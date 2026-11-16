#!/usr/bin/env python3
"""Bounded native rosbag recording; never opens hardware or sends commands.

Only an owned directory named ``latest`` is replaced. The readable bag is its
``bag`` child. Keep segment zero and the latest seven segments. 64 MiB is a
split trigger, not an exact filesystem cap: a write/cache batch can cross it,
and a closed segment can await the 250 ms cleanup poll. Native SQLite/DDS and
metadata add overhead; the 4 MiB cache uses two buffers (up to 8 MiB).
"""

import argparse
from collections import Counter
from contextlib import closing
import fcntl
import json
import os
from pathlib import Path
import re
import shutil
import signal
import sqlite3
import stat
import subprocess
import sys
import time


OWNER = '.custom_dog_latest_rosbag.json'
TOOL = 'custom_dog_control.record_latest_rosbag'
SEGMENT = re.compile(r'bag_(\d+)\.db3')
MAX_BAG_SIZE = 64 * 1024 * 1024
MAX_SEGMENTS = 8
CACHE_SIZE = 4 * 1024 * 1024
POLL_SECONDS = 0.25
TOPICS = (
    '/imu', '/joint_states', '/joy', '/cmd_vel',
    '/nmpc_wbc_controller/diagnostics',
    '/nmpc_wbc_controller/control_mode',
    '/nmpc_wbc_controller/contact_plan',
    '/odom', '/tf', '/tf_static', '/robot_description',
)


def report(message):
    print('[运行录包] ' + message, flush=True)


def safe_path(value):
    path = Path(value).expanduser()
    if not path.is_absolute() or '..' in path.parts or path.name != 'latest':
        raise ValueError('output_dir 必须是以 latest 结尾、不含 .. 的绝对专用目录')
    for component in (*reversed(path.parents), path):
        if component.is_symlink():
            raise ValueError(f'拒绝符号链接路径：{component}')
    return path


class LatestDirectory:
    """Hold a parent lock while replacing/recording/finalizing our own run."""

    def __init__(self, value):
        self.path = safe_path(value)
        self.fd = None

    def __enter__(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        safe_path(self.path)
        lock = self.path.parent / '.latest-rosbag.lock'
        self.fd = os.open(lock, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
        try:
            info = os.fstat(self.fd)
            if not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid():
                raise ValueError('录包锁不是当前用户的普通文件')
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.prepare()
            return self
        except Exception:
            self.__exit__(None, None, None)
            raise

    def prepare(self):
        marker = {'tool': TOOL, 'version': 1, 'output_dir': str(self.path)}
        if self.path.exists():
            if not self.path.is_dir() or self.path.stat().st_uid != os.getuid():
                raise ValueError('latest 不是当前用户的专用目录')
            owner = self.path / OWNER
            if (owner.is_symlink() or not owner.is_file() or
                    owner.stat().st_uid != os.getuid() or owner.stat().st_size > 4096 or
                    json.loads(owner.read_text()) != marker):
                raise ValueError('拒绝删除没有本工具 owner marker 的 latest')
            for child in self.path.iterdir():
                if child.is_symlink() or child.name not in (OWNER, 'bag'):
                    raise ValueError(f'latest 含非本工具文件，拒绝清理：{child}')
                if child.name == 'bag':
                    if not child.is_dir():
                        raise ValueError('bag 不是普通目录')
                    for item in child.iterdir():
                        allowed = (item.name in ('metadata.yaml', 'metadata.yaml.tmp') or
                                   re.fullmatch(r'bag_\d+\.db3(?:-(?:journal|wal|shm))?', item.name))
                        if item.is_symlink() or not item.is_file() or not allowed:
                            raise ValueError(f'bag 含未知文件，拒绝清理：{item}')
            if not shutil.rmtree.avoids_symlink_attacks:
                raise RuntimeError('当前平台不能安全清理目录')
            shutil.rmtree(self.path)
            report('已删除上一份本工具 latest；其他日志保留。')
        self.path.mkdir()
        with (self.path / OWNER).open('x', encoding='utf-8') as stream:
            json.dump(marker, stream)
            stream.write('\n')

    def __exit__(self, *_):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


def segment_paths(bag):
    result = []
    if not bag.exists():
        return result
    if bag.is_symlink() or not bag.is_dir():
        raise ValueError('录包输出被替换为非普通目录')
    for path in bag.iterdir():
        match = SEGMENT.fullmatch(path.name)
        if match:
            if path.is_symlink() or not path.is_file():
                raise ValueError(f'拒绝非普通分片：{path}')
            result.append((int(match[1]), path))
    return sorted(result)


def open_files(pid):
    """Native ros2 CLI owns its SQLite FDs; threads share the same FD table."""
    directory = Path('/proc') / str(pid) / 'fd'
    result = set()
    for fd in directory.iterdir():
        try:
            result.add(os.readlink(fd))
        except FileNotFoundError:
            pass  # A descriptor closed during inspection.
    return result


class SegmentRetention:
    def __init__(self, bag, keep=MAX_SEGMENTS):
        if not 2 <= keep <= MAX_SEGMENTS:
            raise ValueError('保留分片数必须在 2 到 8 之间')
        self.bag = Path(bag)
        self.keep = keep
        self.removed = 0

    def prune(self, opened):
        paths = segment_paths(self.bag)
        if not paths:
            return 0
        if paths[0][0] != 0:
            raise ValueError('首片 bag_0.db3 缺失，停止录制以免丢失静态模型')
        newest = paths[-1][0]
        retain = {0, *(number for number, _ in paths[-(self.keep - 1):])}
        removed = 0
        for number, path in paths:
            if number in retain or number == newest:
                continue
            # Even after a newer file appears, do not unlink a file until the
            # writer has actually closed it. Sidecars indicate unfinished IO.
            if (str(path) in opened or
                    any(Path(str(path) + suffix).exists()
                        for suffix in ('-journal', '-wal', '-shm'))):
                continue
            path.unlink()
            removed += 1
            self.removed += 1
        return removed


def verify_metadata(bag):
    """Verify reindex/native metadata against closed SQLite files, without CDR decoding."""
    import yaml
    metadata = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    paths = segment_paths(bag)
    names = [path.name for _, path in paths]
    if metadata['relative_file_paths'] != names:
        raise ValueError('metadata 分片列表与实际文件或编号顺序不一致')
    total = 0
    starts, ends = [], []
    topics = Counter()
    files = {item['path']: item for item in metadata['files']}
    if set(files) != set(names):
        raise ValueError('metadata files 与实际分片不一致')
    for _, path in paths:
        with closing(sqlite3.connect(path.as_uri() + '?mode=ro&immutable=1', uri=True)) as db:
            count, first, last = db.execute('SELECT count(*), min(timestamp), max(timestamp) FROM messages').fetchone()
            for name, kind, serialization, amount in db.execute(
                    'SELECT t.name,t.type,t.serialization_format,count(m.id) '
                    'FROM topics t LEFT JOIN messages m ON m.topic_id=t.id GROUP BY t.id'):
                topics[(name, kind, serialization)] += amount
        total += count
        entry = files[path.name]
        if entry['message_count'] != count:
            raise ValueError(f'metadata 分片消息数不符：{path.name}')
        if count:
            starts.append(first)
            ends.append(last)
            if (entry['starting_time']['nanoseconds_since_epoch'] != first or
                    entry['duration']['nanoseconds'] != last - first):
                raise ValueError(f'metadata 分片时间不符：{path.name}')
    recorded_topics = Counter()
    for item in metadata['topics_with_message_count']:
        topic = item['topic_metadata']
        recorded_topics[(topic['name'], topic['type'], topic['serialization_format'])] += item['message_count']
    if total != metadata['message_count'] or topics != recorded_topics:
        raise ValueError('metadata 总消息数或各主题消息数不符')
    if starts and (metadata['starting_time']['nanoseconds_since_epoch'] != min(starts) or
                   metadata['duration']['nanoseconds'] != max(ends) - min(starts)):
        raise ValueError('metadata 总起止时间不符')
    return total, len(paths)


def complete_reindexed_metadata(bag):
    """Humble reindex leaves files=[] and sums durations, omitting removed gaps."""
    import yaml
    path = bag / 'metadata.yaml'
    document = yaml.safe_load(path.read_text())
    metadata = document['rosbag2_bagfile_information']
    files = []
    starts, ends = [], []
    for _, segment in segment_paths(bag):
        with closing(sqlite3.connect(segment.as_uri() + '?mode=ro&immutable=1', uri=True)) as db:
            count, first, last = db.execute('SELECT count(*), min(timestamp), max(timestamp) FROM messages').fetchone()
        files.append({'path': segment.name, 'message_count': count,
                      'starting_time': {'nanoseconds_since_epoch': first if count else 0},
                      'duration': {'nanoseconds': last - first if count else 0}})
        if count:
            starts.append(first)
            ends.append(last)
    metadata['files'] = files
    metadata['relative_file_paths'] = [item['path'] for item in files]
    metadata['starting_time'] = {'nanoseconds_since_epoch': min(starts) if starts else 0}
    metadata['duration'] = {'nanoseconds': max(ends) - min(starts) if starts else 0}
    temporary = bag / 'metadata.yaml.tmp'
    try:
        with temporary.open('x', encoding='utf-8') as stream:
            yaml.safe_dump(document, stream, sort_keys=False)
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def finalize(bag, retention):
    metadata = bag / 'metadata.yaml'
    try:
        retention.prune(set())
        if not segment_paths(bag):
            raise RuntimeError('原生录包没有生成 SQLite 分片')
        if retention.removed or not metadata.exists():
            from rosbag2_py import Reindexer, StorageOptions
            # Never leave stale metadata referring to deleted segments if
            # repair fails. Preserve native types/QoS and correct Humble's
            # missing per-file metadata and duration across deleted gaps.
            metadata.unlink(missing_ok=True)
            Reindexer().reindex(StorageOptions(uri=str(bag), storage_id='sqlite3'))
            complete_reindexed_metadata(bag)
        count, files = verify_metadata(bag)
    except Exception:
        metadata.unlink(missing_ok=True)
        raise
    report(f'已正常封存：{bag}；{files} 个分片，{count} 条消息。')


def lower_priority():
    try:
        os.nice(5)
    except OSError as error:
        report(f'无法降低录包优先级，继续记录：{error}')
    try:
        compatible = Path('/proc/device-tree/compatible')
        if not compatible.exists() or b'rk3588' not in compatible.read_bytes():
            return
        capacity = {int(path.parent.name[3:]): int(path.read_text())
                    for path in Path('/sys/devices/system/cpu').glob('cpu[0-9]*/cpu_capacity')}
        if capacity:
            cpus = {cpu for cpu, value in capacity.items() if value == min(capacity.values())}
            os.sched_setaffinity(0, cpus)
            report('录包使用小核：' + ','.join(map(str, sorted(cpus))))
    except (OSError, ValueError) as error:
        report(f'无法设置录包小核亲和，继续记录：{error}')


def recorder_command(executable, bag, qos, size):
    return [executable, 'bag', 'record', '--storage', 'sqlite3', '--output', str(bag),
            '--max-bag-size', str(size), '--max-cache-size', str(CACHE_SIZE),
            '--compression-mode', 'none', '--qos-profile-overrides-path', str(qos), *TOPICS]


def stop_child(process):
    if process.poll() is not None:
        return False
    for sig, timeout in ((signal.SIGINT, 20), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            return False
        try:
            process.wait(timeout=timeout)
            return sig != signal.SIGINT
        except subprocess.TimeoutExpired:
            report(f'录包子进程未在 {timeout}s 内退出，升级终止信号。')
    raise RuntimeError('无法结束本工具的录包子进程')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--qos-profile-overrides-path', required=True, type=Path)
    parser.add_argument('--max-bag-size', type=int, default=MAX_BAG_SIZE,
                        help='分片触发字节数，默认64MiB；测试可缩小，最大64MiB')
    parser.add_argument('--max-segments', type=int, default=MAX_SEGMENTS)
    args = parser.parse_args(argv)
    if not 128 * 1024 <= args.max_bag_size <= MAX_BAG_SIZE:
        parser.error('--max-bag-size 必须在 128KiB 到 64MiB 之间')
    if not 2 <= args.max_segments <= MAX_SEGMENTS:
        parser.error('--max-segments 必须在 2 到 8 之间')
    stop = False

    def request_stop(_signum, _frame):
        nonlocal stop
        stop = True

    previous = {sig: signal.signal(sig, request_stop)
                for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    failed = False
    try:
        executable = shutil.which('ros2')
        qos = args.qos_profile_overrides_path.resolve(strict=True)
        if executable is None or not qos.is_file():
            raise ValueError('缺少 ros2 或 QoS 配置文件')
        # Preflight before removing a prior valid run. This import creates no node.
        from rosbag2_py import Reindexer, StorageOptions  # noqa: F401
        lower_priority()
        with LatestDirectory(args.output_dir) as owned:
            bag = owned.path / 'bag'
            retention = SegmentRetention(bag, args.max_segments)
            report(f'输出：{bag}；仅保留本次运行，首片及最近 {args.max_segments - 1} 片。')
            report(f'分片触发 {args.max_bag_size} 字节；默认约512MiB加写批次/轮询边界和元数据开销，'
                   '不是硬磁盘配额。4MiB双缓冲最多约8MiB；不压缩，不发送控制指令。')
            process = subprocess.Popen(recorder_command(executable, bag, qos, args.max_bag_size),
                                       start_new_session=True, pass_fds=(owned.fd,))
            # The native writer shares the flock's open-file description. If
            # this wrapper is killed, a new run cannot delete its live bag;
            # the lock is released only after the native writer also exits.
            try:
                while not stop and process.poll() is None:
                    if len(segment_paths(bag)) > retention.keep:
                        retention.prune(open_files(process.pid))
                    time.sleep(POLL_SECONDS)
                if not stop:
                    raise RuntimeError(f'原生 ros2 bag record 意外退出，状态 {process.returncode}')
            except Exception as error:
                failed = True
                report(f'录包失败：{error}；仅退出记录器，控制器不受本工具指令影响。')
            finally:
                if stop_child(process):
                    failed = True
                if process.returncode != 0:
                    failed = True
                    report(f'原生录包退出状态：{process.returncode}')
                finalize(bag, retention)
    except Exception as error:
        failed = True
        report(f'记录器退出：{error}；未停止或复位控制器。')
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)
    return 1 if failed else 0


if __name__ == '__main__':
    raise SystemExit(main())
