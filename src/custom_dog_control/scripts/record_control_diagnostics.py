#!/usr/bin/env python3
"""Persist controller diagnostics without opening hardware or issuing commands.

The requested output file is the initial segment and is never rotated away.
After it reaches 16 MiB, numbered segments retain only the latest three files.
Each segment is bounded by complete JSON lines; an oversized record or any
write/flush/rotation failure is reported and stops this recorder explicitly.
The controller is not stopped, reset, or otherwise commanded by this process.
"""

from collections import deque
import json
from pathlib import Path
import sys
import time


MAX_SEGMENT_BYTES = 16 * 1024 * 1024
MAX_SEGMENTS = 4
CONTROL_STATUS_NAME = 'custom_dog_control/nmpc_wbc'


def diagnostic_level(value):
    """Humble exposes DiagnosticStatus.level as a one-byte bytes object."""
    if isinstance(value, (bytes, bytearray, memoryview)):
        if len(value) != 1:
            raise ValueError('diagnostic level must contain exactly one byte')
        return int(value[0])
    result = int(value)
    if not 0 <= result <= 255:
        raise ValueError('diagnostic level is outside the byte range')
    return result


def diagnostic_record(message, received_wall_time_ns=None):
    """Copy one real DiagnosticArray; string values and duplicate keys survive."""
    return {
        'schema_version': 1,
        'received_wall_time_ns': (
            time.time_ns() if received_wall_time_ns is None else received_wall_time_ns),
        'header': {
            'stamp': {'sec': int(message.header.stamp.sec),
                      'nanosec': int(message.header.stamp.nanosec)},
            'frame_id': message.header.frame_id,
        },
        'status': [
            {'level': diagnostic_level(status.level), 'name': status.name,
             'message': status.message, 'hardware_id': status.hardware_id,
             'values': [{'key': item.key, 'value': item.value}
                        for item in status.values]}
            for status in message.status
        ],
    }


def status_values(status):
    return {item['key']: item['value'] for item in status['values']}


def requires_immediate_flush(record):
    return any(status['level'] >= 2 or status_values(status).get('mode') == 'FAULT'
               for status in record['status'])


class JsonlSegments:
    """One writer, at most four files, no accumulated diagnostic history."""

    def __init__(self, output_file, max_segment_bytes=MAX_SEGMENT_BYTES,
                 max_segments=MAX_SEGMENTS):
        if not 0 < max_segment_bytes <= MAX_SEGMENT_BYTES:
            raise ValueError('segment limit must be in (0, 16 MiB]')
        if not 2 <= max_segments <= MAX_SEGMENTS:
            raise ValueError('segment count must be in [2, 4]')
        self.initial_path = Path(output_file).expanduser().resolve()
        self.initial_path.parent.mkdir(parents=True, exist_ok=True)
        self.max_segment_bytes = max_segment_bytes
        self.max_segments = max_segments
        self._recent = deque()
        self._part_number = 0
        self._bytes = 0
        self._closed = False
        # Exclusive creation protects logs from a previous launch. The launch
        # supplies a unique directory; never silently truncate an existing run.
        self._stream = self._open(self.initial_path)

    @staticmethod
    def _open(path):
        return open(path, 'x', encoding='utf-8', newline='\n', buffering=65536)

    @property
    def retained_paths(self):
        return (self.initial_path, *self._recent)

    def _rotate(self):
        self.flush()
        self._stream.close()
        self._stream = None
        if len(self._recent) >= self.max_segments - 1:
            self._recent.popleft().unlink()
        self._part_number += 1
        path = self.initial_path.with_name(
            f'{self.initial_path.stem}.part{self._part_number:06d}'
            f'{self.initial_path.suffix}')
        self._stream = self._open(path)
        self._recent.append(path)
        self._bytes = 0

    def append(self, record, force_flush=False):
        if self._closed:
            raise ValueError('diagnostic writer is closed')
        line = json.dumps(record, ensure_ascii=False, allow_nan=False,
                          separators=(',', ':')) + '\n'
        size = len(line.encode('utf-8'))
        if size > self.max_segment_bytes:
            raise ValueError('one diagnostic record exceeds the segment size limit')
        if self._bytes + size > self.max_segment_bytes:
            self._rotate()
        written = self._stream.write(line)
        if written != len(line):
            raise OSError('short diagnostic file write')
        self._bytes += size
        if force_flush:
            self.flush()

    def flush(self):
        if not self._closed and self._stream is not None:
            self._stream.flush()

    def close(self):
        if self._closed:
            return
        self._closed = True
        if self._stream is not None:
            try:
                self._stream.flush()
            finally:
                self._stream.close()


class ModeNotices:
    """Track only the designated controller's last mode and arming flag."""

    def __init__(self):
        self._mode = None
        self._trot_enabled = None

    def update(self, record):
        notices = []
        for status in record['status']:
            if status['name'] != CONTROL_STATUS_NAME:
                continue
            values = status_values(status)
            mode = values.get('mode')
            trot_enabled = values.get('trot_enabled')
            if mode != self._mode:
                if mode == 'FAULT':
                    notices.append(('error', '控制器故障已锁存：' + status['message']))
                elif mode:
                    descriptions = {
                        'PASSIVE': '控制器进入 PASSIVE（被动）。',
                        'CALIBRATION': '控制器进入标定流程。',
                        'STAND_UP': '控制器正在执行起身流程。',
                        'MPC_STANCE': '控制器进入站立控制模式。',
                        'MPC_TROT': '控制器进入 Trot 控制模式。',
                    }
                    notices.append(('info', descriptions.get(mode, '控制器模式：' + mode)))
            if trot_enabled != self._trot_enabled:
                if trot_enabled == 'true' and mode in ('MPC_STANCE', 'MPC_TROT'):
                    notices.append(('info', '已使能行走；摇杆回中时保持站立，需推动摇杆发出速度指令。'))
                elif trot_enabled == 'false' and self._trot_enabled == 'true':
                    notices.append(('info', '行走使能已取消。'))
            self._mode = mode
            self._trot_enabled = trot_enabled
        return notices


def main(args=None):
    # Unit tests exercise serialization/retention with real ROS message objects
    # without constructing a node, discovery, hardware, or command publishers.
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from rclpy.clock import Clock, ClockType
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node

    class Recorder(Node):
        def __init__(self):
            super().__init__('control_diagnostics_recorder', enable_rosout=False,
                             start_parameter_services=False)
            self.failed = False
            self.writer = None
            self.notices = ModeNotices()
            try:
                self.declare_parameter('output_file', '')
                output = self.get_parameter('output_file').value
                if not isinstance(output, str) or not output.strip():
                    raise ValueError('output_file must name a unique diagnostic log file')
                self.writer = JsonlSegments(output)
                self.subscription = self.create_subscription(
                    DiagnosticArray, '/nmpc_wbc_controller/diagnostics', self.receive, 10)
                self.flush_timer = self.create_timer(
                    1.0, self.flush, clock=Clock(clock_type=ClockType.STEADY_TIME))
                self.get_logger().info(
                    f'诊断日志：{self.writer.initial_path}；每段最多 16 MiB，'
                    '保留初始段及最近三个分段。此记录器只订阅诊断，不发送控制指令。')
            except Exception:
                if self.writer is not None:
                    self.writer.close()
                self.destroy_node()
                raise

        def fail(self, error):
            self.failed = True
            self.get_logger().error(
                f'诊断记录失败，记录器将退出：{error}。控制器未被本记录器停止或复位。')

        def receive(self, message):
            if self.failed:
                return
            try:
                record = diagnostic_record(message)
                self.writer.append(record, force_flush=requires_immediate_flush(record))
                for severity, notice in self.notices.update(record):
                    # rclpy caches severity per source call site. Keep info and
                    # error on separate lines so a later FAULT can be reported.
                    if severity == 'error':
                        self.get_logger().error(notice)
                    else:
                        self.get_logger().info(notice)
            except Exception as error:
                self.fail(error)

        def flush(self):
            if not self.failed:
                try:
                    self.writer.flush()
                except Exception as error:
                    self.fail(error)

    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = Recorder()
        while rclpy.ok() and not node.failed:
            rclpy.spin_once(node, timeout_sec=1.0)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        print(f'诊断记录器启动/运行失败：{error}', file=sys.stderr, flush=True)
        exit_code = 1
    finally:
        if node is not None:
            try:
                node.writer.close()
            except Exception as error:
                node.fail(error)
            exit_code = 1 if node.failed else exit_code
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
