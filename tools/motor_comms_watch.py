#!/usr/bin/env python3
"""Start, inspect, or stop a persistent BRAKE-mode motor communication probe.

Default: /dev/ttyS3, IDs 0/1/2, 50 polls per second per ID. All transmitted
position, velocity, torque and gain values are zero; no FOC or calibration.
Use start/status/stop --all for all four buses, or --leg FR/FL/RR/RL.
"""

import argparse
from contextlib import contextmanager
from datetime import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import platform
import signal
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
LOG_ROOT = ROOT / "log"
STATE = LOG_ROOT / "motor-comms-watch.state.json"
PORTS = {"FR": "/dev/ttyS3", "FL": "/dev/ttyS4", "RR": "/dev/ttyS7", "RL": "/dev/ttyS8"}


def write_json(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n")
    temporary.replace(path)


def publish_link(path, target):
    if path.exists() and not path.is_symlink():
        raise RuntimeError("拒绝覆盖现有文件：" + str(path))
    temporary = path.with_name(path.name + f".tmp.{os.getpid()}")
    temporary.symlink_to(target)
    temporary.replace(path)


def publish_state_links(state, leg):
    snapshot, log_path = Path(state["snapshot"]), Path(state["log"])
    prefix = "motor-comms-watch-" + leg.lower()
    publish_link(LOG_ROOT / (prefix + "-latest.json"), snapshot)
    publish_link(LOG_ROOT / (prefix + ".log"), log_path)
    if leg == "FR":
        publish_link(LOG_ROOT / "motor-comms-watch-latest.json", snapshot)
        publish_link(LOG_ROOT / "motor-comms-watch.log", log_path)


def load_state():
    return json.loads(STATE.read_text()) if STATE.exists() else None


def start_ticks(pid):
    # Fields after the final ')' start at field 3 (process state).
    value = Path(f"/proc/{pid}/stat").read_text()
    return value[value.rfind(")") + 2:].split()[19]


def is_running(state):
    if not state:
        return False
    try:
        pid = int(state["pid"])
        return (
            start_ticks(pid) == state["start_ticks"]
            and os.readlink(f"/proc/{pid}/exe") == state["binary"]
        )
    except (FileNotFoundError, ProcessLookupError):
        return False


@contextmanager
def control_lock():
    LOG_ROOT.mkdir(exist_ok=True)
    with (LOG_ROOT / "motor-comms-watch.lock").open("a") as handle:
        fcntl.flock(handle, fcntl.LOCK_EX)
        yield


def show_status(state):
    if not state:
        print("持续通信测试尚未启动。")
        return
    print("状态：" + ("运行中" if is_running(state) else "已停止"))
    print(f"PID：{state['pid']}；串口：{state['port']}")
    print(f"实时数据：{state['snapshot']}")
    print(f"日志：{state['log']}")
    snapshot = Path(state["snapshot"])
    if snapshot.exists():
        data = json.loads(snapshot.read_text())
        for motor in data["motors"]:
            delta = motor["delta"]
            print(
                f"ID {motor['id']}：累计 {motor['valid']}/{motor['attempts']} 有效；"
                f"最近一秒 {delta['valid']}/{delta['attempts']}；"
                f"电机错误码 {motor['last_motor_error']}"
            )


def find_sdk(argument):
    if argument:
        return argument.resolve()
    configured = os.environ.get("UNITREE_ACTUATOR_SDK_ROOT")
    if configured:
        return Path(configured).resolve()
    return ROOT.parent / "unitree_actuator_sdk-main/unitree_actuator_sdk-main"


def build_probe(argument):
    sdk = find_sdk(argument)
    library = sdk / "lib/libUnitreeMotorSDK_Arm64.so"
    if not library.is_file():
        raise RuntimeError("找不到 ARM64 SDK：" + str(library))
    source = ROOT / "tools/motor_comms_watch.cpp"
    digest = hashlib.sha256(source.read_bytes())
    digest.update(str(sdk.resolve()).encode())
    digest.update(f"{library.stat().st_mtime_ns}:{library.stat().st_size}".encode())
    for header in sorted((sdk / "include").rglob("*.h")):
        digest.update(str(header.relative_to(sdk)).encode() + b"\0" + header.read_bytes())
    build = LOG_ROOT / "motor-comms-watch-bin"
    build.mkdir(exist_ok=True)
    # Immutable names let another leg reuse the probe while existing processes
    # keep their executable and stable /proc/PID/exe identity.
    binary = build / ("motor_comms_watch-" + digest.hexdigest()[:20])
    with (build / "build.log").open("a") as log:
        if not binary.exists():
            temporary = binary.with_name(binary.name + f".tmp.{os.getpid()}")
            try:
                subprocess.run(
                    [
                        "c++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-O2",
                        str(source), "-I", str(sdk / "include"), "-L", str(sdk / "lib"),
                        "-Wl,-rpath," + str(sdk / "lib"),
                        "-lUnitreeMotorSDK_Arm64", "-pthread", "-o", str(temporary),
                    ],
                    stdout=log, stderr=subprocess.STDOUT, check=True,
                )
                temporary.replace(binary)
            finally:
                temporary.unlink(missing_ok=True)
        subprocess.run(
            [str(binary), "--check"],
            stdout=log, stderr=subprocess.STDOUT, check=True,
        )
    return binary


def start(args):
    previous = load_state()
    if is_running(previous):
        if Path(args.port).resolve() != Path(previous["port"]):
            raise RuntimeError("已有监视任务运行，请先执行 stop 再切换串口。")
        publish_state_links(previous, args.leg)
        show_status(previous)
        return
    if platform.machine() not in ("aarch64", "arm64"):
        raise RuntimeError("此探针使用 ARM64 Unitree SDK。")
    if not 1 <= args.hz <= 100:
        raise RuntimeError("轮询频率必须为 1..100 Hz。")
    port = Path(args.port).resolve()
    if not str(port).startswith("/dev/") or not port.is_char_device():
        raise RuntimeError("串口必须是 /dev 下现有的字符设备。")
    check = subprocess.run(["fuser", str(port)], capture_output=True, text=True)
    if check.returncode != 1 or check.stdout.strip() or check.stderr.strip():
        raise RuntimeError("串口被占用或占用检查失败：" + check.stdout + check.stderr)

    binary = build_probe(args.sdk)

    output = LOG_ROOT / ("motor-comms-watch-" + datetime.now().strftime("%Y%m%d-%H%M%S-%f"))
    output.mkdir()
    snapshot, log_path = output / "latest.json", output / "monitor.log"
    with log_path.open("a") as log:
        process = subprocess.Popen(
            [str(binary), str(port), str(args.hz), str(snapshot)],
            stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True, close_fds=True,
        )
    try:
        state = {
            "pid": process.pid,
            "start_ticks": start_ticks(process.pid),
            "binary": str(binary.resolve()),
            "port": str(port),
            "leg": args.leg,
            "cycle_hz": args.hz,
            "snapshot": str(snapshot),
            "log": str(log_path),
        }
        write_json(STATE, state)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError("通信任务启动失败：\n" + log_path.read_text()[-2000:])
            if snapshot.exists():
                publish_state_links(state, args.leg)
                show_status(state)
                return
            time.sleep(0.05)
        raise RuntimeError("通信任务未产生状态数据。日志：" + str(log_path))
    except BaseException:
        # Any launcher error after Popen must release its serial child too.
        if process.poll() is None:
            process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
        raise


def stop():
    state = load_state()
    if not is_running(state):
        show_status(state)
        return
    try:
        descriptor = os.pidfd_open(int(state["pid"]))
    except ProcessLookupError:
        show_status(state)
        return
    try:
        # Recheck identity after opening the stable process handle. The signal
        # cannot target an unrelated process if the numeric PID gets reused.
        if is_running(state):
            signal.pidfd_send_signal(descriptor, signal.SIGTERM)
    finally:
        os.close(descriptor)
    deadline = time.monotonic() + 5
    while is_running(state) and time.monotonic() < deadline:
        time.sleep(0.05)
    if is_running(state):
        raise RuntimeError("进程尚未退出，请检查日志：" + state["log"])
    show_status(state)


def main():
    global STATE
    parser = argparse.ArgumentParser(description=__doc__)
    subcommands = parser.add_subparsers(dest="command", required=True)
    launch = subcommands.add_parser("start", help="后台启动持续通信")
    launch.add_argument("--port", help="覆盖所选腿的串口；不可与 --all 一起使用")
    launch.add_argument("--hz", type=int, default=50, help="每个 ID 每秒查询次数")
    launch.add_argument("--sdk", type=Path)
    status_parser = subcommands.add_parser("status", help="显示最新回包统计")
    stop_parser = subcommands.add_parser("stop", help="停止本工具启动的任务并释放串口")
    for command_parser in (launch, status_parser, stop_parser):
        selection = command_parser.add_mutually_exclusive_group()
        selection.add_argument("--all", action="store_true", help="四条腿全部执行")
        selection.add_argument("--leg", choices=tuple(PORTS), help="单条腿，默认 FR")
    args = parser.parse_args()
    if args.command == "start" and args.all and args.port:
        parser.error("--all 使用四路固定映射，不可同时指定 --port")
    selected = list(PORTS) if args.all else [args.leg or "FR"]
    explicit_port = args.port if args.command == "start" else None
    errors = []
    with control_lock():
        for leg in selected:
            STATE = LOG_ROOT / (
                "motor-comms-watch.state.json" if leg == "FR"
                else "motor-comms-watch-" + leg.lower() + ".state.json"
            )
            args.leg = leg
            print(f"[{leg}]", flush=True)
            try:
                if args.command == "start":
                    args.port = explicit_port or PORTS[leg]
                    start(args)
                elif args.command == "status":
                    show_status(load_state())
                else:
                    stop()
            except (OSError, ValueError, KeyError, RuntimeError, subprocess.SubprocessError) as error:
                errors.append(leg)
                print(f"{leg}: {error}", file=sys.stderr)
    if errors:
        raise RuntimeError("以下腿操作未完成：" + ", ".join(errors))


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, KeyError, RuntimeError, subprocess.SubprocessError) as error:
        print(str(error), file=sys.stderr)
        sys.exit(1)
