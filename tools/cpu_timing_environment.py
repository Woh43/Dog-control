#!/usr/bin/env python3
"""Capture CPU conditions around an isolated timing command.

This wrapper changes only process CPU affinity. It never writes sysfs,
changes a governor, elevates privileges, or changes the command's scheduler.
The command itself may select a scheduler for individual benchmark threads.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
from pathlib import Path
import resource
import signal
import subprocess
import sys
import time


CPU_ROOT = Path("/sys/devices/system/cpu")
THERMAL_ROOT = Path("/sys/class/thermal")


def read_text(path: Path) -> str | None:
    try:
        return path.read_text().strip()
    except OSError:
        return None


def write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")


def parse_cpus(value: str) -> set[int]:
    cpus: set[int] = set()
    for item in value.split(","):
        parts = item.split("-")
        if len(parts) == 1:
            cpus.add(int(parts[0]))
        elif len(parts) == 2:
            first, last = map(int, parts)
            if last < first:
                raise ValueError("CPU range is reversed")
            cpus.update(range(first, last + 1))
        else:
            raise ValueError("invalid CPU list")
    if not cpus or min(cpus) < 0:
        raise ValueError("CPU list must be nonempty and nonnegative")
    return cpus


def cpu_times() -> dict[str, list[int]]:
    text = read_text(Path("/proc/stat")) or ""
    return {parts[0]: list(map(int, parts[1:])) for line in text.splitlines()
            if (parts := line.split()) and parts[0].startswith("cpu")}


def snapshot() -> dict:
    policies = {}
    for policy in sorted((CPU_ROOT / "cpufreq").glob("policy*")):
        policies[policy.name] = {name: read_text(policy / name) for name in (
            "affected_cpus", "related_cpus", "scaling_available_governors",
            "scaling_governor", "scaling_cur_freq", "scaling_min_freq",
            "scaling_max_freq", "cpuinfo_min_freq", "cpuinfo_max_freq")}
    status = read_text(Path("/proc/self/status")) or ""
    cgroup = read_text(Path("/proc/self/cgroup")) or ""
    cgroup_paths = {Path("/sys/fs/cgroup")}
    for line in cgroup.splitlines():
        if line.startswith("0::"):
            here = Path("/sys/fs/cgroup") / line[3:].lstrip("/")
            while here.is_relative_to(Path("/sys/fs/cgroup")):
                cgroup_paths.add(here)
                if here == Path("/sys/fs/cgroup"):
                    break
                here = here.parent
    return {
        "monotonic_ns": time.monotonic_ns(), "unix_time_ns": time.time_ns(),
        "uid": os.getuid(), "kernel": os.uname().release,
        "allowed_cpus": sorted(os.sched_getaffinity(0)),
        "scheduler": os.sched_getscheduler(0),
        "rtprio_limit": resource.getrlimit(resource.RLIMIT_RTPRIO),
        "memlock_limit": resource.getrlimit(resource.RLIMIT_MEMLOCK),
        "process_status": [line for line in status.splitlines()
                           if line.startswith(("Cap", "Cpus_allowed", "NoNewPrivs", "Seccomp"))],
        "policies": policies,
        "thermal": {zone.name: {"type": read_text(zone / "type"),
                                 "temp_millicelsius": read_text(zone / "temp")}
                    for zone in sorted(THERMAL_ROOT.glob("thermal_zone*"))},
        "sysctl": {name: read_text(Path("/proc/sys/kernel") / name) for name in (
            "sched_rt_runtime_us", "sched_rt_period_us", "perf_event_paranoid")},
        "cgroup_membership": cgroup,
        "cgroups": {str(path): {name: value for name in (
            "cgroup.controllers", "cgroup.subtree_control", "cpu.max", "cpu.weight",
            "cpu.stat", "cpu.pressure", "cpu.rt_runtime_us", "cpuset.cpus.effective")
            if (value := read_text(path / name)) is not None} for path in sorted(cgroup_paths)},
        "loadavg": list(os.getloadavg()),
    }


def thread_snapshot(pid: int) -> dict:
    threads = []
    for task in sorted(Path(f"/proc/{pid}/task").glob("*")):
        try:
            tid = int(task.name)
            stat = (task / "stat").read_text().rsplit(")", 1)[1].split()
            threads.append({
                "tid": tid, "comm": read_text(task / "comm"),
                "allowed_cpus": sorted(os.sched_getaffinity(tid)),
                "scheduler": os.sched_getscheduler(tid),
                "priority": os.sched_getparam(tid).sched_priority,
                "last_cpu": int(stat[36]), "user_ticks": int(stat[11]),
                "system_ticks": int(stat[12]),
            })
        except (OSError, ProcessLookupError, ValueError):
            continue
    return {"monotonic_ns": time.monotonic_ns(), "pid": pid, "threads": threads}


def dynamic_sample(previous: dict[str, list[int]] | None) -> tuple[dict, dict]:
    row: dict[str, object] = {"monotonic_ns": time.monotonic_ns(), "unix_time_ns": time.time_ns()}
    row.update({f"load_{label}": value for label, value in zip(("1m", "5m", "15m"), os.getloadavg())})
    for policy in sorted((CPU_ROOT / "cpufreq").glob("policy*")):
        for name in ("scaling_cur_freq", "scaling_governor", "scaling_min_freq", "scaling_max_freq"):
            row[f"{policy.name}_{name}"] = read_text(policy / name)
    for zone in sorted(THERMAL_ROOT.glob("thermal_zone*")):
        row[f"{zone.name}_millicelsius"] = read_text(zone / "temp")
    current = cpu_times()
    for cpu, values in current.items():
        old = previous.get(cpu) if previous else None
        if old:
            # guest/guest_nice are already included in user/nice.
            total = sum(values[:8]) - sum(old[:8])
            idle = sum(values[3:5]) - sum(old[3:5])
            row[f"{cpu}_busy_percent"] = round(100 * (total - idle) / total, 3) if total > 0 else None
        else:
            row[f"{cpu}_busy_percent"] = None
    return row, current


def rt_permission_probe() -> dict:
    try:
        result = subprocess.run(["chrt", "-f", "10", "/bin/true"], text=True,
                                capture_output=True, timeout=3)
        return {"command": ["chrt", "-f", "10", "/bin/true"],
                "returncode": result.returncode, "stdout": result.stdout,
                "stderr": result.stderr,
                "scope": "short-lived child only; does not alter runner scheduler"}
    except (OSError, subprocess.TimeoutExpired) as error:
        return {"error": str(error)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--cpus", default="4-7", help="Child process affinity; default 4-7")
    parser.add_argument("--sampler-cpus", help="Optional separate affinity for this sampling process")
    parser.add_argument("--interval", type=float, default=0.1)
    parser.add_argument("--timeout", type=float, default=900.0)
    parser.add_argument("--probe-rt", action="store_true", help="Run one child-only FIFO permission check")
    parser.add_argument("--expect-governor", choices=("any", "performance"), default="any",
                        help="Observe governor condition; never changes sysfs")
    parser.add_argument("--snapshot-only", action="store_true")
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if not 0.05 <= args.interval <= 5 or not 0 < args.timeout <= 7200:
        parser.error("interval must be 0.05..5 seconds and timeout 0..7200 seconds")
    try:
        requested = parse_cpus(args.cpus)
        original_affinity = os.sched_getaffinity(0)
        sampler_affinity = parse_cpus(args.sampler_cpus) if args.sampler_cpus else original_affinity
    except ValueError as error:
        parser.error(str(error))
    if not requested <= original_affinity or not sampler_affinity <= original_affinity:
        parser.error("requested CPUs are outside this runner's allowed affinity")
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if not command and not args.snapshot_only:
        parser.error("provide a command after --, or use --snapshot-only")
    args.output.mkdir(parents=True, exist_ok=False)
    before = snapshot()
    write_json(args.output / "before.json", before)
    if args.probe_rt:
        write_json(args.output / "rt_permission.json", rt_permission_probe())
    big_policies = {name: policy for name, policy in before["policies"].items()
                    if set(map(int, (policy["related_cpus"] or "").split())) & requested}
    governor_ok = bool(big_policies) and all(policy["scaling_governor"] == "performance" for policy in big_policies.values())
    conditions = {
        "governor_request": args.expect_governor,
        "performance_governor_present": governor_ok,
        "fixed_performance_comparison": "available" if governor_ok else "skipped_not_performance",
        "system_mutations": [], "restoration_required": False,
        "scope": "Observation only. CPU frequency may change under thermal or power limits even with performance governor.",
    }
    write_json(args.output / "conditions.json", conditions)
    if args.snapshot_only:
        write_json(args.output / "after.json", snapshot())
        return 0

    interrupted = 0
    process = None

    def receive_signal(signum: int, _frame: object) -> None:
        nonlocal interrupted
        interrupted = signum

    previous_handlers = {sig: signal.signal(sig, receive_signal) for sig in (signal.SIGINT, signal.SIGTERM)}
    begin = time.monotonic()
    next_sample = begin
    next_threads = begin
    previous_times = None
    count = 0
    timed_out = False
    stop_sent_at = None
    error = None
    try:
        os.sched_setaffinity(0, sampler_affinity)
        with (args.output / "stdout.log").open("w") as stdout, (args.output / "stderr.log").open("w") as stderr, \
                (args.output / "samples.csv").open("w", newline="") as samples, \
                (args.output / "threads.jsonl").open("w") as threads:
            # No sampling threads exist: preexec_fn only sets child affinity.
            process = subprocess.Popen(command, stdout=stdout, stderr=stderr, start_new_session=True,
                                       preexec_fn=lambda: os.sched_setaffinity(0, requested))
            writer = None
            while True:
                now = time.monotonic()
                if now >= next_sample:
                    row, previous_times = dynamic_sample(previous_times)
                    if writer is None:
                        writer = csv.DictWriter(samples, fieldnames=list(row))
                        writer.writeheader()
                    writer.writerow(row)
                    count += 1
                    next_sample += args.interval
                    if next_sample < now:
                        next_sample = now + args.interval
                if now >= next_threads:
                    threads.write(json.dumps(thread_snapshot(process.pid)) + "\n")
                    next_threads = now + 1.0
                if process.poll() is not None:
                    break
                timed_out = now - begin > args.timeout
                if (interrupted or timed_out) and stop_sent_at is None:
                    os.killpg(process.pid, signal.SIGTERM)
                    stop_sent_at = now
                elif stop_sent_at is not None and now - stop_sent_at > 3:
                    os.killpg(process.pid, signal.SIGKILL)
                time.sleep(min(0.05, max(0.001, next_sample - time.monotonic())))
    except BaseException as exception:
        error = repr(exception)
        raise
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        os.sched_setaffinity(0, original_affinity)
        after = snapshot()
        write_json(args.output / "after.json", after)
        write_json(args.output / "result.json", {
            "command": command, "child_pid": process.pid if process else None,
            "child_requested_cpus": sorted(requested), "returncode": process.returncode if process else None,
            "sampler_requested_cpus": sorted(sampler_affinity),
            "elapsed_s": time.monotonic() - begin, "sample_interval_s": args.interval,
            "samples": count, "interrupted_signal": interrupted, "timed_out": timed_out,
            "error": error, "system_mutations": [], "restoration_required": False,
            "governors_before": {p: v["scaling_governor"] for p, v in before["policies"].items()},
            "governors_after": {p: v["scaling_governor"] for p, v in after["policies"].items()},
            "limitations": ["Userspace polling is not a kernel scheduler trace.",
                            "Thread snapshots occur once per second and can miss short-lived threads.",
                            "Sampler consumes CPU time on sampler_requested_cpus."],
        })
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)
    return 128 + interrupted if interrupted else 124 if timed_out else process.returncode


if __name__ == "__main__":
    sys.exit(main())
