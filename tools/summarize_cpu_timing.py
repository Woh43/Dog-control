#!/usr/bin/env python3
"""Summarize CPU telemetry inside each timing probe's measured window.

The inputs are immutable captures from cpu_timing_environment.py and the
matching probe's metadata.json. Policy frequencies are sysfs polling samples;
they are not measurements of the frequency experienced by each thread.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
import json
from pathlib import Path
import statistics


def cpus(value: str) -> list[int]:
    result = set()
    for part in value.split(","):
        if not part:
            continue
        if "-" in part:
            first, last = map(int, part.split("-"))
            result.update(range(first, last + 1))
        else:
            result.add(int(part))
    return sorted(result)


def read_json(path: Path) -> dict:
    return json.loads(path.read_text())


def frequency_summary(rows: list[dict], policy: str) -> dict:
    key = f"{policy}_scaling_cur_freq"
    values = [int(row[key]) for row in rows if row.get(key)]
    governors = Counter(row[f"{policy}_scaling_governor"] for row in rows
                        if row.get(f"{policy}_scaling_governor"))
    return {
        "samples": len(values), "min_khz": min(values) if values else None,
        "max_khz": max(values) if values else None,
        "median_khz": statistics.median(values) if values else None,
        "at_2400000_khz_samples": values.count(2400000),
        "at_2400000_khz_sample_fraction": values.count(2400000) / len(values) if values else None,
        "frequency_sample_counts_khz": dict(sorted(Counter(values).items())),
        "governor_sample_counts": dict(governors),
    }


def thread_summary(snapshots: list[dict], meta: dict) -> dict:
    control_cpus = cpus(meta["cpu_affinity"])
    nmpc_cpus = cpus(meta.get("nmpc_cpu_affinity_request", "")) or control_cpus
    split = control_cpus != nmpc_cpus
    mpc_threads = int(meta.get("mpc_threads", 0))
    io_threads = 4 if meta.get("motor_ports_opened") else 0
    expected_threads = 1 + io_threads + mpc_threads
    expected_named_workers = 1 if mpc_threads else 0
    records: dict[int, dict] = {}
    violations = []
    main_ids = set()
    complete = 0
    size_counts = Counter()
    observed_schedulers = Counter()
    for snapshot in snapshots:
        main_id = int(snapshot["pid"])
        main_ids.add(main_id)
        threads = snapshot["threads"]
        size_counts[len(threads)] += 1
        if len(threads) == expected_threads:
            complete += 1
        control_count = 0
        nmpc_count = 0
        named_workers = 0
        for thread in threads:
            tid = int(thread["tid"])
            allowed = sorted(thread["allowed_cpus"])
            if allowed == control_cpus:
                control_count += 1
            if allowed == nmpc_cpus:
                nmpc_count += 1
            named_workers += thread.get("comm") == "dog-timing-mpc"
            observed_schedulers[int(thread["scheduler"])] += 1
            record = records.setdefault(tid, {"tid": tid, "names": set(), "affinity_masks": set(),
                                              "schedulers": set(), "priorities": set(), "last_cpus": set(),
                                              "observations": 0, "is_main": tid == main_id})
            record["names"].add(thread["comm"])
            record["affinity_masks"].add(tuple(allowed))
            record["schedulers"].add(thread["scheduler"])
            record["priorities"].add(thread["priority"])
            record["last_cpus"].add(thread["last_cpu"])
            record["observations"] += 1
            if tid == main_id and allowed != control_cpus:
                violations.append({"monotonic_ns": snapshot["monotonic_ns"], "tid": tid,
                                   "reason": "main thread affinity differs", "allowed_cpus": allowed})
            if thread["comm"] == "dog-timing-mpc" and allowed != nmpc_cpus:
                violations.append({"monotonic_ns": snapshot["monotonic_ns"], "tid": tid,
                                   "reason": "NMPC worker affinity differs", "allowed_cpus": allowed})
            if thread["last_cpu"] not in allowed:
                violations.append({"monotonic_ns": snapshot["monotonic_ns"], "tid": tid,
                                   "reason": "last CPU outside current affinity; /proc observation may race"})
        if len(threads) == expected_threads:
            counts_ok = ((control_count == 1 + io_threads and nmpc_count == mpc_threads) if split
                         else control_count == expected_threads)
            if not counts_ok or named_workers != expected_named_workers:
                violations.append({"monotonic_ns": snapshot["monotonic_ns"],
                                   "reason": "thread grouping differs", "control_mask_count": control_count,
                                   "nmpc_mask_count": nmpc_count, "named_nmpc_workers": named_workers})
    result_records = []
    for tid, record in sorted(records.items()):
        record = {key: sorted(value) if isinstance(value, set) else value for key, value in record.items()}
        if record["is_main"]:
            record["role"] = "control main"
        elif "dog-timing-mpc" in record["names"]:
            record["role"] = "NMPC worker"
        elif split and record["affinity_masks"] == [tuple(nmpc_cpus)]:
            record["role"] = "NMPC helper inferred from construction affinity"
        elif split and record["affinity_masks"] == [tuple(control_cpus)]:
            record["role"] = "IO worker inferred from mask and process thread inventory"
        else:
            record["role"] = "helper or IO worker; shared masks do not identify role"
        result_records.append(record)
    return {
        "snapshots": len(snapshots), "complete_inventory_snapshots": complete,
        "thread_count_histogram": dict(sorted(size_counts.items())),
        "expected_total_threads": expected_threads, "expected_control_cpus": control_cpus,
        "expected_nmpc_cpus": nmpc_cpus, "expected_control_plus_io_threads": 1 + io_threads,
        "expected_nmpc_worker_plus_helpers": mpc_threads, "split_masks": split,
        "expected_masks_are_disjoint": not bool(set(control_cpus) & set(nmpc_cpus)) if mpc_threads else None,
        "scheduler_observation_counts": dict(sorted(observed_schedulers.items())),
        "verified_at_all_complete_snapshots": bool(complete) and not violations,
        "violations": violations, "threads": result_records,
    }


def summarize_case(probe: Path, cpu: Path) -> dict:
    meta = read_json(probe / "metadata.json")
    begin, finish = int(meta["begin_ns"]), int(meta["finish_ns"])
    if meta.get("clock") != "CLOCK_MONOTONIC" or finish <= begin:
        raise ValueError(f"invalid monotonic interval for {probe}")
    with (cpu / "samples.csv").open(newline="") as source:
        all_rows = list(csv.DictReader(source))
    rows = [row for row in all_rows if begin <= int(row["monotonic_ns"]) <= finish]
    snapshots = [snapshot for line in (cpu / "threads.jsonl").read_text().splitlines()
                 if (snapshot := json.loads(line)) and begin <= int(snapshot["monotonic_ns"]) <= finish]
    thermal_values = defaultdict(list)
    for row in rows:
        for key, value in row.items():
            if key.startswith("thermal_zone") and key.endswith("_millicelsius") and value:
                thermal_values[key.removesuffix("_millicelsius")].append(int(value))
    maximums = {name: max(values) / 1000 for name, values in sorted(thermal_values.items())}
    intervals_ms = [(int(right["monotonic_ns"]) - int(left["monotonic_ns"])) / 1e6
                    for left, right in zip(rows, rows[1:])]
    wrapper = read_json(cpu / "result.json")
    return {
        "probe_directory": str(probe), "cpu_directory": str(cpu),
        "begin_ns": begin, "finish_ns": finish, "measured_duration_s": (finish - begin) / 1e9,
        "target_hz": meta.get("target_hz"), "mpc_hz": meta.get("mpc_hz"),
        "wbc_reuse": meta.get("wbc_reuse"), "phase": meta.get("phase"),
        "completed_requested_duration": meta.get("completed_requested_duration"),
        "wrapper_returncode": wrapper.get("returncode"),
        "window_samples": len(rows), "all_capture_samples": len(all_rows),
        "first_sample_ns": int(rows[0]["monotonic_ns"]) if rows else None,
        "last_sample_ns": int(rows[-1]["monotonic_ns"]) if rows else None,
        "sample_interval_ms": {"min": min(intervals_ms) if intervals_ms else None,
                               "median": statistics.median(intervals_ms) if intervals_ms else None,
                               "max": max(intervals_ms) if intervals_ms else None},
        "policy4": frequency_summary(rows, "policy4"),
        "policy6": frequency_summary(rows, "policy6"),
        "maximum_thermal_celsius": max(maximums.values()) if maximums else None,
        "thermal_zone_maximums_celsius": maximums,
        "thread_affinity": thread_summary(snapshots, meta),
        "governors_unchanged_at_endpoints": wrapper.get("governors_before") == wrapper.get("governors_after"),
        "system_mutations": wrapper.get("system_mutations"),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path, help="Parent containing CASE and CASE_cpu directories")
    parser.add_argument("--case", action="append", help="Select case names; repeat for multiple cases")
    parser.add_argument("--output", type=Path, help="Write derived JSON; inputs remain untouched")
    args = parser.parse_args()
    cases = {}
    pending = []
    cpu_paths = ([args.directory / f"{name}_cpu" for name in args.case] if args.case
                 else sorted(args.directory.glob("*_cpu")))
    for cpu in cpu_paths:
        probe = cpu.with_name(cpu.name.removesuffix("_cpu"))
        required = [probe / "metadata.json", cpu / "result.json", cpu / "samples.csv", cpu / "threads.jsonl"]
        missing = [str(path) for path in required if not path.is_file()]
        if missing:
            pending.append({"case": probe.name, "missing": missing})
        else:
            cases[probe.name] = summarize_case(probe, cpu)
    result = {
        "cases": cases, "pending": pending,
        "limits": [
            "All CPU samples and thread snapshots are filtered to metadata.begin_ns..finish_ns.",
            "Frequency is a sysfs policy-level polling observation, not each thread's actual frequency at each instant.",
            "2.4 GHz ratios count samples; they are not time-weighted residency estimates.",
            "Thread snapshots occur about once per second and do not constitute scheduler tracing.",
            "Helper/IO roles are inferred from configured construction masks and the expected thread inventory; comm names are inherited.",
            "Governor and affinity observations do not establish controlled-frequency or hard-real-time conditions.",
        ],
    }
    content = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(content)
    else:
        print(content, end="")


if __name__ == "__main__":
    main()
