#!/usr/bin/env python3
"""Summarize isolated OCS2 transcription-stage counters without conflating threads.

Usage: analyze_nmpc_profile.py capture/nmpc_stages.csv [--output summary.json]
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path


CHILDREN = ['dynamics_discretization', 'precomputation', 'cost', 'state_equality',
            'state_input_equality', 'state_inequality', 'state_input_inequality']


def analyze(path):
    rows = list(csv.DictReader(path.open()))
    if not rows:
        raise ValueError('empty profile; verify the instrumented object was linked before the static OCS2 archives')
    totals = {}
    tids = set()
    for row in rows:
        tids.add(int(row['tid']))
        stage = row['stage']
        count, wall, cpu, minimum, maximum = [int(row[k]) for k in
            ['calls', 'wall_ns', 'cpu_ns', 'wall_min_ns', 'wall_max_ns']]
        if count <= 0 or min(wall, cpu, minimum, maximum) < 0 or maximum < minimum:
            raise ValueError('invalid profile counters')
        value = totals.setdefault(stage, dict(calls=0, wall_ns=0, cpu_ns=0,
                                            wall_min_ns=minimum, wall_max_ns=0))
        value['calls'] += count
        value['wall_ns'] += wall
        value['cpu_ns'] += cpu
        value['wall_min_ns'] = min(value['wall_min_ns'], minimum)
        value['wall_max_ns'] = max(value['wall_max_ns'], maximum)
    if 'intermediate_inclusive' not in totals:
        raise ValueError('no intermediate node counters found')
    intermediate_calls = totals['intermediate_inclusive']['calls']
    for stage in CHILDREN:
        if totals.get(stage, {}).get('calls') != intermediate_calls:
            raise ValueError(f'incomplete node-stage counters: {stage}')
    cpu_denominator = sum(totals[s]['cpu_ns'] for s in CHILDREN)
    wall_denominator = sum(totals[s]['wall_ns'] for s in CHILDREN)
    stages = {}
    for stage, value in totals.items():
        c = value['calls']
        stages[stage] = {
            **value, 'mean_wall_us': value['wall_ns'] / c / 1000,
            'mean_cpu_us': value['cpu_ns'] / c / 1000,
            'sum_wall_ms_across_workers': value['wall_ns'] / 1e6,
            'sum_cpu_ms_across_workers': value['cpu_ns'] / 1e6,
        }
        if stage in CHILDREN:
            stages[stage]['fraction_of_intermediate_child_cpu'] = value['cpu_ns'] / cpu_denominator
            stages[stage]['fraction_of_intermediate_child_accumulated_wall'] = value['wall_ns'] / wall_denominator
    exclusive_cpu = sum(totals[s]['cpu_ns'] for s in
                        CHILDREN + ['projection', 'terminal_inclusive', 'event_inclusive'] if s in totals)
    for stage in stages:
        if stage != 'intermediate_inclusive':
            stages[stage]['fraction_of_instrumented_exclusive_cpu'] = totals[stage]['cpu_ns'] / exclusive_cpu
    inclusive = totals['intermediate_inclusive']
    return {
        'schema_version': 1, 'source_csv': str(path),
        'source_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        'clock': 'CLOCK_MONOTONIC and CLOCK_THREAD_CPUTIME_ID',
        'worker_tids': sorted(tids), 'stages': stages,
        'unattributed_intermediate_inclusive_cpu_ms': (inclusive['cpu_ns'] - cpu_denominator) / 1e6,
        'unattributed_intermediate_inclusive_accumulated_wall_ms': (inclusive['wall_ns'] - wall_denominator) / 1e6,
        'limits': [
            'Generated copy of unchanged vendor transcription math with inserted scope timers; production archives/plugins are untouched.',
            'Accumulated wall time sums overlapping workers and must not be interpreted as SQP elapsed wall time or CPU utilization.',
            'Thread CPU time separates time executing from scheduler preemption but includes instrumentation overhead.',
            'Inclusive intermediate totals overlap their seven child stages and are excluded from fraction denominators.',
            'The denominator excludes SQP scheduling, metrics, vector moves, horizon allocation, QP and line search.',
            'Clock reads and counter bookkeeping add overhead; use an uninstrumented run for end-to-end timing.',
            'Final in-flight solve is drained before counters are collected; warmup is excluded when begin is called after initialization.',
            'Synthetic stationary inputs establish a profiling workload, not dynamic motion performance or control stability.',
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = analyze(args.csv)
    text = json.dumps(result, indent=2) + '\n'
    if args.output:
        args.output.write_text(text)
    else:
        print(text, end='')


if __name__ == '__main__':
    main()
