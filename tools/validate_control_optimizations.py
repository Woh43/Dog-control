#!/usr/bin/env python3
"""Finite zero-output WBC reuse / thread-affinity comparison; never run Run Dog.

Keeps installed plugins, production parameters, governors and RT permissions
unchanged. Uses a separately built timing executable with encoded zero guards.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

from analyze_timing_probe import analyze
from run_timing_identification import run, runtime_environment


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def free_ports():
    check = subprocess.run(['fuser', '/dev/ttyS3', '/dev/ttyS4', '/dev/ttyS7',
                            '/dev/ttyS8', '/dev/ttyUSB0'], capture_output=True, text=True)
    if check.returncode != 1 or check.stdout.strip() or check.stderr.strip():
        raise RuntimeError('serial device ownership check failed; stop other hardware processes')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--duration', type=float, default=30)
    parser.add_argument('--repetitions', type=int, choices=[1, 2], default=2)
    parser.add_argument('--resume', action='store_true', help='reuse complete retained windows after recorder interruption')
    args = parser.parse_args()
    if not 5 <= args.duration <= 120:
        parser.error('duration must be 5..120 seconds')
    root = Path(__file__).resolve().parents[1]
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=args.resume)
    binary = args.binary.resolve()
    environment = runtime_environment(root)
    watched = [binary, *sorted((root / 'tools').glob('*.hpp')),
               *[root / p for p in [
                   'tools/probe_control_timing.cpp', 'tools/build_timing_probe.py',
                   'tools/analyze_timing_probe.py', 'tools/cpu_timing_environment.py',
                   'tools/validate_control_optimizations.py',
                   'src/custom_dog_control/src/nmpc/NmpcBackend.cpp',
                   'src/custom_dog_control/third_party/legged_wbc/src/WeightedWbc.cpp',
                   'src/custom_dog_control/third_party/legged_wbc/include/legged_wbc/WeightedWbc.h',
                   'src/custom_dog_control/config/controllers.yaml',
                   'src/custom_dog_control/config/real_controller.yaml',
                   'src/custom_dog_control/config/nmpc/task.info',
                   'src/custom_dog_control/config/nmpc/reference.info']]]
    watched += sorted((root / 'install/custom_dog_control/lib').glob('libcustom_dog_control*.so'))
    provenance = {'git_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip(),
                  'sha256': {str(p): digest(p) for p in watched},
                  'output_mode': 'mode 0, all fields zero; numerical output discarded',
                  'system_settings_changed': False, 'installed_plugins_changed': False,
                  'cpu_sampler_cpus': '0-3', 'outer_hz': 250, 'mpc_requested_hz': 50}
    if args.resume:
        previous_path = output / 'provenance.json'
        previous = json.loads(previous_path.read_text())
        runner = str(Path(__file__).resolve())
        for path, expected in previous['sha256'].items():
            if path != runner and digest(Path(path)) != expected:
                raise RuntimeError(f'cannot resume after source/binary change: {path}')
        initial = output / 'provenance_initial.json'
        if not initial.exists():
            initial.write_bytes(previous_path.read_bytes())
        provenance['recorder_revisions'] = [previous['sha256'][runner], digest(Path(runner))]
        provenance['resume_reason'] = 'Record freshness misses as measured outcomes while keeping the original 40 ms gate and zero hardware output.'
    (output / 'provenance.json').write_text(json.dumps(provenance, indent=2) + '\n')
    run([binary, '--self-test'], output / 'zero_guard_tests.log', root, environment, timeout=20)
    completed = {}

    def capture(name, reuse, split, *, paired=False):
        free_ports()
        target = output / name
        command = [binary, '--phase', 'offline' if paired else 'control', '--workspace', root,
                   '--output', target, '--duration', '10' if paired else str(args.duration),
                   '--hz', '250', '--mpc-hz', '50', '--wbc-reuse', str(reuse).lower()]
        if paired:
            command += ['--wbc-compare', 'true']
        if split:
            command += ['--control-cpus', '4-5', '--mpc-cpus', '6-7']
        monitor = output / (name + '_cpu')
        wrapped = [sys.executable, root / 'tools/cpu_timing_environment.py', '--output', monitor,
                   '--cpus', '4-7', '--sampler-cpus', '0-3', '--expect-governor', 'performance',
                   '--timeout', str(args.duration + 90)]
        if paired:
            wrapped += ['--probe-rt']
        if args.resume and (target / 'metadata.json').exists() and (monitor / 'result.json').exists():
            print('Reusing retained complete window:', name, flush=True)
        else:
            run([*wrapped, '--', *command], output / (name + '_wrapper.log'), root,
                environment, timeout=args.duration + 100)
        result = analyze(target)
        if not result['metadata']['completed_requested_duration'] or result['metadata'].get('wbc_failed') or result['metadata'].get('comparison_failed'):
            raise RuntimeError(f'{name}: incomplete or failed probe window')
        if result['rounds_with_motor_failures'] or result.get('captured_unsafe_packets', 0):
            raise RuntimeError(f'{name}: motor/encoded packet validation failed')
        n = result['executed_cycles']
        if any(result['control'][key + '_cycles'] != n for key in [
                'estimate_valid', 'policy_valid', 'solver_healthy']):
            raise RuntimeError(f'{name}: control validity failure; inspect retained capture')
        if result['control']['wbc_valid_cycles'] != result['control']['wbc_executed_cycles']:
            raise RuntimeError(f'{name}: invalid WBC output')
        result['all_control_cycles_fresh'] = result['control']['policy_fresh_cycles'] == n
        if result['wbc']['failed_cycles'] or result['wbc']['paired_failures']:
            raise RuntimeError(f'{name}: WBC solve/equivalence failure')
        if paired and result['wbc']['paired_cycles'] != n:
            raise RuntimeError('paired validation did not cover every executed control cycle')
        cpu = json.loads((monitor / 'result.json').read_text())
        if cpu['governors_before'] != cpu['governors_after']:
            raise RuntimeError('governor changed externally during the measurement')
        completed[name] = result
        (output / 'result.json').write_text(json.dumps(
            {'provenance': provenance, 'completed': completed}, indent=2, allow_nan=False) + '\n')
        print(json.dumps({'case': name, 'loop_p99_ms': result['loop_work_ms']['p99'],
                          'completion_miss_fraction': result['executed_completion_miss_fraction'],
                          'wbc_p99_ms': result['wbc']['total_ms']['p99'],
                          'wbc_solver_mean_ms': result['wbc']['solver_ms']['mean'],
                          'policy_age_over_40ms_cycles': result['control']['policy_age_over_40ms_cycles'],
                          'new_policy_hz': result['control']['new_policy_hz']}, ensure_ascii=False), flush=True)

    capture('paired_same_inputs', True, False, paired=True)
    order = [('shared_cold', False, False), ('shared_hot', True, False),
             ('split_hot', True, True), ('split_cold', False, True)]
    for repeat in range(args.repetitions):
        for name, reuse, split in order if repeat == 0 else reversed(order):
            capture(f'r{repeat+1}_{name}', reuse, split)
    free_ports()
    for path, expected in provenance['sha256'].items():
        if digest(Path(path)) != expected:
            raise RuntimeError(f'source/binary/plugin changed during validation: {path}')
    print('All finite cases recorded; assess freshness/misses in results. Zero-output hardware stopped; sources/plugins unchanged.', flush=True)


if __name__ == '__main__':
    main()
