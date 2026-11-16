#!/usr/bin/env python3
"""Run ordered, finite zero-output IO/control/receive-only IMU measurements.

Builds a standalone probe; never launches Run Dog, ROS controllers, startup
alignment, calibration or FOC. Large captures stay in the ignored log tree.
"""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys


def runtime_environment(root):
    shell = 'set -e; source "$1/src/custom_dog_control/scripts/lib/workspace.sh"; dog_source_runtime; env -0'
    result = subprocess.run(['bash', '--noprofile', '--norc', '-c', shell,
                             'timing-environment', str(root)], capture_output=True, check=True)
    environment = {}
    for item in result.stdout.split(b'\0'):
        if b'=' in item:
            key, value = item.split(b'=', 1)
            environment[os.fsdecode(key)] = os.fsdecode(value)
    environment.update(OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='2')
    return environment


def run(command, log_path, root, environment, timeout=1200):
    print('Running:', ' '.join(map(str, command)), flush=True)
    with log_path.open('w') as log:
        process = subprocess.Popen(list(map(str, command)), cwd=root, env=environment,
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            code = process.wait(timeout=timeout)
        except (subprocess.TimeoutExpired, KeyboardInterrupt):
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            raise
    if code:
        raise RuntimeError(f'Command exited {code}; details: {log_path}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--duration', type=float, default=30,
                        help='seconds per measured phase, 1..120 (model initialization excluded)')
    parser.add_argument('--hz', type=float, default=1000)
    parser.add_argument('--cpus', default='4-7')
    args = parser.parse_args()
    if not 1 <= args.duration <= 120 or not 1 <= args.hz <= 2000:
        parser.error('duration must be 1..120 and hz 1..2000')
    root = Path(__file__).resolve().parents[1]
    output = (args.output or root / 'log' / ('timing-identification-' +
              datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))).resolve()
    if output.exists():
        raise RuntimeError('choose a new output directory; existing captures are preserved')
    output.mkdir(parents=True)
    environment = runtime_environment(root)
    ports = ['/dev/ttyS3', '/dev/ttyS4', '/dev/ttyS7', '/dev/ttyS8', '/dev/ttyUSB0']
    ownership = subprocess.run(['fuser', *ports], capture_output=True, text=True)
    if ownership.returncode != 1 or ownership.stdout.strip() or ownership.stderr.strip():
        raise RuntimeError('serial devices occupied or ownership check failed')
    run([sys.executable, root / 'tools/build_timing_probe.py', '--output', output / 'bin'],
        output / 'build_console.log', root, environment)
    binary = output / 'bin/probe_control_timing'
    run([binary, '--self-test'], output / 'zero_guard_tests.log', root, environment)
    run([sys.executable, root / 'tools/probe_imu_timing.py', '--self-test'],
        output / 'imu_protocol_tests.log', root, environment)
    from analyze_timing_probe import analyze
    completed = []
    for phase, directory in [('io', '01_parallel_io'), ('control', '02_control')]:
        target = output / directory
        run(['taskset', '--cpu-list', args.cpus, binary, '--phase', phase,
             '--workspace', root, '--output', target, '--duration', args.duration, '--hz', args.hz],
            output / (directory + '_console.log'), root, environment)
        result = analyze(target)
        if result.get('captured_unsafe_packets') != 0:
            raise RuntimeError('final packet verification failed')
        completed.append(directory)
    run([sys.executable, root / 'tools/probe_imu_timing.py', '--port', '/dev/ttyUSB0',
         '--duration', args.duration, '--consumer-hz', args.hz, '--consumer-cpus', args.cpus,
         '--consumer-binary', binary, '--output', output / '03_imu'],
        output / '03_imu_console.log', root, environment)
    completed.append('03_imu')
    sources = [root / 'tools' / name for name in ['probe_control_timing.cpp', 'control_timing_workload.hpp',
               'probe_imu_timing.py', 'analyze_timing_probe.py', 'build_timing_probe.py', 'run_timing_identification.py']]
    sources += [root / 'src/custom_dog_control' / name for name in [
        'src/hardware/IOSDK.cpp', 'include/custom_dog_control/hardware/IOSDK.hpp',
        'config/controllers.yaml', 'config/real_controller.yaml', 'config/nmpc/task.info', 'config/nmpc/reference.info']]
    sources.append(binary)
    provenance = {'git_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip(),
                  'completed': completed, 'sha256': {str(p.relative_to(root)) if p.is_relative_to(root) else str(p):
                  hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
                  'cpu_affinity': args.cpus, 'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '2'}
    (output / 'provenance.json').write_text(json.dumps(provenance, indent=2) + '\n')
    print('Completed all three phases:', output)


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, subprocess.SubprocessError) as error:
        print(error, file=sys.stderr)
        raise SystemExit(2)
