#!/usr/bin/env python3
"""Build the standalone zero-output timing executable; never run hardware."""
import argparse
import json
import os
from pathlib import Path
import shlex
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--extra-object', type=Path, action='append', default=[],
                        help='isolated instrumentation object placed before the unchanged archives')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    build = root / 'build/custom_dog_control'
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    entries = json.loads((build / 'compile_commands.json').read_text())
    cache = (build / 'CMakeCache.txt').read_text().splitlines()
    sdk = Path(next(line.split('=', 1)[1] for line in cache
                    if line.startswith('UNITREE_ACTUATOR_SDK_ROOT:PATH=')))
    jobs = [
        ('/test/test_model_validator.cpp', root / 'tools/probe_control_timing.cpp', 'probe.o'),
        ('/src/hardware/IOSDK.cpp', root / 'src/custom_dog_control/src/hardware/IOSDK.cpp', 'iosdk.o'),
        ('/src/nmpc/NmpcBackend.cpp', root / 'src/custom_dog_control/src/nmpc/NmpcBackend.cpp', 'backend.o'),
        ('/third_party/legged_wbc/src/WeightedWbc.cpp', root / 'src/custom_dog_control/third_party/legged_wbc/src/WeightedWbc.cpp', 'wbc.o'),
    ]
    env = os.environ.copy()
    env['LIBRARY_PATH'] = '/opt/ros/humble/lib/aarch64-linux-gnu:' + env.get('LIBRARY_PATH', '')
    with (output / 'build.log').open('w') as log:
        for suffix, source, obj in jobs:
            entry = next(item for item in entries if item['file'].endswith(suffix))
            command = shlex.split(entry['command'])
            command[command.index('-o') + 1] = str(output / obj)
            command[command.index('-c') + 1] = str(source)
            command += ['-DCOMPILE_WITH_REAL_ROBOT', '-DCUSTOM_DOG_TIMING_PROBE',
                        '-I' + str(sdk / 'include'), '-I' + str(root / 'tools')]
            subprocess.run(command, cwd=entry['directory'], env=env,
                           stdout=log, stderr=subprocess.STDOUT, check=True)
        link = shlex.split((build / 'CMakeFiles/test_model_validator.dir/link.txt').read_text())
        link = [str(output / 'probe.o') if arg.endswith('test_model_validator.cpp.o') else arg for arg in link]
        link = [arg for arg in link if arg not in ('gtest/libgtest_main.a', 'gtest/libgtest.a')]
        link[link.index('-o') + 1] = str(output / 'probe_control_timing')
        # WeightedWbc's member layout changes with the opt-in implementation.
        # Recompile its constructing backend as well; never mix old sizeof with
        # new methods or modify an installed archive/plugin in place.
        index = link.index('libcustom_dog_control_core.a')
        link[index:index] = [str(output / 'backend.o'), str(output / 'wbc.o'),
                             *[str(p.resolve()) for p in args.extra_object]]
        link += [str(output / 'iosdk.o'), '-L' + str(sdk / 'lib'),
                 '-Wl,-rpath,' + str(sdk / 'lib'), '-lUnitreeMotorSDK_Arm64', '-pthread']
        subprocess.run(link, cwd=build, env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
    print(output / 'probe_control_timing')


if __name__ == '__main__':
    main()
