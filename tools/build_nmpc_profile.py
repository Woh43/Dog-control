#!/usr/bin/env python3
"""Build an isolated OCS2 node-stage profiling object for the timing probe.

Original vendor source, archives, plugins, and controller configuration are read
only. The generated copy retains its original copyright and license notice.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def transform(source, header):
    """Insert scope timers, leaving every original executable statement intact."""
    text = source.read_text()
    text = text.replace('#include "ocs2_oc/multiple_shooting/Transcription.h"',
                        '#include "ocs2_oc/multiple_shooting/Transcription.h"\n'
                        f'#include "{header}"', 1)
    begin = text.index('Transcription setupIntermediateNode(')
    end = text.index('\nvoid projectTranscription(', begin)
    section = text[begin:end]
    section = section.replace('  // Results and short-hand notation',
                              '  dog_nmpc_profile::Scope whole(dog_nmpc_profile::Intermediate);\n'
                              '  // Results and short-hand notation', 1)
    markers = [('  // Dynamics\n', 'Dynamics'),
               ('  // Precomputation for other terms\n', 'Precomputation'),
               ('  // Costs:', 'Cost'),
               ('  // State equality constraints\n', 'StateEquality'),
               ('  // State-input equality constraints\n', 'StateInputEquality'),
               ('  // State inequality constraints.\n', 'StateInequality'),
               ('  // State-input inequality constraints.\n', 'StateInputInequality')]
    for i, (marker, stage) in enumerate(markers):
        if section.count(marker) != 1:
            raise RuntimeError(f'OCS2 source structure changed: {marker!r}')
        section = section.replace(marker, ('  }\n' if i else '') +
                                  '  { dog_nmpc_profile::Scope part(dog_nmpc_profile::' + stage + ');\n' + marker, 1)
    section = section.replace('  return transcription;', '  }\n  return transcription;', 1)
    text = text[:begin] + section + text[end:]
    for signature, stage in [('void projectTranscription(', 'Projection'),
                             ('TerminalTranscription setupTerminalNode(', 'Terminal'),
                             ('EventTranscription setupEventNode(', 'Event')]:
        start = text.index(signature)
        opening = text.index('{', start)
        text = text[:opening + 1] + '\n  dog_nmpc_profile::Scope whole(dog_nmpc_profile::' + stage + ');' + text[opening + 1:]
    return text


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    build = root / 'build/custom_dog_control'
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    source = root / 'external/ocs2_ws/src/ocs2/ocs2_oc/src/multiple_shooting/Transcription.cpp'
    original_hash = sha(source)
    generated = output / 'Transcription.profile.cpp'
    generated.write_text(transform(source, root / 'tools/profile_nmpc_stages.hpp'))
    entries = json.loads((build / 'compile_commands.json').read_text())
    entry = next(e for e in entries if e['file'].endswith('/test/test_model_validator.cpp'))
    command = shlex.split(entry['command'])
    env = os.environ.copy()
    env['LIBRARY_PATH'] = '/opt/ros/humble/lib/aarch64-linux-gnu:' + env.get('LIBRARY_PATH', '')
    with (output / 'build.log').open('w') as log:
        command[command.index('-c') + 1] = str(generated)
        command[command.index('-o') + 1] = str(output / 'transcription.o')
        subprocess.run(command, cwd=entry['directory'], env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
    if sha(source) != original_hash:
        raise RuntimeError('vendor source changed during profiling-object compilation')
    metadata = {
        'vendor_source': str(source.relative_to(root)), 'vendor_sha256': original_hash,
        'generated_sha256': sha(generated), 'original_vendor_unchanged': True,
        'method': 'Isolated instrumented transcription object precedes unchanged static OCS2 archives.',
        'sources': {str(p.relative_to(root)): sha(p) for p in [
            root / 'tools/build_nmpc_profile.py',
            root / 'tools/profile_nmpc_stages.hpp', root / 'tools/control_timing_workload.hpp']},
        'object_sha256': sha(output / 'transcription.o'),
    }
    (output / 'build_metadata.json').write_text(json.dumps(metadata, indent=2) + '\n')
    print(output / 'transcription.o')


if __name__ == '__main__':
    main()
