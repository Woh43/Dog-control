#!/usr/bin/env python3
"""Build the offline runtime harness using the completed package build flags.

Does not open serial ports or run the resulting binary. Build custom_dog_control
first. TASK_FILE is supplied when executing the binary; reuse a valid existing
content-addressed codegen cache only if model/task/reference bytes still match.
"""
from pathlib import Path
import argparse,json,shlex,subprocess
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--workspace',type=Path,default=Path(__file__).resolve().parents[1])
parser.add_argument('--output',type=Path)
args=parser.parse_args()
root=args.workspace.resolve();build=root/'build/custom_dog_control'
output=(args.output or root/'log/basic-motion-20261102').resolve();output.mkdir(parents=True,exist_ok=True)
source=root/'tools/test_nmpc_runtime.cpp';obj=output/'test_nmpc_runtime.cpp.o';binary=output/'test_nmpc_runtime'
entries=json.loads((build/'compile_commands.json').read_text())
entry=next(item for item in entries if item['file'].endswith('/test/test_model_validator.cpp'))
compile_argv=shlex.split(entry['command']);compile_argv[compile_argv.index('-o')+1]=str(obj);compile_argv[-1]=str(source)
with (output/'runtime-build.log').open('w') as log:
    subprocess.run(compile_argv,cwd=entry['directory'],stdout=log,stderr=subprocess.STDOUT,check=True)
    link_argv=shlex.split((build/'CMakeFiles/test_model_validator.dir/link.txt').read_text())
    link_argv=[str(obj) if arg.endswith('test_model_validator.cpp.o') else arg for arg in link_argv]
    link_argv[link_argv.index('-o')+1]=str(binary)
    link_argv=[arg for arg in link_argv if arg not in ('gtest/libgtest_main.a','gtest/libgtest.a')]
    link_argv.append('-Wl,--wrap=_ZN4ocs217MPC_MRT_Interface10advanceMpcEv')
    subprocess.run(link_argv,cwd=build,stdout=log,stderr=subprocess.STDOUT,check=True)
print(binary)
