#!/usr/bin/env bash
set -euo pipefail

# Match the verified RK3588 runtime workload; avoid library thread pools
# oversubscribing the two-thread SQP solver. Explicit task-specific overrides
# are available for profiling without inheriting an unrelated shell default.
export OPENBLAS_NUM_THREADS="${DOG_OPENBLAS_NUM_THREADS:-1}"
export OMP_NUM_THREADS="${DOG_OMP_NUM_THREADS:-2}"

# shellcheck source=lib/workspace.sh
source "$(dirname "${BASH_SOURCE[0]}")/lib/workspace.sh"
dog_source_runtime

if [[ -z "${UNITREE_ACTUATOR_SDK_ROOT:-}" ]]; then
  for sdk_candidate in \
    "${DOG_WORKSPACE}/../unitree_actuator_sdk-main/unitree_actuator_sdk-main" \
    "${DOG_WORKSPACE}/../unitree_actuator_sdk-main" \
    "${DOG_WORKSPACE}/../unitree_actuator_sdk"; do
    if [[ -f "${sdk_candidate}/include/unitreeMotor/unitreeMotor.h" ]]; then
      UNITREE_ACTUATOR_SDK_ROOT="$(cd "$sdk_candidate" && pwd -P)"
      break
    fi
  done
fi
export UNITREE_ACTUATOR_SDK_ROOT="${UNITREE_ACTUATOR_SDK_ROOT:-}"
export CUSTOM_DOG_CONTROL_REAL_HARDWARE=1
"${DOG_SCRIPTS_DIR}/check_dependencies.sh"
python3 -c 'import evdev'

controller_prefix="$(ros2 pkg prefix custom_dog_control)"
case "$controller_prefix" in
  "${DOG_WORKSPACE}/install"|"${DOG_WORKSPACE}/install/"*) ;;
  *)
    echo "ERROR: custom_dog_control resolves outside this workspace: ${controller_prefix}" >&2
    exit 2
    ;;
esac
hardware_library="${controller_prefix}/lib/libcustom_dog_control_unitree_system.so"
controller_library="${controller_prefix}/lib/libcustom_dog_control_nmpc_wbc_controller.so"
hardware_description="${controller_prefix}/share/custom_dog_control/custom_dog_control_hardware_plugins.xml"
if [[ ! -f "$hardware_library" || ! -f "$controller_library" || ! -f "$hardware_description" ]]; then
  echo "ERROR: Missing real-hardware plugin. Run ${DOG_SCRIPTS_DIR}/build_real.sh first." >&2
  exit 2
fi
for plugin_library in "$hardware_library" "$controller_library"; do
  if ! library_dependencies="$(ldd "$plugin_library" 2>&1)"; then
    echo "ERROR: Cannot inspect plugin dependencies: $plugin_library" >&2
    echo "$library_dependencies" >&2
    exit 2
  fi
  if [[ "$library_dependencies" == *"not found"* ]]; then
    echo "ERROR: Plugin has missing shared libraries: $plugin_library" >&2
    echo "$library_dependencies" >&2
    exit 2
  fi
done

# Keep all launched RK3588 nodes on the large cores. Unrestricted scheduling
# moved WBC onto A55 cores and missed the 4 ms compute budget in offline tests.
# This is process affinity, not a realtime policy or system-wide CPU isolation.
# Set DOG_CPU_AFFINITY=none to disable, or an explicit taskset CPU list to override.
dog_cpu_affinity="${DOG_CPU_AFFINITY:-auto}"
if [[ "$dog_cpu_affinity" == auto ]]; then
  dog_cpu_affinity="$(python3 - <<'PY'
import os
from pathlib import Path
compatible = Path('/proc/device-tree/compatible')
capacities = {}
if compatible.exists() and b'rk3588' in compatible.read_bytes():
    for cpu in os.sched_getaffinity(0):
        path = Path(f'/sys/devices/system/cpu/cpu{cpu}/cpu_capacity')
        if path.exists():
            capacities[cpu] = int(path.read_text())
if capacities:
    maximum = max(capacities.values())
    print(','.join(str(cpu) for cpu in sorted(capacities) if capacities[cpu] == maximum))
PY
)"
fi

# Keep real.launch.py's physical_estop_verified=false default. Only an operator
# who has verified the independent emergency stop may pass an explicit override.
if [[ -n "$dog_cpu_affinity" && "$dog_cpu_affinity" != none ]]; then
  echo "Runtime CPU affinity: ${dog_cpu_affinity} (no realtime scheduling guarantee)"
  exec taskset --cpu-list "$dog_cpu_affinity" \
    ros2 launch custom_dog_control real.launch.py "$@"
fi
exec ros2 launch custom_dog_control real.launch.py "$@"
