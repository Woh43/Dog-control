#!/usr/bin/env bash
set -euo pipefail

# Native ARM64 Gazebo Fortress. This entry never selects the Unitree plugin.
source "$(dirname "${BASH_SOURCE[0]}")/lib/workspace.sh"
dog_source_runtime
export OPENBLAS_NUM_THREADS="${DOG_OPENBLAS_NUM_THREADS:-1}"
export OMP_NUM_THREADS="${DOG_OMP_NUM_THREADS:-2}"
dog_description_prefix="$(ros2 pkg prefix custom_dog_description)"
export IGN_GAZEBO_RESOURCE_PATH="${dog_description_prefix}/share:${IGN_GAZEBO_RESOURCE_PATH:-}"
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="/opt/ros/humble/lib:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:-}"
for simulation_package in ros_gz_sim ros_gz_bridge gz_ros2_control; do
  ros2 pkg prefix "$simulation_package" >/dev/null
done

# Match the measured RK3588 physics/solver run on the available large cores.
# This sets process affinity only; it does not grant realtime scheduling.
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
if [[ -n "$dog_cpu_affinity" && "$dog_cpu_affinity" != none ]]; then
  echo "Simulation CPU affinity: ${dog_cpu_affinity}"
  exec taskset --cpu-list "$dog_cpu_affinity" \
    ros2 launch custom_dog_control fortress.launch.py "$@"
fi
exec ros2 launch custom_dog_control fortress.launch.py "$@"
