#!/usr/bin/env bash
set -euo pipefail

# Build only: this entry point never starts a controller or opens a motor port.
# shellcheck source=lib/workspace.sh
source "$(dirname "${BASH_SOURCE[0]}")/lib/workspace.sh"

if (( $# )); then
  echo "Usage: build_real.sh (configure paths through environment variables)" >&2
  exit 2
fi

model_package="$DOG_MODEL_SOURCE"

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
if [[ ! -f "${UNITREE_ACTUATOR_SDK_ROOT:-}/include/unitreeMotor/unitreeMotor.h" ]]; then
  echo "ERROR: Set UNITREE_ACTUATOR_SDK_ROOT to the actuator SDK containing include/unitreeMotor/unitreeMotor.h." >&2
  exit 2
fi
UNITREE_ACTUATOR_SDK_ROOT="$(cd "$UNITREE_ACTUATOR_SDK_ROOT" && pwd -P)"
export UNITREE_ACTUATOR_SDK_ROOT
export CUSTOM_DOG_CONTROL_REAL_HARDWARE=1

export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
if [[ ! "$CMAKE_BUILD_PARALLEL_LEVEL" =~ ^[1-9][0-9]*$ ]]; then
  echo "ERROR: CMAKE_BUILD_PARALLEL_LEVEL must be a positive integer." >&2
  exit 2
fi
export MAKEFLAGS="-j${CMAKE_BUILD_PARALLEL_LEVEL}"

dog_source_ros
dog_build_local_model
dog_source_dependencies
"${DOG_SCRIPTS_DIR}/check_dependencies.sh"

multiarch="$(gcc -print-multiarch)"
ros_multiarch_lib="/opt/ros/humble/lib/${multiarch}"
if [[ -d "$ros_multiarch_lib" ]]; then
  export LIBRARY_PATH="${ros_multiarch_lib}:${LIBRARY_PATH:-}"
fi

cd "$DOG_WORKSPACE"
colcon build --symlink-install --packages-up-to custom_dog_control \
  --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
    -DCUSTOM_DOG_CONTROL_CANONICAL_URDF="${model_package}/urdf/custom_dog.urdf" \
    -DCUSTOM_DOG_CONTROL_BUILD_REAL_HARDWARE=ON \
    -DUNITREE_ACTUATOR_SDK_ROOT="$UNITREE_ACTUATOR_SDK_ROOT"

echo "Real-hardware build completed. No controller was started."
echo "Runtime entry: ${DOG_SCRIPTS_DIR}/run_real.sh"
