#!/usr/bin/env bash
set -euo pipefail

# shellcheck source=lib/workspace.sh
source "$(dirname "${BASH_SOURCE[0]}")/lib/workspace.sh"
script_dir="$DOG_SCRIPTS_DIR"
workspace="$DOG_WORKSPACE"
model_package="$DOG_MODEL_SOURCE"

dog_source_ros

dog_build_local_model
if [[ -d "${DOG_DEPS_WS}" ]]; then
  dog_source_dependencies
fi
if [[ -d "${workspace}/src/ocs2" ]]; then
  export CUSTOM_DOG_CONTROL_ALLOW_OCS2_SOURCE_BUILD=1
fi
"${script_dir}/check_dependencies.sh"

multiarch="$(gcc -print-multiarch 2>/dev/null || true)"
ros_multiarch_lib="/opt/ros/humble/lib/${multiarch}"
if [[ -n "${multiarch}" && -d "${ros_multiarch_lib}" ]]; then
  export LIBRARY_PATH="${ros_multiarch_lib}:${LIBRARY_PATH:-}"
fi
export GAZEBO_PLUGIN_PATH="/opt/ros/humble/lib:${GAZEBO_PLUGIN_PATH:-}"
export GAZEBO_MODEL_DATABASE_URI=""
# Pinocchio/CppAD translation units are memory intensive on desktop systems.
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
export MAKEFLAGS="${MAKEFLAGS:--j${CMAKE_BUILD_PARALLEL_LEVEL}}"

cd "${workspace}"
colcon build --symlink-install --packages-up-to custom_dog_control \
  --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
               -DCUSTOM_DOG_CONTROL_CANONICAL_URDF="${model_package}/urdf/custom_dog.urdf" \
               -DCUSTOM_DOG_CONTROL_BUILD_REAL_HARDWARE=OFF

# Only source a freshly successful build. set -e above prevents stale installs
# from being used after a compiler or linker failure.
dog_source "${workspace}/install/local_setup.bash"
if [[ "${CUSTOM_DOG_BUILD_ONLY:-0}" == "1" ]]; then
  exit 0
fi
dog_check_local_model
dog_prepare_cppad_cache
exec ros2 launch custom_dog_control gazebo.launch.py use_rviz:=false "$@"
