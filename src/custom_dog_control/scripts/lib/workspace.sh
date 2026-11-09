# Shared by source-workspace entry points; does not change caller shell options.
# All paths are derived from this file, so commands also work outside the repo.
DOG_SCRIPTS_DIR="$(cd -P "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DOG_WORKSPACE="$(cd -P "${DOG_SCRIPTS_DIR}/../../.." && pwd)"
DOG_MODEL_SOURCE="${DOG_WORKSPACE}/src/custom_dog_description"
DOG_DEPS_WS="${CUSTOM_DOG_CONTROL_DEPS_WS:-${DOG_WORKSPACE}/external/ocs2_ws}"

# ROS/colcon setup scripts may reference unset variables. Restore nounset even
# on failure, and report missing underlays instead of loading a partial stack.
dog_source() {
  local setup_file="$1" had_nounset=0 status=0
  if [[ ! -f "$setup_file" ]]; then
    echo "ERROR: Missing environment: $setup_file" >&2
    return 2
  fi
  [[ $- == *u* ]] && had_nounset=1
  set +u
  # shellcheck disable=SC1090
  source "$setup_file" || status=$?
  if (( had_nounset )); then set -u; fi
  return "$status"
}

# Remove the retired model underlay before sourcing or building. Also remove
# this model's existing entries so local_setup can put the local package first
# even when the caller sourced an older workspace afterwards.
dog_prepare_model_environment() {
  local dog_path_variable dog_filtered_path
  for dog_path_variable in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH; do
    dog_filtered_path="$(python3 - "$DOG_WORKSPACE" "$dog_path_variable" <<'PY'
import os
from pathlib import Path
import sys

workspace = Path(sys.argv[1]).resolve()
retired = workspace / 'external/model_ws/install'
local = {workspace / 'install', workspace / 'install/custom_dog_description'}
kept = []
for entry in os.environ.get(sys.argv[2], '').split(':'):
    if not entry:
        continue
    path = Path(entry).resolve()
    if path == retired or retired in path.parents or path in local:
        continue
    kept.append(entry)
print(':'.join(kept))
PY
)" || return "$?"
    export "${dog_path_variable}=${dog_filtered_path}"
  done
}

dog_source_ros() {
  dog_prepare_model_environment && dog_source /opt/ros/humble/setup.bash
}

dog_source_dependencies() {
  local prefix="$DOG_DEPS_WS"
  [[ -f "$prefix/install/local_setup.bash" ]] && prefix="$prefix/install"
  dog_source "$prefix/local_setup.bash"
}

# The robot description belongs to this workspace. An older model underlay in
# the caller's shell must never silently replace the local 15.15 kg model.
dog_check_local_model() {
  python3 - "$DOG_WORKSPACE" <<'PY'
import hashlib
from pathlib import Path
import sys
from ament_index_python.packages import get_package_prefix, get_package_share_directory

workspace = Path(sys.argv[1]).resolve()
try:
    prefix = Path(get_package_prefix('custom_dog_description')).resolve()
    allowed = {workspace / 'install', workspace / 'install/custom_dog_description'}
    if prefix not in allowed:
        raise ValueError(f'custom_dog_description resolves outside this workspace: {prefix}')
    source = workspace / 'src/custom_dog_description/urdf/custom_dog.urdf'
    installed = Path(get_package_share_directory('custom_dog_description')) / 'urdf/custom_dog.urdf'
    if hashlib.sha256(source.read_bytes()).digest() != hashlib.sha256(installed.read_bytes()).digest():
        raise ValueError('installed robot model differs from src/custom_dog_description')
except Exception as error:
    print(f'ERROR: {error}. Rebuild with scripts/build_real.sh or build_simulation.sh.', file=sys.stderr)
    sys.exit(2)
PY
}

dog_build_local_model() {
  if [[ ! -f "${DOG_MODEL_SOURCE}/package.xml" ||
        ! -f "${DOG_MODEL_SOURCE}/urdf/custom_dog.urdf" ]]; then
    echo "ERROR: Missing workspace model package: ${DOG_MODEL_SOURCE}" >&2
    return 2
  fi
  colcon --log-base "${DOG_WORKSPACE}/log" build \
    --base-paths "$DOG_MODEL_SOURCE" \
    --build-base "${DOG_WORKSPACE}/build" \
    --install-base "${DOG_WORKSPACE}/install" \
    --symlink-install --executor sequential \
    --packages-select custom_dog_description \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON || return "$?"
  dog_source "${DOG_WORKSPACE}/install/local_setup.bash" &&
    dog_check_local_model
}

# Generated native libraries survive /tmp cleanup and reboot. Explicit caller
# overrides remain supported; normalize once so child launches share one root.
dog_prepare_cppad_cache() {
  local dog_cache_directory="${CUSTOM_DOG_CPPAD_CACHE_DIR:-${DOG_WORKSPACE}/.cache/cppad}"
  if ! mkdir -p -- "$dog_cache_directory"; then
    echo "ERROR: Cannot create CppAD cache directory: $dog_cache_directory" >&2
    return 2
  fi
  if [[ ! -d "$dog_cache_directory" || ! -w "$dog_cache_directory" ]]; then
    echo "ERROR: CppAD cache directory is not writable: $dog_cache_directory" >&2
    return 2
  fi
  CUSTOM_DOG_CPPAD_CACHE_DIR="$(cd -P "$dog_cache_directory" && pwd)" || return "$?"
  export CUSTOM_DOG_CPPAD_CACHE_DIR
}

dog_source_runtime() {
  dog_source_ros &&
    dog_source_dependencies &&
    dog_source "$DOG_WORKSPACE/install/local_setup.bash" &&
    dog_check_local_model &&
    dog_prepare_cppad_cache
}
