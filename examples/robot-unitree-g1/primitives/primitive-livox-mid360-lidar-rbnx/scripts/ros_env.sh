#!/usr/bin/env bash
# Source with PKG set. Remove this package's generated standard-message overlay,
# including paths inherited from rbnx, before loading the system ROS ABI.
for env_key in PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH; do
    IFS=: read -r -a env_paths <<< "${!env_key:-}"
    kept_paths=()
    for env_path in "${env_paths[@]}"; do
        [[ -z "$env_path" || "$env_path" == "$PKG/rbnx-build/codegen/ros2_idl/"* ]] && continue
        kept_paths+=("$env_path")
    done
    printf -v "$env_key" '%s' "$(IFS=:; echo "${kept_paths[*]}")"
    export "$env_key"
done
export PATH="/usr/bin:$PATH"
ROS_DISTRO="${ROS_DISTRO:-humble}"
set +u
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "$PKG/rbnx-build/ws/install/local_setup.bash"
set -u
