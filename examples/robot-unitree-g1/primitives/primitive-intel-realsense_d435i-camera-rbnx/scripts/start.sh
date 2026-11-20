#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

ROS_DISTRO="${ROS_DISTRO:-humble}"
# shellcheck disable=SC1091
set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u
if [[ -f "$PKG/rbnx-build/codegen/ros2_idl/install/setup.bash" ]]; then
    set +u; source "$PKG/rbnx-build/codegen/ros2_idl/install/setup.bash"; set -u
fi
if [[ -f "$PKG/rbnx-build/ws/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    set +u; source "$PKG/rbnx-build/ws/install/setup.bash"; set -u
fi

if ROBONIX_API="$(rbnx path robonix-api 2>/dev/null)"; then
    export PYTHONPATH="$ROBONIX_API:$PKG:${PYTHONPATH:-}"
fi

exec python3 -m realsense_camera.main
