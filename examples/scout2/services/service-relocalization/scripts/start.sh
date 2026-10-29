#!/usr/bin/env bash
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
ROS_SETUP="${RELOCALIZATION_ROS_SETUP:-/opt/ros/${ROS_DISTRO:-humble}/setup.bash}"
OVERLAY="$PKG/rbnx-build/codegen/ros2_idl/install/local_setup.bash"
if [[ ! -x "$PKG/rbnx-build/venv/bin/python" || ! -f "$OVERLAY" ]]; then
    echo "[relocalization] build the package before starting it" >&2
    exit 1
fi
set +u
source "$ROS_SETUP"
source "$OVERLAY"
set -u
export PYTHONPATH="$PKG:$PKG/rbnx-build/search:$PKG/rbnx-build/codegen/proto_gen:${PYTHONPATH:-}"
"$PKG/rbnx-build/venv/bin/python" -m relocalization_rbnx.native_backend \
    "$PKG/rbnx-build/search"
exec "$PKG/rbnx-build/venv/bin/python" -m relocalization_rbnx.main
