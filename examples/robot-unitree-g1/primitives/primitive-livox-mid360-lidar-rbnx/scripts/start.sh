#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Spawn the mid360_lidar capability process. The Livox ROS driver is NOT
# launched here — it's spawned inside the cap's on_init handler, after
# rbnx boot delivers config via Driver(CMD_INIT).
#
# Layout invariant (populated by scripts/build.sh):
#   rbnx-build/ws/install/setup.bash   colcon overlay (livox_ros_driver2)
#   rbnx-build/codegen/proto_gen/      atlas_pb2.py + robonix_contracts_pb2*
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

# Standard sensor messages come from the ROS distribution, not generated IDL.
source "$PKG/scripts/ros_env.sh"

# robonix_api is on the host at `rbnx path robonix-api`.
if ROBONIX_API="$(rbnx path robonix-api 2>/dev/null)"; then
    export PYTHONPATH="$ROBONIX_API:$PKG:${PYTHONPATH:-}"
fi

export PYTHONPATH="$PKG/rbnx-build/codegen/proto_gen:${PYTHONPATH:-}"
exec /usr/bin/python3 -m mid360_driver.main
