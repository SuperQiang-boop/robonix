#!/usr/bin/env bash
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
if [[ -n "${CAMERA_RTSP_ROS_SETUP:-}" ]]; then
    set +u
    source "$CAMERA_RTSP_ROS_SETUP"
    set -u
fi
export PYTHONPATH="$PKG:$PKG/rbnx-build/codegen/proto_gen:${PYTHONPATH:-}"
exec "$PKG/rbnx-build/venv/bin/python" -m camera_rtsp.main
