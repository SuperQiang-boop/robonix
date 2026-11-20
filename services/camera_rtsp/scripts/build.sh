#!/usr/bin/env bash
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
SOURCE="${ROBONIX_SOURCE_PATH:-$(git -C "$PKG" rev-parse --show-toplevel)}"
PYTHON="${CAMERA_RTSP_PYTHON:-/usr/bin/python3}"
uv venv --python "$PYTHON" --system-site-packages "$PKG/rbnx-build/venv"
uv pip install --python "$PKG/rbnx-build/venv/bin/python" "$SOURCE/pylib/robonix-api" "$PKG"
# Codegen runs on rbnx's own pinned toolchain. Naming this venv's Python would
# fail the pin check: the venv inherits the host ROS 2 packages, so its
# protobuf and grpcio are whatever the host distribution shipped.
rbnx codegen -p "$PKG"
