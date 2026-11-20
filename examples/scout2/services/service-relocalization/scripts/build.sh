#!/usr/bin/env bash
# Build package interfaces and native matching with the service interpreter.
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
SOURCE="${ROBONIX_SOURCE_PATH:-$(git -C "$PKG" rev-parse --show-toplevel)}"
PYTHON="${RELOCALIZATION_PYTHON:-/usr/bin/python3}"
ROS_SETUP="${RELOCALIZATION_ROS_SETUP:-/opt/ros/${ROS_DISTRO:-humble}/setup.bash}"
export UV_CACHE_DIR="${UV_CACHE_DIR:-$PKG/rbnx-build/uv-cache}"
set +u
source "$ROS_SETUP"
set -u
uv venv --allow-existing --python "$PYTHON" --system-site-packages "$PKG/rbnx-build/venv"
uv pip install --python "$PKG/rbnx-build/venv/bin/python" \
    "$SOURCE/pylib/robonix-api" "$PKG[build]"
rbnx codegen -p "$PKG" --ros2 --python "$PKG/rbnx-build/venv/bin/python"
IDL="$PKG/rbnx-build/codegen/ros2_idl"
cd "$IDL"
colcon build --packages-select relocalization \
    --cmake-args -DPython3_EXECUTABLE="$PYTHON" -DPYTHON_EXECUTABLE="$PYTHON"
RELOCALIZATION_PYTHON="$PKG/rbnx-build/venv/bin/python" \
    bash "$PKG/scripts/build_search.sh"
# Scout2 prepares saved-map indices at startup, so the exporter is required
# even when matching indices happened to exist during an earlier deployment.
RELOCALIZATION_BUILD_IMAGE="${RELOCALIZATION_BUILD_IMAGE:-${ROBONIX_MAPPING_IMAGE:-robonix-mapping:latest}}" \
    bash "$PKG/scripts/build_backend.sh"
