#!/usr/bin/env bash
# Unitree G1 deployment — build entry point.
#
# Usage:
#   bash build.sh                  # build all primitives and services
#   bash build.sh --clean          # clean previous build artifacts first
#
set -euo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
MANIFEST="$DIR/robonix_manifest.yaml"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # The package build scripts invoke colcon and need the ROS environment.
  source /opt/ros/humble/setup.bash
fi

# Keep manifest-referenced caches inside this deployment when build.sh is used.
export ROBONIX_DEPLOY_DIR="$DIR"
export XDG_CACHE_HOME="${XDG_CACHE_HOME:-$DIR/.cache/xdg}"
mkdir -p "$XDG_CACHE_HOME" "$DIR/.cache/modelscope" \
  "$DIR/.cache/huggingface" "$DIR/.cache/torch"

FLAGS=()
if [[ "${1:-}" == "--clean" ]]; then
  FLAGS+=(--clean)
fi

echo "[g1] building deployment from $MANIFEST..."
rbnx build -f "$MANIFEST" "${FLAGS[@]}"
echo "[g1] build complete."
