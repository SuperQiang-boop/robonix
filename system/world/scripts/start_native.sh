#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# world native start — run the service directly on the host (no docker).
#
# Used on Jetson (and any host with ROS 2 + a CUDA-capable python that already
# has torch/torchvision/ultralytics/open3d). Selected by the jetson-native
# manifest (`ROBONIX_WORLD_FORCE=native bash scripts/start.sh`, which execs
# this). The build phase (build_native.sh) created rbnx-build/venv with
# --system-site-packages so the heavy CUDA wheels come from the host JetPack
# stack and only the light pure-python deps live in the venv.
#
# Unlike the docker path this does NOT force a specific RMW / SHM profile:
# native world must share whatever RMW the other host nodes use so it actually
# sees the camera / pointcloud topics they publish.
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

# Native world runs on the host python (the build installed the light deps
# into the user site next to the host JetPack torch — no venv; see build.sh).
PY="${WORLD_NATIVE_PYTHON:-python3}"
"$PY" -c "import torch" 2>/dev/null || {
    echo "[world/native] error: '$PY' has no torch — run the jetson-native build" >&2
    echo "               and install the JetPack CUDA stack first (see World README)." >&2
    exit 1
}

ROS_DISTRO="${ROS_DISTRO:-humble}"
# shellcheck disable=SC1091
set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u

# Codegen stubs (proto_gen + robonix_mcp_types) + robonix-api + the package.
export PYTHONPATH="$PKG/rbnx-build/codegen/proto_gen:$PKG/rbnx-build/codegen/robonix_mcp_types:$PKG:${PYTHONPATH:-}"
if ROBONIX_API="$(rbnx path robonix-api 2>/dev/null)"; then
    export PYTHONPATH="$ROBONIX_API:$PYTHONPATH"
fi

# Model weights: build.sh fetches yolo + mobile_sam to docker/_weights and
# pre-warms both CLIP paths. Keep these envs aligned with build.sh so start is
# offline-fast and never downloads model weights during robot boot.
W="$PKG/docker/_weights"
export WORLD_YOLO_WORLD_WEIGHTS="${WORLD_YOLO_WORLD_WEIGHTS:-$W/yolov8l-world.pt}"
export WORLD_MOBILE_SAM_WEIGHTS="${WORLD_MOBILE_SAM_WEIGHTS:-$W/mobile_sam.pt}"
export WORLD_CLIP_MODEL="${WORLD_CLIP_MODEL:-ViT-B-32}"
export WORLD_CLIP_PRETRAINED="${WORLD_CLIP_PRETRAINED:-$W/open_clip_pytorch_model.bin}"
export YOLO_CONFIG_DIR="${YOLO_CONFIG_DIR:-$PKG/rbnx-build/data/ultralytics}"
ULTRALYTICS_CLIP_WEIGHTS="$YOLO_CONFIG_DIR/weights/clip/ViT-B-32.pt"
for model_path in \
    "$WORLD_YOLO_WORLD_WEIGHTS" \
    "$WORLD_MOBILE_SAM_WEIGHTS" \
    "$WORLD_CLIP_PRETRAINED" \
    "$ULTRALYTICS_CLIP_WEIGHTS"; do
    if [[ ! -s "$model_path" ]]; then
        echo "[world/native] error: missing model weight $model_path" >&2
        echo "               run system/world/scripts/build.sh before starting world." >&2
        exit 1
    fi
done

# Host-persisted World state (object-memory DB + scene-graph caches).
mkdir -p "$PKG/rbnx-build/data/robonix"
export WORLD_GRAPH_CACHE_DIR="${WORLD_GRAPH_CACHE_DIR:-$PKG/rbnx-build/data/robonix/scene_graph/cache}"
export WORLD_ANNOTATIONS_DIR="${WORLD_ANNOTATIONS_DIR:-$PKG/rbnx-build/data/robonix/scene_annotations}"
mkdir -p "$WORLD_ANNOTATIONS_DIR"

# Service knobs (same defaults the docker run wires via -e).
export WORLD_WEB_PORT="${WORLD_WEB_PORT:-50107}"
export WORLD_WEB_HOST="${WORLD_WEB_HOST-0.0.0.0}"
export WORLD_LOG_LEVEL="${WORLD_LOG_LEVEL:-INFO}"

echo "[world/native] python=$PY ros=$ROS_DISTRO rmw=${RMW_IMPLEMENTATION:-<default>} web=$WORLD_WEB_HOST:$WORLD_WEB_PORT"
exec "$PY" -m world_service.service
