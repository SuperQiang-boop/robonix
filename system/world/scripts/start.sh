#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# World service start phase — `rbnx boot` calls this via package_manifest.yaml.
#
# Runs the `robonix-world` container on the host network so it shares
# the host ROS 2 graph with whatever sim/robot container publishes the
# observation topics. The runtime RMW is passed through so World uses
# the same transport as the rest of the deploy.
#
# Trap discipline: when boot SIGTERMs our PGID, this script's TERM
# trap stops + removes the container. `docker run --rm` alone is not
# enough; docker leaks containers if the host wrapper dies before
# sending stop.

set -euo pipefail

# ── Platform dispatch (same scheme as mapping_rbnx) ─────────────────────────
#   ROBONIX_WORLD_FORCE=native|docker     explicit hard pin
#   else auto: ROBONIX_WORLD_PLATFORM=jetson_orin → native, otherwise docker
# native → scripts/start_native.sh (host venv + host JetPack torch, no docker).
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
is_native_platform() { case "$1" in jetson_orin|jetson*|orin*) return 0 ;; *) return 1 ;; esac }
case "${ROBONIX_WORLD_FORCE:-}" in
    native) MODE=native ;;
    docker) MODE=docker ;;
    "") if is_native_platform "${ROBONIX_WORLD_PLATFORM:-}"; then MODE=native; else MODE=docker; fi ;;
    *) echo "[world/start] ROBONIX_WORLD_FORCE=${ROBONIX_WORLD_FORCE} not in {native,docker}" >&2; exit 2 ;;
esac
echo "[world/start] mode=${MODE} (FORCE=${ROBONIX_WORLD_FORCE:-} PLATFORM=${ROBONIX_WORLD_PLATFORM:-})"
if [[ "$MODE" == "native" ]]; then
    exec bash "${PKG}/scripts/start_native.sh"
fi

CT="${ROBONIX_WORLD_CONTAINER:-robonix_world}"
IMG="${ROBONIX_WORLD_IMAGE:-robonix-world}"
RUNTIME_PROTO_TMP=""
WORLD_DEPLOY_MANIFEST=""

if [[ -n "${ROBONIX_DEPLOY_DIR:-}" ]]; then
    WORLD_DEPLOY_MANIFEST="${ROBONIX_DEPLOY_DIR}/robonix_manifest.yaml"
    if [[ ! -f "$WORLD_DEPLOY_MANIFEST" || ! -r "$WORLD_DEPLOY_MANIFEST" ]]; then
        echo "[world/start] deployment manifest is not readable: $WORLD_DEPLOY_MANIFEST" >&2
        exit 2
    fi
fi

# Docker does not inherit the host's Fast DDS profile or its file. Validate
# before touching containers; an unset override preserves the image default.
declare -a FASTRTPS_ARGS=()
if [[ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]]; then
    if [[ ! -f "$FASTRTPS_DEFAULT_PROFILES_FILE" || ! -r "$FASTRTPS_DEFAULT_PROFILES_FILE" ]]; then
        echo "[world/start] Fast DDS profile is not a readable file: $FASTRTPS_DEFAULT_PROFILES_FILE" >&2
        exit 2
    fi
    FASTRTPS_PROFILE="$(realpath -- "$FASTRTPS_DEFAULT_PROFILES_FILE")"
    FASTRTPS_ARGS=(
        -v "$FASTRTPS_PROFILE:/tmp/robonix-fastrtps.xml:ro"
        -e FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/robonix-fastrtps.xml
    )
fi

cleanup() {
    timeout 15s docker stop "$CT" >/dev/null 2>&1 || true
    if [[ -n "$RUNTIME_PROTO_TMP" ]]; then
        rm -rf -- "$RUNTIME_PROTO_TMP"
    fi
}
trap cleanup EXIT INT TERM

# Drop a stopped container from a previous run.
docker rm -f "$CT" >/dev/null 2>&1 || true

# Python protobuf generated code is only forward-compatible within protobuf's
# documented runtime window.  `rbnx codegen` intentionally uses the host's
# active python3, which can be newer than this image (for example gencode
# 7.35.0 with a 6.33.6 runtime).  Never suppress protobuf's version check and
# never guess a package pin here.  Instead, regenerate World's Python stubs
# with the exact grpc_tools/protobuf stack that will import them, without
# network access, then validate every generated module before World starts.
# Keep these stubs separate from the host build output: other packages may
# legitimately use a different Python runtime.
prepare_runtime_proto_gen() {
    local proto_staging="$PKG/rbnx-build/proto-staging"
    local runtime_proto
    local runtime_proto_gen="$PKG/rbnx-build/codegen/world_proto_gen"

    runtime_proto="$(rbnx path runtime-proto)" || {
        echo "[world/start] cannot resolve Robonix runtime proto directory" >&2
        return 1
    }
    [[ -d "$runtime_proto" && -f "$runtime_proto/src.proto" ]] || {
        echo "[world/start] missing runtime src.proto: $runtime_proto" >&2
        return 1
    }
    [[ -d "$proto_staging" ]] \
        && find "$proto_staging" -maxdepth 1 -type f -name '*.proto' -print -quit \
            | grep -q . || {
        echo "[world/start] missing staged package protos; run rbnx build first" >&2
        return 1
    }

    mkdir -p "$PKG/rbnx-build/codegen"
    RUNTIME_PROTO_TMP="$(mktemp -d "${runtime_proto_gen}.tmp.XXXXXX")"

    docker run --rm \
        --network none \
        --entrypoint sh \
        --user "$(id -u):$(id -g)" \
        -e HOME=/tmp \
        -v "$runtime_proto:/runtime-proto:ro" \
        -v "$proto_staging:/proto-staging:ro" \
        -v "$RUNTIME_PROTO_TMP:/proto-gen" \
        "$IMG" -ec '
            python3 -m grpc_tools.protoc \
                -I/runtime-proto \
                -I/proto-staging \
                --python_out=/proto-gen \
                --grpc_python_out=/proto-gen \
                /runtime-proto/*.proto \
                /proto-staging/*.proto
            PYTHONPATH=/proto-gen python3 -c '\''import importlib, pathlib; p = pathlib.Path("/proto-gen"); modules = sorted({f.stem for f in p.glob("*_pb2.py")} | {f.stem for f in p.glob("*_pb2_grpc.py")}); assert modules; [importlib.import_module(name) for name in modules]'\''
        '

    rm -rf -- "$runtime_proto_gen"
    mv -- "$RUNTIME_PROTO_TMP" "$runtime_proto_gen"
    RUNTIME_PROTO_TMP=""
    echo "[world/start] runtime-compatible protobuf stubs ready"
}

prepare_runtime_proto_gen

mkdir -p rbnx-build/data
# Host-persisted World state. Mounted at /data/robonix in the container so the
# object-memory DB — and the scene-graph JSON caches, which otherwise die with
# the --rm container — survive across boots. CI sets WORLD_DATA_DIR to keep
# semantic/object state isolated per run.
WORLD_HOST_DATA_DIR="${WORLD_DATA_DIR:-$(pwd)/rbnx-build/data/robonix}"
mkdir -p "$WORLD_HOST_DATA_DIR"

declare -a EXTRA_MOUNTS=()
if [[ -n "${RBNX_CONFIG_FILE:-}" ]]; then
    EXTRA_MOUNTS+=(-v "${RBNX_CONFIG_FILE}:${RBNX_CONFIG_FILE}:ro")
fi
if [[ -n "$WORLD_DEPLOY_MANIFEST" && "$WORLD_DEPLOY_MANIFEST" != "${RBNX_CONFIG_FILE:-}" ]]; then
    EXTRA_MOUNTS+=(-v "$WORLD_DEPLOY_MANIFEST:$WORLD_DEPLOY_MANIFEST:ro")
fi

declare -a ZENOH_ARGS=()
if [[ -n "${ROBONIX_ZENOH_ROUTER:-}" ]]; then
    ZENOH_ARGS=(-e "ROBONIX_ZENOH_ROUTER=${ROBONIX_ZENOH_ROUTER}")
fi
if [[ -n "${ROBONIX_ZENOH_MODE:-}" ]]; then
    ZENOH_ARGS+=(-e "ROBONIX_ZENOH_MODE=${ROBONIX_ZENOH_MODE}")
fi
if [[ -n "${ROBONIX_ZENOH_LISTEN:-}" ]]; then
    ZENOH_ARGS+=(-e "ROBONIX_ZENOH_LISTEN=${ROBONIX_ZENOH_LISTEN}")
fi

declare -a MAP_ID_ARGS=()
if [[ -n "${WORLD_MAP_ID:-}" ]]; then
    MAP_ID_ARGS=(-e "WORLD_MAP_ID=${WORLD_MAP_ID}")
fi

# GPU passthrough: ConceptGraphs perception (YOLO-World + MobileSAM +
# CLIP) wants CUDA. Auto-detect via nvidia-smi; opt out by setting
# ROBONIX_FORCE_CPU=1. Without this flag the container sees CPU only
# and CLIP/YOLO run ~5x slower.
declare -a GPU_ARGS=()
if [[ "${ROBONIX_FORCE_CPU:-0}" != "1" ]]; then
    # NVIDIA_DRIVER_CAPABILITIES=all is REQUIRED: with just `--gpus all` (or
    # `--runtime nvidia`) and the capability unset, the NVIDIA runtime injects
    # only "utility" — nvidia-smi works but the CUDA compute libs are NOT
    # mounted, so torch.cuda.is_available() is False and ConceptGraphs silently
    # falls back to CPU (~5x slower). Requesting all caps (compute+utility+
    # graphics) makes CUDA actually available.
    if is_native_platform "${ROBONIX_WORLD_PLATFORM:-}" || [[ -e /etc/nv_tegra_release ]]; then
        # Jetson / L4T: the container gets the GPU via the NVIDIA container
        # runtime (which bind-mounts the host CUDA libs); `--gpus all` is x86.
        GPU_ARGS=(--runtime nvidia -e NVIDIA_DRIVER_CAPABILITIES=all)
    elif command -v nvidia-smi &>/dev/null && nvidia-smi &>/dev/null; then
        GPU_ARGS=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all)
    fi
    # Forward CUDA_VISIBLE_DEVICES ONLY when explicitly set (e.g. to pin one
    # GPU). The old unconditional `-e CUDA_VISIBLE_DEVICES=${CUDA_VISIBLE_DEVICES:-}`
    # passed it EMPTY when unset on the host — which tells CUDA "no GPUs" and
    # disabled the GPU even though --gpus all had mounted it (torch.cuda → False
    # while nvidia-smi still worked). Omitting it lets all mounted GPUs show.
    if [[ ${#GPU_ARGS[@]} -gt 0 && -n "${CUDA_VISIBLE_DEVICES:-}" ]]; then
        GPU_ARGS+=(-e "CUDA_VISIBLE_DEVICES=${CUDA_VISIBLE_DEVICES}")
    fi
fi

exec docker run --rm \
    --name "$CT" \
    --entrypoint /world/docker/entrypoint.sh \
    --network host \
    --ipc=host \
    "${GPU_ARGS[@]}" \
    -e ROBONIX_SRC="${ROBONIX_SRC:-127.0.0.1:50051}" \
    -e ROBONIX_PROVIDER_BIND_HOST="${ROBONIX_PROVIDER_BIND_HOST:-0.0.0.0}" \
    -e ROBONIX_ADVERTISE_HOST="${ROBONIX_ADVERTISE_HOST:-}" \
    -e ROBONIX_CAPABILITY_ID="${ROBONIX_CAPABILITY_ID:-com.robonix.system.world}" \
    -e ROBONIX_PKG_HOST_DIR="$(pwd)" \
    -e ROBONIX_DEPLOY_DIR="${ROBONIX_DEPLOY_DIR:-}" \
    -e WORLD_WEB_PORT="${WORLD_WEB_PORT:-50107}" \
    -e WORLD_WEB_HOST="${WORLD_WEB_HOST-0.0.0.0}" \
    -e WORLD_LOG_LEVEL="${WORLD_LOG_LEVEL:-INFO}" \
    -e WORLD_CG_FORCE_CPU="${WORLD_CG_FORCE_CPU:-}" \
    -e WORLD_CG_OBJ_MIN_POINTS="${WORLD_CG_OBJ_MIN_POINTS:-}" \
    -e WORLD_CG_MAX_MERGE_DIST_M="${WORLD_CG_MAX_MERGE_DIST_M:-}" \
    -e WORLD_CG_CROSS_CLASS_CENTROID_MAX_M="${WORLD_CG_CROSS_CLASS_CENTROID_MAX_M:-}" \
    -e WORLD_CG_CROSS_CLASS_IOU_THRESH="${WORLD_CG_CROSS_CLASS_IOU_THRESH:-}" \
    -e WORLD_CG_CROSS_CLASS_OVERLAP_THRESH="${WORLD_CG_CROSS_CLASS_OVERLAP_THRESH:-}" \
    -e WORLD_CG_MERGE_OVERLAP_THRESH="${WORLD_CG_MERGE_OVERLAP_THRESH:-}" \
    -e WORLD_CG_MERGE_VISUAL_SIM_THRESH="${WORLD_CG_MERGE_VISUAL_SIM_THRESH:-}" \
    -e WORLD_CG_SAME_CLASS_MERGE_DIST_M="${WORLD_CG_SAME_CLASS_MERGE_DIST_M:-}" \
    -e WORLD_CG_MERGE_CLASS_GROUPS="${WORLD_CG_MERGE_CLASS_GROUPS:-}" \
    -e WORLD_CG_MISSING_DETECTIONS="${WORLD_CG_MISSING_DETECTIONS:-3}" \
    -e WORLD_OBJECT_TTL_SEC="${WORLD_OBJECT_TTL_SEC:-}" \
    -e WORLD_DETECT_PERIOD_S="${WORLD_DETECT_PERIOD_S:-3.0}" \
    -e WORLD_CG_OBJ_MAX_POINTS="${WORLD_CG_OBJ_MAX_POINTS:-1500}" \
    -e WORLD_CG_MERGE_THRESHOLD="${WORLD_CG_MERGE_THRESHOLD:-0.55}" \
    -e WORLD_OBJECT_WATCHDOG="${WORLD_OBJECT_WATCHDOG:-0}" \
    -e WORLD_GRAPH_ENABLED="${WORLD_GRAPH_ENABLED:-true}" \
    -e WORLD_PERCEPTION_WAIT_S="${WORLD_PERCEPTION_WAIT_S:-30}" \
    -e WORLD_OPEN_VOCAB_CLASSES="${WORLD_OPEN_VOCAB_CLASSES:-}" \
    -e VLM_BASE_URL="${VLM_BASE_URL:-}" \
    -e VLM_API_KEY="${VLM_API_KEY:-}" \
    -e VLM_MODEL="${VLM_MODEL:-}" \
    -e VLM_REASONING_EFFORT="${VLM_REASONING_EFFORT:-}" \
    -e WORLD_GRAPH_CAPTION_ENABLED="${WORLD_GRAPH_CAPTION_ENABLED:-true}" \
    -e WORLD_GRAPH_RELATION_ENABLED="${WORLD_GRAPH_RELATION_ENABLED:-true}" \
    -e WORLD_GRAPH_INTERVAL_SEC="${WORLD_GRAPH_INTERVAL_SEC:-30}" \
    -e WORLD_GRAPH_CACHE_DIR="${WORLD_GRAPH_CACHE_DIR:-/data/robonix/scene_graph/cache}" \
    -e WORLD_ANNOTATIONS_DIR="${WORLD_ANNOTATIONS_DIR:-/data/robonix/scene_annotations}" \
    -e WORLD_MAP_LOAD_TIMEOUT_S="${WORLD_MAP_LOAD_TIMEOUT_S:-240}" \
    -e WORLD_GRAPH_MIN_OBSERVATIONS="${WORLD_GRAPH_MIN_OBSERVATIONS:-2}" \
    -e WORLD_GRAPH_MAX_OBJECTS="${WORLD_GRAPH_MAX_OBJECTS:-80}" \
    -e WORLD_GRAPH_MAX_LLM_RELATIONS_PER_CYCLE="${WORLD_GRAPH_MAX_LLM_RELATIONS_PER_CYCLE:-20}" \
    -e WORLD_OBJECT_MEMORY_ENABLED="${WORLD_OBJECT_MEMORY_ENABLED:-true}" \
    -e WORLD_OBJECT_MEMORY_DB="${WORLD_OBJECT_MEMORY_DB:-/data/robonix/scene_memory/objects.db}" \
    -e WORLD_RESTORE_ON_START="${WORLD_RESTORE_ON_START:-false}" \
    "${MAP_ID_ARGS[@]}" \
    -e RBNX_CONFIG_FILE="${RBNX_CONFIG_FILE:-}" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}" \
    -e CYCLONEDDS_URI="${CYCLONEDDS_URI:-}" \
    "${FASTRTPS_ARGS[@]}" \
    "${ZENOH_ARGS[@]}" \
    -v "$(pwd)":/world \
    -v "$PKG/rbnx-build/codegen/world_proto_gen:/world/rbnx-build/codegen/proto_gen:ro" \
    -v "$PKG/rbnx-build/codegen/robonix_mcp_types:/world/rbnx-build/codegen/robonix_mcp_types:ro" \
    -v "$WORLD_HOST_DATA_DIR":/data/robonix \
    -v "$(rbnx path robonix-api)":/robonix-api:ro \
    "${EXTRA_MOUNTS[@]}" \
    "$IMG"
