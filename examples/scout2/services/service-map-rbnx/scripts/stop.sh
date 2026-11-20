#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

NATIVE_PLATFORMS=("jetson_orin")
is_native_platform() {
    local p="$1"
    for w in "${NATIVE_PLATFORMS[@]}"; do
        [[ "$p" == "$w" ]] && return 0
    done
    return 1
}

MODE=""
case "${ROBONIX_MAPPING_FORCE:-}" in
    native) MODE=native ;;
    docker) MODE=docker ;;
    "") ;;
    *) echo "[mapping/stop] ROBONIX_MAPPING_FORCE=${ROBONIX_MAPPING_FORCE} not in {native,docker}" >&2; exit 2 ;;
esac
if [[ -z "$MODE" ]]; then
    if is_native_platform "${ROBONIX_MAPPING_PLATFORM:-}"; then MODE=native; else MODE=docker; fi
fi

echo "[mapping/stop] mode=${MODE}"
if [[ "$MODE" == "docker" ]]; then
    CT="${ROBONIX_MAPPING_CONTAINER:-robonix_mapping}"
    if docker container inspect "$CT" >/dev/null 2>&1; then
        AUTO_REMOVE="$(docker container inspect --format '{{.HostConfig.AutoRemove}}' "$CT")"
        # Fail visibly if the daemon cannot stop it; never force-remove a DB writer.
        docker stop --time "${ROBONIX_MAPPING_STOP_TIMEOUT_S:-180}" "$CT" >/dev/null
        if [[ "$AUTO_REMOVE" == true ]]; then
            # --rm owns deletion; wait until its name can safely be reused.
            for _ in {1..100}; do
                docker container inspect "$CT" >/dev/null 2>&1 || break
                sleep 0.1
            done
            if docker container inspect "$CT" >/dev/null 2>&1; then
                echo "[mapping/stop] timed out waiting for Docker auto-removal: $CT" >&2
                exit 1
            fi
            docker info >/dev/null
        else
            docker rm "$CT" >/dev/null
        fi
    elif ! docker info >/dev/null 2>&1; then
        echo "[mapping/stop] cannot contact Docker; map shutdown is unconfirmed" >&2
        exit 1
    fi
    exit 0
fi

pkill -TERM -f "${PKG}.*mapping_rbnx.atlas_bridge" 2>/dev/null || true
pkill -TERM -f "${PKG}/scripts/start_engine.sh" 2>/dev/null || true
sleep 1
pkill -KILL -f "${PKG}.*mapping_rbnx.atlas_bridge" 2>/dev/null || true
pkill -KILL -f "${PKG}/scripts/start_engine.sh" 2>/dev/null || true
