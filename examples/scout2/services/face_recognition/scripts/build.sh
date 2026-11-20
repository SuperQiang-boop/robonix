#!/usr/bin/env bash
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
SOURCE="${ROBONIX_SOURCE_PATH:-$(git -C "$PKG" rev-parse --show-toplevel)}"
BUILD_ROOT="$PKG/rbnx-build"
VSIS_SOURCE="$(cd "$PKG/vsis_video_service" && pwd -P)"
VSIS_BUILD="$BUILD_ROOT/vsis"

if [[ "${RBNX_BUILD_CLEAN:-}" == "1" ]]; then
    echo "[face_recognition/build] clean: removing $BUILD_ROOT"
    rm -rf "$BUILD_ROOT"
fi

# CMake caches the absolute source directory. A copied package can otherwise
# reuse a cache from its old location and fail before configuration starts.
if [[ -f "$VSIS_BUILD/CMakeCache.txt" ]]; then
    cached_source="$(awk -F= '$1 == "CMAKE_HOME_DIRECTORY:INTERNAL" { print substr($0, index($0, "=") + 1); exit }' "$VSIS_BUILD/CMakeCache.txt")"
    if [[ "$cached_source" != "$VSIS_SOURCE" ]]; then
        echo "[face_recognition/build] stale CMake cache: $cached_source"
        rm -rf "$VSIS_BUILD"
    fi
fi

PYTHON="${FACE_RECOGNITION_PYTHON:-/usr/bin/python3}"
if [[ -d "$BUILD_ROOT/venv" && ! -x "$BUILD_ROOT/venv/bin/python" ]]; then
    rm -rf "$BUILD_ROOT/venv"
fi
if [[ ! -x "$BUILD_ROOT/venv/bin/python" ]]; then
    uv venv --python "$PYTHON" --system-site-packages "$BUILD_ROOT/venv"
fi
uv pip install --python "$BUILD_ROOT/venv/bin/python" "$SOURCE/pylib/robonix-api" "$PKG"
RBNX_CODEGEN_PYTHON="$BUILD_ROOT/venv/bin/python" rbnx codegen -p "$PKG"

# Rebuild the bundled executable so source fixes do not leave a stale binary.
cmake -S "$VSIS_SOURCE" -B "$VSIS_BUILD" -DCMAKE_BUILD_TYPE=Release
cmake --build "$VSIS_BUILD" --parallel 2
