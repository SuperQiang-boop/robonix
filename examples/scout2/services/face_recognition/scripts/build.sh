#!/usr/bin/env bash
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
SOURCE="${ROBONIX_SOURCE_PATH:-$(git -C "$PKG" rev-parse --show-toplevel)}"
PYTHON="${FACE_RECOGNITION_PYTHON:-/usr/bin/python3}"
uv venv --python "$PYTHON" --system-site-packages "$PKG/rbnx-build/venv"
uv pip install --python "$PKG/rbnx-build/venv/bin/python" "$SOURCE/pylib/robonix-api" "$PKG"
RBNX_CODEGEN_PYTHON="$PKG/rbnx-build/venv/bin/python" rbnx codegen -p "$PKG"

# Rebuild the bundled executable so source fixes do not leave a stale binary.
cmake -S "$PKG/vsis_video_service" -B "$PKG/rbnx-build/vsis" -DCMAKE_BUILD_TYPE=Release
cmake --build "$PKG/rbnx-build/vsis" --parallel 2
