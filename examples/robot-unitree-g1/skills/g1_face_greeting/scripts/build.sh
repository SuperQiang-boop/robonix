#!/usr/bin/env bash
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
SOURCE="${ROBONIX_SOURCE_PATH:-$(git -C "$PKG" rev-parse --show-toplevel)}"
BUILD_ROOT="$PKG/rbnx-build"
cd "$PKG"
PYTHON="${G1_FACE_GREETING_PYTHON:-/usr/bin/python3}"
if [[ -d "$BUILD_ROOT/venv" && ! -x "$BUILD_ROOT/venv/bin/python" ]]; then
  rm -rf "$BUILD_ROOT/venv"
fi
if [[ ! -x "$BUILD_ROOT/venv/bin/python" ]]; then
  uv venv --python "$PYTHON" --system-site-packages "$BUILD_ROOT/venv"
fi
uv pip install --python "$BUILD_ROOT/venv/bin/python" \
  "$SOURCE/pylib/robonix-api" 'mcp>=1,<2' 'fastmcp[client]>=3,<4'
RBNX_CODEGEN_PYTHON="$BUILD_ROOT/venv/bin/python" rbnx codegen -p "$PKG" --mcp
ROBONIX_API_ROOT="$(rbnx path robonix-api)"
PYTHONPATH="$PKG:$ROBONIX_API_ROOT:$PKG/rbnx-build/codegen/proto_gen:$PKG/rbnx-build/codegen/robonix_mcp_types:${PYTHONPATH:-}" \
  "$BUILD_ROOT/venv/bin/python" -m py_compile "$PKG/g1_face_greeting/main.py"
