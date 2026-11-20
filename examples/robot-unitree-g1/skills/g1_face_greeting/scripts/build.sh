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
# `[codegen]` keeps grpcio-tools inside this venv: codegen runs under the venv
# Python, so its protoc must pair with the venv's protobuf runtime. Falling back
# to a system grpcio-tools can emit gencode newer than the pinned protobuf and
# make import validation fail.
uv pip install --python "$BUILD_ROOT/venv/bin/python" \
  "robonix-api[codegen] @ file://$SOURCE/pylib/robonix-api" -r "$PKG/requirements.txt"
RBNX_CODEGEN_PYTHON="$BUILD_ROOT/venv/bin/python" rbnx codegen -p "$PKG" --mcp
ROBONIX_API_ROOT="$(rbnx path robonix-api)"
PYTHONPATH="$PKG:$ROBONIX_API_ROOT:$PKG/rbnx-build/codegen/proto_gen:$PKG/rbnx-build/codegen/robonix_mcp_types:${PYTHONPATH:-}" \
  "$BUILD_ROOT/venv/bin/python" -m py_compile "$PKG/g1_face_greeting/main.py"
