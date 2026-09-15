#!/usr/bin/env bash
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
cd "$PKG"
rbnx codegen -p "$PKG" --mcp

ROBONIX_API_ROOT="$(rbnx path robonix-api)"
PYTHONPATH="$PKG:$ROBONIX_API_ROOT:$PKG/rbnx-build/codegen/proto_gen:$PKG/rbnx-build/codegen/robonix_mcp_types:${PYTHONPATH:-}" \
  python3 -m py_compile "$PKG/g1_basic_movement/main.py"
