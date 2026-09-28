#!/usr/bin/env bash
set -euo pipefail
PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROBONIX_API_ROOT="$(rbnx path robonix-api)"
export PYTHONPATH="$PKG:$ROBONIX_API_ROOT:$PKG/rbnx-build/codegen/proto_gen:$PKG/rbnx-build/codegen/robonix_mcp_types:${PYTHONPATH:-}"
exec "$PKG/rbnx-build/venv/bin/python" -m g1_face_greeting.main
