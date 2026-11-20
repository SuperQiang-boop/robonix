#!/usr/bin/env bash
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
export PYTHONPATH="$PKG:$PKG/rbnx-build/codegen/proto_gen:${PYTHONPATH:-}"
exec "$PKG/rbnx-build/venv/bin/python" -m face_recognition.main
