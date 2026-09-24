#!/usr/bin/env bash
set -euo pipefail
PKG="$(cd "$(dirname "$0")/.." && pwd)"
PYTHON="${FACE_RECOGNITION_TEST_PYTHON:-$PKG/rbnx-build/venv/bin/python}"
export PYTHONPATH="$PKG:$PKG/rbnx-build/codegen/proto_gen:${PYTHONPATH:-}"
"$PYTHON" -m unittest discover -s "$PKG/tests" -v
"$PYTHON" -m py_compile "$PKG/face_recognition/main.py"
echo 'face recognition bridge smoke OK'
