#!/usr/bin/env bash
# Uses the built Python environment; offline preparation does not require ROS.
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
export PYTHONPATH="$PKG:${PYTHONPATH:-}"
exec "$PKG/rbnx-build/venv/bin/python" -m relocalization_rbnx.prepare "$@"
