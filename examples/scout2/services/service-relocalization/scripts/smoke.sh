#!/usr/bin/env bash
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
ROS_SETUP="${RELOCALIZATION_ROS_SETUP:-/opt/ros/${ROS_DISTRO:-humble}/setup.bash}"
set +u
source "$ROS_SETUP"
source "$PKG/rbnx-build/codegen/ros2_idl/install/local_setup.bash"
set -u
export PYTHONPATH="$PKG:$PKG/rbnx-build/search:$PKG/rbnx-build/codegen/proto_gen:${PYTHONPATH:-}"
"$PKG/rbnx-build/venv/bin/python" -m unittest discover -s "$PKG/tests" -v
if [[ "${1:-}" == "--e2e" ]]; then
    "$PKG/rbnx-build/venv/bin/python" "$PKG/tests/e2e_atlas.py"
    "$PKG/rbnx-build/venv/bin/python" "$PKG/tests/e2e_observations.py"
    "$PKG/rbnx-build/venv/bin/python" "$PKG/tests/e2e_production.py"
elif [[ "${1:-}" == "--production-e2e" ]]; then
    "$PKG/rbnx-build/venv/bin/python" "$PKG/tests/e2e_production.py"
elif [[ "${1:-}" == "--observations-e2e" ]]; then
    "$PKG/rbnx-build/venv/bin/python" "$PKG/tests/e2e_observations.py"
elif [[ "${1:-}" == "--probe-e2e" ]]; then
    "$PKG/rbnx-build/venv/bin/python" "$PKG/tests/e2e_probe.py"
elif [[ "${1:-}" == "--computation-e2e" ]]; then
    # Build scripts/build_search.sh first; missing native bindings fail explicitly.
    export PYTHONPATH="$PKG/rbnx-build/search:$PYTHONPATH"
    "$PKG/rbnx-build/venv/bin/python" "$PKG/tests/e2e_computation.py"
fi
