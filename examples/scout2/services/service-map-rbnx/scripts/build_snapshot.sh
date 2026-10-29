#!/usr/bin/env bash
# Build against the exact RTAB-Map/ROS libraries used by this deployment target.
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cmake -S "$PKG/cpp" -B "$PKG/rbnx-build/snapshot" -DCMAKE_BUILD_TYPE=Release
cmake --build "$PKG/rbnx-build/snapshot" --parallel 2
