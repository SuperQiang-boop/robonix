#!/usr/bin/env bash
# Build the internal Python search extension without RTAB-Map or live ROS.
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cmake -S "$PKG/cpp" -B "$PKG/rbnx-build/search" \
    -DRELOCALIZATION_BUILD_EXPORTER=OFF -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release \
    -DPython3_EXECUTABLE="${RELOCALIZATION_PYTHON:-/usr/bin/python3}"
cmake --build "$PKG/rbnx-build/search" --parallel 2
ctest --test-dir "$PKG/rbnx-build/search" --output-on-failure
