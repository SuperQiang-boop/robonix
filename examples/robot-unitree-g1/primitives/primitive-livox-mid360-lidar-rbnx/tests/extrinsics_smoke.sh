#!/usr/bin/env bash
# Compile and run packet-level checks without starting the hardware driver.
set -euo pipefail
pkg_dir="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$pkg_dir/rbnx-build/tests"
g++ -std=c++14 -pthread -I/usr/local/include \
  -I"$pkg_dir/src/livox_ros_driver2/src" \
  "$pkg_dir/tests/extrinsics_smoke.cpp" \
  "$pkg_dir/src/livox_ros_driver2/src/comm/pub_handler.cpp" \
  "$pkg_dir/src/livox_ros_driver2/src/comm/comm.cpp" \
  -L/usr/local/lib -Wl,-rpath,/usr/local/lib -llivox_lidar_sdk_shared \
  -o "$pkg_dir/rbnx-build/tests/extrinsics_smoke"
"$pkg_dir/rbnx-build/tests/extrinsics_smoke"
