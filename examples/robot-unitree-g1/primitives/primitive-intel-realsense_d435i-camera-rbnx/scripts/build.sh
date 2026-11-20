#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Build phase: colcon-build the vendored realsense2_camera, then
# rbnx codegen so atlas_bridge can import atlas_pb2 + lifecycle_pb2.
#
# Vendored under src/realsense-ros — IntelRealSense/realsense-ros at the
# version that built cleanly on the Jetson (we hit no upstream issues
# that needed local patches; if anything diverges, drop a *.patch
# alongside src/ documenting the diff).
#
# Output goes into rbnx-build/{ws/install,codegen}/. start.sh sources
# rbnx-build/ws/install/setup.bash before launching atlas_bridge.
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"
CLEAN="${RBNX_BUILD_CLEAN:-}"

if [[ "$CLEAN" == "1" ]]; then
    echo "[realsense_camera/build] clean: removing rbnx-build/"
    rm -rf rbnx-build
fi
mkdir -p rbnx-build/ws/src rbnx-build/data

ln -snf "$PKG/src/realsense-ros" "$PKG/rbnx-build/ws/src/realsense-ros"

ROS_DISTRO="${ROS_DISTRO:-humble}"
# shellcheck disable=SC1091
set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u

echo "[realsense_camera/build] colcon build (realsense2_camera + msgs)"
cd "$PKG/rbnx-build/ws"
# Skip examples; they need rclcpp_components_register_node we don't care about.
colcon build --symlink-install \
    --packages-skip realsense2_description \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
cd "$PKG"

FLAGS=(--mcp --ros2 --out-dir "$PKG/rbnx-build/codegen")
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)
echo "[realsense_camera/build] rbnx codegen ${FLAGS[*]}"
rbnx codegen -p "$PKG" "${FLAGS[@]}"

ROS2_IDL="$PKG/rbnx-build/codegen/ros2_idl"
echo "[realsense_camera/build] colcon build (Robonix ROS 2 interfaces)"
(cd "$ROS2_IDL" && colcon build)

touch "$PKG/rbnx-build/.rbnx-built"
echo "[realsense_camera/build] done."
