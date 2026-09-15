#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Build phase: colcon-build the vendored livox_ros_driver2, then
# rbnx codegen so atlas_bridge can import atlas_pb2.
#
# Vendored under src/livox_ros_driver2 — includes our local fixes
# on top of upstream Livox-SDK/livox_ros_driver2 (config IPs +
# topic name + xfer_format default). See src/livox_ros_driver2.patch
# for the diff against upstream HEAD at the time of vendoring.
#
# Output goes into rbnx-build/{ws/install,codegen}/. start.sh
# sources rbnx-build/ws/install/setup.bash before launching.
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"
CLEAN="${RBNX_BUILD_CLEAN:-}"

if [[ "$CLEAN" == "1" ]]; then
    echo "[mid360_lidar/build] clean: removing rbnx-build/"
    rm -rf rbnx-build
fi
mkdir -p rbnx-build/ws/src rbnx-build/data

# Materialise package.xml from package_ROS2.xml. Livox upstream gitignores
# package.xml because it supports dual ROS1/ROS2 builds — package.xml is a
# build-time copy of the chosen variant. Skip if user already produced one.
if [[ ! -f "$PKG/src/livox_ros_driver2/package.xml" ]]; then
    cp "$PKG/src/livox_ros_driver2/package_ROS2.xml" \
       "$PKG/src/livox_ros_driver2/package.xml"
fi

# Symlink the vendored source into a scratch ws so colcon can find it
# without polluting our src/ tree with build artefacts.
ln -snf "$PKG/src/livox_ros_driver2" "$PKG/rbnx-build/ws/src/livox_ros_driver2"

# Source ROS env. Distro overridable for non-humble setups.
ROS_DISTRO="${ROS_DISTRO:-humble}"
# shellcheck disable=SC1091
set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u

echo "[mid360_lidar/build] colcon build (livox_ros_driver2)"
cd "$PKG/rbnx-build/ws"
/usr/bin/python3 -m colcon build --symlink-install --cmake-clean-cache --cmake-args \
    -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_EXECUTABLE=/usr/bin/python3
cd "$PKG"

# Robonix codegen for atlas_bridge.py imports.
FLAGS=(--ros2 --out-dir "$PKG/rbnx-build/codegen")
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)
echo "[mid360_lidar/build] rbnx codegen ${FLAGS[*]}"
rbnx codegen -p "$PKG" "${FLAGS[@]}"

# This primitive uses system sensor_msgs and protobuf contracts only.
# Do not build/source generated copies of standard ROS message packages.

touch "$PKG/rbnx-build/.rbnx-built"
echo "[mid360_lidar/build] done."
