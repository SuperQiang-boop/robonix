#!/usr/bin/env bash
# Check the startup environment even when rbnx inherits the old generated overlay.
set -euo pipefail
PKG="$(cd "$(dirname "$0")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
if [[ -f "$PKG/rbnx-build/codegen/ros2_idl/install/setup.bash" ]]; then
    source "$PKG/rbnx-build/codegen/ros2_idl/install/setup.bash"
fi
set -u
source "$PKG/scripts/ros_env.sh"
export PYTHONPATH="$PKG/rbnx-build/codegen/proto_gen:$(rbnx path robonix-api):${PYTHONPATH:-}"
/usr/bin/python3 - <<'PY'
import sys
import rcl_interfaces
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import PointCloud2, Imu
from rclpy.type_support import check_for_type_support
import robonix_api
import robonix_contracts_pb2
for cls in (GetParameters, PointCloud2, Imu):
    check_for_type_support(cls)
assert rcl_interfaces.__file__.startswith('/opt/ros/'), rcl_interfaces.__file__
print('ROS type support OK:', sys.executable, rcl_interfaces.__file__)
PY
