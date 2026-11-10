"""Check native ROS type support without connecting to a robot or camera."""
import os
from pathlib import Path
import subprocess
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
ROS_SETUP = Path("/opt/ros") / os.environ.get("ROS_DISTRO", "humble") / "setup.bash"


@unittest.skipUnless(ROS_SETUP.is_file(), "requires a system ROS installation")
class RosOverlayTest(unittest.TestCase):
    def test_system_python_loads_native_type_support(self):
        """Source the built overlays and eagerly import their native ROS extensions."""
        command = '''
set -e
source "$1"
source "$2"
source "$3"
/usr/bin/python3 - <<'PY'
from rosidl_generator_py import import_type_support
for package in ("builtin_interfaces", "rcl_interfaces", "sensor_msgs", "geometry_msgs"):
    support = import_type_support(package)
    print(package, support.__file__)
PY
'''
        result = subprocess.run(
            [
                "bash", "-c", command, "check-ros-overlay",
                str(ROS_SETUP),
                str(PACKAGE_ROOT / "rbnx-build/codegen/ros2_idl/install/setup.bash"),
                str(PACKAGE_ROOT / "rbnx-build/ws/install/setup.bash"),
            ],
            capture_output=True,
            text=True,
            timeout=60,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
