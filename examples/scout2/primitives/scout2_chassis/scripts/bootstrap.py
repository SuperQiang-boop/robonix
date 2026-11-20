#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Bootstrap entry for scout2_chassis primitive — adds matching ROS2 paths + driver dir."""
from __future__ import annotations

import os
import subprocess
import sys

# The selected interpreter already supplies its own standard library and packages.
# Add only ROS paths matching that interpreter's Python version.
_python_version = f"python{sys.version_info.major}.{sys.version_info.minor}"
_ros_root = os.path.join('/opt/ros', os.environ.get('ROS_DISTRO', 'humble'))
for _p in (
    os.path.join(_ros_root, 'lib', _python_version, 'site-packages'),
    os.path.join(_ros_root, 'local/lib', _python_version, 'dist-packages'),
):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)
_parent = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _parent)
sys.path.insert(0, os.path.join(_parent, 'rbnx-build/codegen/proto_gen'))

# Resolve the current source tree through the same CLI used to launch the package.
_api_path = os.environ.get('ROBONIX_API') or subprocess.check_output(
    ['rbnx', 'path', 'robonix-api'], text=True,
).strip()
if not _api_path or not os.path.isdir(os.path.join(_api_path, 'robonix_api')):
    raise RuntimeError('Cannot locate robonix-api; run rbnx setup from the repository root')
sys.path.insert(0, _api_path)

from chassis_driver import driver as _driver  # noqa: E402

if __name__ == '__main__':
    _driver.scout2_chassis.run()
