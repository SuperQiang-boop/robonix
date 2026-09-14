from __future__ import annotations

import importlib.util
import io
import json
import tempfile
import sys
import types
import unittest
from pathlib import Path
from unittest import mock


class _Primitive:
    def __init__(self, **_kwargs):
        self.declared_topics: list[tuple[tuple, dict]] = []

    @staticmethod
    def on_init(func):
        return func

    @staticmethod
    def on_shutdown(func):
        return func

    def declare_ros2_topic(self, *args, **kwargs):
        self.declared_topics.append((args, kwargs))

    @staticmethod
    def run():
        return None


def _load_driver():
    api = types.ModuleType("robonix_api")
    api.Primitive = _Primitive
    api.Ok = lambda value=None: ("ok", value)
    api.Err = lambda value=None: ("err", value)

    module_path = Path(__file__).parents[1] / "mid360_driver" / "main.py"
    spec = importlib.util.spec_from_file_location("mid360_driver_main_test", module_path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)

    with mock.patch.dict(sys.modules, {"robonix_api": api}):
        spec.loader.exec_module(module)
    return module


class InitLifecycleTests(unittest.TestCase):
    def test_default_transfer_format_is_ros2_pointcloud2(self):
        driver = _load_driver()
        proc = mock.Mock(pid=1234, stdout=io.BytesIO(b""))

        with (
            mock.patch.object(driver, "_resolve_livox_config", return_value="/tmp/mid360.json"),
            mock.patch.object(driver.subprocess, "Popen", return_value=proc) as popen,
        ):
            driver._spawn_livox({})

        self.assertEqual(popen.call_args.kwargs["env"]["LIVOX_XFER_FORMAT"], "0")

    def test_init_starts_one_static_transform_publisher(self):
        driver = _load_driver()

        with (
            mock.patch.object(driver, "_spawn_livox"),
            mock.patch.object(driver, "_wait_for_pointcloud", return_value=True),
            mock.patch.object(driver, "_spawn_stp") as spawn_stp,
        ):
            result = driver.init({"extrinsics": {"x": 0.1}})

        self.assertEqual(result[0], "ok")
        spawn_stp.assert_called_once_with({"extrinsics": {"x": 0.1}})
        self.assertEqual(len(driver.cap.declared_topics), 1)

    def test_network_config_preserves_inverted_mount_extrinsics(self):
        """Generated network settings must retain the configured data rotation."""
        driver = _load_driver()
        source = driver._pkg_root / "src/livox_ros_driver2/config/MID360_config.json"
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            target = root / "src/livox_ros_driver2/config/MID360_config.json"
            target.parent.mkdir(parents=True)
            target.write_bytes(source.read_bytes())
            with mock.patch.object(driver, "_pkg_root", root):
                output = driver._resolve_livox_config({
                    "lidar_ip": "192.168.123.120", "host_ip": "192.168.123.100"
                })
            config = json.loads(Path(output).read_text())
        self.assertEqual(config["lidar_configs"][0]["extrinsic_parameter"], {
            "roll": 180.0, "pitch": 0.0, "yaw": 0.0, "x": 0, "y": 0, "z": 0
        })
        self.assertEqual(config["MID360"]["host_net_info"]["imu_data_ip"],
                         "192.168.123.100")

    def test_manifest_uses_implicit_shared_lifecycle_driver(self):
        manifest = (Path(__file__).parents[1] / "package_manifest.yaml").read_text()
        self.assertNotIn("- name: robonix/primitive/lidar/driver", manifest)
        self.assertNotIn("- name: robonix/lifecycle/driver", manifest)


if __name__ == "__main__":
    unittest.main()
