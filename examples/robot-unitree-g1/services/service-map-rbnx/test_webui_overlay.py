# SPDX-License-Identifier: MulanPSL-2.0
"""Projection of live range data into the map frame for the page overlay.

The overlay exists so an operator can see whether localization is right: if the
returns do not sit on the walls of the map, the pose is wrong. That only works
if the points are placed correctly, so the geometry is pinned here. Point cloud
input is covered too -- a deployment with a 3-D lidar (the Ranger's mid360) has
no 2-D scan, and a simulator with a 2-D scan has no cloud, so neither path can
be verified by running only one of them.
"""
from __future__ import annotations

import math
import types
import unittest
from unittest.mock import patch

from mapping_rbnx import webui


def _scan(ranges, angle_min=0.0, inc=math.pi / 2, rmin=0.1, rmax=10.0,
          frame="lidar"):
    return types.SimpleNamespace(
        ranges=ranges, angle_min=angle_min, angle_increment=inc,
        range_min=rmin, range_max=rmax,
        header=types.SimpleNamespace(frame_id=frame))


class ScanProjectionTest(unittest.TestCase):
    def test_a_scan_becomes_cartesian_points_in_the_sensor_frame(self):
        out = webui._scan_points(_scan([1.0, 2.0]))
        self.assertEqual(out["frame"], "lidar")
        (x0, y0), (x1, y1) = out["pts"]
        self.assertAlmostEqual(x0, 1.0, places=6)
        self.assertAlmostEqual(y0, 0.0, places=6)
        self.assertAlmostEqual(x1, 0.0, places=6)
        self.assertAlmostEqual(y1, 2.0, places=6)

    def test_no_return_readings_are_dropped_rather_than_drawn_as_walls(self):
        # inf is what a lidar reports for "nothing there". Drawing those at
        # range_max paints a fake wall in a ring around the robot, which is
        # exactly the artefact that would make a good pose look wrong.
        out = webui._scan_points(_scan([float("inf"), float("nan"), 99.0, 0.01, 1.0]))
        self.assertEqual(len(out["pts"]), 1)

    def test_a_dense_scan_is_subsampled_to_the_overlay_budget(self):
        out = webui._scan_points(_scan([1.0] * 9000, inc=0.001))
        self.assertLessEqual(len(out["pts"]), webui.MAX_OVERLAY_POINTS + 1)
        self.assertGreater(len(out["pts"]), 0)


class CloudProjectionTest(unittest.TestCase):
    def _cloud(self, pts, frame="velodyne"):
        msg = types.SimpleNamespace(width=len(pts), height=1,
                                    header=types.SimpleNamespace(frame_id=frame))
        fake = types.ModuleType("sensor_msgs_py")
        fake.point_cloud2 = types.SimpleNamespace(
            read_points=lambda *a, **k: iter(pts))
        return msg, fake

    def test_returns_far_above_or_below_the_sensor_are_dropped(self):
        # The overlay is compared against a 2-D occupancy grid; ceiling and
        # floor returns only obscure it.
        pts = [(1.0, 0.0, 0.0), (2.0, 0.0, 3.0), (3.0, 0.0, -2.5)]
        msg, fake = self._cloud(pts)
        with patch.dict("sys.modules", {"sensor_msgs_py": fake}):
            out = webui._cloud_points(msg)
        self.assertEqual(out["pts"], [(1.0, 0.0)])
        self.assertEqual(out["frame"], "velodyne")

    def test_a_missing_sensor_msgs_py_yields_an_empty_overlay_not_a_crash(self):
        msg, _ = self._cloud([(1.0, 0.0, 0.0)])
        with patch.dict("sys.modules", {"sensor_msgs_py": None}):
            out = webui._cloud_points(msg)
        self.assertEqual(out["pts"], [])


class MapFrameTest(unittest.TestCase):
    def setUp(self):
        webui._latest["scan"] = {"frame": "lidar", "pts": [(1.0, 0.0)],
                                 "t": 1e12}
        webui.set_sensor_topics(scan="/scan")
        self.addCleanup(webui.set_sensor_topics, "", "")
        self.addCleanup(webui._latest.__setitem__, "scan", None)

    def test_points_are_placed_using_the_sensor_transform(self):
        # Sensor at (2, 3) rotated 90 degrees: a return 1 m straight ahead of
        # the sensor belongs at (2, 4) on the map.
        with patch.object(webui, "_frame_to_map",
                          return_value=(2.0, 3.0, math.pi / 2)):
            out = webui._overlay_in_map("scan")
        self.assertEqual(out["pts"], [[2.0, 4.0]])

    def test_a_missing_transform_reports_itself_instead_of_drawing_nothing(self):
        with patch.object(webui, "_frame_to_map", return_value=None):
            out = webui._overlay_in_map("scan")
        self.assertEqual(out["pts"], [])
        self.assertEqual(out["frame"], "lidar")
        self.assertIn("transform", out["why"])
        # The reason has to point at localization, not at the sensor.
        self.assertIn("map frame", out["why"])

    def test_stale_data_is_flagged(self):
        webui._latest["scan"] = {"frame": "lidar", "pts": [(1.0, 0.0)], "t": 0.0}
        with patch.object(webui, "_frame_to_map", return_value=(0.0, 0.0, 0.0)):
            self.assertTrue(webui._overlay_in_map("scan")["stale"])

    def test_no_data_at_all_names_the_topic_it_is_waiting_on(self):
        webui._latest["scan"] = None
        out = webui._overlay_in_map("scan")
        self.assertEqual(out["pts"], [])
        self.assertTrue(out["stale"])
        self.assertIn("/scan", out["why"])

    def test_an_unbound_sensor_says_so_rather_than_blaming_the_data(self):
        webui.set_sensor_topics()
        out = webui._overlay_in_map("scan")
        self.assertIn("no capability bound", out["why"])


class TopicBindingTest(unittest.TestCase):
    def test_topics_come_from_the_bridge_and_default_to_none(self):
        self.addCleanup(webui.set_sensor_topics, "", "")
        webui.set_sensor_topics(scan="/scanner_normalized", cloud="/livox/lidar")
        self.assertEqual(webui._sensor_topics["scan"], "/scanner_normalized")
        self.assertEqual(webui._sensor_topics["cloud"], "/livox/lidar")
        webui.set_sensor_topics()
        self.assertEqual(webui._sensor_topics, {"scan": "", "cloud": ""})


if __name__ == "__main__":
    unittest.main()


class ScanDiscoveryTest(unittest.TestCase):
    """Finding a 2-D scan when no capability declares one.

    The Ranger's mid360 declares only robonix/primitive/lidar/lidar3d; its 2-D
    scan is projected from the cloud by the navigation service and never
    declared. Field deployments update the mapping package alone, so the page
    has to find that scan itself rather than wait for another repository to
    declare it.
    """

    def test_the_filtered_scan_wins_over_the_raw_projection(self):
        topics = [("/scanner/cloud", ["sensor_msgs/msg/PointCloud2"]),
                  ("/scanner/scan_raw", ["sensor_msgs/msg/LaserScan"]),
                  ("/scanner/scan", ["sensor_msgs/msg/LaserScan"])]
        self.assertEqual(webui.pick_scan_topic(topics), "/scanner/scan")

    def test_a_raw_scan_is_still_better_than_no_scan(self):
        topics = [("/scanner/scan_raw", ["sensor_msgs/msg/LaserScan"])]
        self.assertEqual(webui.pick_scan_topic(topics), "/scanner/scan_raw")

    def test_a_graph_with_no_scan_yields_nothing(self):
        topics = [("/scanner/cloud", ["sensor_msgs/msg/PointCloud2"]),
                  ("/map", ["nav_msgs/msg/OccupancyGrid"])]
        self.assertEqual(webui.pick_scan_topic(topics), "")

    def test_the_shortest_name_wins_among_equals(self):
        topics = [("/robot/front/scan_filtered", ["sensor_msgs/msg/LaserScan"]),
                  ("/scan", ["sensor_msgs/msg/LaserScan"])]
        self.assertEqual(webui.pick_scan_topic(topics), "/scan")

    def test_an_explicit_topic_beats_the_resolved_capability(self):
        # A deployment that pins one must not be second-guessed by discovery.
        self.addCleanup(setattr, webui, "SCAN_TOPIC_OVERRIDE",
                        webui.SCAN_TOPIC_OVERRIDE)
        self.addCleanup(webui.set_sensor_topics, "", "")
        webui.SCAN_TOPIC_OVERRIDE = "/pinned/scan"
        webui.set_sensor_topics(scan="/resolved/scan", cloud="")
        self.assertEqual(webui._sensor_topics["scan"], "/pinned/scan")
