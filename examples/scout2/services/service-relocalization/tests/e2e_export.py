"""Validate exporter geometry and fail-closed behavior against real RTAB-Map DBs."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

FIXTURE, EXPORTER = sys.argv[1:3]
del sys.argv[1:3]


class ExportTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def export(self, mode, anchor=0):
        """Generate an API-written snapshot and assert exporter never mutates it."""
        database = self.root / "snapshot.db"
        subprocess.run([FIXTURE, str(database), mode], check=True, capture_output=True)
        before = hashlib.sha256(database.read_bytes()).hexdigest()
        output = self.root / "export"
        result = subprocess.run([EXPORTER, str(database), str(output), str(anchor)],
                                capture_output=True, text=True)
        self.assertEqual(hashlib.sha256(database.read_bytes()).hexdigest(), before)
        return result, output

    def test_coordinates_and_unoptimized_exclusion(self):
        """Use optimized poses plus scan extrinsics; raw odometry is deliberately wrong."""
        result, output = self.export("connected")
        self.assertEqual(result.returncode, 0, result.stderr)
        report = json.loads((output / "export.json").read_text())
        self.assertEqual(report["excluded_ids"], [3])
        self.assertEqual([node["id"] for node in report["nodes"]], [1, 2])
        scans = np.fromfile(output / "scans.bin", dtype="<f4").reshape(-1, 3)
        cloud = np.fromfile(output / "cloud.bin", dtype="<f4").reshape(-1, 3)
        np.testing.assert_allclose(scans[0], [1.5, 2.25, 3.75])
        np.testing.assert_allclose(cloud[0], [11.5, 22.25, 3.75])
        np.testing.assert_allclose(cloud[3], [12.5, 22.25, 3.75])
        self.assertEqual(report["grid"]["origin_x"], -2.5)
        self.assertEqual(report["grid"]["origin_y"], -1.25)
        grid = np.fromfile(output / "grid.bin", dtype="i1").reshape(8, 9)
        self.assertEqual(grid[0, 0], -1)
        self.assertEqual(grid[1, 2], 100)

    def test_disconnected_is_rejected(self):
        """Preserve coverage diagnostics but never silently select a component."""
        result, output = self.export("disconnected")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("select a verified anchor", result.stderr)
        coverage = json.loads((output / "coverage.json").read_text())
        self.assertEqual(coverage["optimized_components"], [[1, 2], [3]])
        self.assertFalse((output / "cloud.bin").exists())

    def test_anchor_export_remains_diagnostic(self):
        """Allow scan inspection of one component but refuse a ready map index."""
        from relocalization_rbnx.map_index import build_index
        result, output = self.export("disconnected", anchor=1)
        self.assertEqual(result.returncode, 0, result.stderr)
        with self.assertRaisesRegex(ValueError, "diagnostic only"):
            build_index(output, 0.05, 4)

    def test_missing_scan_is_rejected(self):
        result, _ = self.export("missing_scan")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("scan", result.stderr)

    def test_missing_optimized_poses_is_rejected(self):
        result, _ = self.export("missing_poses")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("no saved optimized poses", result.stderr)

    def test_missing_grid_is_rejected(self):
        result, _ = self.export("missing_grid")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("occupancy grid", result.stderr)

    def test_complete_preparation_and_cached_load(self):
        """Exercise private DB copy, real exporter, voxel/pyramid build and reuse."""
        from relocalization_rbnx.prepare import prepare
        maps = self.root / "maps"
        saved = maps / "test_map"
        saved.mkdir(parents=True)
        (saved / "generation").write_text("1")
        subprocess.run([FIXTURE, str(saved / "rtabmap.db"), "connected"],
                       check=True, capture_output=True)
        index = prepare(maps, "test_map", self.root / "indices", Path(EXPORTER), levels=4)
        self.assertTrue((index / "index.json").is_file())
        self.assertEqual(prepare(maps, "test_map", self.root / "indices", Path(EXPORTER), levels=4), index)


if __name__ == "__main__":
    unittest.main(verbosity=2)
