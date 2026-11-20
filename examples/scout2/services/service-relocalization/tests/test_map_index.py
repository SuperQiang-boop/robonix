"""Verify grid bounds, immutable snapshot identity and interrupted publication."""
import json
from pathlib import Path
import sqlite3
import subprocess
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from relocalization_rbnx.map_index import build_index, max_pyramid, sha256, verify_index
from relocalization_rbnx.prepare import export_snapshot, prepare, saved_snapshot


def write_export(directory):
    """Create tiny checked geometry, including negative origins and unknown space."""
    directory.mkdir()
    pose = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0]
    points = np.array([[0.1, 0.2, 0.3], [0.11, 0.21, 0.31], [1, 2, 3]], dtype="<f4")
    points.tofile(directory / "cloud.bin")
    points.tofile(directory / "scans.bin")
    np.array([[-1, 0, 100], [0, 100, 0]], dtype="i1").tofile(directory / "grid.bin")
    report = dict(format_version=1, database_version="0.23.7", total_nodes=1,
        optimized_components=[[1]], graph_components=[[1]], excluded_ids=[], point_count=3,
        grid=dict(width=3, height=2, origin_x=-2.0, origin_y=-3.0, resolution=0.05),
        nodes=[dict(id=1, internal_map_id=0, offset=0, count=3,
                    map_from_base=pose, base_from_sensor=pose)])
    (directory / "export.json").write_text(json.dumps(report))
    (directory / "coverage.json").write_text(json.dumps(dict(optimized_components=[[1]])))


class PyramidTests(unittest.TestCase):
    def test_bounds_equal_brute_force(self):
        """Check maxima at every offset, including unknown and edge-padded cells."""
        grid = np.array([[0, -1, 100, 0, 0], [100, 0, 0, 0, 100], [0, 0, 0, 0, 0]])
        for level, actual in enumerate(max_pyramid(grid, 16)):
            expected = np.zeros(grid.shape, dtype=np.uint8)
            for row in range(grid.shape[0]):
                for col in range(grid.shape[1]):
                    expected[row, col] = (grid[row:row+2**level, col:col+2**level] == 100).any()
            np.testing.assert_array_equal(actual, expected)

    def test_corrupt_export_is_rejected(self):
        """Byte counts and scan identities must agree before any index is built."""
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp) / "export"
            write_export(output)
            (output / "cloud.bin").write_bytes(b"broken")
            with self.assertRaisesRegex(ValueError, "byte count"):
                build_index(output, 0.05, 4)


class PreparationTests(unittest.TestCase):
    def setUp(self):
        """Use a metadata-only DB; the export mock stands in for the separately tested codec."""
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.maps = self.root / "maps"
        map_dir = self.maps / "test_map"
        map_dir.mkdir(parents=True)
        self.database = map_dir / "rtabmap.db"
        connection = sqlite3.connect(self.database)
        connection.execute("create table Admin(version text)")
        connection.execute("insert into Admin values ('0.23.7')")
        connection.commit()
        connection.close()
        (map_dir / "generation").write_text("1")
        self.exporter = self.root / "exporter"
        self.exporter.write_text("#!/bin/sh\nexit 1\n")
        self.exporter.chmod(0o755)
        self.indices = self.root / "indices"

    def run_prepare(self):
        return prepare(self.maps, "test_map", self.indices, self.exporter, levels=4)

    def test_atomic_publication_and_reuse(self):
        """Verify snapshot digest, geometry artifacts and reuse without a second export."""
        before = sha256(self.database)
        with patch("relocalization_rbnx.prepare.export_snapshot",
                   side_effect=lambda _, output, *args: write_export(output)) as export:
            result = self.run_prepare()
            self.assertEqual(result, self.run_prepare())
            export.assert_called_once()
        manifest = json.loads((result / "index.json").read_text())
        self.assertEqual(manifest["identity"]["database_sha256"], before)
        self.assertEqual(sha256(self.database), before)
        self.assertEqual(manifest["summary"]["indexed_points"], 2)
        verify_index(result, manifest["identity"])
        self.assertFalse(list(result.parent.glob(".prepare-*")))
        self.assertEqual(np.load(result / "cloud.npy").shape, (2, 3))

    def test_corrupt_cache_is_not_reused_or_overwritten(self):
        """An intact identity does not excuse corrupted cached payload bytes."""
        with patch("relocalization_rbnx.prepare.export_snapshot",
                   side_effect=lambda _, output, *args: write_export(output)) as export:
            result = self.run_prepare()
            (result / "cloud.bin").write_bytes(b"bad")
            with self.assertRaisesRegex(ValueError, "corrupted"):
                self.run_prepare()
            export.assert_called_once()
        self.assertEqual((result / "cloud.bin").read_bytes(), b"bad")

    def test_failed_export_does_not_publish(self):
        """Keep graph diagnostics while removing all incomplete map artifacts."""
        def fail(_database, output, *args):
            """Simulate a decoded but disconnected graph."""
            write_export(output)
            raise subprocess.CalledProcessError(1, "export")
        with patch("relocalization_rbnx.prepare.export_snapshot", side_effect=fail):
            with self.assertRaises(subprocess.CalledProcessError):
                self.run_prepare()
        parent = self.indices / "test_map"
        self.assertEqual(len(list((parent / "reports").glob("*.json"))), 1)
        self.assertFalse(list(parent.glob("*/index.json")))
        self.assertFalse(list(parent.glob(".prepare-*")))

    def test_map_changed_during_export_does_not_publish(self):
        """Changing coordinate generation invalidates an otherwise valid export."""
        def change(_database, output, *args):
            """Simulate a concurrent map reload changing persisted identity."""
            write_export(output)
            (self.database.parent / "generation").write_text("2")
        with patch("relocalization_rbnx.prepare.export_snapshot", side_effect=change):
            with self.assertRaisesRegex(ValueError, "map changed"):
                self.run_prepare()
        self.assertFalse(list(self.indices.glob("**/index.json")))

    def test_docker_timeout_removes_owned_container(self):
        """A timed-out Docker client must not leave the exporter running."""
        from relocalization_rbnx.prepare import PKG
        calls = []
        def run(command, **kwargs):
            """Simulate client timeout followed by successful owned-container removal."""
            calls.append(command)
            if command[1] == "run":
                raise subprocess.TimeoutExpired(command, kwargs["timeout"])
        with patch("relocalization_rbnx.prepare.subprocess.run", side_effect=run):
            with self.assertRaises(subprocess.TimeoutExpired):
                export_snapshot(self.database, self.root / "export",
                    PKG / "rbnx-build/backend-container/relocalization_export", "sha256:test", 0, 1)
        self.assertEqual(calls[1][:3], ["docker", "rm", "--force"])
        self.assertEqual(calls[1][3], calls[0][calls[0].index("--name") + 1])

    def test_missing_exporter_image_reports_rebuild_without_publication(self):
        """Expose the missing dependency before exporting or publishing an index."""
        error = subprocess.CalledProcessError(1, ['docker', 'image', 'inspect'],
                                             stderr='No such image: sha256:old\n')
        with patch('relocalization_rbnx.prepare.subprocess.check_output', side_effect=error), \
                patch('relocalization_rbnx.prepare.export_snapshot') as export:
            with self.assertRaisesRegex(ValueError, 'No such image.*rbnx build'):
                prepare(self.maps, 'test_map', self.indices, self.exporter, image='sha256:old')
            export.assert_not_called()
        self.assertFalse(self.indices.exists())

    def test_snapshot_mutation_does_not_publish(self):
        """A misbehaving exporter cannot silently change the private snapshot."""
        def mutate(database, output, *args):
            """Simulate an exporter violating the read-only snapshot contract."""
            write_export(output)
            database.chmod(0o644)
            database.write_bytes(b"changed")
        with patch("relocalization_rbnx.prepare.export_snapshot", side_effect=mutate):
            with self.assertRaisesRegex(ValueError, "modified the snapshot"):
                self.run_prepare()
        self.assertFalse(list(self.indices.glob("**/index.json")))

    def test_live_or_escaped_sources_are_rejected(self):
        """Sidecars and links to runtime databases cannot enter offline preparation."""
        sidecar = self.database.with_name("rtabmap.db-wal")
        sidecar.touch()
        with self.assertRaisesRegex(ValueError, "sidecars"):
            saved_snapshot(self.maps, "test_map")
        sidecar.unlink()
        outside = self.root / "outside.db"
        self.database.rename(outside)
        self.database.symlink_to(outside)
        with self.assertRaisesRegex(ValueError, "inside maps_dir"):
            saved_snapshot(self.maps, "test_map")


if __name__ == "__main__":
    unittest.main()
