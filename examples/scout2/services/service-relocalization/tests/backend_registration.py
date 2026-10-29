"""Exercise planar fitting, fixed-pose validation and conservative rejection."""
import math
import threading
import time
import unittest
import tempfile
import json
from pathlib import Path

import numpy as np
import _relocalization_bbs as native


def config(**values):
    """Set thresholds for widely separated synthetic three-dimensional points."""
    options = native.RegistrationOptions()
    options.max_correspondence_m = 0.4
    options.max_rmse_m = 0.08
    options.min_overlap = 0.8
    for name, value in values.items():
        setattr(options, name, value)
    return options


def transform(points, pose, options):
    """Construct the independent Rz(yaw) Ry(pitch) Rx(roll) reference."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    cp, sp = math.cos(options.pitch), math.sin(options.pitch)
    cr, sr = math.cos(options.roll), math.sin(options.roll)
    rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return points @ (rz @ ry @ rx).T + [pose[0], pose[1], options.base_z]


class RegistrationTests(unittest.TestCase):
    def setUp(self):
        """Create an asymmetric cloud and trusted fixed height/tilt."""
        self.scan = np.random.default_rng(718).uniform([-4, -3, -0.2], [4, 3, 1.5], (180, 3))
        self.pose = [-2.3, 1.7, math.pi - 0.02]
        self.options = config(base_z=0.7, roll=0.09, pitch=-0.12)
        self.map = transform(self.scan, self.pose, self.options)
        self.registration = native.PlanarRegistration(self.map)

    def test_loads_registration_cloud_bound_to_checked_snapshot(self):
        """Index loader verifies hashes and gives the native matcher owned map data."""
        from relocalization_rbnx.map_index import ARTIFACTS, build_index, sha256
        from relocalization_rbnx.search import load_planar_registration
        from test_map_index import write_export
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp) / "index"
            write_export(directory)
            self.map.astype("<f4").tofile(directory / "cloud.bin")
            self.scan.astype("<f4").tofile(directory / "scans.bin")
            report = json.loads((directory / "export.json").read_text())
            report["point_count"] = len(self.scan)
            report["nodes"][0]["count"] = len(self.scan)
            (directory / "export.json").write_text(json.dumps(report))
            identity = dict(map_id="small", generation=1, database_sha256="f" * 64,
                            pyramid_levels=3, voxel_size=0.05)
            summary = build_index(directory, 0.05, 3)
            manifest = dict(format_version=1, identity=identity, summary=summary,
                            files={name: sha256(directory / name) for name in ARTIFACTS})
            (directory / "index.json").write_text(json.dumps(manifest))
            registration = load_planar_registration(directory, identity)
            result = registration.evaluate(self.scan, self.pose, self.options,
                                           1, native.CancellationToken())
            self.assertEqual(result.correspondences, len(self.scan))
            with self.assertRaisesRegex(ValueError, "identity"):
                load_planar_registration(directory, dict(identity, generation=2))

    def refine(self, **kwargs):
        """Fit near the known pose while allowing individual failure-path overrides."""
        args = dict(points=self.scan, initial=[-2.22, 1.64, -math.pi + 0.01],
                    options=self.options, timeout_s=2, cancelled=native.CancellationToken())
        args.update(kwargs)
        return self.registration.refine(**args)

    def test_recovers_pose_with_fixed_height_and_tilt(self):
        """Recover a wrapped yaw and translations without fitting height or tilt."""
        result = self.refine()
        np.testing.assert_allclose([result.x, result.y], self.pose[:2], atol=1e-6)
        self.assertLess(abs(math.remainder(result.yaw - self.pose[2], 2 * math.pi)), 1e-6)
        self.assertLess(result.rmse_m, 1e-6)
        self.assertEqual(result.overlap, 1)
        self.assertGreater(result.iterations, 0)

    def test_independent_scan_is_evaluated_without_fitting(self):
        """New noisy points validate at the supplied fixed pose, which never changes."""
        scan = self.scan + np.random.default_rng(812).normal(0, 0.01, self.scan.shape)
        result = self.registration.evaluate(scan, self.pose, self.options, 2, native.CancellationToken())
        np.testing.assert_allclose([result.x, result.y, result.yaw], self.pose, atol=1e-12)
        self.assertEqual(result.iterations, 0)
        self.assertLess(result.rmse_m, 0.03)

    def test_fixed_pose_validation_rejects_residual(self):
        with self.assertRaisesRegex(RuntimeError, 'residual'):
            self.registration.evaluate(self.scan, [-2.15, 1.7, self.pose[2]],
                                       self.options, 2, native.CancellationToken())

    def test_wrong_height_cannot_be_optimized_away(self):
        self.options.base_z += 0.25
        with self.assertRaisesRegex(RuntimeError, 'residual|overlap'):
            self.refine(initial=self.pose)

    def test_similar_xy_with_different_3d_structure_is_rejected(self):
        """An identical projection cannot hide incompatible vertical geometry."""
        scan = self.scan.copy()
        scan[:, 2] += np.where(np.arange(len(scan)) % 2, 1.0, -1.0)
        with self.assertRaisesRegex(RuntimeError, 'residual|overlap'):
            self.refine(points=scan, initial=self.pose)

    def test_collapsed_target_support_is_rejected(self):
        """A spread-out scan cannot validate against repeated copies of one target."""
        registration = native.PlanarRegistration(np.zeros((30, 3)))
        scan = np.random.default_rng(21).uniform(-0.1, 0.1, (30, 3))
        with self.assertRaisesRegex(RuntimeError, 'degenerate'):
            registration.evaluate(scan, [0, 0, 0], config(), 1, native.CancellationToken())

    def test_unmatched_points_stay_in_overlap_denominator(self):
        scan = np.concatenate([self.scan, self.scan + [30, 30, 0]])
        with self.assertRaisesRegex(RuntimeError, 'overlap'):
            self.refine(points=scan)

    def test_degenerate_line_and_duplicates_are_rejected(self):
        """Low residual cannot excuse a line or repeated point as spatial evidence."""
        for scan in (np.zeros((30, 3)), np.column_stack([np.arange(30), np.zeros((30, 2))])):
            registration = native.PlanarRegistration(scan)
            with self.assertRaisesRegex(RuntimeError, 'degenerate'):
                registration.refine(scan, [0, 0, 0], config(), 2, native.CancellationToken())

    def test_escape_and_nonconvergence_are_rejected(self):
        self.options.max_translation_m = 0.01
        with self.assertRaisesRegex(RuntimeError, 'bounds'):
            self.refine()
        self.options.max_translation_m = 0.75
        self.options.max_iterations = 1
        with self.assertRaisesRegex(RuntimeError, 'converge'):
            self.refine()

    def test_invalid_geometry_and_options(self):
        """Reject invalid data and thresholds without crashing the native query."""
        for points in (np.zeros((2, 3)), np.full((30, 3), np.nan), np.full((30, 3), 1e30)):
            with self.assertRaises(ValueError):
                native.PlanarRegistration(points)
        for values in (dict(min_overlap=0), dict(max_rmse_m=float('nan')),
                       dict(max_iterations=0), dict(min_correspondences=2)):
            with self.assertRaises(ValueError):
                self.refine(options=config(**values))

    def test_native_map_owns_its_input(self):
        self.map[:] = 10000
        self.assertLess(self.refine().rmse_m, 1e-6)

    def test_cancel_and_expired_deadline(self):
        """Interrupted calls raise instead of returning partial geometric evidence."""
        token = native.CancellationToken()
        token.cancel()
        with self.assertRaises(native.RegistrationCancelled):
            self.refine(cancelled=token)
        with self.assertRaises(native.RegistrationTimeout):
            self.refine(timeout_s=1e-12)

    def test_running_evaluation_releases_gil_for_cancellation(self):
        """Cancel a costly nearest-neighbour evaluation from a Python thread."""
        points = np.random.default_rng(19).uniform(-4, 4, (100000, 3))
        registration = native.PlanarRegistration(points)
        token, errors, entered = native.CancellationToken(), [], threading.Event()

        def work():
            """Record the terminal outcome for the main thread."""
            entered.set()
            try:
                registration.evaluate(points, [0, 0, 0], config(), 5, token)
            except Exception as exc:
                errors.append(exc)

        worker = threading.Thread(target=work)
        worker.start()
        self.assertTrue(entered.wait(1))
        time.sleep(0.002)
        token.cancel()
        worker.join(3)
        self.assertFalse(worker.is_alive())
        self.assertEqual(len(errors), 1)
        self.assertIsInstance(errors[0], native.RegistrationCancelled)


if __name__ == '__main__':
    unittest.main(verbosity=2)
