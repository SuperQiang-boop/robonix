"""Compare the native BBS search against independent exhaustive enumeration."""
import math
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest

import numpy as np

from relocalization_rbnx.map_index import max_pyramid
import _relocalization_bbs as native


def options(**values):
    """Configure a coarse heading lattice for small exhaustive fixtures."""
    value = native.SearchOptions()
    value.yaw_step = 0.7
    value.min_score = 0.1
    value.separation_m = 0.6
    value.separation_rad = 0.8
    value.max_candidates = 5
    for key, item in values.items():
        setattr(value, key, item)
    return value


def exhaustive(grid, points, config, resolution, origin):
    """Score every discrete pose, sort, then suppress nearby accepted poses.

    This uses world coordinates directly and no pyramid or search implementation.
    Unknown and out-of-map endpoints stay in the score denominator.
    """
    height, width = grid.shape
    headings = math.ceil(2 * math.pi / config.yaw_step)
    scored = []
    for heading in range(headings):
        yaw = -math.pi + heading * (2 * math.pi / headings)
        c, s = math.cos(yaw), math.sin(yaw)
        for row in range(height):
            for col in range(width):
                x, y = origin[0] + (col + 0.5) * resolution, origin[1] + (row + 0.5) * resolution
                hits = 0
                for px, py in points:
                    gx = math.floor((x + c * px - s * py - origin[0]) / resolution)
                    gy = math.floor((y + s * px + c * py - origin[1]) / resolution)
                    hits += int(0 <= gx < width and 0 <= gy < height and grid[gy, gx] == 100)
                score = hits / len(points)
                if score >= config.min_score:
                    scored.append((-score, heading, row, col, x, y, yaw))
    accepted = []
    for negative_score, _, _, _, x, y, yaw in sorted(scored):
        if any(math.hypot(x - ax, y - ay) <= config.separation_m and
               abs(math.remainder(yaw - ayaw, 2 * math.pi)) <= config.separation_rad
               for ax, ay, ayaw, _ in accepted):
            continue
        accepted.append((x, y, yaw, -negative_score))
        if len(accepted) == config.max_candidates:
            break
    return accepted


class SearchTests(unittest.TestCase):
    def compare(self, grid, points, config=None, levels=4, resolution=0.3, origin=(-2.3, -4.1)):
        """Require identical candidate order, geometry and score to exhaustive search."""
        config = config or options()
        search = native.BbsSearch(max_pyramid(grid, levels), resolution, *origin)
        result = search.search(np.asarray(points, dtype=float), config, 5, native.CancellationToken())
        actual = [(v.x, v.y, v.yaw, v.score) for v in result.candidates]
        expected = exhaustive(grid, points, config, resolution, origin)
        self.assertEqual(len(actual), len(expected))
        np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-12)
        return result

    def test_random_maps_match_exhaustive(self):
        """Exercise clipped windows, odd dimensions, multiple headings and pyramid depths."""
        rng = np.random.default_rng(601)
        for trial in range(40):
            height, width = rng.integers(2, 10, size=2)
            grid = rng.choice([-1, 0, 100], (height, width))
            grid[0, 0] = 100
            points = rng.uniform(-2, 2, (13, 2))
            with self.subTest(trial=trial):
                self.compare(grid, points, levels=(1, 2, 4, 8, 16)[trial % 5])

    def test_negative_window_start_still_intersects_map(self):
        """The root starts left/below the grid, but its child is a perfect match."""
        grid = np.array([[100, 0, 0], [0, 100, 0], [0, 0, -1]])
        result = self.compare(grid, [[-1.1, -1.1], [-0.1, -0.1]],
                              options(yaw_step=math.pi / 2, min_score=1), resolution=1)
        self.assertEqual(result.candidates[0].score, 1)

    def test_keeps_lower_scored_separated_candidate(self):
        """A perfect first candidate must not prune a weaker second location."""
        grid = np.zeros((2, 12), dtype=int)
        grid[0, [1, 2, 3, 8, 9]] = 100
        result = self.compare(grid, [[0.03, 0.02], [1.03, 0.02], [2.03, 0.02]],
                              options(yaw_step=math.pi / 2, min_score=0.5,
                                      separation_m=3, separation_rad=math.pi, max_candidates=2),
                              resolution=1)
        self.assertEqual([v.score for v in result.candidates], [1, 2 / 3])

    def test_heading_wrap_and_position_separation(self):
        """Suppression wraps at pi and retains different headings at the same location."""
        grid = np.full((3, 4), 100)
        result = self.compare(grid, [[0.01, 0.02]],
                              options(yaw_step=0.4, separation_rad=0.5, max_candidates=20))
        self.assertEqual(len(result.candidates), 20)

    def test_unknown_and_outside_points_are_not_evidence(self):
        """Unmatched endpoints cannot inflate the occupied fraction."""
        grid = np.array([[100, -1], [-1, 0]])
        result = self.compare(grid, [[0.01, 0.01], [10.01, 10.01]], options(min_score=0.75))
        self.assertEqual(result.candidates, [])

    def test_recovers_known_pose_at_full_heading_range(self):
        """An asymmetric synthetic scan must recover its planted map pose."""
        rng = np.random.default_rng(722)
        points = rng.uniform(-1.7, 1.7, (41, 2))
        grid = np.zeros((41, 47), dtype=int)
        resolution, origin = 0.1, (-2.3, -1.7)
        yaw, col, row = math.pi / 2, 22, 23
        for px, py in points:
            grid[row + math.floor(0.5 + px / resolution),
                 col + math.floor(0.5 - py / resolution)] = 100
        search = native.BbsSearch(max_pyramid(grid, 7), resolution, *origin)
        result = search.search(points, options(yaw_step=math.pi / 2, min_score=1),
                               1, native.CancellationToken())
        self.assertEqual(len(result.candidates), 1)
        candidate = result.candidates[0]
        np.testing.assert_allclose([candidate.x, candidate.y, candidate.yaw, candidate.score],
                                   [origin[0] + (col + 0.5) * resolution,
                                    origin[1] + (row + 0.5) * resolution, yaw, 1], atol=1e-12)

    def test_rejects_invalid_bounds_and_empty_map(self):
        """A corrupted upper bound must be rejected before it can hide candidates."""
        grid = np.array([[0, 100], [-1, 0]])
        layers = max_pyramid(grid, 3)
        layers[1][0, 0] = 0
        with self.assertRaisesRegex(ValueError, 'maximum pyramid'):
            native.BbsSearch(layers, 0.1, 0, 0)
        with self.assertRaisesRegex(ValueError, 'no occupied'):
            native.BbsSearch([np.zeros((2, 2), dtype='u1')], 0.1, 0, 0)
        with self.assertRaises(ValueError):
            native.BbsSearch(max_pyramid(grid, 2), float('nan'), 0, 0)

    def test_rejects_invalid_scans_and_options(self):
        """Reject nonfinite/oversized scans and invalid limits without native crashes."""
        search = native.BbsSearch([np.ones((2, 2), dtype='u1')], 1, 0, 0)
        for points in ([], [[float('nan'), 0]], [[1e100, 0]], [[0, 0, 0]]):
            with self.subTest(points=points), self.assertRaises(ValueError):
                search.search(np.array(points), options(), 1, native.CancellationToken())
        for config in (options(min_score=0), options(yaw_step=0), options(max_candidates=0),
                       options(separation_rad=4), options(separation_m=float('inf'))):
            with self.assertRaises(ValueError):
                search.search(np.zeros((1, 2)), config, 1, native.CancellationToken())

    def test_pre_cancel_and_deadline_discard_results(self):
        """An interrupted operation raises instead of returning usable partial poses."""
        search = native.BbsSearch([np.ones((2, 2), dtype='u1')], 1, 0, 0)
        token = native.CancellationToken()
        token.cancel()
        with self.assertRaises(native.SearchCancelled):
            search.search(np.zeros((1, 2)), options(), 1, token)
        with self.assertRaises(native.SearchTimeout):
            search.search(np.zeros((1, 2)), options(), 1e-12, native.CancellationToken())

    def test_running_search_obeys_deadline(self):
        """Stop costly rotation/search work at a monotonic deadline."""
        grid = np.zeros((128, 129), dtype=int)
        grid[0, 0] = 100
        search = native.BbsSearch(max_pyramid(grid, 8), 0.05, -3, -3)
        points = np.random.default_rng(31).uniform(-4, 4, (1500, 2))
        started = time.monotonic()
        with self.assertRaises(native.SearchTimeout):
            search.search(points, options(yaw_step=0.002, min_score=1),
                          0.01, native.CancellationToken())
        self.assertLess(time.monotonic() - started, 1)

    def test_native_search_owns_map_layers(self):
        """Changing Python input layers cannot invalidate native bounds."""
        layers = max_pyramid(np.array([[100, 0], [0, 0]]), 3)
        search = native.BbsSearch(layers, 1, -2, -3)
        for layer in layers:
            layer[:] = 0
        result = search.search(np.zeros((1, 2)),
                               options(yaw_step=2 * math.pi, min_score=1),
                               1, native.CancellationToken())
        self.assertEqual(len(result.candidates), 1)
        self.assertEqual((result.candidates[0].x, result.candidates[0].y), (-1.5, -2.5))

    def test_concurrent_cancel_runs_while_native_search_releases_gil(self):
        """Python must be able to cancel a running native search from another thread."""
        grid = np.zeros((128, 129), dtype=int)
        grid[0, 0] = 100
        search = native.BbsSearch(max_pyramid(grid, 8), 0.05, -3, -3)
        points = np.random.default_rng(94).uniform(-4, 4, (1500, 2))
        config = options(yaw_step=0.002, min_score=1)
        token, entered, errors = native.CancellationToken(), threading.Event(), []

        def work():
            """Record the native terminal exception so the main thread can assert it."""
            entered.set()
            try:
                search.search(points, config, 2, token)
            except Exception as exc:
                errors.append(exc)

        worker = threading.Thread(target=work)
        started = time.monotonic()
        worker.start()
        self.assertTrue(entered.wait(1))
        time.sleep(0.02)
        config.max_candidates = 0  # The running operation owns its options.
        token.cancel()
        worker.join(3)
        self.assertFalse(worker.is_alive())
        self.assertEqual(len(errors), 1)
        self.assertIsInstance(errors[0], native.SearchCancelled)
        self.assertLess(time.monotonic() - started, 1.5)


class IndexLoadTests(unittest.TestCase):
    def setUp(self):
        """Publish a tiny real index without a live robot or RTAB-Map codec."""
        from relocalization_rbnx.map_index import ARTIFACTS, build_index, sha256
        from test_map_index import write_export
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name) / 'index'
        write_export(self.directory)
        self.identity = dict(map_id='synthetic', generation=2, database_sha256='a' * 64,
                             pyramid_levels=4)
        self.manifest = dict(format_version=1, identity=self.identity,
                             summary=build_index(self.directory, 0.05, 4),
                             files={name: sha256(self.directory / name) for name in ARTIFACTS})
        self.publish_manifest()

    def publish_manifest(self):
        (self.directory / 'index.json').write_text(json.dumps(self.manifest))

    def test_load_and_search_prepared_index(self):
        """Use saved pyramid origin and resolution to recover both occupied cells."""
        from relocalization_rbnx.search import load_search_index
        search = load_search_index(self.directory, self.identity)
        result = search.search(np.zeros((1, 2)),
                               options(yaw_step=2 * math.pi, min_score=1, separation_m=0),
                               1, native.CancellationToken())
        np.testing.assert_allclose([(c.x, c.y) for c in result.candidates],
                                   [(-1.875, -2.975), (-1.925, -2.925)])

    def test_identity_and_artifact_corruption_are_rejected(self):
        """A different map generation or corrupted pyramid cannot be searched."""
        from relocalization_rbnx.search import load_search_index
        with self.assertRaisesRegex(ValueError, 'identity'):
            load_search_index(self.directory, dict(self.identity, generation=3))
        (self.directory / 'pyramid.npz').write_bytes(b'bad')
        with self.assertRaisesRegex(ValueError, 'corrupted'):
            load_search_index(self.directory, self.identity)

    def test_valid_digest_does_not_excuse_wrong_grid_bounds(self):
        """Even self-consistent file hashes cannot excuse an incorrect score grid."""
        from relocalization_rbnx.map_index import sha256
        from relocalization_rbnx.search import load_search_index
        layers = max_pyramid(np.array([[100, 0, 0], [0, 100, 0]]), 4)
        np.savez(self.directory / 'pyramid.npz', **{f'level_{i}': v for i, v in enumerate(layers)})
        self.manifest['files']['pyramid.npz'] = sha256(self.directory / 'pyramid.npz')
        self.publish_manifest()
        with self.assertRaisesRegex(ValueError, 'occupancy differs'):
            load_search_index(self.directory, self.identity)


if __name__ == '__main__':
    unittest.main(verbosity=2)
