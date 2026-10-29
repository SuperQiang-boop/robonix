"""Offline candidate matching only; no robot actions or localization validity."""
from __future__ import annotations

import copy
import math
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import _relocalization_bbs as native

from .map_index import read_export, sha256, verify_index
from .search import load_planar_registration, load_search_index
from .observations import Observation


@dataclass(frozen=True)
class MatchingOptions:
    """Internal offline thresholds; live calibration and acceptance are outstanding."""

    min_height_m: float = 0.15
    max_height_m: float = 1.5
    min_known_fraction: float = 0.7
    min_occupied_fraction: float = 0.3
    max_blocked_ray_fraction: float = 0.1
    ray_margin_m: float = 0.15
    same_position_m: float = 0.2
    same_heading_rad: float = 0.1
    max_frame_gap_s: float = 2.0


@dataclass(frozen=True)
class MatchResult:
    """A unique offline geometric candidate, pending live checks and pose handoff."""

    x: float
    y: float
    yaw: float
    rmse_m: float
    overlap: float
    observation_stamp_ns: int
    rejected: tuple[str, ...]


class MatchRejected(RuntimeError):
    """Expected absence of a unique geometric match, without a usable pose."""


def _remaining(deadline: float, token) -> float:
    """Use the shared token and whole-operation monotonic deadline at every stage."""
    if token.cancelled:
        raise native.SearchCancelled("offline matching cancelled")
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise native.SearchTimeout("offline matching deadline exceeded")
    return min(remaining, 120.0)


def _validate_options(options: MatchingOptions) -> None:
    """Reject nonfinite thresholds and limits that could disable evidence checks."""
    for name, value in vars(options).items():
        if type(value) not in (float, int) or not math.isfinite(value):
            raise ValueError(f"invalid matching option: {name}")
    if not -10 <= options.min_height_m < options.max_height_m <= 10:
        raise ValueError("invalid matching height interval")
    for value in (options.min_known_fraction, options.min_occupied_fraction):
        if not 0 < value <= 1:
            raise ValueError("matching evidence fractions must be in (0, 1]")
    for value in (options.max_blocked_ray_fraction,):
        if not 0 <= value < 1:
            raise ValueError("matching conflict fractions must be in [0, 1)")
    if (not 0 < options.ray_margin_m <= 1 or not 0 < options.same_position_m <= 0.5 or
            not 0 < options.same_heading_rad <= 0.35 or not 0 < options.max_frame_gap_s <= 5):
        raise ValueError("invalid matching distance, heading or time bounds")


def _owned_observation(value: Observation) -> Observation:
    """Copy scan buffers; validate size, time bounds and finite base-frame geometry."""
    points = np.array(value.points, dtype=float, order="C", copy=True)
    origin = np.asarray(value.sensor_origin, dtype=float)
    if (points.ndim != 2 or points.shape[1] != 3 or not 3 <= len(points) <= 100_000 or
            not np.isfinite(points).all() or np.max(np.abs(points)) > 1000 or
            origin.shape != (3,) or not np.isfinite(origin).all() or np.max(np.abs(origin)) > 10):
        raise ValueError("invalid matching scan or sensor origin")
    if (type(value.start_stamp_ns) is not int or type(value.end_stamp_ns) is not int or
            not 0 < value.start_stamp_ns <= value.end_stamp_ns < 2**63 or
            value.end_stamp_ns - value.start_stamp_ns > 2_000_000_000):
        raise ValueError("invalid observation capture window")
    points.flags.writeable = False
    return Observation(points, tuple(origin), value.start_stamp_ns, value.end_stamp_ns)


def _tilt(options) -> np.ndarray:
    """Match the native Ry(pitch) Rx(roll) convention before height selection."""
    cp, sp = math.cos(options.pitch), math.sin(options.pitch)
    cr, sr = math.cos(options.roll), math.sin(options.roll)
    return np.array([[cp, sp * sr, sp * cr], [0, cr, -sr], [-sp, cp * sr, cp * cr]])


class OfflineMatcher:
    """Load one immutable prepared snapshot and match two stationary observations.

    Map identity must be bound to the actual mapping snapshot by the future
    task layer. Timestamps alone cannot prove independent acquisition, static
    robot pose, TF correctness or freshness; those are caller obligations.
    """

    def __init__(self, directory: Path, identity: dict):
        """Verify files around loading so grid, BBS and cloud share one snapshot."""
        directory = Path(directory)
        identity = copy.deepcopy(identity)
        before = sha256(directory / "index.json")
        verify_index(directory, identity)
        self._search = load_search_index(directory, identity)
        self._registration = load_planar_registration(directory, identity)
        report, grid, _ = read_export(directory)
        verify_index(directory, identity)
        if sha256(directory / "index.json") != before:
            raise ValueError("map index changed during matcher loading")
        self._grid = grid.copy()
        self._grid.flags.writeable = False
        self._geometry = dict(report["grid"])
        self._identity = identity

    @property
    def identity(self) -> dict:
        """Return an owned identity of the snapshot validated during loading."""
        return copy.deepcopy(self._identity)

    def _cells(self, xy: np.ndarray) -> np.ndarray:
        """Return occupied/free/unknown values; outside endpoints remain unknown."""
        g = self._geometry
        cells = np.floor((xy - [g["origin_x"], g["origin_y"]]) / g["resolution"])
        inside = ((cells[:, 0] >= 0) & (cells[:, 0] < g["width"]) &
                  (cells[:, 1] >= 0) & (cells[:, 1] < g["height"]))
        indices = cells[inside].astype(np.int64)
        values = np.full(len(xy), -1, dtype=np.int8)
        values[inside] = self._grid[indices[:, 1], indices[:, 0]]
        return values

    def evaluate(self, observation: Observation, pose, registration_options,
                 options: MatchingOptions, timeout_s: float, cancelled) -> MatchResult:
        """Score a fresh observation at mapping's fixed pose without fitting it.

        Reuses the snapshot cloud, height selection and occupancy/ray checks
        used during global matching. Caller verifies acquisition freshness,
        stationary motion and actual mapping identity before and after use.
        """
        if type(timeout_s) not in (int, float) or not math.isfinite(timeout_s) or not 0 < timeout_s <= 120:
            raise ValueError('evaluation timeout_s must be in (0, 120]')
        deadline = time.monotonic()+timeout_s
        _remaining(deadline, cancelled)
        _validate_options(options)
        pose = np.asarray(pose, dtype=float).copy()
        if pose.shape != (3,) or not np.isfinite(pose).all():
            raise ValueError('mapping pose must contain finite x/y/yaw')
        observation = _owned_observation(observation)
        registration_options = copy.copy(registration_options)
        options = copy.deepcopy(options)
        tilt = _tilt(registration_options)
        projected = observation.points @ tilt.T
        heights = projected[:, 2]+registration_options.base_z
        mask = (heights >= options.min_height_m) & (heights <= options.max_height_m)
        if not registration_options.min_correspondences <= np.count_nonzero(mask) <= 10000:
            raise ValueError('insufficient or excessive points in the matching height interval')
        measured = self._registration.evaluate(observation.points[mask], pose,
            registration_options, _remaining(deadline, cancelled), cancelled)
        self._check_grid(projected[mask], np.asarray(observation.sensor_origin) @ tilt.T,
                         pose, options, deadline, cancelled)
        _remaining(deadline, cancelled)
        return MatchResult(float(pose[0]), float(pose[1]), float(pose[2]),
            measured.rmse_m, measured.overlap, observation.end_stamp_ns, ())

    def _check_grid(self, points: np.ndarray, origin: np.ndarray, pose, options, deadline, token) -> None:
        """Check base cell and scan endpoints against the saved 2D occupancy grid.

        Unknown/outside endpoints never increase known or occupied evidence.
        A VLP-16 PointCloud2 contains rays at multiple elevations, while this
        grid collapses all heights into one plane; projected 2D ray crossings
        cannot establish a 3D visibility contradiction. Free endpoint counts
        are diagnostic for the same reason. Same-height point-cloud registration
        checks observed surfaces; known coverage and occupied support remain required.
        """
        evidence = self._grid_evidence(points, origin, pose, options, deadline, token,
                                       include_projected_rays=False)
        if evidence['base_cell'] != 0:
            raise MatchRejected("candidate base cell is occupied, unknown or outside map")
        if (evidence['known_fraction'] < options.min_known_fraction or
                evidence['occupied_fraction'] < options.min_occupied_fraction):
            raise MatchRejected("candidate endpoints contradict saved occupancy or lack known coverage; "
                f"known={evidence['known_fraction']:.3f} occupied={evidence['occupied_fraction']:.3f} "
                f"free={evidence['free_fraction']:.3f}")

    def _grid_evidence(self, points, origin, pose, options, deadline, token,
                       include_projected_rays=True):
        """Measure fixed-pose endpoint evidence and optional 2D ray diagnostics.

        Ray and free endpoint fractions are diagnostic only because the saved
        2D grid loses the vertical separation of the VLP-16 beams. Fixed-pose
        point-cloud registration checks 3D surface agreement; known coverage
        and occupied endpoint support remain acceptance checks.
        """
        _remaining(deadline, token)
        if len(points) < 3:
            raise ValueError('grid evidence needs at least three scan points')
        c, s = math.cos(pose[2]), math.sin(pose[2])
        rotation = np.array([[c, -s], [s, c]])
        xy = points[:, :2] @ rotation.T + pose[:2]
        sensor = origin[:2] @ rotation.T + pose[:2]
        base = int(self._cells(np.asarray([pose[:2]]))[0])
        values = self._cells(xy)
        evidence = dict(base_cell=base, known_fraction=float(np.mean(values != -1)),
                        occupied_fraction=float(np.mean(values == 100)),
                        free_fraction=float(np.mean(values == 0)))
        if include_projected_rays:
            blocked = samples = 0
            step = self._geometry["resolution"] * 0.5
            for i, endpoint in enumerate(xy):
                if i % 32 == 0:
                    _remaining(deadline, token)
                vector = endpoint - sensor
                distance = float(np.linalg.norm(vector))
                count = max(0, math.ceil((distance - 2 * options.ray_margin_m) / step))
                samples += count
                if samples > 2_000_000:
                    raise ValueError("grid ray diagnostic exceeds two million samples; downsample scan")
                if count:
                    offsets = options.ray_margin_m + np.arange(count) * step
                    cells = self._cells(sensor + offsets[:, None] * (vector / distance))
                    blocked += bool(np.any(cells == 100))
            evidence.update(blocked_ray_fraction=blocked / len(points),
                            rays=len(points), ray_samples=samples)
        _remaining(deadline, token)
        return evidence

    def match(self, fitting: Observation, validation: Observation, search_options, registration_options,
              options: MatchingOptions, timeout_s: float, cancelled) -> MatchResult:
        """Find one geometric candidate under a single whole-operation deadline.

        Both scans must describe a stationary robot in the same base frame.
        Validation window follows fitting without overlap and never participates
        in fitting. All accepted candidates are checked; multiple separated
        solutions or a saturated search budget reject instead of selecting one.
        Unexpected errors and interruption propagate, never masquerade as rejection.
        """
        if type(timeout_s) not in (float, int) or not math.isfinite(timeout_s) or not 0 < timeout_s <= 120:
            raise ValueError("matching timeout_s must be in (0, 120]")
        deadline = time.monotonic() + timeout_s
        _remaining(deadline, cancelled)
        _validate_options(options)
        search_options, registration_options = copy.copy(search_options), copy.copy(registration_options)
        if not 2 <= search_options.max_candidates <= 100:
            raise ValueError("matching requires at least two BBS candidates to check ambiguity")
        # Do not suppress a coarse location more widely than the final check
        # considers equivalent. The conservative candidate cap may reject more.
        search_options.separation_m = min(search_options.separation_m, options.same_position_m)
        search_options.separation_rad = min(search_options.separation_rad, options.same_heading_rad)
        fitting, validation = _owned_observation(fitting), _owned_observation(validation)
        if (validation.start_stamp_ns <= fitting.end_stamp_ns or
                validation.end_stamp_ns - fitting.end_stamp_ns > options.max_frame_gap_s * 1e9):
            raise ValueError("validation must follow fitting in a separate nearby capture window")
        tilt = _tilt(registration_options)
        selected = []
        for observation in (fitting, validation):
            points = observation.points @ tilt.T
            heights = points[:, 2] + registration_options.base_z
            mask = (heights >= options.min_height_m) & (heights <= options.max_height_m)
            if np.count_nonzero(mask) < max(3, registration_options.min_correspondences):
                raise ValueError("insufficient scan points in the matching height interval")
            if np.count_nonzero(mask) > 10000:
                raise ValueError("matching height slice exceeds 10000 points; downsample scan")
            selected.append((observation.points[mask], points[mask], np.asarray(observation.sensor_origin) @ tilt.T))
        fit, check = selected
        coarse = self._search.search(fit[1][:, :2], search_options, _remaining(deadline, cancelled), cancelled)
        if len(coarse.candidates) >= search_options.max_candidates:
            raise MatchRejected("BBS candidate limit reached; alternative locations remain unchecked")
        accepted, rejected = [], []
        for number, candidate in enumerate(coarse.candidates):
            _remaining(deadline, cancelled)
            try:
                refined = self._registration.refine(fit[0], [candidate.x, candidate.y, candidate.yaw],
                    registration_options, _remaining(deadline, cancelled), cancelled)
                pose = [refined.x, refined.y, refined.yaw]
                verified = self._registration.evaluate(check[0], pose, registration_options,
                    _remaining(deadline, cancelled), cancelled)
                self._check_grid(fit[1], fit[2], pose, options, deadline, cancelled)
                self._check_grid(check[1], check[2], pose, options, deadline, cancelled)
            except (native.RegistrationRejected, MatchRejected) as exc:
                rejected.append(f"candidate {number}: {exc}")
                continue
            accepted.append((refined, verified))
        _remaining(deadline, cancelled)
        if not accepted:
            raise MatchRejected("no candidate passed fitting and independent validation; " + "; ".join(rejected))
        # Require every result to be close to every other result; do not merge
        # chains of neighbouring poses into one apparently unambiguous location.
        for i, (first, _) in enumerate(accepted):
            for second, _ in accepted[i + 1:]:
                if (math.hypot(first.x - second.x, first.y - second.y) > options.same_position_m or
                        abs(math.remainder(first.yaw - second.yaw, 2 * math.pi)) > options.same_heading_rad):
                    raise MatchRejected("multiple separated candidates passed independent validation; ambiguous map location")
        refined, verified = min(accepted, key=lambda pair: (max(pair[0].rmse_m, pair[1].rmse_m),
                                                          -min(pair[0].overlap, pair[1].overlap)))
        _remaining(deadline, cancelled)
        return MatchResult(refined.x, refined.y, refined.yaw, max(refined.rmse_m, verified.rmse_m),
                           min(refined.overlap, verified.overlap), validation.end_stamp_ns, tuple(rejected))
