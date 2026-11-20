"""Build and verify snapshot-bound map artifacts without a live ROS dependency."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import numpy as np

ARTIFACTS = ("export.json", "coverage.json", "cloud.bin", "scans.bin", "grid.bin",
             "cloud.npy", "voxel_keys.npy", "pyramid.npz")


def sha256(path: Path) -> str:
    """Hash file bytes incrementally, including databases larger than memory."""
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical(value: dict) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def max_pyramid(grid: np.ndarray, levels: int) -> list[np.ndarray]:
    """Store forward square maxima at every cell, retaining the original origin.

    Level L covers [x,x+2**L) by [y,y+2**L). Unknown and out-of-grid cells
    contribute zero. These are translation score bounds, not resized images.
    """
    if grid.ndim != 2 or not np.isin(grid, (-1, 0, 100)).all():
        raise ValueError("grid must be a 2D occupancy array of -1, 0 or 100")
    if type(levels) is not int or not 1 <= levels <= 16:
        raise ValueError("levels must be in [1, 16]")
    result = [(grid == 100).astype(np.uint8)]
    for level in range(1, levels):
        previous = result[-1]
        stride = 2 ** (level - 1)
        height, width = previous.shape
        layer = previous.copy()
        if stride < height:
            layer[:-stride, :] = np.maximum(layer[:-stride, :], previous[stride:, :])
        if stride < width:
            layer[:, :-stride] = np.maximum(layer[:, :-stride], previous[:, stride:])
        if stride < height and stride < width:
            layer[:-stride, :-stride] = np.maximum(layer[:-stride, :-stride], previous[stride:, stride:])
        result.append(layer)
    return result


def read_export(directory: Path) -> tuple[dict, np.ndarray, np.ndarray]:
    """Check report dimensions, scan layout and finite geometry before indexing."""
    report = json.loads((directory / "export.json").read_text())
    if report.get("format_version") != 1 or report.get("database_version") != "0.23.7":
        raise ValueError("unsupported export format or RTAB-Map version")
    grid = report["grid"]
    width, height = grid["width"], grid["height"]
    if any(type(v) is not int or v <= 0 for v in (width, height)) or width * height > 100_000_000:
        raise ValueError("invalid or oversized occupancy grid")
    if not all(math.isfinite(grid[key]) for key in ("origin_x", "origin_y", "resolution")) or grid["resolution"] <= 0:
        raise ValueError("invalid occupancy origin or resolution")
    if (directory / "grid.bin").stat().st_size != width * height:
        raise ValueError("occupancy grid byte count mismatch")
    occupancy = np.fromfile(directory / "grid.bin", dtype="i1").reshape(height, width)
    if not np.isin(occupancy, (-1, 0, 100)).all() or not (occupancy == 100).any():
        raise ValueError("occupancy grid has invalid values or no occupied cells")
    count = report["point_count"]
    if isinstance(count, bool) or not isinstance(count, (int, float)) or not math.isfinite(count) or count < 1 or int(count) != count:
        raise ValueError("invalid point count")
    count = int(count)
    for filename in ("cloud.bin", "scans.bin"):
        if (directory / filename).stat().st_size != count * 12:
            raise ValueError(f"{filename} byte count mismatch")
    points = np.memmap(directory / "cloud.bin", dtype="<f4", mode="r", shape=(count, 3))
    scans = np.memmap(directory / "scans.bin", dtype="<f4", mode="r", shape=(count, 3))
    if not np.isfinite(points).all() or not np.isfinite(scans).all():
        raise ValueError("nonfinite exported geometry")
    offset, ids = 0, set()
    for node in report["nodes"]:
        if (type(node["id"]) is not int or node["id"] in ids or node["id"] <= 0 or
                node["offset"] != offset or type(node["count"]) is not int or node["count"] <= 0):
            raise ValueError("invalid per-node scan offsets or IDs")
        ids.add(node["id"])
        offset += node["count"]
        for key in ("map_from_base", "base_from_sensor"):
            transform = np.asarray(node[key], dtype=float)
            if transform.shape != (12,) or not np.isfinite(transform).all():
                raise ValueError("invalid scan transform")
            rotation = transform.reshape(3, 4)[:, :3]
            if not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-3) or not np.isclose(np.linalg.det(rotation), 1, atol=1e-3):
                raise ValueError("nonrigid scan transform")
    if offset != count:
        raise ValueError("per-node scans do not cover the cloud")
    return report, occupancy, points


def build_index(directory: Path, voxel_size: float, levels: int) -> dict:
    """Create persistent voxel lookup and BBS bounds from one validated export."""
    if not math.isfinite(voxel_size) or not 0.01 <= voxel_size <= 1.0:
        raise ValueError("voxel_size must be in [0.01, 1.0] metres")
    report, grid, points = read_export(directory)
    if len(report["optimized_components"]) != 1:
        raise ValueError("disconnected optimized graph: export is diagnostic only; no index published")
    cells = np.floor(points.astype(np.float64) / voxel_size)
    if np.max(np.abs(cells)) >= 2**31:
        raise ValueError("cloud exceeds voxel coordinate limits")
    keys, first = np.unique(cells.astype(np.int32), axis=0, return_index=True)
    # Lexicographic voxel keys and corresponding representative map points.
    # A runtime PCL spatial index can be created without decoding the database.
    np.save(directory / "cloud.npy", np.asarray(points[first], dtype="<f4"), allow_pickle=False)
    np.save(directory / "voxel_keys.npy", keys.astype("<i4"), allow_pickle=False)
    pyramid = max_pyramid(grid, levels)
    np.savez_compressed(directory / "pyramid.npz", **{f"level_{i}": layer for i, layer in enumerate(pyramid)})
    extrinsics = {str(node["id"]): node["base_from_sensor"] for node in report["nodes"]}
    return dict(indexed_points=len(keys), source_points=int(report["point_count"]),
                exported_nodes=len(report["nodes"]), excluded_ids=report["excluded_ids"],
                grid=report["grid"], sensor_extrinsics_sha256=hashlib.sha256(canonical(extrinsics)).hexdigest())


def verify_index(directory: Path, identity: dict) -> dict:
    """Reject mismatched, incomplete, linked or corrupted cached map artifacts."""
    manifest = json.loads((directory / "index.json").read_text())
    if manifest.get("format_version") != 1 or manifest.get("identity") != identity:
        raise ValueError("index identity or format does not match the requested snapshot")
    if set(manifest.get("files", {})) != set(ARTIFACTS):
        raise ValueError("index artifact list is incomplete")
    for filename in ARTIFACTS:
        path = directory / filename
        if path.is_symlink() or not path.is_file() or sha256(path) != manifest["files"][filename]:
            raise ValueError(f"index artifact missing or corrupted: {filename}")
    return manifest
