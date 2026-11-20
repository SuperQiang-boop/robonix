"""Load snapshot-bound BBS data; coarse candidates alone never validate a pose."""
from __future__ import annotations

from pathlib import Path

import numpy as np

from .map_index import read_export, verify_index


def load_search_index(directory: Path, identity: dict):
    """Check exact prepared identity, artifacts and bounds before native loading.

    This is an internal offline entry point, not an Atlas capability. Callers
    must bind identity to the actual loaded mapping snapshot before using it
    for a robot task. The returned search owns its layers and releases the GIL
    while computing; its candidates still require registration and handoff.
    """
    from _relocalization_bbs import BbsSearch

    manifest = verify_index(directory, identity)
    report, grid, _points = read_export(directory)
    if len(report["optimized_components"]) != 1:
        raise ValueError("disconnected optimized graph: export is diagnostic only")
    geometry = report["grid"]
    if manifest["summary"]["grid"] != geometry:
        raise ValueError("index grid geometry differs from its snapshot export")
    count = identity["pyramid_levels"]
    if type(count) is not int or not 1 <= count <= 16:
        raise ValueError("invalid index pyramid level count")
    with np.load(directory / "pyramid.npz", allow_pickle=False) as archive:
        if set(archive.files) != {f"level_{i}" for i in range(count)}:
            raise ValueError("index pyramid layer list is incomplete")
        layers = [archive[f"level_{i}"] for i in range(count)]
    if any(layer.dtype != np.dtype('u1') or layer.shape != grid.shape for layer in layers):
        raise ValueError("index pyramid shape or dtype differs from its grid")
    if not np.array_equal(layers[0], grid == 100):
        raise ValueError("index pyramid occupancy differs from its snapshot grid")
    return BbsSearch(layers, geometry["resolution"], geometry["origin_x"], geometry["origin_y"])


def load_planar_registration(directory: Path, identity: dict):
    """Load the checked map cloud from the same identity-bound index.

    The native registration object copies its input. Callers must supply
    calibrated base height/tilt and independent live scans at execution time.
    """
    from _relocalization_bbs import PlanarRegistration

    manifest = verify_index(directory, identity)
    report, _grid, source = read_export(directory)
    if len(report["optimized_components"]) != 1:
        raise ValueError("disconnected optimized graph: export is diagnostic only")
    points = np.load(directory / "cloud.npy", allow_pickle=False)
    if (points.dtype != np.dtype("<f4") or points.ndim != 2 or points.shape[1] != 3 or
            not 3 <= len(points) <= 2_000_000 or not np.isfinite(points).all()):
        raise ValueError("indexed registration cloud has invalid shape, dtype or values")
    voxel_size = identity["voxel_size"]
    if not np.isfinite(voxel_size) or not 0.01 <= voxel_size <= 1:
        raise ValueError("invalid registration voxel size")
    keys, first = np.unique(np.floor(source.astype(np.float64) / voxel_size),
                            axis=0, return_index=True)
    if (manifest["summary"]["indexed_points"] != len(keys) or
            not np.array_equal(points, source[first])):
        raise ValueError("indexed registration cloud differs from snapshot export")
    return PlanarRegistration(points)
