"""Check saved scans at known poses; this is not independent accuracy evidence."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

import numpy as np

from .backend import native_options, read_profile
from .map_index import sha256
from .matching import MatchingOptions, OfflineMatcher, _tilt, _validate_options
from .observations import CaptureOptions, Observation
from .prepare import saved_snapshot


def check(profile_file, maps_dir, output):
    """Compare beginning/middle/end scans with the production fixed-pose gates.

    Reproduce range/voxel filtering and report capture size separately from
    matching rejection. Keep existing thresholds and the source map unchanged.
    Timing is unnecessary for this offline geometric diagnostic, not inferred.
    """
    import _relocalization_bbs as native
    profile_file = Path(profile_file).resolve(strict=True)
    if Path(output).exists():
        raise ValueError('map-check output must be a new file')
    profile = read_profile(profile_file)
    index = Path(profile['index'])
    index = (profile_file.parent/index).resolve() if not index.is_absolute() else index
    identity = json.loads((index/'index.json').read_text())['identity']
    database, generation = saved_snapshot(Path(maps_dir), identity['map_id'])
    if generation != identity['generation'] or sha256(database) != identity['database_sha256']:
        raise ValueError('prepared index does not match saved map checkpoint')
    matcher = OfflineMatcher(index, identity)
    report = json.loads((index/'export.json').read_text())
    # CaptureOptions supplies geometric defaults only. Driver timing is not
    # consumed by this command and remains untouched in the real profile.
    geometry = {k: v for k, v in profile['capture'].items()
                if k not in ('scan_duration_ns', 'header_is_end')}
    capture = CaptureOptions(scan_duration_ns=1, header_is_end=False, **geometry)
    capture.validate()
    registration = native_options(native.RegistrationOptions, profile['registration'])
    options = MatchingOptions(**profile.get('matching', {}))
    _validate_options(options)
    scans = np.memmap(index/'scans.bin', dtype='<f4', mode='r').reshape(-1, 3)
    results = []
    for number in sorted({0, len(report['nodes'])//2, len(report['nodes'])-1}):
        node = report['nodes'][number]
        pose = np.asarray(node['map_from_base']).reshape(3, 4)
        truth = [float(pose[0, 3]), float(pose[1, 3]), math.atan2(pose[1, 0], pose[0, 0])]
        origin = np.asarray(node['base_from_sensor']).reshape(3, 4)[:, 3]
        points = np.array(scans[int(node['offset']):int(node['offset'])+node['count']], dtype=float)
        ranges = np.linalg.norm(points-origin, axis=1)
        points = points[(ranges >= capture.min_range_m) & (ranges <= capture.max_range_m)]
        _, first = np.unique(np.floor(points/capture.voxel_size_m), axis=0, return_index=True)
        points = np.ascontiguousarray(points[np.sort(first)])
        tilted = points @ _tilt(registration).T
        heights = tilted[:, 2]+registration.base_z
        mask = (heights >= options.min_height_m) & (heights <= options.max_height_m)
        item = dict(node_id=node['id'], pose=truth, filtered_points=len(points),
                    capture_size_passed=20 <= len(points) <= 10000,
                    matching_points=int(mask.sum()), fixed_pose_geometry_passed=False)
        start = time.monotonic()
        token = native.CancellationToken()
        try:
            item['grid_evidence'] = matcher._grid_evidence(tilted[mask], origin @ _tilt(registration).T,
                                                          truth, options, start+3, token)
            result = matcher.evaluate(Observation(points, tuple(origin), 1_000_000_000, 1_100_000_000),
                                      truth, registration, options, 3, token)
            item.update(fixed_pose_geometry_passed=True, rmse_m=result.rmse_m, overlap=result.overlap)
        except (ValueError, native.RegistrationRejected, native.RegistrationTimeout,
                RuntimeError) as exc:
            item['error'] = type(exc).__name__+': '+str(exc)
        item['seconds'] = time.monotonic()-start
        results.append(item)
    output_report = dict(identity=identity,
        note='Same-map scan geometry diagnostic only; not independent relocalization accuracy evidence.',
        passed=all(r['capture_size_passed'] and r['fixed_pose_geometry_passed'] for r in results),
        matching_limits=dict(vars(options)), capture_voxel_size_m=capture.voxel_size_m,
        results=results)
    with Path(output).open('x') as stream:
        stream.write(json.dumps(output_report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    return output_report


def main():
    """Preserve rejection diagnostics and return nonzero if any selected scan fails."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--maps-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    try:
        report = check(args.profile, args.maps_dir, args.output)
        print(json.dumps(dict(report=str(args.output), passed=report['passed'],
            nodes=[dict(node_id=r['node_id'], capture_size_passed=r['capture_size_passed'],
                        fixed_pose_geometry_passed=r['fixed_pose_geometry_passed'],
                        error=r.get('error')) for r in report['results']]), ensure_ascii=False))
        if not report['passed']:
            raise SystemExit(1)
    except (OSError, ValueError, KeyError, TypeError, RuntimeError) as exc:
        parser.exit(1, f'map geometry check failed: {exc}\n')


if __name__ == '__main__':
    main()
