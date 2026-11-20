"""Prepare a profile from checked map geometry without inventing driver timing."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from .map_index import read_export, sha256, verify_index
from .observations import CaptureOptions
from .prepare import saved_snapshot


def fixed_transform(urdf: Path, base_frame: str, sensor_frame: str) -> np.ndarray:
    """Resolve base_from_sensor through fixed URDF joints in either direction.

    Require a tree and declared links. Movable joints on the requested path
    are rejected because their current position cannot be read from URDF.
    """
    root = ET.fromstring(Path(urdf).read_text())
    links = [link.attrib['name'] for link in root.findall('link')]
    if root.tag != 'robot' or len(links) != len(set(links)) or not links:
        raise ValueError('URDF must contain a robot with unique links')
    if base_frame not in links or sensor_frame not in links or base_frame == sensor_frame:
        raise ValueError('distinct base and sensor frames must exist in URDF')
    parents, edges = {}, {name: [] for name in links}
    for joint in root.findall('joint'):
        parent = joint.find('parent').attrib['link']
        child = joint.find('child').attrib['link']
        if parent not in edges or child not in edges or child in parents or parent == child:
            raise ValueError('URDF joints must form a tree of declared links')
        parents[child] = parent
        origin = joint.find('origin')
        xyz = np.array([float(v) for v in (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()])
        rpy = np.array([float(v) for v in (origin.get('rpy', '0 0 0') if origin is not None else '0 0 0').split()])
        if xyz.shape != (3,) or rpy.shape != (3,) or not np.isfinite(np.r_[xyz, rpy]).all():
            raise ValueError('URDF origins must contain three finite xyz and rpy numbers')
        cr, cp, cy = np.cos(rpy)
        sr, sp, sy = np.sin(rpy)
        matrix = np.eye(4)
        matrix[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                          [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                          [-sp, cp*sr, cp*cr]]
        matrix[:3, 3] = xyz
        fixed = joint.get('type') == 'fixed'
        edges[parent].append((child, matrix, fixed))
        edges[child].append((parent, np.linalg.inv(matrix), fixed))
    for link in links:
        seen, current = set(), link
        while current in parents:
            if current in seen:
                raise ValueError('URDF joints contain a cycle')
            seen.add(current)
            current = parents[current]
    if sum(link not in parents for link in links) != 1:
        raise ValueError('URDF joints must form one connected tree')
    pending, seen = [(base_frame, np.eye(4), True)], set()
    while pending:
        frame, matrix, fixed = pending.pop()
        if frame == sensor_frame:
            if not fixed:
                raise ValueError('sensor path contains a movable joint')
            return matrix
        seen.add(frame)
        pending.extend((child, matrix @ step, fixed and is_fixed)
                       for child, step, is_fixed in edges[frame] if child not in seen)
    raise ValueError('sensor frame is disconnected from base frame')


def prepare_profile(index: Path, maps_dir: Path, urdf: Path, *, base_frame='base_link',
                    sensor_frame='velodyne', scan_duration_ns=None, header_is_end=None,
                    timing_evidence=None) -> tuple[dict, dict]:
    """Verify identity, full coverage, URDF extrinsics and the fixed planar model.

    Height/tilt are extracted only when every optimized pose agrees. Timing
    stays null unless both explicit values and their operator evidence exist.
    The result is a draft until the existing deployment check accepts it.
    """
    index, urdf = Path(index).resolve(strict=True), Path(urdf).resolve(strict=True)
    identity = json.loads((index / 'index.json').read_text())['identity']
    verify_index(index, identity)
    database, generation = saved_snapshot(Path(maps_dir), identity['map_id'])
    if generation != identity['generation'] or sha256(database) != identity['database_sha256']:
        raise ValueError('prepared index does not match the saved map checkpoint')
    report, _, _ = read_export(index)
    ids = {node['id'] for node in report['nodes']}
    components = report['optimized_components']
    if (len(components) != 1 or set(components[0]) != ids or report['excluded_ids'] or
            report['total_nodes'] != len(ids)):
        raise ValueError('profile requires a connected index covering every map node')
    expected = fixed_transform(urdf, base_frame, sensor_frame)
    for node in report['nodes']:
        if not np.allclose(np.asarray(node['base_from_sensor']).reshape(3, 4),
                           expected[:3], atol=1e-4, rtol=0):
            raise ValueError(f'URDF differs from saved scan extrinsics at node {node["id"]}')
    poses = np.asarray([node['map_from_base'] for node in report['nodes']]).reshape(-1, 3, 4)
    height = poses[:, 2, 3]
    roll = np.arctan2(poses[:, 2, 1], poses[:, 2, 2])
    pitch = np.arctan2(-poses[:, 2, 0], np.hypot(poses[:, 2, 1], poses[:, 2, 2]))
    values = dict(base_z=height, roll=roll, pitch=pitch)
    registration = {key: float(np.median(value)) for key, value in values.items()}
    for key, value in values.items():
        difference = value-registration[key]
        if key != 'base_z':
            difference = (difference+math.pi) % (2*math.pi)-math.pi
        if np.max(np.abs(difference)) > 0.01:
            raise ValueError('map height/tilt varies; fixed planar calibration needs review')
    provided = (scan_duration_ns is not None, header_is_end is not None, timing_evidence is not None)
    if any(provided):
        if not all(provided) or not isinstance(timing_evidence, str) or not timing_evidence.strip():
            raise ValueError('duration, header convention and timing evidence must be supplied together')
        CaptureOptions(scan_duration_ns=scan_duration_ns, header_is_end=header_is_end).validate()
    profile = dict(index=str(index), capture=dict(scan_duration_ns=scan_duration_ns,
        header_is_end=header_is_end), base_from_sensor=expected.reshape(-1).tolist(),
        registration=registration)
    evidence = dict(identity=identity, index=str(index), optimized_nodes=len(ids),
        urdf=str(urdf), urdf_sha256=sha256(urdf), base_frame=base_frame,
        sensor_frame=sensor_frame, base_from_sensor=profile['base_from_sensor'],
        registration=registration,
        saved_pose_ranges={key: [float(min(value)), float(max(value))] for key, value in values.items()},
        timing_evidence=timing_evidence, missing_inputs=[] if all(provided) else
        ['capture.scan_duration_ns', 'capture.header_is_end', 'driver timing evidence'],
        note='URDF/map consistency does not verify physical mounting; run live input and deployment checks.')
    return profile, evidence


def main():
    """Write new profile/evidence files only; never modify maps or start services."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--index', type=Path, required=True)
    parser.add_argument('--maps-dir', type=Path, required=True)
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--sensor-frame', default='velodyne')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--scan-duration-ns', type=int)
    parser.add_argument('--header-is-end', choices=('true', 'false'))
    parser.add_argument('--timing-evidence')
    args = parser.parse_args()
    try:
        if args.output.resolve() == args.report.resolve() or args.output.exists() or args.report.exists():
            raise ValueError('profile and report must be distinct new files')
        profile, evidence = prepare_profile(args.index, args.maps_dir, args.urdf,
            base_frame=args.base_frame, sensor_frame=args.sensor_frame,
            scan_duration_ns=args.scan_duration_ns,
            header_is_end=None if args.header_is_end is None else args.header_is_end == 'true',
            timing_evidence=args.timing_evidence)
        with args.report.open('x') as stream:
            stream.write(json.dumps(evidence, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
        with args.output.open('x') as stream:
            stream.write(json.dumps(profile, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
        print(json.dumps(dict(profile=str(args.output), report=str(args.report),
                              missing_inputs=evidence['missing_inputs']), ensure_ascii=False))
    except (OSError, ValueError, KeyError, TypeError, AttributeError, ET.ParseError) as exc:
        parser.exit(1, f'cannot prepare calibration profile: {exc}\n')


if __name__ == '__main__':
    main()
