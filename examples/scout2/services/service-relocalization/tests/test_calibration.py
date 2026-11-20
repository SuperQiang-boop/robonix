"""Check coordinate conventions and reject unsupported deployment calibration."""
import json
from pathlib import Path
import sqlite3
import tempfile
import unittest

import numpy as np

from relocalization_rbnx.calibration import fixed_transform, prepare_profile
from relocalization_rbnx.map_index import ARTIFACTS, build_index, sha256
from test_map_index import write_export


class CalibrationTests(unittest.TestCase):
    def setUp(self):
        """Prepare a complete one-node index, stable DB and fixed-joint model."""
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.urdf = self.root/'robot.urdf'
        self.urdf.write_text('<robot name="test"><link name="base"/><link name="lidar"/>'
            '<joint name="sensor" type="fixed"><parent link="base"/><child link="lidar"/>'
            '<origin xyz="0 0 0" rpy="0 0 0"/></joint></robot>')
        self.maps = self.root/'maps'
        saved = self.maps/'test_map'
        saved.mkdir(parents=True)
        self.database = saved/'rtabmap.db'
        with sqlite3.connect(self.database) as c:
            c.execute('create table Admin(version text)')
            c.execute("insert into Admin values ('0.23.7')")
        (saved/'generation').write_text('1')
        self.index = self.root/'index'
        write_export(self.index)
        self.identity = dict(map_id='test_map', generation=1, database_sha256=sha256(self.database),
                             voxel_size=.05, pyramid_levels=4)
        self.publish()

    def publish(self, change=None):
        """Republish intentionally changed geometry with correct artifact hashes."""
        if change:
            p=self.index/'export.json'
            report=json.loads(p.read_text())
            change(report)
            p.write_text(json.dumps(report))
        summary=build_index(self.index, .05, 4)
        (self.index/'index.json').write_text(json.dumps(dict(format_version=1,
            identity=self.identity, summary=summary,
            files={name:sha256(self.index/name) for name in ARTIFACTS})))

    def prepare(self, **kwargs):
        return prepare_profile(self.index, self.maps, self.urdf,
                               base_frame='base', sensor_frame='lidar', **kwargs)

    def test_draft_does_not_guess_timing(self):
        """A correct map and URDF cannot supply scan time or header convention."""
        profile, report=self.prepare()
        self.assertEqual(profile['capture'],dict(scan_duration_ns=None,header_is_end=None))
        self.assertEqual(profile['registration'],dict(base_z=0.,roll=0.,pitch=0.))
        self.assertEqual(len(report['missing_inputs']),3)
        np.testing.assert_array_equal(np.array(profile['base_from_sensor']).reshape(4,4),np.eye(4))

    def test_timing_requires_evidence_and_bounded_values(self):
        """Partial, unverified or invalid time configurations cannot become ready."""
        for kwargs in [dict(scan_duration_ns=100_000_000),
                       dict(scan_duration_ns=100_000_000,header_is_end=False),
                       dict(scan_duration_ns=True,header_is_end=False,timing_evidence='source'),
                       dict(scan_duration_ns=100_000_000,header_is_end=False,timing_evidence=' ')]:
            with self.assertRaises(ValueError):self.prepare(**kwargs)
        profile, report=self.prepare(scan_duration_ns=100_000_000,
                                    header_is_end=False,timing_evidence='driver source verified')
        self.assertFalse(profile['capture']['header_is_end'])
        self.assertEqual(report['missing_inputs'],[])

    def test_map_check_preserves_failed_geometry_report_and_source(self):
        """A structurally valid index with too little geometry remains diagnostic."""
        from relocalization_rbnx.map_check import check
        points=np.array([[1,0,.3],[2,0,.4],[3,0,.5]],dtype='<f4')
        points.tofile(self.index/'cloud.bin')
        points.tofile(self.index/'scans.bin')
        self.publish()
        profile, _=self.prepare()
        profile_file=self.root/'profile.json'
        profile_file.write_text(json.dumps(profile))
        before=sha256(self.database)
        result=check(profile_file,self.maps,self.root/'geometry.json')
        self.assertFalse(result['passed'])
        self.assertFalse(result['results'][0]['capture_size_passed'])
        self.assertIn('error',result['results'][0])
        self.assertEqual(sha256(self.database),before)
        self.assertEqual(json.loads((self.root/'geometry.json').read_text())['identity'],self.identity)

    def test_rejects_stale_map_and_mismatched_extrinsics(self):
        """A valid cached index cannot authorize a different snapshot or sensor."""
        self.identity['database_sha256']='f'*64
        self.publish()
        with self.assertRaisesRegex(ValueError,'checkpoint'):self.prepare()
        self.identity['database_sha256']=sha256(self.database)
        self.publish(lambda r:r['nodes'][0]['base_from_sensor'].__setitem__(3, .25))
        with self.assertRaisesRegex(ValueError,'extrinsics'):self.prepare()

    def test_rejects_incomplete_coverage_and_variable_height(self):
        """Full graph coverage and a common height/tilt are needed for this model."""
        self.publish(lambda r:r.update(total_nodes=2))
        with self.assertRaisesRegex(ValueError,'every map node'):self.prepare()
        def second_node(r):
            r['nodes'][0]['count']=1
            other=dict(r['nodes'][0],id=2,offset=1,count=2)
            other['map_from_base']=list(other['map_from_base'])
            other['map_from_base'][11]=.2
            r['nodes'].append(other)
            r['optimized_components']=[[1,2]]
        self.publish(second_node)
        with self.assertRaisesRegex(ValueError,'height/tilt'):self.prepare()

    def test_multihop_rotation_translation_and_inverse(self):
        """Joint origins compose in parent coordinates; reverse queries invert."""
        self.urdf.write_text('<robot name="test"><link name="base"/><link name="mount"/>'
            '<link name="lidar"/><joint name="a" type="fixed"><parent link="base"/>'
            '<child link="mount"/><origin xyz="1 0 0" rpy="0 0 1.5707963267948966"/>'
            '</joint><joint name="b" type="fixed"><parent link="mount"/>'
            '<child link="lidar"/><origin xyz="2 0 0"/></joint></robot>')
        result=fixed_transform(self.urdf,'base','lidar')
        np.testing.assert_allclose(result[:3,3],[1,2,0],atol=1e-10)
        np.testing.assert_allclose(fixed_transform(self.urdf,'lidar','base'),np.linalg.inv(result))

    def test_rejects_movable_cycle_and_malformed_origin(self):
        """URDF cannot silently substitute a pose for an unknown movable joint."""
        original=self.urdf.read_text()
        for updated in [original.replace('type="fixed"','type="revolute"'),
                        original.replace('xyz="0 0 0"','xyz="0 0 0 junk"'),
                        original.replace('</robot>','<joint name="back" type="fixed">'
                            '<parent link="lidar"/><child link="base"/></joint></robot>')]:
            self.urdf.write_text(updated)
            with self.assertRaises(ValueError):fixed_transform(self.urdf,'base','lidar')
