"""Verify diagnostics preserve timing uncertainty and expose corrupt input."""
import unittest
from types import SimpleNamespace as NS

import numpy as np

from relocalization_rbnx.probe import InputProbe, point_time_ranges
from test_observations import cloud, odom


class ProbeTests(unittest.TestCase):
    def test_intervals_are_not_scan_duration(self):
        """Observable 10 Hz headers cannot establish scan length or timestamp type."""
        probe=InputProbe()
        for stamp in [1_000_000_000,1_100_000_000,1_200_000_000]:
            probe.receive(cloud(np.ones((3,3)),stamp),stamp+200_000_000,cloud=True)
            probe.receive(odom(stamp),stamp+5_000_000,cloud=False)
        report=probe.report()
        self.assertEqual(report['lidar']['header_interval_s']['median'],.1)
        self.assertEqual(report['lidar']['header_age_s']['median'],.2)
        self.assertNotIn('scan_duration_ns',report['lidar'])
        self.assertEqual(report['error_count'],0)

    def test_errors_latch_and_empty_input_is_visible(self):
        """Duplicate stamps and odometry frame errors cannot look like clean input."""
        probe=InputProbe()
        self.assertEqual(probe.report()['lidar']['samples'],0)
        for _ in range(2):probe.receive(cloud(np.ones((3,3)),1_000_000_000),1_100_000_000,cloud=True)
        probe.receive(odom(1_000_000_000,child='wrong'),1_100_000_000,cloud=False)
        self.assertEqual(probe.report()['error_count'],2)
        self.assertEqual(probe.report()['odom']['samples'],0)

    def test_point_timing_respects_endian_and_padding_without_unit_guess(self):
        """Read raw padded big-endian per-point times and refuse invalid fields."""
        message=cloud(np.ones((3,3)),1_000_000_000,big=True,row_padding=5)
        message.fields.append(NS(name='time',offset=12,datatype=7,count=1))
        data=bytearray(message.data)
        for i,value in enumerate([0.,.05,.1]):data[i*16+12:i*16+16]=np.array(value,dtype='>f4').tobytes()
        message.data=bytes(data)
        result=point_time_ranges(message)['time']
        self.assertAlmostEqual(result['max'],.1)
        self.assertEqual(result['units'],'unverified')
        message.fields[-1].offset=16
        with self.assertRaisesRegex(ValueError,'stride'):point_time_ranges(message)
