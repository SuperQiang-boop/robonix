"""Mapping feedback timing bounds without a ROS process or pose injection."""
import threading
import time
from types import SimpleNamespace as NS
import unittest

from _relocalization_bbs import SearchTimeout
from relocalization_rbnx.mapping_io import MappingIO


class MappingFeedbackTests(unittest.TestCase):
    def setUp(self):
        """Provide a bounded feedback buffer with a fixed valid ROS clock."""
        self.now = 2_000_000_000
        self.mapping = MappingIO.__new__(MappingIO)
        self.mapping.node = NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=self.now)))
        self.mapping._condition = threading.Condition()
        self.mapping._closed, self.mapping._error = False, None
        self.mapping._last_clock = 0
        self.mapping.max_age_s = .5
        self.mapping._poses = []
        self.token = NS(cancelled=False)
        self.scan = self.now-50_000_000

    def add_pose(self, stamp, age=0):
        self.mapping._poses.append(dict(stamp_ns=stamp, received=time.monotonic()-age))

    def read(self, **kwargs):
        return self.mapping.read_pose(self.scan, time.monotonic()+.01, self.token, **kwargs)

    def test_tracking_can_transport_nearest_previous_feedback(self):
        """A skipped 10 Hz tick can still use nearby historical feedback."""
        self.add_pose(self.scan-90_000_000)
        self.add_pose(self.scan-1_000_000)
        self.add_pose(self.scan+40_000_000)
        result = self.read(allow_before=True)
        self.assertEqual(result['stamp_ns'], self.scan-1_000_000)
        self.assertIsNot(result, self.mapping._poses[1])

    def test_handoff_still_requires_feedback_after_scan(self):
        """Default callers cannot satisfy handoff using preceding feedback."""
        self.add_pose(self.scan-1_000_000)
        with self.assertRaises(SearchTimeout):
            self.read()
        self.add_pose(self.scan+40_000_000)
        self.assertEqual(self.read()['stamp_ns'], self.scan+40_000_000)

    def test_tracking_rejects_excessive_skew_and_expired_feedback(self):
        """Two-sided timing never expands skew or either freshness bound."""
        for stamp, age in ((self.scan-100_000_001, 0),
                           (self.scan+100_000_001, 0), (self.scan-1_000_000, .6)):
            with self.subTest(stamp=stamp, age=age):
                self.mapping._poses = []
                self.add_pose(stamp, age)
                with self.assertRaises(SearchTimeout):
                    self.read(allow_before=True)
        self.now += 600_000_000
        self.mapping._poses = []
        self.add_pose(self.scan)
        with self.assertRaises(SearchTimeout):
            self.read(allow_before=True)


if __name__ == '__main__':
    unittest.main()
