"""Pose handoff waits for mapping convergence without weakening verification."""
from types import SimpleNamespace as NS
import unittest

from relocalization_rbnx.handoff import handoff_candidate
from relocalization_rbnx.matching import MatchingOptions


class HandoffTests(unittest.TestCase):
    def test_retries_new_pair_until_mapping_feedback_matches_candidate(self):
        """A fresh but pre-convergence map pose cannot reject a correct candidate."""
        candidate = NS(x=1.0, y=2.0, yaw=0.3)
        options = MatchingOptions()
        observations = [
            (NS(start_stamp_ns=201, end_stamp_ns=211), NS(start_stamp_ns=221, end_stamp_ns=231)),
            (NS(start_stamp_ns=301, end_stamp_ns=311), NS(start_stamp_ns=321, end_stamp_ns=331)),
        ]

        class Collector:
            def __init__(self):
                self.captures = 0
                self.validated = None

            def validate_current(self, _timeout, _token):
                return 101

            def collect(self, _timeout, _token):
                value = observations[self.captures]
                self.captures += 1
                return value

            def validate_after_matching(self, value, _timeout, _token, *, stationary_since_ns):
                self.validated = value, stationary_since_ns

        class Mapping:
            def __init__(self):
                self.reads = 0
                self.injected = 0

            def inject(self, _candidate, _deadline, _token, _authorize):
                self.injected += 1
                return 200

            def read_pose(self, stamp, _deadline, _token):
                self.reads += 1
                if self.reads == 1:
                    return dict(stamp_ns=stamp, x=0.0, y=0.0, yaw=0.0,
                                z=0.0, roll=0.0, pitch=0.0)
                return dict(stamp_ns=stamp, x=1.0, y=2.0, yaw=0.3,
                            z=0.0, roll=0.0, pitch=0.0)

        class Matcher:
            def __init__(self):
                self.evaluated = []

            def evaluate(self, observation, pose, *_args):
                self.evaluated.append((observation, pose))
                return NS(observation_stamp_ns=observation.end_stamp_ns)

        collector, mapping, matcher = Collector(), Mapping(), Matcher()
        result = handoff_candidate(collector, matcher, mapping, candidate, lambda: None,
            registration_options=NS(base_z=0.0, roll=0.0, pitch=0.0), options=options,
            timeout_s=2.0, cancelled=NS(cancelled=False))

        self.assertEqual(mapping.injected, 1)
        self.assertEqual(collector.captures, 2)
        self.assertEqual(len(matcher.evaluated), 2)
        self.assertEqual(result.observation_stamp_ns, 331)
        self.assertEqual(collector.validated, (observations[1], 101))


if __name__ == '__main__':
    unittest.main()
