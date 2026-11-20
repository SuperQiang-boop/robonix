"""Run the read-only input command against isolated real ROS publishers."""
import json
from pathlib import Path
import threading
import unittest

from e2e_observations import RosObservationTests
from relocalization_rbnx.probe import run


class ProbeIntegrationTests(RosObservationTests):
    def test_readonly_probe(self):
        """Check live TF/capture, unknown timing and cleanup with real ROS messages."""
        root=Path(self.temp.name)
        profile=root/'profile.json'
        value=dict(index='unused-by-input-probe',capture=dict(scan_duration_ns=50_000_000,
            header_is_end=True),base_from_sensor=self.expected.reshape(-1).tolist(),
            registration=dict(base_z=0,roll=0,pitch=0))
        profile.write_text(json.dumps(value))
        thread=threading.Thread(target=self.executor.spin,daemon=True)
        thread.start()
        try:
            result=run(profile,root/'valid.json',seconds=1.5,
                lidar_topic='/fixture/cloud',odom_topic='/fixture/odom')
            self.assertEqual(result['error_count'],0,result['errors'])
            self.assertTrue(result['stationary_capture_passed'])
            self.assertTrue(result['live_extrinsics_match'])
            self.assertEqual(len(result['capture_points']),2)
            value['capture']=dict(scan_duration_ns=None,header_is_end=None)
            profile.write_text(json.dumps(value))
            result=run(profile,root/'unknown.json',seconds=1.5,
                lidar_topic='/fixture/cloud',odom_topic='/fixture/odom')
            self.assertGreater(result['lidar']['samples'],0)
            self.assertFalse(result['stationary_capture_passed'])
            self.assertIn('explicit scan duration',' '.join(result['errors']))
        finally:
            self.executor.shutdown(timeout_sec=2)
            thread.join(3)
        self.assertFalse(thread.is_alive())


if __name__ == '__main__':
    unittest.main(defaultTest='ProbeIntegrationTests.test_readonly_probe')
