"""Exercise mapping readiness separately from permission for autonomous motion."""
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from relocalization_rbnx.coordination import MapReader, MapWriter, write_record
from relocalization_rbnx.navigation import check, mapping_active, mapping_navigation_ready


class CoordinationTests(unittest.TestCase):
    def test_mapping_admission_and_localization_transition(self):
        """Fresh mapping admits guarded navigation before localization exists."""
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            writer = MapWriter(directory, 'mapping')
            try:
                reader = MapReader(directory, 'mapping')
                writer.mapping('first_map')
                self.assertEqual(reader.state()['map_id'], 'first_map')
                self.assertTrue(mapping_active(directory))
                self.assertTrue(check(directory)[0])
                self.assertFalse(mapping_navigation_ready(directory))
                write_record(directory / 'navigation.json', dict(
                    ready=True, mapping_process_id=reader.state()['process_id'],
                    load_id=reader.state()['load_id']))
                self.assertTrue(mapping_navigation_ready(directory))
                write_record(directory / 'navigation.json', dict(
                    ready=True, mapping_process_id=reader.state()['process_id'],
                    load_id='old-load'))
                self.assertFalse(mapping_navigation_ready(directory))
                (directory / 'navigation.json').unlink()
                with self.assertRaisesRegex(RuntimeError, 'localization snapshot'):
                    reader.current()
                with writer.transition():
                    self.assertFalse(mapping_active(directory))
                    with self.assertRaisesRegex(RuntimeError, 'ready map state'):
                        reader.state()
                    self.assertFalse(check(directory)[0])
                    identity = dict(map_id='custom_map', generation=3, database_sha256='a'*64)
                    writer.loaded(identity)
                self.assertFalse(mapping_active(directory))
                self.assertEqual(reader.current()['map_id'], 'custom_map')
                status = dict(identity, provider_id='relocalization', active=True,
                    busy=False, localization_valid=True,
                    mapping_process_id=reader.current()['process_id'],
                    load_id=reader.current()['load_id'])
                write_record(directory / 'status.json', status)
                self.assertTrue(check(directory)[0])
                writer.mapping('custom_map')
                self.assertTrue(check(directory)[0])
                self.assertNotIn('database_sha256', reader.state())
            finally:
                writer.close()

    def test_mapping_detection_rejects_missing_and_stale_heartbeats(self):
        """Only fresh ready mapping suppresses periodic navigation stop output."""
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            self.assertFalse(mapping_active(directory))
            write_record(directory / 'mapping.json', dict(ready=True, mode='mapping'))
            self.assertTrue(mapping_active(directory))
            self.assertFalse(check(directory)[0])
            with patch('relocalization_rbnx.coordination.time.monotonic_ns', return_value=0):
                self.assertFalse(mapping_active(directory))
