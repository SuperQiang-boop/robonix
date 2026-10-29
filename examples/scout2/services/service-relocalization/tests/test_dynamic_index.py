"""Verify map identity selects only its exact prepared index."""
import json
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from relocalization_rbnx.backend import Backend


class DynamicIndexTests(unittest.TestCase):
    def test_loaded_identity_selects_matching_index_without_configured_map_id(self):
        """Resolve a user's saved map by its loaded identity and verify calibration."""
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            identity = dict(map_id='lab_map', generation=4, database_sha256='a' * 64)
            directory = root / 'lab_map' / 'index-key'
            directory.mkdir(parents=True)
            (directory / 'index.json').write_text(json.dumps({'identity': identity}))
            backend = Backend.__new__(Backend)
            backend.settings = SimpleNamespace(index_dir=root, maps_dir=root / 'maps', map_id=None)
            backend.expected = np.eye(4)
            backend.profile = {}
            backend._matcher_lock = __import__('threading').RLock()
            backend._matcher_identity = None
            backend.matcher = None
            matcher = object()
            with patch('relocalization_rbnx.map_index.verify_index') as verify, \
                    patch('relocalization_rbnx.backend.OfflineMatcher', return_value=matcher) as factory, \
                    patch('relocalization_rbnx.backend.read_export', return_value=(
                        {'nodes': [{'base_from_sensor': np.eye(4)[:3].reshape(-1).tolist()}]}, None, None)):
                selected = backend._matcher_for(identity)
            self.assertIs(selected, matcher)
            self.assertEqual(backend._matcher_identity, identity)
            factory.assert_called_once_with(directory, identity)
            verify.assert_called_once_with(directory, identity)


if __name__ == '__main__':
    unittest.main()
