"""Verify startup rejects unusable or unrelated native modules before Atlas."""
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch

from relocalization_rbnx.native_backend import verify


class NativeBackendTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='relocalization-native-')
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.origin = self.directory / '_relocalization_bbs.so'
        self.origin.touch()

    def test_missing_module_is_actionable(self):
        """Reject a missing extension with build instructions before import."""
        with patch('relocalization_rbnx.native_backend.importlib.util.find_spec', return_value=None), \
                patch('relocalization_rbnx.native_backend.importlib.import_module') as importer:
            with self.assertRaisesRegex(ImportError, 'run scripts/build.sh'):
                verify(self.directory)
        importer.assert_not_called()

    def test_module_elsewhere_is_never_imported(self):
        """A similarly named system installation cannot replace this build."""
        spec = SimpleNamespace(origin=__file__)
        with patch('relocalization_rbnx.native_backend.importlib.util.find_spec', return_value=spec), \
                patch('relocalization_rbnx.native_backend.importlib.import_module') as importer:
            with self.assertRaisesRegex(ImportError, 'outside the package build'):
                verify(self.directory)
        importer.assert_not_called()

    def test_loader_failure_is_not_hidden(self):
        """Shared-library/ABI errors retain their actionable loader diagnostics."""
        spec = SimpleNamespace(origin=str(self.origin))
        with patch('relocalization_rbnx.native_backend.importlib.util.find_spec', return_value=spec), \
                patch('relocalization_rbnx.native_backend.importlib.import_module',
                      side_effect=ImportError('libpcl_kdtree missing')):
            with self.assertRaisesRegex(ImportError, 'libpcl_kdtree missing'):
                verify(self.directory)

    def test_outdated_module_is_rejected(self):
        """A stale extension missing planar registration cannot start service."""
        spec = SimpleNamespace(origin=str(self.origin))
        with patch('relocalization_rbnx.native_backend.importlib.util.find_spec', return_value=spec), \
                patch('relocalization_rbnx.native_backend.importlib.import_module',
                      return_value=SimpleNamespace(BbsSearch=object)):
            with self.assertRaisesRegex(ImportError, 'lacks PlanarRegistration'):
                verify(self.directory)
