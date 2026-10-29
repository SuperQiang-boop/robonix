"""Keep the service wired to the single Scout2 deployment manifest."""
import json
from pathlib import Path
import tempfile
import unittest

import yaml

from relocalization_rbnx.profile import read_profile
from relocalization_rbnx.settings import Settings


class ManifestConfigurationTests(unittest.TestCase):
    def test_single_manifest_controls_startup_mode_and_relocalization(self):
        """The shipped manifest owns mapping mode and automatic startup behavior."""
        root = Path(__file__).resolve().parents[3]
        manifest = yaml.safe_load((root / 'robonix_manifest.yaml').read_text())
        services = manifest['service']
        mapping = next(item for item in services if item['name'] == 'mapping')
        relocation = next(item for item in services if item['name'] == 'relocalization')
        self.assertEqual(mapping['config']['map_mode'], 'auto')
        self.assertTrue(relocation['config']['auto_relocalize'])
        settings = Settings.parse(relocation['config'], deployment_dir=root)
        self.assertTrue(settings.auto_relocalize)
        manual_config = {**relocation['config'], 'auto_relocalize': False}
        manual_settings = Settings.parse(manual_config, deployment_dir=root)
        self.assertFalse(manual_settings.auto_relocalize)
        self.assertNotIn('map_id', relocation['config'])
        names = [item['name'] for item in services]
        self.assertLess(names.index('mapping'), names.index('relocalization'))
        self.assertLess(names.index('relocalization'), names.index('nav2'))

    def test_calibration_profile_does_not_configure_startup_behavior(self):
        """Keep the automatic/manual switch exclusively in the manifest."""
        with tempfile.TemporaryDirectory() as temporary:
            profile = Path(temporary) / 'profile.json'
            values = dict(capture={}, base_from_sensor=[], registration={},
                          auto_relocalize=False)
            profile.write_text(json.dumps(values))
            with self.assertRaisesRegex(ValueError, 'profile requires'):
                read_profile(profile)


if __name__ == '__main__':
    unittest.main()
