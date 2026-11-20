"""Exercise the normal build entrypoint without ROS, Docker or a live robot."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


PKG = Path(__file__).resolve().parents[1]
IMAGE = 'sha256:' + 'b' * 64


class ExporterBuildTests(unittest.TestCase):
    def setUp(self):
        """Replace external build tools while running the real shell scripts."""
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        scripts = self.root / 'scripts'
        scripts.mkdir()
        for name in ('build.sh', 'build_search.sh', 'build_backend.sh'):
            shutil.copyfile(PKG / 'scripts' / name, scripts / name)
        (self.root / 'rbnx-build/codegen/ros2_idl').mkdir(parents=True)
        (self.root / 'ros-setup.bash').touch()
        self.metadata = self.root / 'rbnx-build/backend-container/image-id'
        self.metadata.parent.mkdir()
        self.metadata.write_text('sha256:' + 'a' * 64 + '\n')
        binary = self.root / 'bin'
        binary.mkdir()
        stub = binary / 'stub'
        stub.write_text(f'#!{sys.executable}\n' + '''
import json
import os
from pathlib import Path
import sys
command = Path(sys.argv[0]).name
with open(os.environ['BUILD_TEST_LOG'], 'a') as stream:
    stream.write(json.dumps([command, *sys.argv[1:]]) + '\\n')
if command == 'docker':
    if sys.argv[1:3] == ['image', 'inspect']:
        if os.environ.get('BUILD_TEST_FAIL') == 'inspect':
            print('No such image', file=sys.stderr)
            sys.exit(1)
        print(os.environ['BUILD_TEST_IMAGE'])
    elif sys.argv[1] == 'run' and os.environ.get('BUILD_TEST_FAIL') == 'compile':
        sys.exit(2)
''')
        stub.chmod(0o755)
        for name in ('uv', 'rbnx', 'colcon', 'cmake', 'ctest', 'docker'):
            (binary / name).symlink_to(stub)
        self.environment = dict(os.environ)
        for name in ('RELOCALIZATION_BUILD_IMAGE', 'ROBONIX_MAPPING_IMAGE'):
            self.environment.pop(name, None)
        self.environment.update(
            PATH=str(binary) + os.pathsep + os.environ['PATH'],
            RBNX_PACKAGE_ROOT=str(self.root), ROBONIX_SOURCE_PATH=str(self.root),
            RELOCALIZATION_ROS_SETUP=str(self.root / 'ros-setup.bash'),
            BUILD_TEST_LOG=str(self.root / 'commands.jsonl'), BUILD_TEST_IMAGE=IMAGE)

    def build(self):
        """Run the complete build script and return the external command log."""
        result = subprocess.run(['bash', str(self.root / 'scripts/build.sh')],
                                env=self.environment, capture_output=True, text=True)
        commands = [json.loads(line) for line in
                    (self.root / 'commands.jsonl').read_text().splitlines()]
        return result, commands

    def test_normal_build_refreshes_stale_exporter_image(self):
        """The standard build must compile and test against the current image ID."""
        result, commands = self.build()
        self.assertEqual(result.returncode, 0, result.stderr)
        inspect = next(command for command in commands if command[:3] == ['docker', 'image', 'inspect'])
        self.assertEqual(inspect[3], 'robonix-mapping:latest')
        run = next(command for command in commands if command[:2] == ['docker', 'run'])
        self.assertIn(IMAGE, run)
        self.assertEqual(run[-1], '--clean-first')
        self.assertIn('ctest --test-dir', run[run.index('-c') + 1])
        self.assertEqual(self.metadata.read_text().strip(), IMAGE)

    def test_explicit_exporter_image_takes_precedence(self):
        """Keep the mapping-image override and explicit exporter override usable."""
        self.environment['ROBONIX_MAPPING_IMAGE'] = 'custom-mapping:latest'
        self.environment['RELOCALIZATION_BUILD_IMAGE'] = 'custom-exporter:latest'
        result, commands = self.build()
        self.assertEqual(result.returncode, 0, result.stderr)
        inspect = next(command for command in commands if command[:3] == ['docker', 'image', 'inspect'])
        self.assertEqual(inspect[3], 'custom-exporter:latest')

    def test_missing_image_fails_build(self):
        """An unavailable exporter image must prevent a successful build stamp."""
        self.environment['BUILD_TEST_FAIL'] = 'inspect'
        result, commands = self.build()
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(any(command[:2] == ['docker', 'run'] for command in commands))

    def test_failed_rebuild_invalidates_old_metadata(self):
        """A partly overwritten executable must never retain its old image record."""
        self.environment['BUILD_TEST_FAIL'] = 'compile'
        result, _ = self.build()
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.metadata.exists())

    def test_unchanged_image_allows_incremental_build(self):
        """Keep normal repeated builds incremental without skipping exporter tests."""
        self.metadata.write_text(IMAGE + '\n')
        result, commands = self.build()
        self.assertEqual(result.returncode, 0, result.stderr)
        run = next(command for command in commands if command[:2] == ['docker', 'run'])
        self.assertNotIn('--clean-first', run)
        self.assertEqual(self.metadata.read_text().strip(), IMAGE)


if __name__ == '__main__':
    unittest.main()
