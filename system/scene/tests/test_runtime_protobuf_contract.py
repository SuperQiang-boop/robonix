# SPDX-License-Identifier: MulanPSL-2.0
"""Offline contract for Scene's image-matched protobuf generation."""

from __future__ import annotations

import os
from pathlib import Path
import shlex
import shutil
import subprocess
import tempfile
import unittest


SCENE_ROOT = Path(__file__).resolve().parents[1]


class RuntimeProtobufContractTests(unittest.TestCase):
    def test_launcher_generates_with_runtime_image_without_network(self):
        """Exercise codegen and profile forwarding without starting real containers."""
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            scene = root / "scene"
            (scene / "scripts").mkdir(parents=True)
            (scene / "docker").mkdir()
            (scene / "rbnx-build" / "proto-staging").mkdir(parents=True)
            (scene / "rbnx-build" / "codegen" / "proto_gen").mkdir(parents=True)
            runtime_proto = root / "runtime-proto"
            runtime_proto.mkdir()

            shutil.copy2(SCENE_ROOT / "scripts" / "start.sh", scene / "scripts")
            shutil.copy2(
                SCENE_ROOT / "docker" / "entrypoint.sh",
                scene / "docker" / "entrypoint.sh",
            )
            (scene / "rbnx-build" / "proto-staging" / "asr.proto").write_text(
                'syntax = "proto3";\n', encoding="utf-8"
            )
            (runtime_proto / "atlas.proto").write_text(
                'syntax = "proto3";\n', encoding="utf-8"
            )
            # The ambient output is intentionally retained and never imported by Scene.
            ambient = scene / "rbnx-build" / "codegen" / "proto_gen" / "asr_pb2.py"
            ambient.write_text("# incompatible host gencode\n", encoding="utf-8")

            fake_bin = root / "fake-bin"
            fake_bin.mkdir()
            docker_log = root / "docker.log"
            (fake_bin / "rbnx").write_text(
                "#!/usr/bin/env bash\n"
                "set -eu\n"
                "[[ \"${1:-}\" == path && \"${2:-}\" == runtime-proto ]]\n"
                "printf '%s\\n' \"$FAKE_RUNTIME_PROTO\"\n",
                encoding="utf-8",
            )
            (fake_bin / "docker").write_text(
                """#!/usr/bin/env bash
set -eu
printf '%q ' "$@" >> "$FAKE_DOCKER_LOG"
printf '\n' >> "$FAKE_DOCKER_LOG"
if [[ " $* " == *" --network none "* ]]; then
    for argument in "$@"; do
        case "$argument" in
            *:/proto-gen) output="${argument%:/proto-gen}" ;;
        esac
    done
    [[ -n "${output:-}" ]]
    printf '# image-matched\n' > "$output/atlas_pb2.py"
    printf '# image-matched\n' > "$output/robonix_contracts_pb2_grpc.py"
fi
exit 0
""",
                encoding="utf-8",
            )
            for executable in (fake_bin / "rbnx", fake_bin / "docker"):
                executable.chmod(0o755)

            env = os.environ.copy()
            profile = root / "Fast DDS profile.xml"
            profile.write_text('<profiles/>', encoding="utf-8")
            env.update(
                {
                    "PATH": f"{fake_bin}{os.pathsep}{env['PATH']}",
                    "RBNX_PACKAGE_ROOT": str(scene),
                    "ROBONIX_SCENE_IMAGE": "scene-runtime:test",
                    "SCENE_DATA_DIR": str(root / "scene-data"),
                    "FAKE_RUNTIME_PROTO": str(runtime_proto),
                    "FAKE_DOCKER_LOG": str(docker_log),
                    "FASTRTPS_DEFAULT_PROFILES_FILE": str(profile),
                    "ROBONIX_SCENE_FORCE": "docker",
                    "ROBONIX_FORCE_CPU": "1",
                }
            )
            subprocess.run(
                ["bash", "scripts/start.sh"],
                cwd=scene,
                env=env,
                check=True,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )

            lines = docker_log.read_text(encoding="utf-8").splitlines()
            codegen = next(line for line in lines if "--network none" in line)
            service = next(line for line in lines if "--network host" in line)
            service_args = shlex.split(service)
            self.assertIn(f"{profile}:/tmp/robonix-fastrtps.xml:ro", service_args)
            self.assertIn(
                "FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/robonix-fastrtps.xml",
                service_args,
            )
            self.assertIn("--entrypoint sh", codegen)
            self.assertIn("grpc_tools.protoc", codegen)
            self.assertIn("--entrypoint /scene/docker/entrypoint.sh", service)
            self.assertIn(
                f"{scene / 'rbnx-build' / 'codegen' / 'scene_proto_gen'}:"
                "/scene/rbnx-build/codegen/proto_gen:ro",
                service,
            )
            self.assertEqual(ambient.read_text(encoding="utf-8"), "# incompatible host gencode\n")
            self.assertTrue(
                (scene / "rbnx-build" / "codegen" / "scene_proto_gen" / "atlas_pb2.py").is_file()
            )

            # Unset exports must retain the Docker image's existing default.
            env.pop("FASTRTPS_DEFAULT_PROFILES_FILE")
            docker_log.write_text("", encoding="utf-8")
            subprocess.run(
                ["bash", "scripts/start.sh"], cwd=scene, env=env, check=True,
                text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            )
            self.assertNotIn("robonix-fastrtps.xml", docker_log.read_text())

            # Missing files and directories fail before docker rm/stop/run.
            for invalid_profile in (root / "missing.xml", root):
                with self.subTest(profile=str(invalid_profile)):
                    env["FASTRTPS_DEFAULT_PROFILES_FILE"] = str(invalid_profile)
                    docker_log.write_text("", encoding="utf-8")
                    result = subprocess.run(
                        ["bash", "scripts/start.sh"], cwd=scene, env=env,
                        text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                    )
                    self.assertEqual(result.returncode, 2)
                    self.assertIn("not a readable file", result.stderr)
                    self.assertEqual(docker_log.read_text(), "")

    def test_entrypoint_has_no_ambient_codegen_fallback(self):
        entrypoint = (SCENE_ROOT / "docker" / "entrypoint.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("SCENE_PROTO_GEN=/scene/rbnx-build/codegen/scene_proto_gen", entrypoint)
        export_line = next(
            line for line in entrypoint.splitlines() if line.startswith("export PYTHONPATH=")
        )
        self.assertIn("$SCENE_PROTO_GEN", export_line)
        self.assertNotIn("/codegen/proto_gen:", export_line)


if __name__ == "__main__":
    unittest.main()
