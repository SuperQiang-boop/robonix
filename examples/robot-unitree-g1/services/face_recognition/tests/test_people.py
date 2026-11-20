"""Exact registered-person resolution tests."""
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
sys.path.insert(0, str(PACKAGE / "rbnx-build/codegen/robonix_mcp_types"))
sys.path.insert(0, str(PACKAGE.parents[3] / "pylib/robonix-api"))

from face_recognition import main  # noqa: E402
from face_recognition_mcp import ResolveFacePerson_Request  # noqa: E402


class ResolvePersonTests(unittest.TestCase):
    def test_exact_unique_name_and_ambiguous_or_missing_names(self):
        with tempfile.TemporaryDirectory() as root:
            base = Path(root)
            (base / "bin").mkdir()
            (base / "person").mkdir()
            binary = base / "bin/vsis_video_service"
            binary.write_text("#!/bin/sh\n")
            binary.chmod(0o755)
            (base / "person/mapping.json").write_text(json.dumps([
                {"id": 8, "name": "小明"},
                {"id": 9, "name": "小明"},
                {"id": 10, "name": "小红"},
            ]), encoding="utf-8")
            with patch.object(main, "_settings", main.Settings.parse({"external_root": root})):
                unique = main.resolve_person(ResolveFacePerson_Request(name="小红"))
                ambiguous = main.resolve_person(ResolveFacePerson_Request(name="小明"))
                missing = main.resolve_person(ResolveFacePerson_Request(name="小"))
            self.assertTrue(unique.found)
            self.assertEqual(unique.person_id, 10)
            self.assertFalse(ambiguous.found)
            self.assertIn("ambiguous", ambiguous.detail)
            self.assertFalse(missing.found)


if __name__ == "__main__":
    unittest.main()
