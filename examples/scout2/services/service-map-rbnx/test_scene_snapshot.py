# SPDX-License-Identifier: MulanPSL-2.0
"""Check the bounded Scene request and explicit reporting of partial saves."""
import io
import json
import unittest
import urllib.error
from unittest.mock import patch

from mapping_rbnx import scene_snapshot


class SceneSnapshotTest(unittest.TestCase):
    """Exercise transport failures without a running robot or real object DB."""

    def test_notification_uses_completed_spatial_save_and_current_identity(self):
        """The notification saves Scene objects without issuing another spatial save."""
        response = {"ok": True, "map_id": "custom", "objects": 2,
                    "snapshot_partition": "custom__s1"}
        with (patch.dict("os.environ", {"SCENE_WEB_URL": "http://scene.test:50107/"}),
              patch.object(scene_snapshot.urllib.request, "urlopen",
                           return_value=io.BytesIO(json.dumps(response).encode())) as send):
            self.assertEqual(scene_snapshot.save_objects("custom", 7), response)
        request = send.call_args.args[0]
        self.assertEqual(request.full_url, "http://scene.test:50107/api/maps/save")
        self.assertEqual(json.loads(request.data), {
            "map_id": "custom", "generation": 7, "spatial_saved": True,
        })
        self.assertEqual(send.call_args.kwargs["timeout"], 10.0)

    def test_failures_report_missing_objects_without_throwing(self):
        """HTTP refusal, unavailable Scene, and malformed replies leave mapping online."""
        for result in (
            TimeoutError("Scene offline"),
            urllib.error.HTTPError("http://scene.test", 409, "stale", {},
                                   io.BytesIO(b'{"detail":"wrong map generation"}')),
            b"not JSON", b"[]", b'{"ok":true,"map_id":"other"}',
            b'{"ok":true,"map_id":"custom"}',
        ):
            with self.subTest(result=result):
                kwargs = ({"side_effect": result} if isinstance(result, Exception)
                          else {"return_value": io.BytesIO(result)})
                with patch.object(scene_snapshot.urllib.request, "urlopen", **kwargs):
                    with self.assertLogs(scene_snapshot.log, level="WARNING"):
                        self.assertFalse(scene_snapshot.save_objects("custom", 7)["ok"])


if __name__ == "__main__":
    unittest.main()
