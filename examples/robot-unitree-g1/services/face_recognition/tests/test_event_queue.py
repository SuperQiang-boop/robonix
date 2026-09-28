"""Checks the face encounter MCP queue adapter."""
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "rbnx-build/codegen/robonix_mcp_types"))
sys.path.insert(0, "/home/szh/robonix/pylib/robonix-api")

from face_recognition.main import _events, next_event  # noqa: E402
from face_recognition_mcp import GetFaceRecognitionEvent_Request  # noqa: E402


class EventQueueTests(unittest.TestCase):
    def setUp(self):
        while True:
            try:
                _events.get_nowait()
                _events.task_done()
            except Exception:
                break

    def test_returns_confirmed_event(self):
        _events.put_nowait({
            "event_id": "12-1",
            "person_id": 12,
            "name": "国强",
            "hello": "强哥您好！",
            "confidence": 0.8,
            "encounter_count": 3,
            "base_intimacy_level": 2,
            "intimacy_level": 2,
            "timestamp": "1",
        })
        response = next_event(GetFaceRecognitionEvent_Request(wait_ms=0))
        self.assertTrue(response.available)
        self.assertEqual(response.person_id, 12)
        self.assertEqual(response.intimacy_level, 2)

    def test_empty_queue_is_not_an_event(self):
        response = next_event(GetFaceRecognitionEvent_Request(wait_ms=0))
        self.assertFalse(response.available)


if __name__ == "__main__":
    unittest.main()
