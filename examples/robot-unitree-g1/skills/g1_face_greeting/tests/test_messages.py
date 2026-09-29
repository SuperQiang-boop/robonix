"""Message persistence, event matching and speech result tests."""
import json
import asyncio
import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
sys.path.insert(0, str(PACKAGE / "rbnx-build/codegen/robonix_mcp_types"))
sys.path.insert(0, str(PACKAGE.parents[3] / "pylib/robonix-api"))

from g1_face_greeting import main  # noqa: E402
from g1_face_greeting.messages import MessageStore  # noqa: E402
from g1_face_greeting_mcp import (  # noqa: E402
    CancelFaceMessage_Request, CreateFaceMessage_Request,
    FaceMessageStatus_Request, ListFaceMessages_Request,
)


class MessageTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / "tasks.sqlite3"
        main._settings = main.Settings.parse({"message_store_path": str(self.path)})
        main._store = MessageStore(self.path)
        main._face_connection = SimpleNamespace(endpoint="face")
        main._scene_connection = None
        main._speech_connection = SimpleNamespace(endpoint="speech")
        main._arm_connection = SimpleNamespace(endpoint="arm")
        main._last_greet.clear()
        main._seen_events.clear()
        self.addCleanup(lambda: setattr(main, "_store", None))

    def test_default_message_store_path_is_absolute(self):
        self.assertTrue(main.Settings.parse({}).message_store_path.is_absolute())

    def test_identical_pending_message_is_idempotent(self):
        first = main._store.create_or_get(8, "小红", "王老师", "下午开会。")
        second = main._store.create_or_get(8, "小红", "王老师", "下午 开会")
        self.assertTrue(first[1])
        self.assertFalse(second[1])
        self.assertEqual(first[0], second[0])
        self.assertEqual(len(main._store.list("pending")), 1)

    def _event(self, observed_at=None, event_id="encounter"):
        return {
            "event_id": event_id, "person_id": 8, "name": "小红", "hello": "小红您好！",
            "intimacy_level": 1, "observed_at": observed_at or time.time(),
        }

    def test_create_via_person_resolution_then_combined_speech_and_status(self):
        def call(connection, tool, payload, timeout):
            if tool == "resolve":
                return {"found": True, "person_id": 8, "name": "小红"}
            return {"ok": True} if tool == "speak" else {"accepted": True}

        async def call_async(connection, tool, payload, timeout):
            return call(connection, tool, payload, timeout)

        with patch.object(main, "_call_connection", side_effect=call_async) as resolve:
            with patch.object(main, "_call_sync", side_effect=call) as invoke:
                first = asyncio.run(main.create_message(CreateFaceMessage_Request(
                    person_name="小红", message="下午开会", sender="王老师",
                )))
                second = asyncio.run(main.create_message(CreateFaceMessage_Request(
                    person_name="小红", message="带上材料", sender="",
                )))
                self.assertTrue(first.accepted)
                self.assertTrue(second.accepted)
                pending = asyncio.run(main.list_messages(ListFaceMessages_Request(status="pending")))
                self.assertEqual(len(json.loads(pending.tasks_json)), 2)
                result = main._greet(self._event())
                self.assertTrue(result["speech_ok"])
                calls = [args.args for args in invoke.call_args_list if args.args[1] == "speak"]
                self.assertEqual(len(calls), 1)
                self.assertIn("小红您好", calls[0][2]["text"])
                self.assertIn("王老师托我告诉您：下午开会", calls[0][2]["text"])
                self.assertIn("有人托我告诉您：带上材料", calls[0][2]["text"])
                status = asyncio.run(main.message_status(FaceMessageStatus_Request(task_id=first.task_id)))
                self.assertEqual(json.loads(status.task_json)["status"], "delivered")

    def test_object_is_bound_at_creation_and_scene_absence_means_removed(self):
        main._scene_connection = SimpleNamespace(endpoint="scene")

        async def call_async(connection, tool, payload, timeout):
            if tool == "resolve":
                return {"found": True, "person_id": 8, "name": "小红"}
            return {"objects": [{"id": "scene.object.cup_001", "label": "cup", "last_seen_unix": 10.0}]}

        with patch.object(main, "_call_connection", side_effect=call_async):
            created = asyncio.run(main.create_message(CreateFaceMessage_Request(
                person_name="小红", message="请拿走杯子", sender="小C", object_label="cup",
            )))
        self.assertTrue(created.accepted)
        self.assertEqual(created.object_id, "scene.object.cup_001")
        self.assertEqual(main._store.get(created.task_id)["object_status"], "present")

        async def absent(connection, tool, payload, timeout):
            return {"objects": []}

        with patch.object(main, "_call_connection", side_effect=absent):
            status = asyncio.run(main.message_status(FaceMessageStatus_Request(task_id=created.task_id)))
        self.assertEqual(status.object_status, "removed")
        self.assertEqual(json.loads(status.task_json)["object_status"], "removed")

    def test_object_scene_query_failure_is_unknown(self):
        task_id, _ = main._store.create_or_get(8, "小红", "", "请拿走杯子", "cup-1", "cup")
        main._scene_connection = SimpleNamespace(endpoint="scene")
        with patch.object(main, "_call_connection", side_effect=RuntimeError("scene offline")):
            status = asyncio.run(main.message_status(FaceMessageStatus_Request(task_id=task_id)))
        self.assertEqual(status.object_status, "unknown")

    def test_old_event_does_not_deliver_and_cooldown_does_not_block_message(self):
        old_time = time.time() - 5
        task_id = main._store.create(8, "小红", "", "请回电")
        def call(connection, tool, payload, timeout):
            return {"ok": True} if tool == "speak" else {"accepted": True}

        with patch.object(main, "_call_sync", side_effect=call) as invoke:
            main._greet(self._event(old_time, "older"))
            self.assertEqual(main._store.get(task_id)["status"], "pending")
            result = main._greet(self._event(time.time(), "newer"))
            self.assertTrue(result["accepted"])
            self.assertEqual(main._store.get(task_id)["status"], "delivered")
            speeches = [args.args[2]["text"] for args in invoke.call_args_list if args.args[1] == "speak"]
            self.assertEqual(len(speeches), 2)
            self.assertNotIn("小红您好", speeches[1])

    def test_failure_and_restart_do_not_auto_repeat(self):
        task_id = main._store.create(8, "小红", "", "请回电")
        with patch.object(main, "_call_sync", side_effect=lambda conn, tool, payload, timeout:
                          {"ok": False, "detail": "speaker failed"} if tool == "speak" else {"accepted": True}):
            main._greet(self._event())
        self.assertEqual(main._store.get(task_id)["status"], "uncertain")
        self.assertEqual(main._store.claim(8, time.time(), "next"), [])
        pending = main._store.create(8, "小红", "", "第二条")
        main._store.claim(8, time.time(), "interrupt")
        self.assertEqual(MessageStore(self.path).get(pending)["status"], "uncertain")

    def test_cancel_pending_and_reject_ambiguous_name(self):
        async def resolve_ambiguous(connection, tool, payload, timeout):
            return {"found": False, "detail": "ambiguous registered name"}

        with patch.object(main, "_call_connection", side_effect=resolve_ambiguous):
            result = asyncio.run(main.create_message(CreateFaceMessage_Request(person_name="小明", message="你好")))
        self.assertFalse(result.accepted)
        self.assertEqual(main._store.list(), [])
        task_id = main._store.create(8, "小红", "", "请回电")
        self.assertTrue(main.cancel_message(CancelFaceMessage_Request(task_id=task_id)).cancelled)
        self.assertEqual(main._store.get(task_id)["status"], "cancelled")
        self.assertEqual(main._store.claim(8, time.time(), "later"), [])


if __name__ == "__main__":
    unittest.main()
