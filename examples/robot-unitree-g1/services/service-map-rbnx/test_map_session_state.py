# SPDX-License-Identifier: MulanPSL-2.0
"""The live-database record that makes save_map work after load_map.

RTAB-Map does not write the saved artifact directly: a load copies the saved
database to a runtime path and switches onto the copy. save_map therefore has
to know which path is live. That record used to be kept by the gRPC and MCP
adapters, so the web UI -- which calls the impls directly -- never updated it,
and a save issued after a web UI load either failed outright or snapshotted the
pre-load database. These tests pin the record to map_ops, where every entry
point shares it.
"""
from __future__ import annotations

import os
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

from mapping_rbnx import map_ops


def _touch(path: str) -> str:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    open(path, "wb").close()
    return path


class ActiveDatabaseRecordTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.addCleanup(map_ops.set_active_db, "")
        os.makedirs(os.path.join(self.tmp.name, "target"))
        self.saved_db = _touch(os.path.join(self.tmp.name, "target", "rtabmap.db"))
        self.runtime_db = _touch(os.path.join(self.tmp.name, "runtime", "target-1.db"))

    def _load(self, publish=(True, "published"), verify=(True, "verified")):
        with (
            patch.object(map_ops, "MAPS_DIR", self.tmp.name),
            patch.object(map_ops, "_sqlite_quick_check", return_value=(True, "ok")),
            patch.object(map_ops, "_get_node", return_value=object()),
            patch.object(map_ops, "_runtime_db_copy", return_value=self.runtime_db),
            patch.object(map_ops, "_set_mode", return_value=(True, "ok")),
            patch.object(map_ops, "_load_database", return_value=(True, "ok")),
            patch.object(map_ops.lifecycle, "set_mode"),
            patch.object(map_ops.lifecycle, "set_state"),
            patch.object(map_ops, "_publish_full_map", return_value=publish),
            patch.object(map_ops, "_begin_target_map_wait", return_value={}),
            patch.object(map_ops, "_finish_target_map_wait", return_value=verify),
        ):
            return map_ops.load_map_impl("target")

    def test_load_records_the_runtime_copy_as_the_live_database(self):
        map_ops.set_active_db("/startup/mapping-1.db")
        self.assertTrue(self._load()["ok"])
        self.assertEqual(map_ops.get_active_db(), self.runtime_db)

    def test_load_records_the_swap_even_when_a_later_stage_fails(self):
        # rtabmap is serving the runtime copy the moment LoadDatabase returns.
        # A failed preview publish afterwards must not leave save_map aimed at
        # the database that is no longer open.
        map_ops.set_active_db("/startup/mapping-1.db")
        out = self._load(publish=(False, "no map"))
        self.assertFalse(out["ok"])
        self.assertEqual(map_ops.get_active_db(), self.runtime_db)

    def test_save_snapshots_the_database_recorded_by_load(self):
        self._load()
        seen = {}

        def _flush(_node, live_db, _timeout):
            seen["live_db"] = live_db
            return True, "flushed", live_db

        with (
            patch.object(map_ops, "MAPS_DIR", os.path.join(self.tmp.name, "maps")),
            patch.object(map_ops, "_get_node", return_value=object()),
            patch.object(map_ops, "_flush_rtabmap_database", side_effect=_flush),
            patch.object(map_ops, "_sqlite_backup", return_value=(True, "ok")),
            patch.object(map_ops, "_sqlite_quick_check", return_value=(True, "ok")),
            patch.object(map_ops, "_publish_full_map", return_value=(True, "published")),
            patch.object(map_ops, "_run_preview_snapshot", return_value=True),
            patch.object(map_ops, "_atomic_publish_map_dir"),
            patch.object(os.path, "isfile", lambda p: not p.endswith("maps/second/rtabmap.db")),
        ):
            out = map_ops.save_map_impl("second")

        self.assertTrue(out["ok"], out)
        self.assertEqual(seen["live_db"], self.runtime_db)

    def test_save_reports_the_paths_it_tried_when_nothing_is_live(self):
        map_ops.set_active_db("/gone/mapping-1.db")
        with (
            patch.object(map_ops, "MAPS_DIR", os.path.join(self.tmp.name, "maps")),
            patch.dict(os.environ, {"RTABMAP_DATABASE_PATH": ""}, clear=False),
        ):
            out = map_ops.save_map_impl("second")
        self.assertFalse(out["ok"])
        self.assertIn("/gone/mapping-1.db", out["detail"])

    def test_a_saved_session_is_marked_finalized(self):
        self.assertFalse(map_ops.map_finalized())
        map_ops._mark_finalized()
        self.assertTrue(map_ops.map_finalized())
        # Opening another database starts an unpublished session again.
        map_ops.set_active_db(self.runtime_db)
        self.assertFalse(map_ops.map_finalized())


class ModeReadTest(unittest.TestCase):
    def test_get_mode_reads_the_lifecycle_broadcast(self):
        # One owner: whatever consumers are told over the lifecycle topic is
        # what get_mode (and the web UI badge) reports.
        with patch.object(map_ops.lifecycle, "current",
                          return_value={"map_id": "x", "mode": "localization"}):
            self.assertEqual(map_ops.get_mode_impl(),
                             {"ok": True, "mode": "localization", "detail": ""})

    def test_get_mode_is_not_ok_before_init(self):
        with patch.object(map_ops.lifecycle, "current",
                          return_value={"map_id": "", "mode": ""}):
            self.assertFalse(map_ops.get_mode_impl()["ok"])


if __name__ == "__main__":
    unittest.main()


class ResetEpochTest(unittest.TestCase):
    """A reset moves the map origin, which invalidates every stored map-frame
    coordinate. Consumers learn that from the lifecycle generation bump, so the
    bump has to happen when the reset happens -- not only on the path where
    everything after it also succeeded."""

    def _reset(self, mode_ok):
        calls = []
        # reset_map_impl imports std_srvs lazily; off-robot it is absent.
        std_srvs = types.ModuleType("std_srvs")
        srv = types.ModuleType("std_srvs.srv")
        srv.Empty = type("Empty", (), {"Request": staticmethod(lambda: object())})
        std_srvs.srv = srv
        with (
            patch.dict(sys.modules, {"std_srvs": std_srvs, "std_srvs.srv": srv}),
            patch.object(map_ops, "_get_node", return_value=object()),
            patch.object(map_ops, "_call_service", return_value=(True, "ok")),
            patch.object(map_ops, "_set_mode",
                         return_value=(mode_ok, "ok" if mode_ok else "no mode service")),
            patch.object(map_ops.lifecycle, "mark_reset",
                         side_effect=lambda *a, **k: calls.append("epoch")),
            patch.object(map_ops.lifecycle, "set_mode",
                         side_effect=lambda *a, **k: calls.append("mode")),
        ):
            return map_ops.reset_map_impl(), calls

    def test_a_successful_reset_bumps_the_epoch_and_reports_mapping(self):
        out, calls = self._reset(mode_ok=True)
        self.assertTrue(out["ok"], out)
        self.assertEqual(calls, ["epoch", "mode"])

    def test_the_epoch_is_bumped_even_when_the_mode_switch_fails(self):
        out, calls = self._reset(mode_ok=False)
        self.assertFalse(out["ok"])
        self.assertEqual(calls, ["epoch"])
        self.assertIn("stale", out["detail"])
