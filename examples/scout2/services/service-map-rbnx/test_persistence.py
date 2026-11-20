"""Exercise SQLite recovery, repeated checkpoints and startup policy offline."""
import ast
import logging
import os
from pathlib import Path
import sqlite3
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

import yaml

from mapping_rbnx import map_ops, scene_snapshot


def create_db(path, value):
    """Create a small, real SQLite map substitute with one distinguishable row."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with sqlite3.connect(path) as con:
        con.execute("create table if not exists sample(value integer)")
        con.execute("delete from sample")
        con.execute("insert into sample values (?)", (value,))


def read_db(path):
    with sqlite3.connect(path) as con:
        return con.execute("select value from sample").fetchone()[0]


class PersistenceTest(unittest.TestCase):
    def setUp(self):
        """Isolate every test from real robot maps and global runtime state."""
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name)
        self.maps = self.root / "maps"
        self.runtime = self.root / "runtime"
        for key, value in (("MAPS_DIR", str(self.maps)),
                           ("RUNTIME_DB_DIR", str(self.runtime)), ("_ACTIVE_DB", "")):
            ctx = patch.object(map_ops, key, value)
            ctx.start()
            self.addCleanup(ctx.stop)

    def test_coordinated_localization_switch_saves_then_uses_checked_load(self):
        """Never advertise a mutable mapping database as a localization snapshot."""
        with (patch.object(map_ops.relocalization, 'writer', object()),
              patch.object(map_ops, '_get_node', return_value=object()),
              patch.object(map_ops.lifecycle, 'current', return_value=dict(map_id='custom', mode='mapping')),
              patch.object(map_ops, 'save_map_impl', return_value=dict(ok=True)) as save,
              patch.object(map_ops.load_map_impl, '__wrapped__', return_value=dict(ok=True)) as load):
            self.assertTrue(map_ops.switch_mode_impl.__wrapped__('localization')['ok'])
            save.assert_called_once_with('custom', active_db='')
            load.assert_called_once_with('custom')

    def test_failed_checkpoint_never_loads_or_grants_localization(self):
        """An incomplete checkpoint must leave localization unavailable."""
        with (patch.object(map_ops.relocalization, 'writer', object()),
              patch.object(map_ops, '_get_node', return_value=object()),
              patch.object(map_ops.lifecycle, 'current', return_value=dict(map_id='custom', mode='mapping')),
              patch.object(map_ops, 'save_map_impl', return_value=dict(ok=False, detail='disconnected graph')),
              patch.object(map_ops.load_map_impl, '__wrapped__') as load):
            result = map_ops.switch_mode_impl.__wrapped__('localization')
            self.assertFalse(result['ok'])
            self.assertIn('disconnected graph', result['detail'])
            load.assert_not_called()

    def test_mapping_switch_announces_mapping_readiness(self):
        """Switching away from localization resumes mapping supervision."""
        with (patch.object(map_ops, '_get_node', return_value=object()),
              patch.object(map_ops, '_prepare_mapping_runtime'),
              patch.object(map_ops, '_set_mode', return_value=(True, 'ok')),
              patch.object(map_ops, 'set_current_mode'),
              patch.object(map_ops.lifecycle, 'set_mode'),
              patch.object(map_ops.lifecycle, 'current', return_value=dict(map_id='custom')),
              patch.object(map_ops.relocalization, 'mapping') as announce):
            self.assertTrue(map_ops.switch_mode_impl.__wrapped__('mapping')['ok'])
            announce.assert_called_once_with('custom')

    def test_saved_checkpoint_is_restored_before_mapping_resumes(self):
        """A boot starts from the operator's last saved map checkpoint."""
        saved = self.maps / "site" / "rtabmap.db"
        create_db(saved, 1)
        runtime = Path(map_ops.persistent_runtime_db("site"))
        self.assertEqual(read_db(runtime), 1)
        create_db(runtime, 2)
        self.assertEqual(map_ops.persistent_runtime_db("site"), str(runtime))
        self.assertEqual(read_db(runtime), 1)
        self.assertEqual(read_db(saved), 1)

    def test_empty_runtime_is_restored_from_checkpoint(self):
        """An empty placeholder is not a valid map and must not hide a checkpoint."""
        saved = self.maps / "site" / "rtabmap.db"
        create_db(saved, 9)
        runtime = self.runtime / "site" / "rtabmap.db"
        runtime.parent.mkdir(parents=True)
        runtime.touch()
        map_ops.persistent_runtime_db("site")
        self.assertEqual(read_db(runtime), 9)

    def test_corrupt_runtime_is_preserved_before_restore(self):
        """A damaged DB is recoverable for investigation, including its sidecars."""
        runtime = self.runtime / "site" / "rtabmap.db"
        runtime.parent.mkdir(parents=True)
        runtime.write_bytes(b"corrupt")
        Path(str(runtime) + ".back").write_bytes(b"old backup")
        create_db(self.maps / "site" / "rtabmap.db", 3)
        map_ops.persistent_runtime_db("site")
        self.assertEqual(read_db(runtime), 3)
        old = next(runtime.parent.glob("previous-*"))
        self.assertEqual((old / "rtabmap.db").read_bytes(), b"corrupt")
        self.assertEqual((old / "rtabmap.db.back").read_bytes(), b"old backup")

    def test_corruption_without_checkpoint_does_not_silently_start_fresh(self):
        """Repeated startup attempts fail until an operator explicitly resets."""
        runtime = self.runtime / "site" / "rtabmap.db"
        runtime.parent.mkdir(parents=True)
        runtime.write_bytes(b"corrupt")
        for _ in range(2):
            with self.assertRaises(RuntimeError):
                map_ops.persistent_runtime_db("site")
        self.assertEqual(map_ops.persistent_runtime_db("site", reset=True), str(runtime))

    def test_checkpoint_twice_preserves_generation_and_live_database(self):
        """Repeated saves update a checkpoint, preserve identity, and retain rollback."""
        runtime = self.runtime / "site" / "rtabmap.db"
        create_db(runtime, 1)
        target = self.maps / "site"
        target.mkdir(parents=True)
        (target / "generation").write_text("4")
        (target / "annotations.json").write_text("{}")
        def export_snapshot(staging):
            """Model a complete independent export without reading a live writer."""
            with sqlite3.connect(runtime) as src, sqlite3.connect(Path(staging) / 'rtabmap.db') as dst:
                src.backup(dst)
            (Path(staging) / 'occupancy.png').write_bytes(b'preview')
            return True, "optimized snapshot ok"

        with (patch.object(map_ops, "_export_live_snapshot", side_effect=export_snapshot) as snapshot,
              patch.object(map_ops, "_sqlite_backup") as sqlite_backup,
              patch.object(map_ops, "_sqlite_quick_check", return_value=(True, "ok")),
              patch.object(map_ops, "_flush_rtabmap_database") as flush):
            self.assertTrue(map_ops.save_map_impl("site", active_db=str(runtime))["ok"])
            create_db(runtime, 2)
            self.assertTrue(map_ops.save_map_impl("site", active_db=str(runtime))["ok"])
        self.assertEqual(snapshot.call_count, 2)
        flush.assert_not_called()
        sqlite_backup.assert_not_called()
        self.assertEqual(read_db(target / "rtabmap.db"), 2)
        self.assertEqual(read_db(target / "rtabmap.previous.db"), 1)
        self.assertEqual(read_db(runtime), 2)
        self.assertEqual((target / "generation").read_text(), "4")
        self.assertEqual((target / "annotations.json").read_text(), "{}")

    def test_failed_live_snapshot_keeps_checkpoint(self):
        """An unavailable engine or incomplete export preserves the checkpoint."""
        runtime = self.runtime / "site" / "rtabmap.db"
        saved = self.maps / "site" / "rtabmap.db"
        create_db(runtime, 2)
        create_db(saved, 1)
        with patch.object(map_ops, "_export_live_snapshot", return_value=(False, "service unavailable")):
            self.assertFalse(map_ops.save_map_impl("site", active_db=str(runtime))["ok"])
        self.assertEqual(read_db(saved), 1)
        self.assertEqual(list(self.maps.glob('.site.staging-*')), [])

    def test_incomplete_snapshot_is_not_published(self):
        """Even an exporter exit zero cannot publish an empty or partial artifact."""
        runtime = self.runtime / 'site/rtabmap.db'
        saved = self.maps / 'site/rtabmap.db'
        create_db(runtime, 2)
        create_db(saved, 1)
        with patch.object(map_ops, '_export_live_snapshot', return_value=(True, 'incomplete')):
            result = map_ops.save_map_impl('site', active_db=str(runtime))
        self.assertFalse(result['ok'])
        self.assertEqual(read_db(saved), 1)

    def test_failed_database_publication_keeps_previous_checkpoint(self):
        """A failed final rename cannot remove the current DB or map identity."""
        target = self.maps / "site"
        staging = self.maps / ".site.staging-test"
        create_db(target / "rtabmap.db", 1)
        create_db(staging / "rtabmap.db", 2)
        (target / "generation").write_text("5")
        replace = os.replace

        def fail_publish(source, destination):
            if str(source) == str(staging / "rtabmap.db"):
                raise OSError("simulated publication failure")
            return replace(source, destination)

        with patch.object(map_ops.os, "replace", side_effect=fail_publish):
            with self.assertRaises(OSError):
                map_ops._atomic_publish_map_dir(str(staging), str(target))
        self.assertEqual(read_db(target / "rtabmap.db"), 1)
        self.assertEqual((target / "generation").read_text(), "5")

    def test_active_db_and_hidden_paths_are_not_replaced(self):
        """Reject direct active-file replacement and path traversal through map ids."""
        saved = self.maps / "site" / "rtabmap.db"
        create_db(saved, 1)
        self.assertFalse(map_ops.save_map_impl("site", active_db=str(saved))["ok"])
        for value in (".", "..", ".runtime"):
            with self.assertRaises(ValueError):
                map_ops.persistent_runtime_db(value)

    def test_startup_uses_same_named_path_and_does_not_delete_on_resume(self):
        """Run the bridge's exact resolver without starting Atlas/MCP servers."""
        source = Path(__file__).parent / "src/mapping_rbnx/atlas_bridge.py"
        tree = ast.parse(source.read_text())
        names = {"_resolve_persistence", "_sanitize_map_id", "_truthy", "_find_latest_map"}
        funcs = [node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name in names]
        ns = dict(os=os, time=time, map_ops=map_ops, MAPS_DIR=str(self.maps),
                  RUNTIME_DB_DIR=str(self.runtime), _VALID_MODES=("mapping", "localization"),
                  log=logging.getLogger(__name__))
        exec(compile(ast.Module(body=funcs, type_ignores=[]), str(source), "exec"), ns)
        first = ns["_resolve_persistence"]({"map_id": "site"})
        create_db(Path(first["database_path"]), 10)
        second = ns["_resolve_persistence"]({"map_id": "site"})
        self.assertEqual(first, second)
        self.assertEqual(second["reset_map"], "false")
        self.assertEqual(read_db(second["database_path"]), 10)

    def test_auto_mode_localizes_latest_saved_map_and_uses_id_only_when_empty(self):
        """Auto mode picks the newest saved map; configured ID names first map."""
        source = Path(__file__).parent / "src/mapping_rbnx/atlas_bridge.py"
        tree = ast.parse(source.read_text())
        names = {"_resolve_persistence", "_sanitize_map_id", "_truthy", "_find_latest_map"}
        funcs = [node for node in tree.body if isinstance(node, ast.FunctionDef)
                 and node.name in names]
        ns = dict(os=os, time=time, map_ops=map_ops, MAPS_DIR=str(self.maps),
                  RUNTIME_DB_DIR=str(self.runtime), _VALID_MODES=("mapping", "localization"),
                  log=logging.getLogger(__name__))
        exec(compile(ast.Module(body=funcs, type_ignores=[]), str(source), "exec"), ns)

        first = ns["_resolve_persistence"]({"map_id": "scout2_main", "map_mode": "auto"})
        self.assertEqual(first["map_mode"], "mapping")
        self.assertEqual(first["reset_map"], "false")

        older_dir = self.maps / "floor_a"
        newer_dir = self.maps / "floor_b"
        create_db(self.maps / "scout2_main" / "rtabmap.db", 12)
        os.utime(self.maps / "scout2_main", (50, 50))
        create_db(older_dir / "rtabmap.db", 24)
        create_db(newer_dir / "rtabmap.db", 42)
        os.utime(older_dir, (100, 100))
        os.utime(newer_dir, (200, 200))
        resumed = ns["_resolve_persistence"](
            {"map_id": "scout2_main", "map_mode": "auto"}
        )
        self.assertEqual(resumed["map_mode"], "localization")
        self.assertEqual(resumed["map_id"], "floor_b")
        self.assertEqual(resumed["reset_map"], "false")
        self.assertEqual(read_db(resumed["database_path"]), 42)
        self.assertNotEqual(Path(resumed["database_path"]), newer_dir / "rtabmap.db")
        self.assertEqual(read_db(newer_dir / "rtabmap.db"), 42)

        selected = ns["_resolve_persistence"](
            {"map_id": "scout2_main", "map_mode": "auto"}
        )
        self.assertEqual(selected["map_id"], "floor_b")
        self.assertEqual(selected["map_mode"], "localization")
        self.assertEqual(read_db(selected["database_path"]), 42)

        preferred = self.maps / "scout2_main" / "rtabmap.db"
        create_db(preferred, 84)
        os.utime(preferred.parent, (300, 300))
        selected = ns["_resolve_persistence"](
            {"map_id": "scout2_main", "map_mode": "auto"}
        )
        self.assertEqual(selected["map_id"], "scout2_main")
        self.assertEqual(read_db(selected["database_path"]), 84)

        localization = ns["_resolve_persistence"](
            {"map_id": "scout2_main", "map_mode": "localization"}
        )
        self.assertEqual(localization["map_mode"], "localization")
        self.assertNotEqual(localization["database_path"], selected["database_path"])

    def test_localization_without_map_id_selects_latest_and_fails_when_empty(self):
        """Localization follows the newest saved map and never falls back to mapping."""
        source = Path(__file__).parent / "src/mapping_rbnx/atlas_bridge.py"
        tree = ast.parse(source.read_text())
        names = {"_resolve_persistence", "_sanitize_map_id", "_truthy", "_find_latest_map"}
        funcs = [node for node in tree.body if isinstance(node, ast.FunctionDef)
                 and node.name in names]
        ns = dict(os=os, time=time, map_ops=map_ops, MAPS_DIR=str(self.maps),
                  RUNTIME_DB_DIR=str(self.runtime), _VALID_MODES=("mapping", "localization"),
                  log=logging.getLogger(__name__))
        exec(compile(ast.Module(body=funcs, type_ignores=[]), str(source), "exec"), ns)

        with self.assertRaisesRegex(RuntimeError, "at least one saved map"):
            ns["_resolve_persistence"]({"map_mode": "localization"})

        create_db(self.maps / "first" / "rtabmap.db", 1)
        create_db(self.maps / "newest" / "rtabmap.db", 2)
        os.utime(self.maps / "first", (100, 100))
        os.utime(self.maps / "newest", (200, 200))
        selected = ns["_resolve_persistence"]({"map_mode": "localization"})
        self.assertEqual(selected["map_id"], "newest")
        self.assertEqual(selected["map_mode"], "localization")
        self.assertEqual(read_db(selected["database_path"]), 2)

    def test_latest_saved_map_ties_use_map_id_order(self):
        """Mapping's no-ID selection is stable when map directory times tie."""
        source = Path(__file__).parent / "src/mapping_rbnx/atlas_bridge.py"
        tree = ast.parse(source.read_text())
        finder = next(node for node in tree.body if isinstance(node, ast.FunctionDef)
                      and node.name == "_find_latest_map")
        ns = dict(os=os, MAPS_DIR=str(self.maps), log=logging.getLogger(__name__))
        exec(compile(ast.Module(body=[finder], type_ignores=[]), str(source), "exec"), ns)
        for name in ("alpha", "zeta"):
            create_db(self.maps / name / "rtabmap.db", 1)
            os.utime(self.maps / name, ns=(100, 100))
        self.assertEqual(ns["_find_latest_map"](), "zeta")

    def test_scout2_single_manifest_uses_auto_mode_and_relocalization(self):
        """One manifest starts mapping on empty storage and loads saved maps."""
        manifest_path = Path(__file__).parents[2] / "robonix_manifest.yaml"
        manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
        mapping = next(item for item in manifest["service"] if item["name"] == "mapping")
        relocalization = next(item for item in manifest["service"]
                              if item["name"] == "relocalization")
        self.assertEqual(mapping["config"]["map_id"], "scout2_main")
        self.assertEqual(mapping["config"]["map_mode"], "auto")
        self.assertEqual(mapping["config"]["checkpoint_interval_s"], 30)
        self.assertTrue(relocalization["config"]["auto_relocalize"])
        self.assertNotIn("map_id", relocalization["config"])
        self.assertNotIn("map_id", manifest["system"]["scene"])
        self.assertNotIn("SCENE_MAP_ID", manifest.get("env", {}))

    def test_loaded_localization_copy_becomes_persistent_before_mapping(self):
        """Mode switching must not strand new nodes in an ephemeral DB."""
        loaded = self.runtime / "localization-copy.db"
        target = self.runtime / "site/rtabmap.db"
        create_db(loaded, 7)
        create_db(target, 3)
        with (patch.object(map_ops, "_ACTIVE_DB", str(loaded)),
              patch.object(map_ops.lifecycle, "current", return_value={"map_id": "site"}),
              patch.object(map_ops, "_flush_rtabmap_database", return_value=(True, "ok", str(loaded))),
              patch.object(map_ops, "_load_database", return_value=(True, "ok")) as load):
            map_ops._prepare_mapping_runtime(object())
            self.assertEqual(map_ops._ACTIVE_DB, str(target))
            self.assertEqual(read_db(target), 7)
            self.assertEqual(read_db(next(target.parent.glob("previous-*/rtabmap.db"))), 3)
            self.assertEqual(load.call_args.args[1], str(target))

    def test_loading_same_map_does_not_unlink_active_localization_database(self):
        """Preparing a second copy must leave the running engine's file intact."""
        saved = self.maps / "site/rtabmap.db"
        create_db(saved, 1)
        active = map_ops._runtime_db_copy(str(saved), "site")
        with patch.object(map_ops, "_ACTIVE_DB", active):
            next_copy = map_ops._runtime_db_copy(str(saved), "site")
        self.assertNotEqual(active, next_copy)
        self.assertEqual(read_db(active), 1)
        self.assertEqual(read_db(next_copy), 1)

    def test_shutdown_is_idempotent_and_does_not_publish_localization_copy(self):
        """Exercise the actual hook without importing Atlas or generated IDLs."""
        source = Path(__file__).parent / "src/mapping_rbnx/atlas_bridge.py"
        tree = ast.parse(source.read_text())
        hook = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "shutdown")
        hook.decorator_list = []
        for mode, expected_saves in (("mapping", 1), ("localization", 0)):
            ns = dict(map_ops=map_ops, lifecycle=map_ops.lifecycle, relocalization=map_ops.relocalization,
                      scene_snapshot=scene_snapshot,
                      _SHUTDOWN_RESULT=None, _SHUTDOWN_LOCK=threading.Lock(),
                      _CHECKPOINT_STOP=threading.Event(), _CHECKPOINT_THREAD=None,
                      _ACTIVE={"algo": "rtabmap"}, _active_db=lambda: "runtime.db",
                      Ok=object, Err=lambda detail: detail, log=logging.getLogger(__name__))
            exec(compile(ast.Module(body=[hook], type_ignores=[]), str(source), "exec"), ns)
            with (patch.object(map_ops.lifecycle, "current", return_value={"map_id": "site", "generation": 7}),
                  patch.object(map_ops, "get_mode_impl", return_value={"mode": mode}),
                  patch.object(scene_snapshot, "save_objects", return_value={"ok": True}) as objects,
                  patch.object(map_ops, "save_map_impl", return_value={"ok": True}) as save):
                first = ns["shutdown"]()
                self.assertIs(ns["shutdown"](), first)
                self.assertEqual(save.call_count, expected_saves)
                self.assertEqual(objects.call_count, expected_saves)
                if expected_saves:
                    objects.assert_called_once_with("site", 7)

    def test_periodic_save_uses_current_map_not_startup_identity(self):
        """A runtime map change must redirect subsequent periodic saves."""
        source = Path(__file__).parent / "src/mapping_rbnx/atlas_bridge.py"
        tree = ast.parse(source.read_text())
        hook = next(n for n in tree.body if isinstance(n, ast.FunctionDef)
                    and n.name == "_start_checkpoint_loop")
        stop = threading.Event()
        ns = dict(map_ops=map_ops, lifecycle=map_ops.lifecycle, threading=threading,
                  scene_snapshot=scene_snapshot,
                  _CHECKPOINT_STOP=stop, _CHECKPOINT_THREAD=None,
                  _ACTIVE={"algo": "rtabmap", "map_id": "old"},
                  _active_db=lambda: "new.db", log=logging.getLogger(__name__))
        exec(compile(ast.Module(body=[hook], type_ignores=[]), str(source), "exec"), ns)
        with (patch.object(map_ops.lifecycle, "current", return_value={"map_id": "new", "generation": 9}),
              patch.object(map_ops, "get_mode_impl", return_value={"mode": "mapping"}),
              patch.object(scene_snapshot, "save_objects", return_value={"ok": False, "detail": "offline"}) as objects,
              patch.object(map_ops, "save_map_impl", side_effect=lambda *a, **kw: (stop.set() or {"ok": True})) as save):
            ns["_start_checkpoint_loop"]({"checkpoint_interval_s": 0.01}, "mapping")
            ns["_CHECKPOINT_THREAD"].join(timeout=2)
            self.assertFalse(ns["_CHECKPOINT_THREAD"].is_alive())
            save.assert_called_once_with("new", active_db="new.db")
            objects.assert_called_once_with("new", 9)

    def test_failed_spatial_checkpoint_never_saves_objects(self):
        """Do not publish new semantics alongside an old spatial checkpoint."""
        source = Path(__file__).parent / "src/mapping_rbnx/atlas_bridge.py"
        hook = next(n for n in ast.parse(source.read_text()).body
                    if isinstance(n, ast.FunctionDef) and n.name == "_start_checkpoint_loop")
        stop = threading.Event()
        ns = dict(map_ops=map_ops, lifecycle=map_ops.lifecycle, threading=threading,
                  scene_snapshot=scene_snapshot, _CHECKPOINT_STOP=stop,
                  _CHECKPOINT_THREAD=None, _ACTIVE={"algo": "rtabmap"},
                  _active_db=lambda: "runtime.db", log=logging.getLogger(__name__))
        exec(compile(ast.Module(body=[hook], type_ignores=[]), str(source), "exec"), ns)
        with (patch.object(map_ops.lifecycle, "current", return_value={"map_id": "site", "generation": 1}),
              patch.object(map_ops, "get_mode_impl", return_value={"mode": "mapping"}),
              patch.object(scene_snapshot, "save_objects") as objects,
              patch.object(map_ops, "save_map_impl", side_effect=lambda *a, **kw: (stop.set() or {"ok": False}))):
            ns["_start_checkpoint_loop"]({"checkpoint_interval_s": 0.01}, "mapping")
            ns["_CHECKPOINT_THREAD"].join(timeout=2)
            self.assertFalse(ns["_CHECKPOINT_THREAD"].is_alive())
            objects.assert_not_called()


if __name__ == "__main__":
    unittest.main()
