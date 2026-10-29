# SPDX-License-Identifier: MulanPSL-2.0
"""Exercise automatic checkpoint snapshots and delayed lifecycle binding."""
import asyncio
import contextlib
import tempfile
from types import SimpleNamespace

from test_map_epoch import (
    _make_env, _obj, _call, _patched, _unavailable, FakeObjectStore,
)
from test_lifecycle_flush import _service_mod, FakeHub
from scene_service.map_binding import MapBinding
from scene_service.map_meta import MapMetaStore, make_meta
from scene_service.state.object_registry import ObjectRegistry


def _checkpoint_env(tmp):
    """Create an active mapping frame without reaching the real map service."""
    return _make_env(
        tmp, binding={"map_id": "scout2_main", "mode": "mapping",
                      "generation": 4, "source": "lifecycle"},
        objs=[_obj("saved_chair", cls="chair")], anno_map_id="scout2_main",
        lifecycle=SimpleNamespace(map_id="scout2_main", generation=4, mode="mapping"),
    )


def _no_rpc(*args, **kwargs):
    """Checkpoint notifications must never call back into mapping."""
    raise AssertionError("automatic Scene snapshot called mapping recursively")


def test_automatic_checkpoint_saves_rotates_and_queries_objects():
    """Repeated geometry saves commit fresh snapshots accessible by map_id."""
    if _unavailable():
        return
    with tempfile.TemporaryDirectory() as tmp:
        web, app, env = _checkpoint_env(tmp)
        request = {"map_id": "scout2_main", "generation": 4, "spatial_saved": True}
        with _patched(web, rpc=_no_rpc, maps=_no_rpc):
            for seq in (1, 2):
                status, body = _call(app, "POST", "/api/maps/save", request)
                assert status == 200, body
                assert body["objects"] == 1
                assert body["snapshot_partition"] == f"scout2_main__s{seq}"
                assert env.meta.read("scout2_main").save_seq == seq
                assert env.binding["source"] == "lifecycle"
            assert "scout2_main__s1" not in env.store.rows
            # Live detections must not replace the saved map's object list.
            env.store.rows["scout2_main"] = {"new_scan": _obj("new_scan")}
            status, body = _call(app, "GET", "/api/maps/scout2_main/objects")
            assert status == 200, body
            assert [obj["id"] for obj in body["objects"]] == ["saved_chair"]


def test_checkpoint_rejects_foreign_and_stale_frames():
    """A wrong map/generation or invalid completion flag cannot change a snapshot."""
    if _unavailable():
        return
    with tempfile.TemporaryDirectory() as tmp:
        web, app, env = _checkpoint_env(tmp)
        with _patched(web, rpc=_no_rpc, maps=_no_rpc):
            for payload, code in [
                ({"map_id": "other", "generation": 4, "spatial_saved": True}, 409),
                ({"map_id": "scout2_main", "generation": 3, "spatial_saved": True}, 409),
                ({"map_id": "scout2_main", "spatial_saved": True}, 400),
                ({"map_id": "scout2_main", "generation": True, "spatial_saved": True}, 400),
                ({"map_id": "scout2_main", "spatial_saved": "true"}, 400),
            ]:
                status, body = _call(app, "POST", "/api/maps/save", payload)
                assert status == code, body
            assert env.meta.read("scout2_main") is None
            assert env.store.rows == {}


def test_failed_automatic_snapshot_preserves_previous_and_can_retry():
    """Incomplete writes keep the old sidecar; the next checkpoint can recover."""
    if _unavailable():
        return
    with tempfile.TemporaryDirectory() as tmp:
        web, app, env = _checkpoint_env(tmp)
        payload = {"map_id": "scout2_main", "generation": 4, "spatial_saved": True}
        with _patched(web, rpc=_no_rpc, maps=_no_rpc):
            assert _call(app, "POST", "/api/maps/save", payload)[0] == 200
            persist = env.store.persist
            env.store.persist = lambda *a, **kw: 0
            status, body = _call(app, "POST", "/api/maps/save", payload)
            assert status == 502 and not body["ok"], body
            assert env.meta.read("scout2_main").save_seq == 1
            env.store.persist = persist
            assert _call(app, "POST", "/api/maps/save", payload)[0] == 200
            assert env.meta.read("scout2_main").save_seq == 2


def test_lifecycle_change_during_snapshot_does_not_commit():
    """A map switch while rows are writing cannot publish a mismatched sidecar."""
    if _unavailable():
        return
    with tempfile.TemporaryDirectory() as tmp:
        web, app, env = _checkpoint_env(tmp)
        persist = env.store.persist

        def switch_during_write(*args, **kwargs):
            """Model a reset during the asynchronous database write."""
            env.hub.lifecycle.generation = 5
            return persist(*args, **kwargs)

        env.store.persist = switch_during_write
        with _patched(web, rpc=_no_rpc, maps=_no_rpc):
            status, body = _call(app, "POST", "/api/maps/save", {
                "map_id": "scout2_main", "generation": 4, "spatial_saved": True,
            })
        assert status == 502 and not body["ok"], body
        assert env.meta.read("scout2_main") is None


def test_default_binding_follows_mapping_and_restores_only_localization_snapshot():
    """Late startup discovery follows real map IDs and restores the saved partition."""
    service = _service_mod()
    if service is None:
        return

    async def scenario(tmp):
        """Mapping starts empty; a later localization load restores saved objects."""
        registry = ObjectRegistry()
        registry._objects["fallback"] = _obj("fallback")
        store = FakeObjectStore()
        meta = MapMetaStore(tmp)
        meta.write(make_meta("scout2_main", "scout2_main__s1", 1, generation=4))
        store.rows["scout2_main__s1"] = {"old_build": _obj("old_build")}
        meta.write(make_meta("floor_b", "floor_b__s2", 2, generation=9))
        store.rows["floor_b__s2"] = {"saved_chair": _obj("saved_chair")}
        store.rows["floor_b"] = {"recent_scan": _obj("recent_scan")}
        live = {"map_id": "default", "source": "default", "mode": "", "generation": None}
        hold = {"reason": "mapping switched before startup completed"}
        hub = FakeHub()
        task = asyncio.create_task(service._lifecycle_watch(
            hub, MapBinding(map_id="default", source="default"),
            registry=registry, object_store=store, map_meta=meta,
            restore_on_start=True, live_binding=live, semantic_hold=hold,
            interval_s=0.005,
        ))
        try:
            hub.set("scout2_main", 4, "mapping")
            await asyncio.sleep(0.08)
            assert live == {"map_id": "scout2_main", "source": "lifecycle",
                            "mode": "mapping", "generation": 4}, live
            assert not registry._objects
            assert hold["reason"] is None
            assert "scout2_main__s1" in store.rows
            hub.set("floor_b", 9, "localization")
            await asyncio.sleep(0.08)
            assert live["map_id"] == "floor_b", live
            assert set(registry._objects) == {"saved_chair"}
            assert "floor_b" not in store.rows
            assert hold["reason"] is None
        finally:
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await task

    with tempfile.TemporaryDirectory() as tmp:
        asyncio.run(scenario(tmp))


def test_confirmed_startup_does_not_restore_another_generation():
    """Restarting mapping or localization with a rebuilt map never restores old objects."""
    service = _service_mod()
    if service is None:
        return

    async def scenario(tmp):
        """Drive the initial same-map confirmation with an older saved snapshot."""
        registry = ObjectRegistry()
        store = FakeObjectStore()
        meta = MapMetaStore(tmp)
        meta.write(make_meta("lab", "lab__s1", 1, generation=1))
        store.rows["lab__s1"] = {"stale": _obj("stale")}
        hub = FakeHub()
        hub.set("lab", 2, "localization")
        live = {"map_id": "lab", "source": "lifecycle", "generation": 2, "mode": "localization"}
        task = asyncio.create_task(service._lifecycle_watch(
            hub, MapBinding(map_id="lab", source="lifecycle", generation=2),
            registry=registry, object_store=store, map_meta=meta,
            restore_on_start=True, live_binding=live, interval_s=0.005,
        ))
        try:
            await asyncio.sleep(0.08)
            assert live["generation"] == 2
            assert not registry._objects
        finally:
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await task

    with tempfile.TemporaryDirectory() as tmp:
        asyncio.run(scenario(tmp))
