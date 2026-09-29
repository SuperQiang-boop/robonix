# SPDX-License-Identifier: MulanPSL-2.0
"""RGB-D object removal regressions, without loading perception models."""

import asyncio
import os
import sys
import time
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from scene_service.ingest.perception_concept_graphs import ConceptGraphsDetector
from scene_service.ingest.perception_vlm import _CamIntrinsics
from scene_service.state.object_registry import ObjectRegistry


@pytest.fixture
def detector(monkeypatch):
    """Build a mapped cup with real registry reconciliation and stubbed models."""
    monkeypatch.delenv("SCENE_CG_DETECTION_INTERVAL", raising=False)
    monkeypatch.delenv("SCENE_CG_MISSING_DETECTIONS", raising=False)
    registry = ObjectRegistry()
    det = ConceptGraphsDetector(
        rgb_fetcher_msg=lambda: None,
        depth_fetcher_msg=lambda: None,
        camera_info_fetcher=lambda: None,
        on_detections=lambda _: None,
        registry=registry,
        world_frame_fn=lambda: "map",
        cfg_overrides={"detection_interval_ticks": 1},
    )
    pts = np.array([
        [x, y, z] for x in (-0.08, 0.08) for y in (-0.08, 0.08) for z in (1.9, 2.0)
    ])
    det._map_objects = [{
        "id": "cup-uuid", "class_name": "cup", "image_idx": [0],
        "num_detections": 1, "conf": [0.9],
        "pcd": SimpleNamespace(points=pts, colors=np.zeros_like(pts)), "bbox": object(),
    }]
    det._frame_observed_at[0] = time.time()
    monkeypatch.setattr(det, "_maybe_periodic_cleanup", lambda: None)
    return det


def _intrinsics():
    return _CamIntrinsics(width=100, height=100, fx=100, fy=100, cx=50, cy=50)


async def _finish(det, *, depth=None, transform=None, boxes=None):
    """Complete one valid round and drain the scheduled registry reconciliation."""
    det._asyncio_loop = asyncio.get_running_loop()
    det._tick_idx += 1
    det._finish_detection(
        _intrinsics(), np.eye(4) if transform is None else transform,
        (100, 100), np.full((100, 100), 3.0) if depth is None else depth,
        boxes or [],
    )
    await asyncio.sleep(0)
    await asyncio.sleep(0)


async def _seed(det):
    """Publish the initial map object through the production snapshot path."""
    det._asyncio_loop = asyncio.get_running_loop()
    det._project_to_registry()
    await asyncio.sleep(0)
    await asyncio.sleep(0)
    return next(iter(det._registry.all_objects()))


def test_visible_misses_remove_registry_and_cloud_without_resurrection(detector):
    """Three successful empty frames remove all representations immediately."""
    async def scenario():
        """Retain the original timestamp/count until deletion, then poll again."""
        obj = await _seed(detector)
        oid, seen, count = obj.object_id, obj.last_seen, obj.observation_count
        for _ in range(2):
            await _finish(detector)
            assert detector._registry.get_object(oid) is obj
            assert obj.last_seen == seen
            assert obj.observation_count == count
            assert len(detector.export_3d_snapshot()["objects"]) == 1
        await _finish(detector)
        assert detector._registry.get_object(oid) is None
        assert detector._map_objects == []
        assert detector._uuid_to_oid == {}
        assert detector.export_3d_snapshot()["objects"] == []
        await _finish(detector)
        assert list(detector._registry.all_objects()) == []

    asyncio.run(scenario())


def test_outside_view_resets_misses_and_survives_ttl(detector):
    """Turning away retains the object indefinitely without refreshing last_seen."""
    async def scenario():
        """Use an expired timestamp to prove retained map objects bypass TTL."""
        obj = await _seed(detector)
        detector._frame_observed_at[0] = obj.last_seen = time.time() - 100
        detector._registry.mark_stale(time.time())
        await _finish(detector)
        await _finish(detector)
        transform = np.eye(4)
        transform[0, 3] = 10
        for _ in range(5):
            await _finish(detector, transform=transform)
        assert detector._missed_detections == {}
        assert detector._registry.get_object(obj.object_id) is obj
        assert obj.missing
        assert obj.observation_count == 1
        await _finish(detector)
        assert detector._missed_detections == {"cup-uuid": 1}

    asyncio.run(scenario())


@pytest.mark.parametrize("depth_value", [0.0, float("nan"), float("inf"), 0.8])
def test_occlusion_and_invalid_depth_pause_misses(detector, depth_value):
    """Blocked or unmeasured surfaces cannot provide evidence of absence."""
    async def scenario():
        """Pause an existing streak, then resume it when the view clears."""
        await _seed(detector)
        await _finish(detector)
        for _ in range(5):
            await _finish(detector, depth=np.full((50, 50), depth_value))
        assert detector._missed_detections == {"cup-uuid": 1}
        assert len(detector._map_objects) == 1
        await _finish(detector)
        await _finish(detector)
        assert detector._map_objects == []

    asyncio.run(scenario())


def test_redetection_resets_misses_and_increments_count_once(detector):
    """A true new observation resumes the stable ID and resets its miss streak."""
    async def scenario():
        """Re-observe before the threshold, then synchronize the same map again."""
        obj = await _seed(detector)
        await _finish(detector)
        await _finish(detector)
        detector._map_objects[0]["image_idx"].append(detector._tick_idx + 1)
        detector._map_objects[0]["num_detections"] += 1
        await _finish(detector)
        assert detector._missed_detections == {}
        assert detector._registry.get_object(obj.object_id) is obj
        assert obj.observation_count == 2
        stamp = obj.last_seen
        detector._project_to_registry()
        await asyncio.sleep(0)
        await asyncio.sleep(0)
        assert obj.observation_count == 2
        assert obj.last_seen == stamp
        await _finish(detector)
        await _finish(detector)
        assert len(detector._map_objects) == 1

    asyncio.run(scenario())


def test_raw_detection_protects_object_when_3d_lifting_fails(detector):
    """A matching YOLO box is not treated as a miss when no new cloud is produced."""
    async def scenario():
        """Repeated protected rounds retain both the map and registry record."""
        obj = await _seed(detector)
        for _ in range(5):
            await _finish(detector, boxes=[("cup", [40, 40, 60, 60])])
        assert detector._missed_detections == {}
        assert detector._registry.get_object(obj.object_id) is obj

    asyncio.run(scenario())


def _image(kind, stamp):
    """Create a distinct stamped camera message with directly decodable bytes."""
    data = np.zeros((100, 100, 3), dtype=np.uint8) if kind == "rgb" else np.full(
        (100, 100), 3.0, dtype=np.float32,
    )
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp, nanosec=0)),
        height=100, width=100, data=data.tobytes(),
        encoding="bgr8" if kind == "rgb" else "32FC1",
    )


def _stub_camera(det, monkeypatch):
    """Use real decoding/tick flow with an empty YOLO result and fixed geometry."""
    # This path only imports torch; no tensor operation or inference engine is used.
    monkeypatch.setitem(sys.modules, "torch", SimpleNamespace())
    frames = [_image("rgb", 1), _image("depth", 1)]
    det._rgb_msg = lambda: frames[0]
    det._depth_msg = lambda: frames[1]
    det._cam_info = _intrinsics
    det._yolo = SimpleNamespace(predict=lambda *_, **__: [SimpleNamespace(boxes=[])])
    monkeypatch.setattr(det, "_build_camera_to_map_transform", lambda **_: np.eye(4))
    return frames


@pytest.mark.parametrize("frozen_channel", [0, 1])
def test_repeated_rgb_or_depth_does_not_count(detector, monkeypatch, frozen_channel):
    """Different message objects with unchanged camera stamps are repeated frames."""
    frames = _stub_camera(detector, monkeypatch)
    detector._tick_once()
    for stamp in range(2, 6):
        frames[0] = _image("rgb", 1 if frozen_channel == 0 else stamp)
        frames[1] = _image("depth", 1 if frozen_channel == 1 else stamp)
        detector._tick_once()
    assert detector._missed_detections == {"cup-uuid": 1}


@pytest.mark.parametrize("failure", ["exception", "no_results", "malformed", "geometry", "camera"])
def test_failed_rounds_do_not_count(detector, monkeypatch, failure):
    """Inference errors, absent inputs and unavailable geometry preserve objects."""
    frames = _stub_camera(detector, monkeypatch)
    if failure == "exception":
        def fail(*_, **__):
            raise RuntimeError("inference failed")
        detector._yolo.predict = fail
    elif failure == "no_results":
        detector._yolo.predict = lambda *_, **__: []
    elif failure == "malformed":
        detector._yolo.predict = lambda *_, **__: [SimpleNamespace(boxes=None)]
    elif failure == "geometry":
        monkeypatch.setattr(detector, "_build_camera_to_map_transform", lambda **_: None)
    else:
        frames[0] = None
    for stamp in range(2, 6):
        if failure != "camera":
            frames[0] = _image("rgb", stamp)
        frames[1] = _image("depth", stamp)
        detector._tick_once()
    assert detector._missed_detections == {}
    assert len(detector._map_objects) == 1


def test_skipped_inference_does_not_count(detector, monkeypatch):
    """Detection interval ticks do not count absence or inflate observations."""
    _stub_camera(detector, monkeypatch)
    detector.cfg["detection_interval_ticks"] = 3
    detector._tick_once()
    detector._tick_once()
    detector._tick_once()
    assert detector._missed_detections == {"cup-uuid": 1}


def test_three_fresh_empty_camera_frames_remove_object(detector, monkeypatch):
    """Exercise real camera decoding, zero-detection handling, and registry writes."""
    frames = _stub_camera(detector, monkeypatch)

    async def scenario():
        """Run fresh frame messages through the complete empty-detection tick path."""
        obj = await _seed(detector)
        for stamp in range(1, 4):
            frames[:] = [_image("rgb", stamp), _image("depth", stamp)]
            detector._tick_once()
            await asyncio.sleep(0)
            await asyncio.sleep(0)
        assert detector._registry.get_object(obj.object_id) is None
        assert detector.export_3d_snapshot()["objects"] == []

    asyncio.run(scenario())


def test_pruning_removes_published_relations_but_keeps_robot_endpoint(tmp_path):
    """Deleted objects disappear from both graph slices, including cached edges."""
    from scene_service.scene_graph.store import SceneGraphStore
    from scene_service.scene_graph.types import SceneGraphEdge

    store = SceneGraphStore(cache_dir=tmp_path)
    keep = SceneGraphEdge("table", "robot", "reachable_by", 0.9)
    remove = SceneGraphEdge("cup", "table", "on_top_of", 0.9)
    store.set_geometric([], [keep])
    store.set_semantic_edges([remove])
    store._relation_cache["cup__table"] = remove
    assert store.prune_relations({"table", "robot"}) == 1
    assert store.get_snapshot().edges == [keep]


def test_removal_is_reflected_in_both_web_apis(detector, tmp_path):
    """Publish perception removal through actual ASGI 2D/3D and relation responses."""
    import httpx

    from scene_service import web
    from scene_service.scene_graph.store import SceneGraphStore
    from scene_service.scene_graph.types import SceneGraphEdge
    from scene_service.state.object_registry import BBox3D, Pose3D

    async def scenario():
        """Keep a robot and stale graph edge to verify endpoint-aware filtering."""
        cup = await _seed(detector)
        robot = detector._registry.insert_object(
            cls="robot", pose=Pose3D(0, 0, 0, frame_id="map"),
            bbox=BBox3D(frame_id="map"), confidence=1, now=time.time(), source="self",
        )
        edge = SceneGraphEdge(cup.object_id, robot.object_id, "reachable_by", 0.9)
        store = SceneGraphStore(cache_dir=tmp_path)
        store.set_geometric([], [edge])
        store.set_semantic_edges([edge])
        app = web.make_app(registry=detector._registry, detector=detector, sg_store=store)
        async with httpx.AsyncClient(
            transport=httpx.ASGITransport(app=app), base_url="http://scene.test",
        ) as client:
            before = (await client.get("/api/state")).json()
            assert len(before["objects"]) == 2
            assert len(before["relations"]) == 1
            assert len(before["scene_graph"]["edges"]) == 1
            assert len((await client.get("/api/objects3d")).json()["objects"]) == 1
            for _ in range(3):
                await _finish(detector)
            after = (await client.get("/api/state")).json()
            assert [o["id"] for o in after["objects"]] == [robot.object_id]
            assert after["relations"] == []
            assert after["scene_graph"]["edges"] == []
            assert (await client.get("/api/objects3d")).json()["objects"] == []

    asyncio.run(scenario())
