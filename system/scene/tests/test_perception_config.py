# SPDX-License-Identifier: MulanPSL-2.0
"""Configuration precedence and validation for visible-object removal."""

import os
import sys
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from scene_service.ingest.perception_concept_graphs import (
    ConceptGraphsDetector,
    _configured_missing_detections,
)
from scene_service.state.object_registry import ObjectRegistry


@pytest.fixture(autouse=True)
def clean_environment(monkeypatch):
    """Isolate the configuration source from the surrounding deployment."""
    monkeypatch.delenv("ROBONIX_DEPLOY_DIR", raising=False)
    monkeypatch.delenv("SCENE_CG_MISSING_DETECTIONS", raising=False)


def _manifest(tmp_path, monkeypatch, scene):
    """Write the same deployment structure selected by the real launcher."""
    (tmp_path / "robonix_manifest.yaml").write_text(
        yaml.safe_dump({"system": {"scene": scene}}), encoding="utf-8",
    )
    monkeypatch.setenv("ROBONIX_DEPLOY_DIR", str(tmp_path))


def test_deployment_threshold_overrides_environment(tmp_path, monkeypatch):
    """The Scene block remains authoritative even when an old env value exists."""
    _manifest(tmp_path, monkeypatch, {"SCENE_CG_MISSING_DETECTIONS": 5})
    monkeypatch.setenv("SCENE_CG_MISSING_DETECTIONS", "99")
    assert _configured_missing_detections({}) == 5


def test_explicit_scene_config_overrides_deployment(tmp_path, monkeypatch):
    """A materialized Scene config preserves the established override mechanism."""
    _manifest(tmp_path, monkeypatch, {"SCENE_CG_MISSING_DETECTIONS": 5})
    assert _configured_missing_detections({"SCENE_CG_MISSING_DETECTIONS": 7}) == 7


def test_environment_and_default_remain_available(tmp_path, monkeypatch):
    """Standalone Scene and deployments without the field retain compatible defaults."""
    assert _configured_missing_detections({}) == 3
    _manifest(tmp_path, monkeypatch, {})
    monkeypatch.setenv("SCENE_CG_MISSING_DETECTIONS", "4")
    assert _configured_missing_detections({}) == 4


@pytest.mark.parametrize("bad", [0, -1, True, False, 1.5, None, "", "abc", "3.5"])
def test_invalid_deployment_threshold_stops_startup(tmp_path, monkeypatch, bad):
    """Explicit mistakes cannot silently select an environment fallback."""
    _manifest(tmp_path, monkeypatch, {"SCENE_CG_MISSING_DETECTIONS": bad})
    monkeypatch.setenv("SCENE_CG_MISSING_DETECTIONS", "3")
    with pytest.raises(ValueError, match="must be a positive integer"):
        _configured_missing_detections({})


@pytest.mark.parametrize("contents", ["[", "[]", "system: []", "system: {scene: []}"])
def test_invalid_deployment_document_stops_startup(tmp_path, monkeypatch, contents):
    """Unreadable YAML or invalid nesting is reported rather than ignored."""
    (tmp_path / "robonix_manifest.yaml").write_text(contents, encoding="utf-8")
    monkeypatch.setenv("ROBONIX_DEPLOY_DIR", str(tmp_path))
    with pytest.raises((RuntimeError, ValueError)):
        _configured_missing_detections({})


def test_missing_deployment_manifest_stops_startup(tmp_path, monkeypatch):
    """An explicitly selected deployment must have a readable manifest."""
    monkeypatch.setenv("ROBONIX_DEPLOY_DIR", str(tmp_path))
    with pytest.raises(RuntimeError, match="failed to read Scene deployment config"):
        _configured_missing_detections({})


def test_detector_uses_explicit_config_before_env(monkeypatch):
    """Exercise the constructor's real precedence after other env knobs are applied."""
    monkeypatch.setenv("SCENE_CG_MISSING_DETECTIONS", "99")
    detector = ConceptGraphsDetector(
        rgb_fetcher_msg=lambda: None, depth_fetcher_msg=lambda: None,
        camera_info_fetcher=lambda: None, on_detections=lambda _: None,
        registry=ObjectRegistry(), cfg_overrides={"missing_detection_limit": 5},
    )
    assert detector.cfg["missing_detection_limit"] == 5


def test_g1_manifest_threshold_reaches_detector(monkeypatch):
    """Load the actual G1 Scene field and build a detector from that deployment."""
    root = (Path(__file__).resolve().parent.parent / ".." / "..").resolve()
    deployment = root / "examples" / "robot-unitree-g1"
    if not (deployment / "robonix_manifest.yaml").is_file():
        pytest.skip("G1 deployment is unavailable in this Scene-only checkout")
    manifest = yaml.safe_load((deployment / "robonix_manifest.yaml").read_text())
    expected = manifest["system"]["scene"]["SCENE_CG_MISSING_DETECTIONS"]
    monkeypatch.setenv("ROBONIX_DEPLOY_DIR", str(deployment))
    monkeypatch.setenv("SCENE_CG_MISSING_DETECTIONS", "99")
    detector = ConceptGraphsDetector(
        rgb_fetcher_msg=lambda: None, depth_fetcher_msg=lambda: None,
        camera_info_fetcher=lambda: None, on_detections=lambda _: None,
        registry=ObjectRegistry(),
    )
    assert detector.cfg["missing_detection_limit"] == expected == 3
