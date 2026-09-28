"""Atlas/lifecycle bridge for the external VSIS face recognition executable."""
from __future__ import annotations

import json
import logging
import math
import os
import subprocess
import tempfile
import threading
import time
import queue
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from robonix_api import Err, Ok, Service
from face_recognition_mcp import (
    GetFaceRecognitionEvent_Request,
    GetFaceRecognitionEvent_Response,
)

log = logging.getLogger("face_recognition")
service = Service(id="face_recognition", namespace="robonix/service/face_recognition")


@dataclass(frozen=True)
class Settings:
    external_root: Path
    binary_path: Path
    startup_timeout: float
    ready_marker: str
    vsis_config: dict[str, Any]

    @classmethod
    def parse(cls, raw: dict[str, Any]) -> "Settings":
        if not isinstance(raw, dict):
            raise ValueError("face_recognition config must be an object")
        allowed = {"external_root", "binary_path", "startup_timeout", "ready_marker", "log", "video", "face", "audio"}
        unknown = sorted(set(raw) - allowed)
        if unknown:
            raise ValueError(f"unknown config keys: {', '.join(unknown)}")
        bundled_root = Path(__file__).resolve().parent.parent / "vsis_video_service"
        root_value = str(raw.get("external_root", bundled_root)).strip()
        if not root_value:
            raise ValueError("external_root must be nonempty")
        root = Path(root_value).expanduser().resolve()
        binary = Path(str(raw.get("binary_path", root / "bin" / "vsis_video_service"))).expanduser()
        if not binary.is_absolute():
            binary = (root / binary).resolve()
        timeout = _number(raw.get("startup_timeout", 45.0), "startup_timeout")
        if not 1.0 <= timeout <= 300.0:
            raise ValueError("startup_timeout must be in [1, 300]")
        marker = str(raw.get("ready_marker", "FACE_READY"))
        if not marker:
            raise ValueError("ready_marker must be nonempty")
        if not root.is_dir():
            raise ValueError(f"external_root does not exist: {root}")
        if not binary.is_file() or not os.access(binary, os.X_OK):
            raise ValueError(f"external binary is not executable: {binary}")
        vsis_config = {
            "log": _section(raw, "log", {"mode": 2, "level": 1}),
            "video": _section(raw, "video", {"source": "", "reconnect_interval_ms": 3000, "width": 1280, "height": 720}),
            "face": _section(raw, "face", {
                "search_threshold": 0.45, "greet_cooldown_sec": 30, "max_faces": 5,
                "confirm_hits": 1, "absent_sec": 3,
                "intimacy_upgrade_encounters": [5, 15, 30, 60],
            }),
            "audio": _section(raw, "audio", {"host": "127.0.0.1", "port": 10086}),
        }
        log_config, video, face, audio = (vsis_config[name] for name in ("log", "video", "face", "audio"))
        _integer(log_config["mode"], "log.mode", 0, 2)
        _integer(log_config["level"], "log.level", 0, 5)
        if not isinstance(video["source"], str):
            raise ValueError("video.source must be a string")
        _integer(video["reconnect_interval_ms"], "video.reconnect_interval_ms", 1)
        _integer(video["width"], "video.width", 0)
        _integer(video["height"], "video.height", 0)
        threshold = _number(face["search_threshold"], "face.search_threshold")
        if not 0.45 <= threshold <= 0.5:
            raise ValueError("face.search_threshold must be in [0.45, 0.5]")
        _integer(face["greet_cooldown_sec"], "face.greet_cooldown_sec", 0)
        _integer(face["max_faces"], "face.max_faces", 1)
        _integer(face["confirm_hits"], "face.confirm_hits", 1)
        _integer(face["absent_sec"], "face.absent_sec", 1)
        encounters = face["intimacy_upgrade_encounters"]
        if not isinstance(encounters, list) or len(encounters) != 4:
            raise ValueError("face.intimacy_upgrade_encounters must contain four thresholds for levels 2–5")
        previous = 0
        for index, count in enumerate(encounters, start=2):
            _integer(count, f"face.intimacy_upgrade_encounters level {index}", previous + 1)
            previous = count
        if not isinstance(audio["host"], str) or not audio["host"]:
            raise ValueError("audio.host must be a nonempty string")
        _integer(audio["port"], "audio.port", 1, 65535)
        return cls(root, binary, timeout, marker, vsis_config)


def _section(raw: dict[str, Any], name: str, defaults: dict[str, Any]) -> dict[str, Any]:
    overrides = raw.get(name, {})
    if not isinstance(overrides, dict):
        raise ValueError(f"{name} must be an object")
    unknown = sorted(set(overrides) - defaults.keys())
    if unknown:
        raise ValueError(f"unknown {name} config keys: {', '.join(unknown)}")
    return {**defaults, **overrides}


def _number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return float(value)


def _integer(value: Any, name: str, minimum: int, maximum: int | None = None) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum or (maximum is not None and value > maximum):
        raise ValueError(f"{name} must be an integer in [{minimum}, {maximum if maximum is not None else '∞'})")


_settings: Settings | None = None
_process: subprocess.Popen[str] | None = None
_config_dir: tempfile.TemporaryDirectory[str] | None = None
_ready = threading.Event()
_lock = threading.RLock()
_events: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=64)


def _drain_output(proc: subprocess.Popen[str], marker: str) -> None:
    assert proc.stdout is not None
    for line in proc.stdout:
        line = line.rstrip()
        if line:
            log.info("vsis: %s", line)
        if "FACE_EVENT " in line:
            payload = line.split("FACE_EVENT ", 1)[1].strip()
            try:
                event = json.loads(payload)
                if event.get("event_type") == "face_encounter":
                    try:
                        _events.put_nowait(event)
                    except queue.Full:
                        log.warning("face event queue full; dropping oldest event")
                        try:
                            _events.get_nowait()
                            _events.task_done()
                        except queue.Empty:
                            pass
                        _events.put_nowait(event)
            except json.JSONDecodeError:
                log.warning("invalid FACE_EVENT payload: %s", payload)
        if marker in line or line.startswith("FACE_READY"):
            _ready.set()
    if proc.poll() is not None and not _ready.is_set():
        log.error("VSIS process exited before ready: rc=%s", proc.returncode)


def _cleanup() -> None:
    global _process, _config_dir
    if _process is not None:
        if _process.poll() is None:
            _process.terminate()
            try:
                _process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                _process.kill()
                _process.wait(timeout=3)
        _process = None
    if _config_dir is not None:
        _config_dir.cleanup()
        _config_dir = None
    _ready.clear()
    while True:
        try:
            _events.get_nowait()
            _events.task_done()
        except queue.Empty:
            break


@service.mcp("robonix/service/face_recognition/events/next")
def next_event(req: GetFaceRecognitionEvent_Request) -> GetFaceRecognitionEvent_Response:
    wait_ms = max(0, min(int(req.wait_ms), 5000))
    try:
        event = _events.get(timeout=wait_ms / 1000.0 if wait_ms else 0)
    except queue.Empty:
        return GetFaceRecognitionEvent_Response(
            available=False,
            detail="no face encounter event available",
        )
    finally:
        # task_done is called after the event is copied below for the success path.
        pass
    try:
        return GetFaceRecognitionEvent_Response(
            available=True,
            event_id=str(event.get("event_id", "")),
            person_id=int(event.get("person_id", -1)),
            name=str(event.get("name", "")),
            hello=str(event.get("hello", "")),
            confidence=float(event.get("confidence", 0.0)),
            encounter_count=int(event.get("encounter_count", 0)),
            base_intimacy_level=int(event.get("base_intimacy_level", 1)),
            intimacy_level=int(event.get("intimacy_level", 1)),
            timestamp=str(event.get("timestamp", "")),
            detail="ok",
        )
    finally:
        _events.task_done()


@service.on_init
def init(config: dict[str, Any]):
    global _settings
    try:
        _settings = Settings.parse(config)
        return Ok()
    except Exception as exc:  # noqa: BLE001
        return Err(str(exc))


@service.on_activate
def activate():
    global _process, _config_dir
    with _lock:
        if _settings is None:
            return Err("service has not been initialized")
        if _process is not None:
            return Err("service is already active")
        try:
            _config_dir = tempfile.TemporaryDirectory(prefix="robonix-face-")
            config_path = Path(_config_dir.name) / "cfg.yaml"
            config_path.write_text(json.dumps(_settings.vsis_config, ensure_ascii=False, allow_nan=False), encoding="utf-8")
            env = os.environ.copy()
            env.setdefault("LD_LIBRARY_PATH", str(_settings.external_root / "bin"))
            env["VSIS_CONFIG_PATH"] = str(config_path)
            _process = subprocess.Popen(
                [str(_settings.binary_path)], cwd=_settings.external_root, env=env,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                encoding="utf-8", errors="replace", bufsize=1,
            )
            _ready.clear()
            threading.Thread(target=_drain_output, args=(_process, _settings.ready_marker), daemon=True).start()
            deadline = time.monotonic() + _settings.startup_timeout
            while time.monotonic() < deadline:
                if _ready.wait(timeout=0.25):
                    log.info("VSIS face recognition ready")
                    return Ok()
                if _process.poll() is not None:
                    raise RuntimeError(f"VSIS process exited during startup (rc={_process.returncode})")
            raise TimeoutError(f"VSIS ready marker not observed within {_settings.startup_timeout:.1f}s")
        except Exception as exc:  # noqa: BLE001
            log.exception("face recognition activation failed")
            _cleanup()
            return Err(str(exc))


@service.on_deactivate
def deactivate():
    with _lock:
        try:
            _cleanup()
            return Ok()
        except Exception as exc:  # noqa: BLE001
            return Err(f"face recognition cleanup failed: {exc}")


@service.on_shutdown
def shutdown():
    return deactivate()


if __name__ == "__main__":
    service.run()
