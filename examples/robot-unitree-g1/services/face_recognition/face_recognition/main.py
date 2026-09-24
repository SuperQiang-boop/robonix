"""Atlas/lifecycle bridge for the external VSIS face recognition executable."""
from __future__ import annotations

import asyncio
import json
import logging
import os
import subprocess
import threading
import time
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

from robonix_api import ATLAS, Err, Ok, Service
from robonix_api.atlas_types import Transport

log = logging.getLogger("face_recognition")
service = Service(id="face_recognition", namespace="robonix/service/face_recognition")
SPEAK = "robonix/service/speech/speak"


@dataclass(frozen=True)
class Settings:
    external_root: Path
    binary_path: Path
    startup_timeout: float
    speak_host: str
    speak_port: int
    speech_provider_id: str | None
    ready_marker: str

    @classmethod
    def parse(cls, raw: dict[str, Any]) -> "Settings":
        if not isinstance(raw, dict):
            raise ValueError("face_recognition config must be an object")
        allowed = {"external_root", "binary_path", "startup_timeout", "speak_host", "speak_port", "speech_provider_id", "ready_marker"}
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
        timeout = float(raw.get("startup_timeout", 45.0))
        if not 1.0 <= timeout <= 300.0:
            raise ValueError("startup_timeout must be in [1, 300]")
        host = str(raw.get("speak_host", "127.0.0.1")).strip()
        port = int(raw.get("speak_port", 10086))
        if not host or not 1 <= port <= 65535:
            raise ValueError("speak_host and speak_port are invalid")
        provider = raw.get("speech_provider_id")
        provider = str(provider).strip() if provider is not None else None
        marker = str(raw.get("ready_marker", "特征库就绪"))
        if not marker:
            raise ValueError("ready_marker must be nonempty")
        if not root.is_dir():
            raise ValueError(f"external_root does not exist: {root}")
        if not binary.is_file() or not os.access(binary, os.X_OK):
            raise ValueError(f"external binary is not executable: {binary}")
        return cls(root, binary, timeout, host, port, provider, marker)


_settings: Settings | None = None
_process: subprocess.Popen[str] | None = None
_proxy: "SpeechProxy | None" = None
_speech_channel = None
_ready = threading.Event()
_lock = threading.RLock()


def _find_speech() -> Any:
    kwargs = {"contract_id": SPEAK, "transport": Transport.MCP}
    if _settings and _settings.speech_provider_id:
        kwargs["provider_id"] = _settings.speech_provider_id
    return ATLAS.find_unique_capability(**kwargs)


async def _call_speech(endpoint: str, text: str) -> dict[str, Any]:
    from fastmcp import Client
    async with Client(endpoint) as client:
        result = await client.call_tool("speak", {"target": "", "text": text})
        if not result.content:
            return {"ok": False, "detail": "speech returned no content"}
        try:
            value = json.loads(result.content[0].text)
        except (TypeError, ValueError) as exc:
            return {"ok": False, "detail": f"invalid speech response: {exc}"}
        return value if isinstance(value, dict) else {"ok": False, "detail": "invalid speech response object"}


class _SpeechHandler(BaseHTTPRequestHandler):
    def do_POST(self):  # noqa: N802
        """Forward a greeting once; a disconnected caller must never trigger a retry."""
        if self.path != "/speak":
            self.send_error(404, "unknown endpoint")
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 65536:
                raise ValueError("invalid request size")
            payload = json.loads(self.rfile.read(length))
            if not isinstance(payload, dict) or not isinstance(payload.get("text"), str):
                raise ValueError("text must be a string")
            text = payload["text"].strip()
            if not text:
                raise ValueError("text is required")
        except (ValueError, TypeError) as exc:
            self._respond(400, {"ok": False, "detail": str(exc)})
            return
        try:
            result = self.server.proxy.speak(text)
            status = 200 if result.get("ok") is True else 502
            log.info("speech result: %s", result)
        except (TimeoutError, asyncio.TimeoutError):
            status, result = 504, {"ok": False, "detail": "speech exceeded 60s; playback outcome unknown"}
            log.warning("speech timed out; do not automatically retry playback")
        except Exception as exc:
            log.exception("speech upstream request failed")
            status, result = 502, {"ok": False, "detail": str(exc)}
        self._respond(status, result)

    def _respond(self, status, result):
        """Write one response, reporting disconnects without writing a second response."""
        body = json.dumps(result, ensure_ascii=False).encode("utf-8")
        try:
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            log.warning("greeting caller disconnected; response status=%s, result=%s", status, result)

    def log_message(self, fmt, *args):
        log.debug("speech proxy: " + fmt, *args)


class SpeechProxy:
    def __init__(self, host: str, port: int, endpoint: str):
        self.endpoint = endpoint
        self.server = ThreadingHTTPServer((host, port), _SpeechHandler)
        self.server.proxy = self  # type: ignore[attr-defined]
        self.thread = threading.Thread(target=self.server.serve_forever, name="face-speech-proxy", daemon=True)

    def start(self) -> None:
        self.thread.start()
        log.info("speech compatibility proxy listening on %s:%d", *self.server.server_address)

    def speak(self, text: str) -> dict[str, Any]:
        async def bounded_call():
            return await asyncio.wait_for(_call_speech(self.endpoint, text), timeout=60)
        return asyncio.run(bounded_call())

    def close(self) -> None:
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=3)


def _drain_output(proc: subprocess.Popen[str], marker: str) -> None:
    assert proc.stdout is not None
    for line in proc.stdout:
        line = line.rstrip()
        if line:
            log.info("vsis: %s", line)
        if marker in line:
            _ready.set()
    if proc.poll() is not None and not _ready.is_set():
        log.error("VSIS process exited before ready: rc=%s", proc.returncode)


def _cleanup() -> None:
    global _process, _proxy, _speech_channel
    if _process is not None:
        if _process.poll() is None:
            _process.terminate()
            try:
                _process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                _process.kill()
                _process.wait(timeout=3)
        _process = None
    if _proxy is not None:
        _proxy.close()
        _proxy = None
    if _speech_channel is not None:
        _speech_channel.close()
        _speech_channel = None
    _ready.clear()


@service.on_init
def init(config: dict[str, Any]):
    global _settings
    try:
        _settings = Settings.parse(config)
        _find_speech()
        return Ok()
    except Exception as exc:  # noqa: BLE001
        return Err(str(exc))


@service.on_activate
def activate():
    global _process, _proxy, _speech_channel
    with _lock:
        if _settings is None:
            return Err("service has not been initialized")
        if _process is not None:
            return Err("service is already active")
        try:
            cap = _find_speech()
            _speech_channel = service.connect_capability(cap, SPEAK, Transport.MCP)
            endpoint = _speech_channel.endpoint
            if not endpoint:
                raise RuntimeError("speech capability returned an empty endpoint")
            _proxy = SpeechProxy(_settings.speak_host, _settings.speak_port, endpoint)
            _proxy.start()
            env = os.environ.copy()
            env.setdefault("LD_LIBRARY_PATH", str(_settings.external_root / "bin"))
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
