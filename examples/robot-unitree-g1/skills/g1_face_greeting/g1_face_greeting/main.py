#!/usr/bin/env python3
"""Stationary G1 face greeting orchestration skill."""
from __future__ import annotations

import asyncio
import concurrent.futures
import json
import logging
import threading
import time
from collections import OrderedDict
from dataclasses import dataclass
from typing import Any

from robonix_api import ATLAS, Err, Ok, Skill
from robonix_api.atlas_types import Transport
from g1_face_greeting_mcp import GreetFace_Request, GreetFace_Response

log = logging.getLogger("g1_face_greeting")
FACE_EVENTS = "robonix/service/face_recognition/events/next"
SPEECH = "robonix/service/speech/speak"
ARM = "robonix/skill/g1_arm_actions/run"
ALLOWED_ACTIONS = {
    "wave_under_head",
    "wave_above_head",
    "make_heart_with_right_hand",
    "make_heart_with_both_hands",
}


@dataclass(frozen=True)
class Settings:
    face_provider_id: str
    speech_provider_id: str
    arm_provider_id: str
    navigation_enabled: bool
    turn_to_face_enabled: bool
    poll_wait_ms: int
    cooldown_sec: float
    actions: dict[int, str]

    @classmethod
    def parse(cls, raw: dict[str, Any]) -> "Settings":
        if not isinstance(raw, dict):
            raise ValueError("g1_face_greeting config must be an object")

        allowed = {
            "face_provider_id",
            "speech_provider_id",
            "arm_provider_id",
            "navigation_enabled",
            "turn_to_face_enabled",
            "poll_wait_ms",
            "cooldown_sec",
            *(f"action_level_{level}" for level in range(1, 6)),
        }
        unknown = sorted(set(raw) - allowed)
        if unknown:
            raise ValueError(f"unknown config keys: {', '.join(unknown)}")

        def text(name: str, default: str) -> str:
            value = str(raw.get(name, default)).strip()
            if not value:
                raise ValueError(f"{name} must not be empty")
            return value

        def flag(name: str) -> bool:
            value = raw.get(name, False)
            if not isinstance(value, bool):
                raise ValueError(f"{name} must be a boolean")
            return value

        poll = raw.get("poll_wait_ms", 1000)
        cooldown = raw.get("cooldown_sec", 30.0)
        if isinstance(poll, bool) or not isinstance(poll, int):
            raise ValueError("poll_wait_ms must be an integer")
        if isinstance(cooldown, bool) or not isinstance(cooldown, (int, float)):
            raise ValueError("cooldown_sec must be a number")
        cooldown = float(cooldown)
        if not 100 <= poll <= 5000:
            raise ValueError("poll_wait_ms must be in [100, 5000]")
        if not 0 <= cooldown <= 3600:
            raise ValueError("cooldown_sec must be in [0, 3600]")
        navigation = flag("navigation_enabled")
        turn = flag("turn_to_face_enabled")
        if navigation or turn:
            raise ValueError(
                "navigation_enabled and turn_to_face_enabled are reserved and must remain false"
            )
        actions = {}
        defaults = {
            1: "wave_under_head",
            2: "wave_under_head",
            3: "wave_above_head",
            4: "make_heart_with_right_hand",
            5: "make_heart_with_both_hands",
        }
        for level in range(1, 6):
            action = str(raw.get(f"action_level_{level}", defaults[level])).strip()
            if not action:
                raise ValueError(f"action_level_{level} must not be empty")
            if action not in ALLOWED_ACTIONS:
                raise ValueError(f"action_level_{level} is not allowlisted: {action}")
            actions[level] = action
        return cls(
            text("face_provider_id", "face_recognition"),
            text("speech_provider_id", "speech"),
            text("arm_provider_id", "g1_arm_actions"),
            navigation,
            turn,
            poll,
            cooldown,
            actions,
        )


_settings: Settings | None = None
_face_connection = None
_speech_connection = None
_arm_connection = None
_worker: threading.Thread | None = None
_stop = threading.Event()
_lock = threading.RLock()
_last_greet: dict[int, float] = {}
_seen_events: OrderedDict[str, None] = OrderedDict()


def _find(contract: str, provider_id: str):
    caps = ATLAS.find_capability(provider_id=provider_id, contract_id=contract, transport=Transport.MCP)
    if len(caps) != 1:
        raise RuntimeError(f"expected one {provider_id!r} provider for {contract}, found {len(caps)}")
    return caps[0]


def _result_value(result: Any) -> dict[str, Any]:
    structured = getattr(result, "structuredContent", None)
    if isinstance(structured, dict):
        return structured
    if not result.content:
        return {}
    for item in result.content:
        text = getattr(item, "text", "")
        if isinstance(text, str):
            try:
                value = json.loads(text)
            except json.JSONDecodeError:
                continue
            if isinstance(value, dict):
                return value
    if getattr(result, "isError", False):
        raise RuntimeError("MCP tool returned an error without a JSON result")
    return {}


async def _call(endpoint: str, tool: str, payload: dict[str, Any], timeout: float) -> dict[str, Any]:
    from fastmcp import Client

    async with Client(endpoint) as client:
        result = await asyncio.wait_for(client.call_tool(tool, payload), timeout=timeout)
        if getattr(result, "isError", False):
            raise RuntimeError(f"MCP tool {tool} returned an error: {_result_value(result)}")
        return _result_value(result)


def _call_sync(connection: Any, tool: str, payload: dict[str, Any], timeout: float) -> dict[str, Any]:
    endpoint = (connection.endpoint or "").strip()
    if not endpoint:
        raise RuntimeError(f"empty endpoint for {tool}")
    return asyncio.run(_call(endpoint, tool, payload, timeout))


def _timed_call(
    label: str,
    connection: Any,
    tool: str,
    payload: dict[str, Any],
    timeout: float,
) -> dict[str, Any]:
    started = time.monotonic()
    log.info("%s call started", label)
    try:
        return _call_sync(connection, tool, payload, timeout)
    finally:
        log.info("%s call finished in %.3fs", label, time.monotonic() - started)


def _valid_level(value: Any) -> int | None:
    if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= 5:
        return None
    return value


def _greet(event: dict[str, Any]) -> dict[str, Any]:
    assert _settings is not None
    try:
        person_id = int(event.get("person_id", -1))
    except (TypeError, ValueError):
        return {"accepted": False, "detail": "invalid person_id"}
    event_id = str(event.get("event_id", ""))
    level = _valid_level(event.get("intimacy_level", 1))
    if level is None:
        return {"accepted": False, "detail": "intimacy level must be an integer in [1, 5]"}
    now = time.monotonic()
    with _lock:
        if event_id and event_id in _seen_events:
            return {"accepted": False, "detail": "duplicate event"}
        if person_id >= 0 and now - _last_greet.get(person_id, -1e9) < _settings.cooldown_sec:
            return {"accepted": False, "detail": "greeting cooldown"}
        if event_id:
            _seen_events[event_id] = None
            if len(_seen_events) > 512:
                _seen_events.popitem(last=False)
        _last_greet[person_id] = now

    action = _settings.actions[level]
    speech_ok = False
    action_ok = False
    details: list[str] = []
    speech_text = str(event.get("hello", "您好！"))
    with concurrent.futures.ThreadPoolExecutor(
        max_workers=2,
        thread_name_prefix="g1-face-greeting",
    ) as executor:
        speech_future = executor.submit(
            _timed_call,
            "speech",
            _speech_connection,
            "speak",
            {"target": "", "text": speech_text},
            65.0,
        )
        action_future = executor.submit(
            _timed_call,
            "arm action",
            _arm_connection,
            "run",
            {"action": action},
            30.0,
        )
        try:
            speech = speech_future.result()
            speech_ok = bool(speech.get("ok", False))
            details.append(f"speech={speech.get('detail', 'ok' if speech_ok else 'failed')}")
        except Exception as exc:  # noqa: BLE001
            details.append(f"speech_error={exc}")
            log.exception("speech failed for person_id=%s", person_id)
        try:
            arm = action_future.result()
            action_ok = bool(arm.get("accepted", False))
            details.append(f"action={arm.get('detail', 'ok' if action_ok else 'failed')}")
        except Exception as exc:  # noqa: BLE001
            details.append(f"action_error={exc}")
            log.exception("arm action failed for person_id=%s", person_id)
    return {
        "accepted": True,
        "speech_ok": speech_ok,
        "action_ok": action_ok,
        "action": action,
        "detail": "; ".join(details),
    }


def _worker_loop() -> None:
    while not _stop.is_set():
        try:
            event = _call_sync(_face_connection, FACE_EVENTS.rsplit("/", 1)[-1], {"wait_ms": _settings.poll_wait_ms}, 8.0)
            if event.get("available"):
                log.info("face encounter: id=%s name=%s intimacy=%s", event.get("person_id"), event.get("name"), event.get("intimacy_level"))
                _greet(event)
        except Exception as exc:  # noqa: BLE001
            if not _stop.is_set():
                log.warning("face event polling failed: %s", exc)
                _stop.wait(1.0)


def _close_connection(connection: Any) -> None:
    if connection is not None:
        connection.close()


skill = Skill(id="g1_face_greeting", namespace="robonix/skill/g1_face_greeting")


@skill.on_init
def init(config: dict[str, Any]):
    global _settings
    try:
        _settings = Settings.parse(config)
        return Ok()
    except Exception as exc:  # noqa: BLE001
        return Err(str(exc))


@skill.on_activate
def activate():
    global _face_connection, _speech_connection, _arm_connection, _worker
    with _lock:
        if _settings is None:
            return Err("skill has not been initialized")
        if _worker is not None:
            return Err("skill is already active")
        try:
            _face_connection = skill.connect_capability(_find(FACE_EVENTS, _settings.face_provider_id), FACE_EVENTS, Transport.MCP)
            _speech_connection = skill.connect_capability(_find(SPEECH, _settings.speech_provider_id), SPEECH, Transport.MCP)
            _arm_connection = skill.connect_capability(_find(ARM, _settings.arm_provider_id), ARM, Transport.MCP)
            _last_greet.clear()
            _seen_events.clear()
            _stop.clear()
            _worker = threading.Thread(target=_worker_loop, name="g1-face-greeting", daemon=True)
            _worker.start()
            log.info("face greeting active in stationary mode")
            return Ok()
        except Exception as exc:  # noqa: BLE001
            deactivate()
            return Err(str(exc))


@skill.on_deactivate
def deactivate():
    global _face_connection, _speech_connection, _arm_connection, _worker
    with _lock:
        _stop.set()
        if _worker is not None and _worker is not threading.current_thread():
            _worker.join(timeout=10)
            if _worker.is_alive():
                return Err("greeting worker is still completing an in-flight MCP request")
        _worker = None
        connections = (_face_connection, _speech_connection, _arm_connection)
        _face_connection = _speech_connection = _arm_connection = None
        errors = []
        for connection in connections:
            try:
                _close_connection(connection)
            except Exception as exc:  # noqa: BLE001
                errors.append(str(exc))
        _last_greet.clear()
        _seen_events.clear()
        if errors:
            return Err("connection cleanup failed: " + "; ".join(errors))
        return Ok()


@skill.on_shutdown
def shutdown():
    return deactivate()


@skill.mcp("robonix/skill/g1_face_greeting/greet")
def greet(req: GreetFace_Request) -> GreetFace_Response:
    if _settings is None:
        return GreetFace_Response(accepted=False, detail="skill has not been initialized")
    event = {
        "event_id": str(req.event_id),
        "person_id": -1,
        "name": str(req.person_name),
        "hello": str(req.hello),
        "intimacy_level": int(req.intimacy_level),
    }
    result = _greet(event)
    return GreetFace_Response(
        accepted=bool(result.get("accepted", False)),
        speech_ok=bool(result.get("speech_ok", False)),
        action_ok=bool(result.get("action_ok", False)),
        action=str(result.get("action", "")),
        detail=str(result.get("detail", "")),
    )


if __name__ == "__main__":
    skill.run()
