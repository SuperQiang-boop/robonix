#!/usr/bin/env python3
"""Stationary G1 face greeting orchestration skill."""
from __future__ import annotations

import asyncio
import concurrent.futures
import json
import logging
import math
import sys
import threading
import time
from collections import OrderedDict
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from robonix_api import SRC, Err, Ok, Skill
from robonix_api.src_types import Transport
from g1_face_greeting_mcp import (
    CancelFaceMessage_Request, CancelFaceMessage_Response,
    CreateFaceMessage_Request, CreateFaceMessage_Response,
    CreateObjectFaceMessage_Request, CreateObjectFaceMessage_Response,
    FaceMessageStatus_Request, FaceMessageStatus_Response,
    GreetFace_Request, GreetFace_Response,
    ListFaceMessages_Request, ListFaceMessages_Response,
)
from g1_face_greeting.messages import MessageStore
from g1_face_greeting import coordination

log = logging.getLogger("g1_face_greeting")
FACE_EVENTS = "robonix/service/face_recognition/events/next"
FACE_RESOLVE = "robonix/service/face_recognition/people/resolve"
WORLD_OBJECTS = "robonix/system/world/list_objects"
SPEECH = "robonix/service/speech/speak"
ARM = "robonix/skill/g1_arm_actions/run"
NAVIGATION = "robonix/service/navigation/navigate"
# Enable arm actions and confirmed release for the navigation comparison.
# Set False to repeat the speech-only SDK isolation trial.
ARM_GREETING_ENABLED = True
ALLOWED_ACTIONS = {
    "wave_under_head",
    "wave_above_head",
    "make_heart_with_right_hand",
    "make_heart_with_both_hands",
}


@dataclass(frozen=True)
class Settings:
    face_provider_id: str
    world_provider_id: str
    speech_provider_id: str
    arm_provider_id: str
    navigation_enabled: bool
    turn_to_face_enabled: bool
    poll_wait_ms: int
    cooldown_sec: float
    actions: dict[int, str]
    message_store_path: Path
    navigation_provider_id: str
    greeting_wait_sec: float

    @classmethod
    def parse(cls, raw: dict[str, Any]) -> "Settings":
        if not isinstance(raw, dict):
            raise ValueError("g1_face_greeting config must be an object")

        allowed = {
            "face_provider_id",
            "world_provider_id",
            "speech_provider_id",
            "arm_provider_id",
            "navigation_enabled",
            "turn_to_face_enabled",
            "poll_wait_ms",
            "cooldown_sec",
            "message_store_path",
            "navigation_provider_id",
            "greeting_wait_sec",
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
        if turn:
            raise ValueError(
                "turn_to_face_enabled must remain false"
            )
        wait = raw.get("greeting_wait_sec", 12.0)
        if isinstance(wait, bool) or not isinstance(wait, (int, float)) or not math.isfinite(wait) or not 1 <= wait <= 240:
            raise ValueError("greeting_wait_sec must be finite and in [1, 240]")
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
        store_path = Path(str(raw.get(
            "message_store_path", Path.home() / ".local/share/robonix/face_messages.sqlite3"
        ))).expanduser()
        if not store_path.is_absolute() or store_path.name in ("", ".", ".."):
            raise ValueError("message_store_path must be an absolute file path")
        return cls(
            text("face_provider_id", "face_recognition"),
            text("world_provider_id", "world"),
            text("speech_provider_id", "speech"),
            text("arm_provider_id", "g1_arm_actions"),
            navigation,
            turn,
            poll,
            cooldown,
            actions,
            store_path,
            text("navigation_provider_id", "nav2"),
            float(wait),
        )


_settings: Settings | None = None
_face_connection = None
_world_connection = None
_speech_connection = None
_arm_connection = None
_worker: threading.Thread | None = None
_stop = threading.Event()
_lock = threading.RLock()
_last_greet: dict[int, float] = {}
_seen_events: OrderedDict[str, None] = OrderedDict()
_store: MessageStore | None = None
_greeting_lock = threading.Lock()


def _find(contract: str, provider_id: str):
    caps = SRC.find_capability(provider_id=provider_id, contract_id=contract, transport=Transport.MCP)
    if len(caps) != 1:
        raise RuntimeError(f"expected one {provider_id!r} provider for {contract}, found {len(caps)}")
    return caps[0]


def _find_optional(contract: str, provider_id: str):
    caps = SRC.find_capability(provider_id=provider_id, contract_id=contract, transport=Transport.MCP)
    if len(caps) > 1:
        raise RuntimeError(f"expected at most one {provider_id!r} provider for {contract}, found {len(caps)}")
    return caps[0] if caps else None


def _result_value(result: Any) -> dict[str, Any]:
    """Decode MCP results and preserve original text when the server reports errors."""
    if getattr(result, "isError", False):
        details = [str(getattr(item, "text", "")) for item in (getattr(result, "content", None) or [])]
        raise RuntimeError("MCP tool error: " + ("; ".join(filter(None, details)) or str(getattr(result, "structuredContent", None))))
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


async def _call_connection(connection: Any, tool: str, payload: dict[str, Any], timeout: float) -> dict[str, Any]:
    """Call an MCP connection without creating a nested event loop in an MCP handler."""
    endpoint = (connection.endpoint or "").strip()
    if not endpoint:
        raise RuntimeError(f"empty endpoint for {tool}")
    return await _call(endpoint, tool, payload, timeout)


def _normal_label(value: Any) -> str:
    return "".join(str(value or "").casefold().split())


async def _resolve_world_object(label: str) -> dict[str, Any]:
    """Resolve one exact World label to its current stable object ID."""
    if _world_connection is None:
        raise RuntimeError("Scene object tracking is unavailable")
    response = await _call_connection(_world_connection, "list_objects", {}, 8.0)
    wanted = _normal_label(label)
    matches = [
        item for item in response.get("objects", [])
        if isinstance(item, dict) and _normal_label(item.get("label")) == wanted
    ]
    if not matches:
        raise RuntimeError(f"Scene does not currently list an object labeled {label!r}")
    if len(matches) > 1:
        ids = ", ".join(str(item.get("id", "")) for item in matches)
        raise RuntimeError(f"Scene lists multiple objects labeled {label!r}: {ids}; use a unique object")
    object_id = str(matches[0].get("id", "")).strip()
    if not object_id:
        raise RuntimeError(f"Scene returned an object labeled {label!r} without an ID")
    return {"id": object_id, "label": str(matches[0].get("label", label))}


async def _refresh_object_status(task: dict[str, Any]) -> dict[str, Any]:
    """Refresh tracked-object existence; query failure remains unknown, never removed."""
    object_id = str(task.get("object_id", ""))
    if not object_id:
        return task
    checked_at = time.time()
    try:
        if _world_connection is None:
            raise RuntimeError("Scene object tracking is unavailable")
        response = await _call_connection(_world_connection, "list_objects", {}, 8.0)
        objects = [item for item in response.get("objects", []) if isinstance(item, dict)]
        current = next((item for item in objects if str(item.get("id", "")) == object_id), None)
        object_status = "present" if current is not None else "removed"
        last_seen = float(current.get("last_seen_unix", 0.0)) if current is not None else None
    except Exception as exc:  # noqa: BLE001
        log.warning("scene object status refresh failed for task_id=%s: %s", task.get("task_id"), exc)
        object_status, last_seen = "unknown", None
    assert _store is not None
    _store.update_object_status(str(task["task_id"]), object_status, checked_at, last_seen)
    refreshed = _store.get(str(task["task_id"]))
    return refreshed or task


async def _refresh_object_statuses(tasks: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Refresh all tracked tasks from one Scene snapshot, preserving unknown on failure."""
    tracked = [task for task in tasks if str(task.get("object_id", ""))]
    if not tracked:
        return tasks
    checked_at = time.time()
    try:
        if _world_connection is None:
            raise RuntimeError("Scene object tracking is unavailable")
        response = await _call_connection(_world_connection, "list_objects", {}, 8.0)
        objects = {
            str(item.get("id", "")): item
            for item in response.get("objects", [])
            if isinstance(item, dict) and str(item.get("id", ""))
        }
        for task in tracked:
            current = objects.get(str(task["object_id"]))
            _store.update_object_status(
                str(task["task_id"]),
                "present" if current is not None else "removed",
                checked_at,
                float(current.get("last_seen_unix", 0.0)) if current is not None else None,
            )
    except Exception as exc:  # noqa: BLE001
        log.warning("scene object status refresh failed for %d task(s): %s", len(tracked), exc)
        for task in tracked:
            _store.update_object_status(str(task["task_id"]), "unknown", checked_at, None)
    refreshed = []
    for task in tasks:
        refreshed.append(_store.get(str(task["task_id"])) or task)
    return refreshed


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


def _greet_stationary(event: dict[str, Any]) -> dict[str, Any]:
    """Speak greetings and messages, optionally running the arm action."""
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
    messages: list[dict] = []
    with _lock:
        if event_id and event_id in _seen_events:
            return {"accepted": False, "detail": "duplicate event"}
        greet_due = person_id < 0 or now - _last_greet.get(person_id, -1e9) >= _settings.cooldown_sec
        observed_at = float(event.get("observed_at", 0.0))
        if _store is not None and person_id >= 0 and observed_at > 0:
            try:
                messages = _store.claim(person_id, observed_at, event_id)
                if messages:
                    log.info("claimed %d face message(s) for person_id=%s event_id=%s", len(messages), person_id, event_id)
            except Exception:  # noqa: BLE001
                log.exception("face message claim failed for person_id=%s", person_id)
        if not greet_due and not messages:
            return {"accepted": False, "detail": "greeting cooldown"}
        if event_id:
            _seen_events[event_id] = None
            if len(_seen_events) > 512:
                _seen_events.popitem(last=False)
        if greet_due:
            _last_greet[person_id] = now

    action = _settings.actions[level] if ARM_GREETING_ENABLED else ""
    speech_ok = False
    action_ok = False
    details: list[str] = []
    speech_parts = [str(event.get("hello", "您好！")).strip() or "您好！"] if greet_due else []
    for message in messages:
        prefix = f"{message['sender']}托我告诉您：" if message["sender"] else "有人托我告诉您："
        speech_parts.append(f"{prefix}{message['body']}")
    if event.get("ask_for_task"):
        speech_parts.append("有什么可以帮助您的吗？")
    speech_text = "。".join(
        part.rstrip("。！？!?；;，,") for part in speech_parts if part.strip()
    ) + "。"
    with concurrent.futures.ThreadPoolExecutor(
        max_workers=2 if ARM_GREETING_ENABLED else 1,
        thread_name_prefix="g1-face-greeting",
    ) as executor:
        speech_future = executor.submit(
            _timed_call,
            "speech",
            _speech_connection,
            "speak",
            {"target": "", "text": speech_text},
            max(65.0, min(240.0, len(speech_text) * 1.2)),
        )
        action_future = None
        if ARM_GREETING_ENABLED:
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
            log.exception("speech failed for person_id=%s: %s", person_id, exc)
        if messages:
            try:
                assert _store is not None
                _store.finish([message["task_id"] for message in messages], speech_ok, details[-1])
                log.info("face message delivery person_id=%s count=%d status=%s", person_id, len(messages), "delivered" if speech_ok else "uncertain")
            except Exception:  # noqa: BLE001
                log.exception("failed to save face message delivery result")
        if action_future is not None:
            try:
                arm = action_future.result()
                action_ok = bool(arm.get("accepted", False))
                details.append(f"action={arm.get('detail', 'ok' if action_ok else 'failed')}")
            except Exception as exc:  # noqa: BLE001
                details.append(f"action_error={exc}")
                log.exception("arm action failed for person_id=%s", person_id)
        else:
            details.append("arm action skipped for SDK isolation test")
    return {
        "accepted": True,
        "speech_ok": speech_ok,
        "action_ok": action_ok,
        "action": action,
        "playback_wait_sec": max(_settings.greeting_wait_sec, len(speech_text) / 3.0),
        "detail": "; ".join(details),
    }


def _navigation_call(tool: str, payload: dict) -> dict:
    """Resolve each navigation operation through Src and release its connection."""
    contract = NAVIGATION + '/' + tool
    log.info('navigation %s requested run_id=%s', tool, payload.get('run_id', ''))
    connection = skill.connect_capability(_find(contract, _settings.navigation_provider_id), contract, Transport.MCP)
    try:
        response = _call_sync(connection, tool, payload, 8.0)
        log.info('navigation %s result: %s', tool, response)
        return response
    finally:
        _close_connection(connection)


def _release_arm() -> dict:
    """Release the preset and require SDK action-state feedback, not acceptance alone."""
    try:
        response = _timed_call('arm release', _arm_connection, 'run', {'action': 'release_arm'}, 30.0)
        detail = str(response.get('detail', ''))
        confirmed = bool(response.get('accepted')) and any(marker in detail for marker in (
            'release_arm confirmed by action state: id=99 holding=false',
            'release_arm confirmed by action state: id=0 holding=false name=""; idle confirmed for >=0.2s',
        ))
        return {'ok': confirmed, 'detail': detail or 'release feedback unavailable'}
    except Exception as exc:  # noqa: BLE001
        log.exception('arm release failed')
        return {'ok': False, 'detail': f'{type(exc).__name__}: {exc}'}


def _wait_for_greeting(result: dict[str, Any], started: float) -> None:
    """Count SDK call time toward playback waiting; shutdown interrupts the remainder."""
    assert _settings is not None
    wait_sec = result.get('playback_wait_sec', _settings.greeting_wait_sec)
    elapsed_sec = time.monotonic() - started
    remaining_sec = max(0.0, wait_sec - elapsed_sec)
    log.info('greeting playback wait: elapsed_sec=%.3f remaining_sec=%.3f total_sec=%.3f',
             elapsed_sec, remaining_sec, wait_sec)
    _stop.wait(remaining_sec)


def _greet(event: dict[str, Any]) -> dict[str, Any]:
    """Serialize greetings and retain the navigation run until greeting finishes."""
    run_id = ''
    result = None
    if not _greeting_lock.acquire(blocking=False):
        return {'accepted': False, 'detail': 'another greeting is running'}
    try:
        if _settings is not None and _settings.navigation_enabled and coordination.enabled():
            return coordination.greet(sys.modules[__name__], event)
        if _settings is None or not _settings.navigation_enabled:
            return _greet_stationary(event)
        if _valid_level(event.get('intimacy_level', 1)) is None:
            return {'accepted': False, 'detail': 'invalid intimacy level'}
        person_id = int(event.get('person_id', -1))
        if person_id < 0:
            return {'accepted': False, 'detail': 'navigation greetings require a recognized person'}
        with _lock:
            if str(event.get('event_id', '')) in _seen_events:
                return {'accepted': False, 'detail': 'duplicate event'}
            if time.monotonic() - _last_greet.get(person_id, -1e9) < _settings.cooldown_sec:
                return {'accepted': False, 'detail': 'greeting cooldown'}
        paused = _navigation_call('pause', {'run_id': ''})
        if (not paused.get('accepted') and not paused.get('run_id')
                and paused.get('detail') == 'no active navigation; stationary=true'):
            if _stop.is_set():
                return {'accepted': False, 'detail': 'skill stopped before greeting'}
            log.info('stationary greeting without navigation person_id=%s', person_id)
            try:
                greeting_started = time.monotonic()
                result = _greet_stationary(event)
                _wait_for_greeting(result, greeting_started)
            finally:
                if ARM_GREETING_ENABLED:
                    release = _release_arm()
                    log.log(logging.INFO if release['ok'] else logging.ERROR,
                            'stationary greeting arm release: %s', release)
                else:
                    log.info('stationary greeting arm release skipped for SDK isolation test')
            if ARM_GREETING_ENABLED:
                result['detail'] += '; arm release: ' + release['detail']
                if not release['ok']:
                    result['detail'] += '; arm release unconfirmed'
                    log.error('%s', result['detail'])
            else:
                result['detail'] += '; arm release skipped for SDK isolation test'
            result['detail'] += '; no navigation to resume'
            return result
        if not paused.get('accepted'):
            return {'accepted': False, 'detail': 'navigation pause rejected: ' + str(paused.get('detail'))}
        run_id = paused['run_id']
        deadline = time.monotonic() + 8.0
        while not _stop.is_set() and time.monotonic() < deadline:
            state = _navigation_call('status', {'run_id': run_id})
            if state.get('state') == 'PAUSED' and state.get('detail', '').endswith('stationary=true'):
                break
            if state.get('state') not in {'RUNNING', 'PAUSED'}:
                raise RuntimeError('navigation ended before greeting: ' + str(state))
            _stop.wait(0.1)
        else:
            raise RuntimeError('navigation did not stop with fresh odometry before greeting')
        if _stop.is_set():
            return {'accepted': False, 'detail': 'skill stopped; navigation remains paused'}
        # Cleanup also runs if greeting raises or shutdown interrupts the wait.
        try:
            greeting_started = time.monotonic()
            result = _greet_stationary(event)
            if not result.get('speech_ok') or (ARM_GREETING_ENABLED and not result.get('action_ok')):
                log.error('navigation greeting failed run_id=%s: %s', run_id, result)
            _wait_for_greeting(result, greeting_started)
        finally:
            if ARM_GREETING_ENABLED:
                release = _release_arm()
                log.log(logging.INFO if release['ok'] else logging.ERROR,
                        'arm release result run_id=%s: %s', run_id, release)
            else:
                log.info('arm release skipped for SDK isolation test run_id=%s', run_id)
        if ARM_GREETING_ENABLED:
            result['detail'] += '; arm release: ' + release['detail']
            if not release['ok']:
                result['detail'] += f'; navigation run_id={run_id} remains paused: arm release unconfirmed'
                log.error('%s', result['detail'])
                return result
        else:
            result['detail'] += '; arm release skipped for SDK isolation test'
        if _stop.is_set():
            result['detail'] += '; navigation remains paused because skill stopped'
            return result
        state = _navigation_call('status', {'run_id': run_id})
        if state.get('state') != 'PAUSED' or not state.get('detail', '').endswith('stationary=true'):
            result['detail'] += f'; navigation run_id={run_id} not resumed: {state}'
            log.error('%s', result['detail'])
            return result
        resumed = _navigation_call('resume', {'run_id': run_id})
        result['detail'] += '; navigation resume: ' + str(resumed.get('detail', ''))
        if not resumed.get('accepted'):
            log.error('navigation resume rejected run_id=%s: %s', run_id, resumed)
            result['detail'] += f'; run_id={run_id}; resume rejected; inspect navigation status before retrying'
        return result
    except Exception as exc:  # noqa: BLE001
        detail = f'{type(exc).__name__}: {exc}'
        if run_id:
            detail += f'; navigation run_id={run_id}; no automatic resume; inspect navigation status'
        log.exception('navigation greeting failed: %s', detail)
        if result is not None:
            result['detail'] += '; ' + detail
            return result
        return {'accepted': False, 'detail': detail}
    finally:
        _greeting_lock.release()


def _worker_loop() -> None:
    while not _stop.is_set():
        try:
            event = _call_sync(_face_connection, FACE_EVENTS.rsplit("/", 1)[-1], {"wait_ms": _settings.poll_wait_ms}, 8.0)
            if event.get("available"):
                log.info("face encounter: id=%s name=%s intimacy=%s", event.get("person_id"), event.get("name"), event.get("intimacy_level"))
                result = _greet(event)
                log.info("face greeting result: %s", result)
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
    global _face_connection, _world_connection, _speech_connection, _arm_connection, _worker, _store
    with _lock:
        if _settings is None:
            return Err("skill has not been initialized")
        if _worker is not None:
            return Err("skill is already active")
        try:
            if _settings.navigation_enabled and coordination.enabled():
                coordination.initialize()
            _store = MessageStore(_settings.message_store_path)
            _face_connection = skill.connect_capability(_find(FACE_EVENTS, _settings.face_provider_id), FACE_EVENTS, Transport.MCP)
            _find(FACE_RESOLVE, _settings.face_provider_id)
            world_capability = _find_optional(WORLD_OBJECTS, _settings.world_provider_id)
            _world_connection = (
                skill.connect_capability(world_capability, WORLD_OBJECTS, Transport.MCP)
                if world_capability is not None else None
            )
            if _world_connection is None:
                log.warning("Scene object tracking is unavailable; ordinary face messages remain usable")
            _speech_connection = skill.connect_capability(_find(SPEECH, _settings.speech_provider_id), SPEECH, Transport.MCP)
            if ARM_GREETING_ENABLED:
                _arm_connection = skill.connect_capability(_find(ARM, _settings.arm_provider_id), ARM, Transport.MCP)
            else:
                _arm_connection = None
                log.info("arm SDK connection disabled for greeting isolation test")
            _last_greet.clear()
            _seen_events.clear()
            _stop.clear()
            _worker = threading.Thread(target=_worker_loop, name="g1-face-greeting", daemon=True)
            _worker.start()
            log.info("face greeting active; navigation_enabled=%s", _settings.navigation_enabled)
            return Ok()
        except Exception as exc:  # noqa: BLE001
            deactivate()
            return Err(str(exc))


@skill.on_deactivate
def deactivate():
    global _face_connection, _world_connection, _speech_connection, _arm_connection, _worker, _store
    with _lock:
        _stop.set()
        if _worker is not None and _worker is not threading.current_thread():
            _worker.join(timeout=10)
            if _worker.is_alive():
                return Err("greeting worker is still completing an in-flight MCP request")
        _worker = None
        connections = (_face_connection, _world_connection, _speech_connection, _arm_connection)
        _face_connection = _world_connection = _speech_connection = _arm_connection = None
        errors = []
        for connection in connections:
            try:
                _close_connection(connection)
            except Exception as exc:  # noqa: BLE001
                errors.append(str(exc))
        _last_greet.clear()
        _seen_events.clear()
        _store = None
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


@skill.mcp("robonix/skill/g1_face_greeting/create_message")
async def create_message(req: CreateFaceMessage_Request) -> CreateFaceMessage_Response:
    """Store an ordinary one-time message without querying Scene."""
    if _settings is None or _store is None or _face_connection is None:
        return CreateFaceMessage_Response(detail="greeting skill is not active")
    person_name, body, sender = req.person_name.strip(), req.message.strip(), req.sender.strip()
    if not person_name or not body or len(body) > 160 or len(sender) > 30:
        return CreateFaceMessage_Response(detail="person_name and message are required; message <=160 and sender <=30 characters")
    if any(ord(char) < 32 for char in body + sender):
        return CreateFaceMessage_Response(detail="message and sender must not contain control characters")
    try:
        response = await _call_connection(_face_connection, "resolve", {"name": person_name}, 8.0)
        if not response.get("found"):
            return CreateFaceMessage_Response(detail=str(response.get("detail", "person not registered")))
        person_id = int(response["person_id"])
        task_id, created = _store.create_or_get(person_id, str(response["name"]), sender, body)
        log.info(
            "face message %s task_id=%s requested_name=%s resolved_name=%s person_id=%s sender=%s",
            "created" if created else "deduplicated",
            task_id,
            person_name,
            response["name"],
            person_id,
            sender or "<unknown>",
        )
        if created:
            detail = "saved; awaiting next face encounter"
        else:
            detail = "an identical pending task already exists; awaiting next face encounter"
        return CreateFaceMessage_Response(
            accepted=True, task_id=task_id, person_id=person_id,
            person_name=str(response["name"]), detail=detail,
        )
    except Exception as exc:  # noqa: BLE001
        log.exception("failed to save face message")
        return CreateFaceMessage_Response(detail=str(exc))


@skill.mcp("robonix/skill/g1_face_greeting/create_object_message")
async def create_object_message(
    req: CreateObjectFaceMessage_Request,
) -> CreateObjectFaceMessage_Response:
    """Store a one-time message bound to one unique current Scene object."""
    if _settings is None or _store is None or _face_connection is None:
        return CreateObjectFaceMessage_Response(detail="greeting skill is not active")
    person_name = req.person_name.strip()
    body = req.message.strip()
    sender = req.sender.strip()
    object_label = req.object_label.strip()
    if not person_name or not body or not object_label or len(body) > 160 or len(sender) > 30:
        return CreateObjectFaceMessage_Response(
            detail="person_name, message, and object_label are required; message <=160 and sender <=30 characters"
        )
    if any(ord(char) < 32 for char in body + sender + object_label):
        return CreateObjectFaceMessage_Response(detail="message, sender, and object_label must not contain control characters")
    try:
        response = await _call_connection(_face_connection, "resolve", {"name": person_name}, 8.0)
        if not response.get("found"):
            return CreateObjectFaceMessage_Response(detail=str(response.get("detail", "person not registered")))
        person_id = int(response["person_id"])
        object_info = await _resolve_world_object(object_label)
        task_id, created = _store.create_or_get(
            person_id, str(response["name"]), sender, body,
            object_id=object_info["id"], object_label=object_info["label"],
        )
        log.info(
            "face object message %s task_id=%s requested_name=%s resolved_name=%s person_id=%s object_id=%s sender=%s",
            "created" if created else "deduplicated",
            task_id,
            person_name,
            response["name"],
            person_id,
            object_info["id"],
            sender or "<unknown>",
        )
        detail = (
            "saved; awaiting next face encounter"
            if created
            else "an identical pending task already exists; awaiting next face encounter"
        )
        return CreateObjectFaceMessage_Response(
            accepted=True,
            task_id=task_id,
            person_id=person_id,
            person_name=str(response["name"]),
            object_id=object_info["id"],
            object_label=object_info["label"],
            detail=detail,
        )
    except Exception as exc:  # noqa: BLE001
        log.exception("failed to save object face message")
        return CreateObjectFaceMessage_Response(detail=str(exc))


@skill.mcp("robonix/skill/g1_face_greeting/list_messages")
async def list_messages(req: ListFaceMessages_Request) -> ListFaceMessages_Response:
    """List recent tasks so Planner can report pending and delivered results."""
    if _store is None:
        return ListFaceMessages_Response(detail="greeting skill is not active")
    status = req.status.strip()
    if status not in ("", "pending", "delivering", "delivered", "uncertain", "cancelled"):
        return ListFaceMessages_Response(detail="invalid status filter")
    try:
        tasks = await _refresh_object_statuses(_store.list(status))
        return ListFaceMessages_Response(tasks_json=json.dumps(tasks, ensure_ascii=False), detail="ok")
    except Exception as exc:  # noqa: BLE001
        return ListFaceMessages_Response(detail=str(exc))


@skill.mcp("robonix/skill/g1_face_greeting/message_status")
async def message_status(req: FaceMessageStatus_Request) -> FaceMessageStatus_Response:
    """Read one task; a speech-success response does not prove human receipt."""
    if _store is None:
        return FaceMessageStatus_Response(detail="greeting skill is not active")
    try:
        task = _store.get(req.task_id.strip())
        if task and task.get("object_id"):
            task = await _refresh_object_status(task)
        return FaceMessageStatus_Response(
            found=task is not None,
            task_json=json.dumps(task, ensure_ascii=False) if task else "",
            object_status=str(task.get("object_status", "not_tracked")) if task else "",
            detail="ok" if task else "task not found",
        )
    except Exception as exc:  # noqa: BLE001
        return FaceMessageStatus_Response(detail=str(exc))


@skill.mcp("robonix/skill/g1_face_greeting/cancel_message")
def cancel_message(req: CancelFaceMessage_Request) -> CancelFaceMessage_Response:
    """Cancel pending work, but never interrupt an in-progress utterance."""
    if _store is None:
        return CancelFaceMessage_Response(detail="greeting skill is not active")
    try:
        cancelled = _store.cancel(req.task_id.strip())
        return CancelFaceMessage_Response(
            cancelled=cancelled, detail="cancelled" if cancelled else "task is not pending or not found"
        )
    except Exception as exc:  # noqa: BLE001
        return CancelFaceMessage_Response(detail=str(exc))


if __name__ == "__main__":
    skill.run()
