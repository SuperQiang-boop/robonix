"""Deployment-local greeting facts shared with Planner and navigation.

No face events are consumed here. Only the existing greeting worker writes this
snapshot; atomic replacement keeps readers from seeing partial updates.
"""
from __future__ import annotations

import json
import os
import threading
import time
import uuid
from pathlib import Path

from robonix_api import SRC
from robonix_api.src_types import Transport

STATE_ENV = "ROBONIX_GREETING_STATE_PATH"
EXECUTOR = "robonix/system/executor/list_active_plans"
VOICE = "robonix/system/gateway/voice"


def enabled() -> bool:
    return bool(os.environ.get(STATE_ENV, "").strip())


def publish(state: dict) -> None:
    """Publish a complete snapshot; a write failure must prevent movement."""
    path = Path(os.environ[STATE_ENV])
    path.parent.mkdir(parents=True, exist_ok=True)
    value = {**state, "updated_at": time.time()}
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8") as output:
        json.dump(value, output, ensure_ascii=False)
        output.flush()
        os.fsync(output.fileno())
    os.replace(temporary, path)


def initialize() -> None:
    """Never erase an unfinished greeting after a process restart."""
    path = Path(os.environ[STATE_ENV])
    if path.exists():
        previous = json.loads(path.read_text(encoding="utf-8"))
        if previous.get("movement_allowed") is not True:
            raise RuntimeError("previous greeting is unconfirmed; verify stopping and arm release before clearing " + str(path))
    publish({"state": "SUCCEEDED", "active": False, "movement_allowed": True,
             "detail": "no greeting in progress", "event_id": "", "session_id": ""})


def connect(runtime, contract: str):
    """Discover existing gRPC contracts through Src, without fixed ports."""
    import grpc

    found = SRC.find_capability(contract_id=contract, transport=Transport.GRPC)
    if len(found) != 1:
        raise RuntimeError(f"expected one gRPC capability for {contract}, found {len(found)}")
    connection = runtime.skill.connect_capability(found[0], contract, Transport.GRPC)
    endpoint = connection.endpoint.removeprefix("http://").removeprefix("https://")
    return connection, grpc.insecure_channel(endpoint)


def owner(runtime) -> dict:
    """Bind greetings to one actual task session; ambiguous owners are rejected."""
    from executor_pb2 import ListActivePlans_Request
    from robonix_contracts_pb2_grpc import RobonixSystemExecutorListActivePlansStub

    connection, channel = connect(runtime, EXECUTOR)
    try:
        response = RobonixSystemExecutorListActivePlansStub(channel).ListActivePlans(
            ListActivePlans_Request(), timeout=3)
        if not response.success:
            raise RuntimeError(response.error)
        plans = json.loads(response.plans_json)["plans"]
        navigation = [plan for plan in plans if any(
            (op.get("contract_id", "").endswith("/navigate")
             or op.get("contract_id", "").endswith("/navigate_landmark"))
            and op.get("state") in {"pending", "running", "paused"}
            for op in plan.get("ops", []))]
        if len(navigation) > 1:
            raise RuntimeError("multiple navigation plans; greeting task ownership is ambiguous")
        if navigation:
            plan = navigation[0]
            if not plan.get("session_id"):
                raise RuntimeError("Executor does not report session_id; rebuild Executor before enabling coordination")
            return {"session_id": plan["session_id"], "plan_id": plan["plan_id"]}
        return {"session_id": "face-" + uuid.uuid4().hex, "plan_id": ""}
    finally:
        channel.close()
        connection.close()


def stop_requested(facts: dict) -> bool:
    """Honor the matching session/event Stop without sharing snapshot ownership."""
    path = Path(os.environ[STATE_ENV] + ".stop")
    try:
        stopped = json.loads(path.read_text(encoding="utf-8"))
        return (stopped.get("session_id") == facts["session_id"]
                and stopped.get("event_id") == facts["event_id"])
    except FileNotFoundError:
        return False
    except (OSError, ValueError, TypeError):
        return True


def listen(runtime, facts: dict) -> bool:
    """Use Gateway's complete authenticated voice pipeline exactly once.

    Keep draining after admission: its stream includes the entire task, not just
    ASR. A missing admission/empty-transcript result leaves movement inhibited.
    """
    from gateway_pb2 import StartVoiceSession_Request
    from robonix_contracts_pb2_grpc import RobonixSystemGatewayVoiceStub

    ready = threading.Event()
    result = {}
    streams = []

    def consume():
        """Own the voice transport until its task stream finishes or fails."""
        connection = channel = None
        try:
            connection, channel = connect(runtime, VOICE)
            request = StartVoiceSession_Request(
                session_id=facts["session_id"], record_seconds=15, language="zh-CN",
                tts_enabled=True, mic_node_id="g1_audio", speaker_node_id="g1_audio",
                context_json=json.dumps({"greeting_event_id": facts["event_id"]}))
            stream = RobonixSystemGatewayVoiceStub(channel).StartVoiceSession(request)
            streams.append(stream)
            for event in stream:
                if event.event_kind == 6:  # Planner accepted this same-session voice turn.
                    result["submitted"] = True
                    ready.set()
                elif event.event_kind == 10:
                    if "empty transcript" in event.error and not result.get("submitted"):
                        result["submitted"] = False
                    else:
                        result["error"] = event.error or "voice session failed"
                    ready.set()
                if runtime._stop.is_set():
                    break
        except Exception as exc:
            result["error"] = str(exc)
            runtime.log.exception("greeting voice session failed")
        finally:
            ready.set()
            if channel is not None:
                channel.close()
            if connection is not None:
                connection.close()

    threading.Thread(target=consume, name="g1-greeting-voice", daemon=True).start()
    deadline = time.monotonic() + 45
    while not ready.wait(0.1):
        if stop_requested(facts):
            if streams:
                streams[0].cancel()
            raise InterruptedError("greeting conversation canceled by Stop")
        if runtime._stop.is_set() or time.monotonic() >= deadline:
            if streams:
                streams[0].cancel()
            raise RuntimeError("voice admission unconfirmed; navigation remains stopped")
    if "submitted" not in result or (result.get("error") and not result.get("submitted")):
        raise RuntimeError(result.get("error", "voice stream ended before admission"))
    return result["submitted"]


def greet(runtime, event: dict) -> dict:
    """Stop, greet, listen, and hand completion to Planner without resuming a goal."""
    if runtime._valid_level(event.get("intimacy_level", 1)) is None:
        return {"accepted": False, "detail": "invalid intimacy level"}
    person_id = int(event.get("person_id", -1))
    if person_id < 0:
        return {"accepted": False, "detail": "navigation greetings require a recognized person"}
    with runtime._lock:
        if str(event.get("event_id", "")) in runtime._seen_events:
            return {"accepted": False, "detail": "duplicate event"}
        if time.monotonic() - runtime._last_greet.get(person_id, -1e9) < runtime._settings.cooldown_sec:
            return {"accepted": False, "detail": "greeting cooldown"}
    facts = {**owner(runtime), "event_id": str(event.get("event_id", "")),
             "person_id": person_id, "person_name": str(event.get("name", "")),
             "run_id": "", "state": "RUNNING", "active": True,
             "movement_allowed": False, "listening": False, "detail": "stopping for greeting"}
    publish(facts)
    result = {"accepted": False, "detail": "greeting not started"}
    release_confirmed = True
    try:
        paused = runtime._navigation_call("pause", {"run_id": ""})
        idle = (not paused.get("accepted") and not paused.get("run_id")
                and paused.get("detail") == "no active navigation; stationary=true")
        if not idle and not paused.get("accepted"):
            raise RuntimeError("navigation pause rejected: " + str(paused.get("detail")))
        facts["run_id"] = paused.get("run_id", "")
        if not idle:
            deadline = time.monotonic() + 8
            while not runtime._stop.is_set() and time.monotonic() < deadline:
                state = runtime._navigation_call("status", {"run_id": facts["run_id"]})
                if state.get("state") == "PAUSED" and state.get("detail", "").endswith("stationary=true"):
                    break
                if state.get("state") not in {"RUNNING", "PAUSED"}:
                    raise RuntimeError("navigation ended before greeting: " + str(state))
                runtime._stop.wait(0.1)
            else:
                raise RuntimeError("navigation stop unconfirmed")
        if runtime._stop.is_set():
            raise RuntimeError("skill stopped before greeting")
        if stop_requested(facts):
            raise InterruptedError("greeting conversation canceled by Stop")
        facts["detail"] = "greeting in progress; playback completion is estimated"
        publish(facts)
        release = {"ok": True, "detail": "speech-only greeting"}
        try:
            release_confirmed = not runtime.ARM_GREETING_ENABLED
            started = time.monotonic()
            result = runtime._greet_stationary({**event, "ask_for_task": True})
            runtime._wait_for_greeting(result, started)
        finally:
            if runtime.ARM_GREETING_ENABLED:
                release = runtime._release_arm()
        if not release["ok"]:
            raise RuntimeError("arm release unconfirmed: " + release["detail"])
        release_confirmed = True
        if runtime._stop.is_set() or not result.get("speech_ok"):
            raise RuntimeError("greeting speech or completion unconfirmed")
        facts.update(listening=True, detail="greeting wait and arm release complete; listening for a request")
        publish(facts)
        if stop_requested(facts):
            raise InterruptedError("greeting conversation canceled by Stop")
        submitted = listen(runtime, facts)
        if runtime._stop.is_set():
            raise RuntimeError("skill stopped during conversation")
        facts.update(state="SUCCEEDED", active=False, movement_allowed=True, listening=False,
                     detail="voice task submitted to the original session" if submitted else "no voice request; Planner may continue the original task")
        publish(facts)
        result["detail"] += "; " + facts["detail"]
        return result
    except Exception as exc:
        stopped_safely = isinstance(exc, InterruptedError) and release_confirmed
        facts.update(state="SUCCEEDED" if stopped_safely else "FAILED", active=False,
                     movement_allowed=stopped_safely, listening=False,
                     detail=str(exc))
        publish(facts)
        runtime.log.exception("coordinated greeting failed; movement remains inhibited")
        result["detail"] += "; " + str(exc)
        return result
