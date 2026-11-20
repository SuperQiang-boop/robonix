#!/usr/bin/env python3
"""Exercise real Atlas/Executor/Pilot against a deterministic local model.

Uses isolated ports and harmless shell markers instead of robot hardware. This
checks execution ordering and failure/Stop behavior, not model language accuracy.
Build the Rust binaries first, then run with this skill's venv Python.
"""
import asyncio
import json
import os
import re
import shlex
import socket
import subprocess
import sys
import tempfile
import threading
import logging
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from types import SimpleNamespace

PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = PACKAGE.parents[3]
sys.path.insert(0, str(PACKAGE / "rbnx-build/codegen/proto_gen"))
sys.path.insert(0, str(PACKAGE))
sys.path.insert(0, str(SOURCE / "pylib/robonix-api"))
import grpc
import pilot_pb2
import robonix_contracts_pb2_grpc as contracts


def port():
    with socket.socket() as connection:
        connection.bind(("127.0.0.1", 0))
        return connection.getsockname()[1]


class Model:
    def __init__(self, directory):
        self.directory = directory
        self.scenario = "success"
        self.trace = directory / "trace"
        self.requests = 0

    def labels(self):
        return self.trace.read_text().splitlines() if self.trace.exists() else []

    def response(self, messages):
        """Return deliberate RTDL; the real harness must admit or defer it."""
        self.requests += 1
        system = "\n".join(message.get("content", "") for message in messages if message["role"] == "system")
        history = "\n".join(str(message.get("content", "")) for message in messages)
        goal = re.search(r"- instruction: ([^\n]+)", system).group(1)
        assert "resume_after=true" in system
        assert "movement_allowed=false" in system
        labels = self.labels()
        rtdl = {"op": "sequence", "children": []}
        status = "in_progress"
        description = ""

        def action(label, seconds=0.15, fail=False):
            """Use one local shell command as a stand-in for an async action."""
            nonlocal rtdl, description
            target = shlex.quote(str(self.trace))
            command = (f"printf '{label}_START\\n' >> {target}; sleep {seconds}; "
                       f"printf '{label}_DONE\\n' >> {target}")
            command += "; printf '%s' " + shlex.quote('{"ok":false}' if fail else label + "_DONE")
            if fail:
                command += "; exit 1"
            rtdl = {"op": "do", "op_id": 0, "description": label,
                    "cap": "executor.builtin_run_command", "args": {"command": command}}
            description = label

        if goal == "go to A":
            if "A_RETURN_DONE" in labels:
                status = "done"
            elif "temporary work succeeded and was verified" in history:
                if "A_RETURN_START" not in labels:
                    action("A_RETURN")
            elif "A_START" not in labels:
                action("A", 60)
        else:
            plans = re.findall(r"- plan_id=(\S+) running:", system)
            if "B_DONE" in labels:
                status = "done"
            elif plans and "B_START" not in labels:
                rtdl = {"op": "cancel_plan", "plan_id": plans[0], "wait_ms": 5000,
                        "resume_after": self.scenario != "replace"}
                description = "interrupt A"
            elif "B_START" not in labels:
                action("B", 60 if self.scenario == "stop" else 0.15, self.scenario == "failure")
        return {"content": "verified" if status == "done" else "working",
                "rtdl_description": description, "rtdl": rtdl,
                "task_update": {"goal": goal, "success_criterion": "observed task result", "status": status,
                                "verified": status == "done"}}


def handler(model):
    """Verify handler."""
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def do_POST(self):
            """Serve an OpenAI-compatible SSE response on localhost only."""
            payload = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
            response = json.dumps(model.response(payload["messages"]))
            with (model.directory / "model.jsonl").open("a") as log:
                log.write(json.dumps({"request": payload, "response": json.loads(response)}) + "\n")
            chunk = {"id": "local-smoke", "object": "chat.completion.chunk", "created": 0,
                     "model": "local-smoke", "choices": [{"index": 0,
                     "delta": {"role": "assistant", "content": response}, "finish_reason": None}]}
            finish = {**chunk, "choices": [{"index": 0, "delta": {}, "finish_reason": "stop"}]}
            body = ("data: " + json.dumps(chunk) + "\n\ndata: " + json.dumps(finish) + "\n\ndata: [DONE]\n\n").encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
    return Handler


async def wait_until(predicate, description, seconds=20):
    """Fail with a bounded timeout instead of leaving subprocesses running."""
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if predicate():
            return
        await asyncio.sleep(0.05)
    raise AssertionError("timeout: " + description)


async def scenario(stub, model, snapshot, name, runtime=None):
    """Run one temporary/replacement/failed/stopped interaction through real RPCs."""
    model.scenario = name
    model.trace = model.directory / (name + ".trace")
    session = "handoff-" + name
    events = []
    frontend_events = []

    def publish(allowed):
        temporary = snapshot.with_suffix(".tmp")
        temporary.write_text(json.dumps({"state": "SUCCEEDED" if allowed else "RUNNING",
            "movement_allowed": allowed, "active": not allowed, "session_id": session,
            "event_id": name, "detail": "fake greeting", "run_id": "A"}))
        temporary.replace(snapshot)

    async def collect(stream, frontend=False):
        async for event in stream:
            events.append(event)
            if frontend:
                frontend_events.append(event)

    publish(True)
    collectors = [asyncio.create_task(collect(stub.SubmitTask(pilot_pb2.Task(
        task_id=name + "-A", session_id=session, text="go to A")), True))]
    await wait_until(lambda: "A_START" in model.labels(), "A started")
    publish(False)
    if name == "voice":
        from g1_face_greeting import coordination
        admitted = await asyncio.to_thread(coordination.listen, runtime,
            {"session_id": session, "event_id": name})
        assert admitted, "Liaison did not admit the greeting voice command"
    else:
        collectors.append(asyncio.create_task(collect(stub.SubmitTask(pilot_pb2.Task(
            task_id=name + "-B", session_id=session, text="first B",
            context_json=json.dumps({"greeting_event_id": name}))))))
    await asyncio.sleep(1)
    assert "B_START" not in model.labels(), "B bypassed greeting hold"
    publish(True)
    await wait_until(lambda: "B_START" in model.labels(), "B started after cancellation")
    if name == "stop":
        async for _ in stub.SubmitTask(pilot_pb2.Task(task_id=name + "-stop", session_id=session,
                context_json='{"abort_turn":true}')):
            pass
        await asyncio.sleep(1)
    else:
        await wait_until(lambda: "B_DONE" in model.labels(), "B finished")
        if name in {"success", "voice"}:
            await wait_until(lambda: "A_RETURN_DONE" in model.labels(), "original task restored")
        await wait_until(lambda: any(event.final_text for event in events), "final response")
    labels = model.labels()
    assert any(event.HasField("plan") and any(node.description == "B" for node in event.plan.nodes)
               for event in frontend_events), "greeting voice detached the frontend status stream"
    assert "A_DONE" not in labels, "original execution was not cancelled"
    if name in {"success", "voice"}:
        assert labels == ["A_START", "B_START", "B_DONE", "A_RETURN_START", "A_RETURN_DONE"], labels
        assert any(event.HasField("task_state") and event.task_state.goal == "go to A"
                   and event.task_state.status == "in_progress" for event in frontend_events)
    else:
        assert "A_RETURN_START" not in labels, labels
    if name == "stop":
        assert "B_DONE" not in labels
        # A late authorized voice transcript from the same encounter must be rejected.
        try:
            async for _ in stub.SubmitTask(pilot_pb2.Task(task_id="late-" + name,
                    session_id=session, text="first B", context_json=json.dumps({"greeting_event_id": name}))):
                pass
            raise AssertionError("late greeting voice task bypassed Stop")
        except grpc.aio.AioRpcError as error:
            assert error.code() == grpc.StatusCode.CANCELLED
    for collector in collectors:
        collector.cancel()
    await asyncio.gather(*collectors, return_exceptions=True)
    print("PASS", name, labels, flush=True)


async def main():
    """Start isolated services and always tear them down, preserving failure logs."""
    work = Path(tempfile.mkdtemp(prefix="g1-task-handoff-"))
    model = Model(work)
    server = ThreadingHTTPServer(("127.0.0.1", 0), handler(model))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    atlas, executor, pilot, liaison = [f"127.0.0.1:{port()}" for _ in range(4)]
    snapshot = work / "greeting.json"
    snapshot.write_text('{"movement_allowed":true}')
    environment = {key: value for key, value in os.environ.items()
                   if key in {"PATH", "HOME", "USER", "LANG", "LD_LIBRARY_PATH"}}
    environment.update(ROBONIX_SOURCE_PATH=str(SOURCE), ROBONIX_GREETING_STATE_PATH=str(snapshot),
                       ROBONIX_LIAISON_VOICE_MOCK="1", ROBONIX_LIAISON_VOICE_MOCK_TEXT="first B",
                       ROBONIX_LIAISON_ACCESS_ENABLED="false",
                       SCRIBE_STDOUT_LEVEL="debug", SCRIBE_FILE_LEVEL="off", XDG_STATE_HOME=str(work))
    commands = [
        ["robonix-atlas", "--listen", atlas, "--capabilities", str(SOURCE / "capabilities")],
        ["robonix-executor", "--atlas", atlas, "--listen", executor, "--id", "executor"],
        ["robonix-pilot", "--atlas", atlas, "--listen", pilot, "--id", "pilot",
         "--vlm-upstream", f"http://127.0.0.1:{server.server_port}/v1", "--vlm-api-key", "local-smoke",
         "--vlm-model", "local-smoke", "--vlm-format", "openai"],
        ["robonix-liaison", "--atlas", atlas, "--listen", liaison, "--pilot-endpoint", pilot],
    ]
    processes = []
    outputs = []
    try:
        for command, endpoint in zip(commands, [atlas, executor, pilot, liaison]):
            output = (work / (command[0] + ".log")).open("w")
            outputs.append(output)
            command[0] = str(SOURCE / "target/debug" / command[0])
            processes.append(subprocess.Popen(command, env=environment, cwd=work, stdout=output, stderr=subprocess.STDOUT))
            async with grpc.aio.insecure_channel(endpoint) as channel:
                await asyncio.wait_for(channel.channel_ready(), 15)
        async with grpc.aio.insecure_channel(pilot) as channel:
            stub = contracts.RobonixSystemPilotStub(channel)
            os.environ["ROBONIX_ATLAS"] = atlas
            os.environ["ROBONIX_GREETING_STATE_PATH"] = str(snapshot)
            from robonix_api import ATLAS, Skill
            ATLAS.register_skill("greeting-probe", "robonix/skill/greeting-probe")
            runtime = SimpleNamespace(skill=Skill(id="greeting-probe", namespace="robonix/skill/greeting-probe"),
                                      _stop=threading.Event(), log=logging.getLogger("greeting-probe"))
            for name in ["success", "failure", "replace", "stop", "voice"]:
                await scenario(stub, model, snapshot, name, runtime)
        print("PASS real Pilot/Executor task handoff; artifacts:", work)
    finally:
        for process in reversed(processes):
            process.terminate()
        for process in processes:
            process.wait(timeout=10)
        for output in outputs:
            output.close()
        server.shutdown()
        print("Logs:", work, flush=True)


if __name__ == "__main__":
    asyncio.run(main())
