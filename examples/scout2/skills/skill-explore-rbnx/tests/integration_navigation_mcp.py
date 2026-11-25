"""Real MCP transport smoke test against a synthetic provider; no ROS or chassis.

Run in a network-isolated container with the Explore runtime dependencies.
The local fake provider implements only navigation/status/cancel/get_speed_limit.
"""
import argparse
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace


def serve(port, events):
    """Host an isolated fake navigation MCP server with persistent event evidence."""
    from fastmcp import FastMCP
    server = FastMCP('explore-budget-fixture')
    state = {'cancelled': False}

    def record(tool, args):
        """Append calls so the parent can verify actual cross-process transport."""
        with Path(events).open('a') as stream:
            stream.write(json.dumps(dict(tool=tool, args=args)) + '\n')

    @server.tool()
    def get_speed_limit() -> dict:
        record('get_speed_limit', {})
        return dict(available=True, effective_linear_speed_mps=.14851485148514854)

    @server.tool()
    def navigate(goal: dict) -> dict:
        record('navigate', goal)
        state['cancelled'] = False
        return dict(accepted=True, run_id='fixture-owned-run')

    @server.tool()
    def status(run_id: str) -> dict:
        record('status', dict(run_id=run_id))
        return dict(known=run_id == 'fixture-owned-run',
                    state='CANCELED' if state['cancelled'] else 'RUNNING')

    @server.tool()
    def cancel(run_id: str) -> dict:
        record('cancel', dict(run_id=run_id))
        state['cancelled'] = True
        return dict(ok=True)

    server.run(transport='streamable-http', host='127.0.0.1', port=port,
               log_level='ERROR', show_banner=False)


def run():
    """Exercise live speed lookup, XY completion and confirmed cancellation over MCP."""
    from explore_skill.configuration import ExploreSettings
    from explore_skill.controller import ExploreController, TaskHandle
    if os.environ.get('ROBONIX_EXPLORE_TEST_ISOLATED') != '1':
        raise RuntimeError('run in the documented network-isolated container')
    with tempfile.TemporaryDirectory() as temporary:
        events = Path(temporary) / 'events.jsonl'
        with socket.socket() as listener:
            listener.bind(('127.0.0.1', 0))
            port = listener.getsockname()[1]
        process = subprocess.Popen([sys.executable, __file__, '--server', str(port),
                                    '--events', str(events)], stdout=subprocess.DEVNULL)
        stop = threading.Event()
        worker = None
        try:
            deadline = time.monotonic() + 15
            while True:
                if process.poll() is not None:
                    raise RuntimeError('synthetic MCP server failed to start')
                try:
                    with socket.create_connection(('127.0.0.1', port), timeout=.1):
                        break
                except OSError:
                    if time.monotonic() >= deadline:
                        raise TimeoutError('synthetic MCP server startup timed out')
                    time.sleep(.05)
            endpoint = f'http://127.0.0.1:{port}/mcp/'
            ctrl = ExploreController(map_topic='/fixture/map',
                nav_navigate_endpoint=endpoint, nav_status_endpoint=endpoint,
                nav_cancel_endpoint=endpoint, nav_get_speed_endpoint=endpoint,
                settings=ExploreSettings(arrival_settle_s=.1))
            ctrl.NAV_POLL_PERIOD_S = .05
            handle = TaskHandle(task_id='transport-test', started_at=time.time(),
                                 timeout_s=30., max_speed_m_s=.3)
            ctrl._latest_pose_xyyaw = (0., 0., 0.)
            ctrl._latest_pose_received_at = time.monotonic()
            ctrl._latest_map = SimpleNamespace(
                data=[0] * (100 * 100),
                info=SimpleNamespace(height=100, width=100, resolution=.1,
                    origin=SimpleNamespace(
                        position=SimpleNamespace(x=-5., y=-5.),
                        orientation=SimpleNamespace(x=0., y=0., z=0., w=1.))))
            speed = ctrl._navigation_speed(handle)
            assert abs(speed - .14851485148514854) < 1e-6
            assert ctrl._estimate_leg_seconds(14.55, speed) > 170

            def observe():
                """Simulate the clear observation map and fresh, stationary TF samples."""
                started = time.monotonic()
                while not stop.wait(.02):
                    with ctrl._lock:
                        if time.monotonic() - started > .1:
                            ctrl._latest_pose_xyyaw = (0., 1.6, .8)
                        ctrl._latest_pose_received_at = time.monotonic()
            worker = threading.Thread(target=observe)
            worker.start()
            ok, detail = ctrl._nav_navigate_blocking(0., 2., yaw=None,
                                                     timeout_s=5., cancel_evt=handle)
            assert ok, detail
            assert not handle.active_nav_run_id
            records = [json.loads(line) for line in events.read_text().splitlines()]
            goal = next(row['args'] for row in records if row['tool'] == 'navigate')
            assert abs(goal['pose']['orientation']['z'] - math.sin(math.pi/4)) < 1e-6
            assert any(row['tool'] == 'cancel' and row['args']['run_id'] == 'fixture-owned-run'
                       for row in records)
            assert records[-1]['tool'] == 'status'
            print('PASS actual MCP transport reads live speed and budgets the long route')
            print('PASS translation heading follows travel and fresh XY arrival ends the leg')
            print('PASS exact run cancellation and actual-pose clearance are verified before completion')
        finally:
            stop.set()
            if worker is not None:
                worker.join(timeout=1)
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--server', type=int)
    parser.add_argument('--events')
    args = parser.parse_args()
    if args.server:
        serve(args.server, args.events)
    else:
        run()
