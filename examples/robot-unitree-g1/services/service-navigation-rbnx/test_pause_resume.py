"""Exercise production pause/cancel/resume callbacks without robot hardware."""
import ast
import math
import json
import os
import queue
import threading
import tempfile
import time
import unittest
import uuid
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

class PauseResumeTest(unittest.TestCase):
    def setUp(self):
        """Load actual bridge functions with fake action transport and measured odometry."""
        selected = {'_pause_impl', '_resume_impl', '_cancel_impl', '_resolve_run_id',
                    '_status_impl', '_stationary', '_observe_odom', '_result_cb',
                    '_canonical_state', '_goal_status_name', '_cancel_response_cb', '_observe_nav_status',
                    '_observe_chassis_health', '_chassis_health_error', '_navigate_impl',
                    '_quat_to_yaw', '_dispatch_goal'}
        selected.add('_greeting_motion_error')
        tree = ast.parse((Path(__file__).parent / 'nav2_wrapper/atlas_bridge.py').read_text())
        module = ast.Module(body=[n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in selected], type_ignores=[])
        environment = patch.dict(os.environ, {'ROBONIX_GREETING_STATE_PATH': ''})
        environment.start()
        self.addCleanup(environment.stop)
        self.n = dict(time=time, math=math, uuid=uuid, os=os, json=json, Path=Path, _state_lock=threading.Lock(),
            _chassis_health_required=False, _chassis_health_sample=None,
            _chassis_health_level=3, _chassis_faulted=False,
            _chassis_health_message='waiting', _ros_node=object(),
            _last_run_id='run', _goal_states={'run': {'state': 'RUNNING'}},
            _nav_status_active=False, _nav_action_ready=True,
            _goal_payloads={'run': {'x': 1, 'y': 2, 'yaw': 0, 'frame_id': 'map'}},
            _goal_handles={'run': SimpleNamespace(goal_id=SimpleNamespace(uuid=bytes(16)))},
            _nav_queue=queue.Queue(), _issue_cancel=Mock(return_value=(True, 'requested')),
            _restore_speed_after_run=Mock(), _odom_sample=time.monotonic(),
            _stationary_since=time.monotonic()-1, _GoalStatus=SimpleNamespace(STATUS_UNKNOWN=0,
                STATUS_ACCEPTED=1, STATUS_EXECUTING=2, STATUS_CANCELING=3,
                STATUS_SUCCEEDED=4, STATUS_CANCELED=5, STATUS_ABORTED=6),
            _guard_trace_dir=None, read_guard_failure=Mock(return_value=None),
            classify_guard_result=lambda state, cancel, failure: (state, ''),
            _nav_diagnostics=[], _local_costmap_snapshot=None,
            format_result_detail=lambda *args: str(args), log=Mock())
        exec(compile(module, '<bridge>', 'exec'), self.n)

    def test_greeting_restriction_survives_cancellation_of_paused_run(self):
        """Cancelling A does not allow B before the greeting releases motion."""
        n = self.n
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'greeting.json'
            path.write_text(json.dumps({'movement_allowed': False, 'detail': 'listening'}))
            with patch.dict(os.environ, {'ROBONIX_GREETING_STATE_PATH': str(path)}):
                n['_goal_states']['run']['state'] = 'PAUSED'
                self.assertFalse(n['_resume_impl']('run')['accepted'])
                n['_cancel_impl']('run')
                goal = SimpleNamespace(header=SimpleNamespace(frame_id='map'), pose=SimpleNamespace(
                    orientation=SimpleNamespace(z=0, w=1), position=SimpleNamespace(x=1, y=2)))
                self.assertFalse(n['_navigate_impl'](goal)['accepted'])
                path.write_text(json.dumps({'movement_allowed': True}))
                self.assertTrue(n['_navigate_impl'](goal)['accepted'])

    def test_g1_waits_for_actual_cancel_before_admitting_another_goal(self):
        """An accepted cancellation is not a terminal Nav2 result."""
        n = self.n
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'greeting.json'
            path.write_text(json.dumps({'movement_allowed': True}))
            with patch.dict(os.environ, {'ROBONIX_GREETING_STATE_PATH': str(path)}):
                n['_cancel_impl']('run')
                goal = SimpleNamespace(header=SimpleNamespace(frame_id='map'), pose=SimpleNamespace(
                    orientation=SimpleNamespace(z=0, w=1), position=SimpleNamespace(x=1, y=2)))
                self.assertFalse(n['_navigate_impl'](goal)['accepted'])
                n['_result_cb'](SimpleNamespace(result=lambda: SimpleNamespace(status=5)), 'run')
                self.assertTrue(n['_navigate_impl'](goal)['accepted'])

    def test_queued_goal_cannot_dispatch_during_greeting_or_unreadable_state(self):
        """The dispatch check covers goals already queued before greeting began."""
        n = self.n
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'greeting.json'
            with patch.dict(os.environ, {'ROBONIX_GREETING_STATE_PATH': str(path)}):
                n['_goal_states']['run']['state'] = 'PENDING'
                n['_dispatch_goal'](None, 'run', n['_goal_payloads']['run'])
                self.assertEqual(n['_goal_states']['run']['state'], 'FAILED')
                self.assertIn('state unavailable', n['_goal_states']['run']['detail'])

    def test_pause_result_resume_keeps_run_and_goal(self):
        """Nav2 cancellation becomes PAUSED and resume queues the saved original goal."""
        n = self.n
        self.assertTrue(n['_pause_impl']('')['accepted'])
        self.assertIsNone(n['_stationary_since'])
        n['_result_cb'](SimpleNamespace(result=lambda: SimpleNamespace(status=5)), 'run')
        self.assertEqual(n['_status_impl']('run')['state'], 'PAUSED')
        n['_stationary_since'] = time.monotonic() - 1
        self.assertTrue(n['_resume_impl']('run')['accepted'])
        self.assertEqual(n['_nav_queue'].get_nowait(), ('run', n['_goal_payloads']['run']))
        n['_restore_speed_after_run'].assert_not_called()

    def test_user_cancel_prevents_resume(self):
        """A user cancellation during greeting cannot be revived by its completion."""
        n = self.n
        n['_goal_states']['run']['state'] = 'PAUSED'
        n['_goal_handles'].clear()
        self.assertTrue(n['_cancel_impl']('run')['accepted'])
        self.assertFalse(n['_resume_impl']('run')['accepted'])
        self.assertEqual(n['_status_impl']('run')['state'], 'CANCELED')

    def test_idle_greeting_requires_no_goals_and_fresh_stopping(self):
        """Only empty selection with no live runs and fresh odometry permits idle greeting."""
        n = self.n
        n['_goal_states'].clear()
        result = n['_pause_impl']('')
        self.assertEqual(result['detail'], 'no active navigation; stationary=true')
        self.assertEqual(result['run_id'], '')
        n['_issue_cancel'].assert_not_called()
        self.assertNotEqual(n['_pause_impl']('missing')['detail'], result['detail'])
        n['_nav_action_ready'] = False
        self.assertNotEqual(n['_pause_impl']('')['detail'], result['detail'])
        n['_nav_action_ready'] = True
        n['_stationary_since'] = None
        self.assertNotEqual(n['_pause_impl']('')['detail'], result['detail'])
        n['_stationary_since'] = time.monotonic() - 1
        n['_odom_sample'] = time.monotonic() - 1
        self.assertNotEqual(n['_pause_impl']('')['detail'], result['detail'])

    def test_external_or_pending_navigation_blocks_idle_greeting(self):
        """RViz action status and queued/paused runs cannot be treated as idle."""
        n = self.n
        n['_goal_states'].clear()
        n['_observe_nav_status'](SimpleNamespace(status_list=[SimpleNamespace(status=2)]))
        self.assertEqual(n['_pause_impl']('')['detail'], 'no active navigation')
        n['_observe_nav_status'](SimpleNamespace(status_list=[SimpleNamespace(status=4)]))
        for state in ('PENDING', 'PAUSED'):
            n['_goal_states']['run'] = {'state': state}
            self.assertEqual(n['_pause_impl']('')['detail'], 'no active navigation')

    def test_ordinary_navigation_terminal_results_are_unchanged(self):
        """Without a greeting pause, success, failure and cancel remain terminal."""
        for status, expected in ((4, 'SUCCEEDED'), (6, 'FAILED'), (5, 'CANCELED')):
            with self.subTest(status=status):
                self.setUp()
                n = self.n
                n['_result_cb'](SimpleNamespace(result=lambda: SimpleNamespace(status=status)), 'run')
                self.assertEqual(n['_status_impl']('run')['state'], expected)
                n['_restore_speed_after_run'].assert_called_once_with('run')
                self.assertTrue(n['_nav_queue'].empty())

    def test_stale_odometry_and_replaced_run_reject_resume(self):
        """Missing measured motion or a different current goal cannot resume old work."""
        n = self.n
        n['_goal_states']['run']['state'] = 'PAUSED'
        n['_odom_sample'] = time.monotonic()-1
        self.assertFalse(n['_resume_impl']('run')['accepted'])
        n['_odom_sample'] = time.monotonic()
        n['_last_run_id'] = 'new'
        self.assertFalse(n['_resume_impl']('run')['accepted'])

    def test_failed_pause_does_not_claim_ownership(self):
        """Transport rejection must not leave the run owned by greeting."""
        n = self.n
        n['_issue_cancel'].return_value = (False, 'offline')
        self.assertFalse(n['_pause_impl']('run')['accepted'])
        self.assertFalse(n['_goal_states']['run']['pause_requested'])

    def test_cancel_rejected_by_nav2_clears_pause(self):
        """A submitted request is not proof of successful cancellation."""
        n = self.n
        n['_pause_impl']('run')
        n['_cancel_response_cb'](SimpleNamespace(result=lambda: SimpleNamespace(goals_canceling=[])), 'run')
        self.assertFalse(n['_goal_states']['run']['pause_requested'])
        self.assertFalse(n['_goal_states']['run']['cancel_requested'])
        self.assertEqual(n['_status_impl']('run')['state'], 'RUNNING')

    def test_old_cancel_reply_cannot_mutate_resumed_run(self):
        """An old reply cannot install cancellation intent on the next action."""
        n = self.n
        old_state = n['_goal_states']['run']
        n['_goal_states']['run'] = {'state': 'RUNNING', 'cancel_requested': False}
        n['_cancel_response_cb'](SimpleNamespace(result=lambda: SimpleNamespace(goals_canceling=[1])), 'run', old_state)
        self.assertFalse(n['_goal_states']['run']['cancel_requested'])

    def test_user_cancel_during_pause_never_becomes_paused(self):
        """User cancellation wins before the Nav2 result arrives."""
        n = self.n
        n['_pause_impl']('run')
        n['_cancel_impl']('run')
        n['_result_cb'](SimpleNamespace(result=lambda: SimpleNamespace(status=5)), 'run')
        self.assertEqual(n['_status_impl']('run')['state'], 'CANCELED')
        self.assertFalse(n['_resume_impl']('run')['accepted'])

    def test_multiple_active_runs_reject_pause(self):
        """Greeting cannot stop one action while another action is queued."""
        n = self.n
        n['_goal_states']['other'] = {'state': 'PENDING'}
        self.assertFalse(n['_pause_impl']('run')['accepted'])
        n['_issue_cancel'].assert_not_called()

    def test_moving_or_missing_odometry_never_confirms_stop(self):
        """Measured motion resets the dwell time and stale samples expire."""
        n = self.n
        msg = SimpleNamespace(twist=SimpleNamespace(twist=SimpleNamespace(
            linear=SimpleNamespace(x=0.1, y=0.0), angular=SimpleNamespace(z=0.0))))
        n['_observe_odom'](msg)
        self.assertFalse(n['_stationary']())
        msg.twist.twist.linear.x = float('nan')
        n['_observe_odom'](msg)
        self.assertFalse(n['_stationary']())

    def test_chassis_fault_blocks_resume_even_with_fresh_stationary_odometry(self):
        """Live display telemetry cannot authorize movement after an SDK fault."""
        n = self.n
        n['_chassis_health_required'] = True
        n['_goal_states']['run']['state'] = 'PAUSED'
        status = SimpleNamespace(name='g1_chassis', hardware_id='g1_chassis',
                                 level=2, message='motion fault')
        n['_observe_chassis_health'](SimpleNamespace(status=[status]))
        n['_stationary_since'] = time.monotonic() - 1
        n['_odom_sample'] = time.monotonic()
        self.assertFalse(n['_stationary']())
        result = n['_resume_impl']('run')
        self.assertFalse(result['accepted'])
        self.assertIn('chassis motion fault', result['detail'])
        self.assertIn('chassis motion fault', n['_status_impl']('run')['detail'])
        status.level = 0
        n['_observe_chassis_health'](SimpleNamespace(status=[status]))
        self.assertFalse(n['_resume_impl']('run')['accepted'])
        self.assertTrue(n['_nav_queue'].empty())

    def test_g1_requires_diagnostics_but_other_chassis_keep_existing_behavior(self):
        """Missing, stale and unrelated diagnostics cannot authorize G1 recovery."""
        n = self.n
        n['_goal_states']['run']['state'] = 'PAUSED'
        n['_chassis_health_required'] = True
        self.assertFalse(n['_resume_impl']('run')['accepted'])
        status = SimpleNamespace(name='other', hardware_id='other', level=0, message='ok')
        n['_observe_chassis_health'](SimpleNamespace(status=[status]))
        self.assertFalse(n['_resume_impl']('run')['accepted'])
        status.name = status.hardware_id = 'g1_chassis'
        n['_observe_chassis_health'](SimpleNamespace(status=[status]))
        n['_chassis_health_sample'] -= 1
        self.assertFalse(n['_resume_impl']('run')['accepted'])
        n['_observe_chassis_health'](SimpleNamespace(status=[status]))
        n['_stationary_since'] = time.monotonic() - 1
        self.assertTrue(n['_resume_impl']('run')['accepted'])

    def test_chassis_fault_rejects_new_and_already_queued_goals(self):
        """Chassis health is checked both before queueing and before sending to Nav2."""
        n = self.n
        n['_chassis_health_required'] = n['_chassis_faulted'] = True
        goal = SimpleNamespace(header=SimpleNamespace(frame_id='map'),
            pose=SimpleNamespace(orientation=SimpleNamespace(z=0, w=1)))
        self.assertFalse(n['_navigate_impl'](goal)['accepted'])
        n['_goal_states']['run']['state'] = 'PENDING'
        n['_dispatch_goal'](object(), 'run', {})
        self.assertEqual(n['_goal_states']['run']['state'], 'FAILED')
        self.assertIn('chassis motion fault', n['_goal_states']['run']['detail'])

    def test_ros_byte_diagnostic_levels_are_decoded(self):
        """Humble's single-byte diagnostic level must enforce the same motion gate."""
        n = self.n
        n['_chassis_health_required'] = True
        status = SimpleNamespace(name='g1_chassis', hardware_id='g1_chassis',
                                 level=b'\x00', message='ok')
        n['_observe_chassis_health'](SimpleNamespace(status=[status]))
        self.assertEqual(n['_chassis_health_error'](), '')
        status.level = b'\x02'
        n['_observe_chassis_health'](SimpleNamespace(status=[status]))
        self.assertIn('chassis motion fault', n['_chassis_health_error']())

if __name__ == '__main__':
    unittest.main()
