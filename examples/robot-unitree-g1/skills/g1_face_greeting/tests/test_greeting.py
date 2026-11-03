import sys
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
sys.path.insert(0, str(PACKAGE / "rbnx-build/codegen/robonix_mcp_types"))
sys.path.insert(0, str(PACKAGE.parents[3] / "pylib/robonix-api"))

from g1_face_greeting import main  # noqa: E402


class GreetingTests(unittest.TestCase):
    def setUp(self):
        """Keep coverage of the arm-enabled path while isolating the SDK with fakes."""
        arm_mode = patch.object(main, 'ARM_GREETING_ENABLED', True)
        arm_mode.start()
        self.addCleanup(arm_mode.stop)
        main._settings = main.Settings.parse({"cooldown_sec": 30})
        main._speech_connection = SimpleNamespace(endpoint="speech")
        main._arm_connection = SimpleNamespace(endpoint="arm")
        main._last_greet.clear()
        main._seen_events.clear()

    def test_config_allows_navigation_and_keeps_turning_disabled(self):
        settings = main.Settings.parse({})
        self.assertFalse(settings.navigation_enabled)
        self.assertFalse(settings.turn_to_face_enabled)
        self.assertTrue(main.Settings.parse({"navigation_enabled": True}).navigation_enabled)
        with self.assertRaisesRegex(ValueError, "must remain false"):
            main.Settings.parse({"turn_to_face_enabled": True})

    def test_intimacy_levels_select_allowlisted_actions(self):
        expected = {
            1: "wave_under_head",
            2: "wave_under_head",
            3: "wave_above_head",
            4: "make_heart_with_right_hand",
            5: "make_heart_with_both_hands",
        }
        for level, action in expected.items():
            with self.subTest(level=level):
                main._last_greet.clear()
                main._seen_events.clear()
                def call(connection, tool, payload, timeout):
                    if tool == "speak":
                        return {"ok": True}
                    return {"accepted": True}

                with patch.object(main, "_call_sync", side_effect=call) as call_mock:
                    result = main._greet({"event_id": str(level), "person_id": level, "intimacy_level": level})
                self.assertTrue(result["accepted"])
                self.assertEqual(result["action"], action)
                action_calls = [item for item in call_mock.call_args_list if item.args[1] == "run"]
                self.assertEqual(action_calls[0].args[2], {"action": action})

    def test_invalid_levels_are_rejected_without_side_effects(self):
        for level in (0, 6, 1.5, "2", True, None):
            with self.subTest(level=level):
                with patch.object(main, "_call_sync") as call:
                    result = main._greet({"person_id": 1, "intimacy_level": level})
                self.assertFalse(result["accepted"])
                call.assert_not_called()

    def test_duplicate_event_and_person_cooldown_are_suppressed(self):
        def call(connection, tool, payload, timeout):
            if tool == "speak":
                return {"ok": True}
            return {"accepted": True}

        with patch.object(main, "_call_sync", side_effect=call) as call_mock:
            accepted = main._greet({"event_id": "event-1", "person_id": 7, "intimacy_level": 1})
            duplicate = main._greet({"event_id": "event-1", "person_id": 7, "intimacy_level": 1})
            cooldown = main._greet({"event_id": "event-2", "person_id": 7, "intimacy_level": 1})
        self.assertTrue(accepted["accepted"])
        self.assertEqual(duplicate["detail"], "duplicate event")
        self.assertEqual(cooldown["detail"], "greeting cooldown")
        self.assertEqual(call_mock.call_count, 2)

    def test_speech_and_arm_failures_are_reported_without_retry(self):
        def call(connection, tool, payload, timeout):
            if tool == "speak":
                raise RuntimeError("speech offline")
            return {"accepted": False, "detail": "busy"}

        with patch.object(main, "_call_sync", side_effect=call) as call_mock:
            result = main._greet({"event_id": "failure", "person_id": 9, "intimacy_level": 2})
        self.assertTrue(result["accepted"])
        self.assertFalse(result["speech_ok"])
        self.assertFalse(result["action_ok"])
        self.assertIn("speech_error=speech offline", result["detail"])
        self.assertIn("action=busy", result["detail"])
        self.assertEqual(call_mock.call_count, 2)

    def test_speech_and_arm_start_without_waiting_for_each_other(self):
        started = {"speak": threading.Event(), "run": threading.Event()}
        release = threading.Event()

        def call(connection, tool, payload, timeout):
            started[tool].set()
            self.assertTrue(release.wait(timeout=1))
            return {"ok": True} if tool == "speak" else {"accepted": True}

        with patch.object(main, "_call_sync", side_effect=call):
            result_holder = {}
            thread = threading.Thread(
                target=lambda: result_holder.setdefault(
                    "result",
                    main._greet({"event_id": "parallel", "person_id": 10, "intimacy_level": 1}),
                )
            )
            thread.start()
            self.assertTrue(started["speak"].wait(timeout=1))
            self.assertTrue(started["run"].wait(timeout=1))
            release.set()
            thread.join(timeout=2)

        self.assertFalse(thread.is_alive())
        self.assertTrue(result_holder["result"]["speech_ok"])
        self.assertTrue(result_holder["result"]["action_ok"])

    def test_mcp_result_reads_structured_or_text_json(self):
        structured = SimpleNamespace(structuredContent={"available": True}, content=[])
        text = SimpleNamespace(structuredContent=None, content=[SimpleNamespace(text='{"ok":true}')])
        self.assertEqual(main._result_value(structured), {"available": True})
        self.assertEqual(main._result_value(text), {"ok": True})

    def test_mcp_error_preserves_original_reason(self):
        """Server text errors reach callers without losing the hardware failure detail."""
        error = SimpleNamespace(isError=True, structuredContent=None,
                                content=[SimpleNamespace(text='SDK RPC timeout code=3103')])
        with self.assertRaisesRegex(RuntimeError, 'SDK RPC timeout code=3103'):
            main._result_value(error)

    def test_event_poll_uses_contract_leaf_as_mcp_tool_name(self):
        main._face_connection = SimpleNamespace(endpoint="face")
        main._stop.clear()

        def one_poll(connection, tool, payload, timeout):
            main._stop.set()
            return {"available": False}

        try:
            with patch.object(main, "_call_sync", side_effect=one_poll) as call:
                main._worker_loop()
            self.assertEqual(call.call_args.args[1], "next")
            self.assertEqual(call.call_args.args[2], {"wait_ms": 1000})
        finally:
            main._stop.clear()


class NavigationGreetingTests(unittest.TestCase):
    def setUp(self):
        """Enable coordination with deterministic fake navigation responses."""
        arm_mode = patch.object(main, 'ARM_GREETING_ENABLED', True)
        arm_mode.start()
        self.addCleanup(arm_mode.stop)
        main._settings = main.Settings.parse({'navigation_enabled': True})
        main._last_greet.clear()
        main._seen_events.clear()
        main._stop.clear()
        self.event = {'person_id': 12, 'event_id': 'new', 'intimacy_level': 4}

    def test_idle_greeting_waits_and_releases_without_resume(self):
        """Confirmed idle encounters greet and clean up, including preset/speech failures."""
        for speech_ok, action_ok, release_ok in ((True, True, True), (False, False, True), (True, True, False)):
            with self.subTest(speech_ok=speech_ok, action_ok=action_ok, release_ok=release_ok):
                order = []
                greeting = {'accepted': True, 'speech_ok': speech_ok, 'action_ok': action_ok,
                            'playback_wait_sec': 15, 'detail': 'original result'}
                idle = {'accepted': False, 'run_id': '', 'detail': 'no active navigation; stationary=true'}
                with patch.object(main, '_navigation_call', return_value=idle) as nav, patch.object(main, '_greet_stationary', side_effect=lambda _: order.append('greet') or greeting), patch.object(main._stop, 'wait', side_effect=lambda _: order.append('wait')) as wait, patch.object(main, '_release_arm', side_effect=lambda: order.append('release') or {'ok': release_ok, 'detail': 'release feedback'}) as release, patch.object(main.time, 'monotonic', return_value=100.0):
                    result = main._greet(self.event)
                nav.assert_called_once_with('pause', {'run_id': ''})
                wait.assert_called_once_with(15)
                release.assert_called_once()
                self.assertEqual(order, ['greet', 'wait', 'release'])
                self.assertEqual(result['speech_ok'], speech_ok)
                self.assertEqual(result['action_ok'], action_ok)
                self.assertIn('original result', result['detail'])
                self.assertIn('no navigation to resume', result['detail'])
                self.assertEqual('arm release unconfirmed' in result['detail'], not release_ok)

    def test_playback_wait_overlaps_calls_before_release_and_resume(self):
        """Idle and paused greetings wait only the remainder, then confirm release."""
        for active in (False, True):
            for call_sec, total_sec in ((0.0, 12.0), (7.745, 12.0), (12.0, 12.0),
                                        (20.0, 12.0), (7.745, 40.0)):
                with self.subTest(active=active, call_sec=call_sec, total_sec=total_sec):
                    clock = SimpleNamespace(now=100.0)
                    order = []

                    def nav(tool, payload):
                        """Confirm stopping and preserve the original navigation run."""
                        order.append(tool)
                        if tool == 'pause':
                            if not active:
                                return {'accepted': False, 'run_id': '',
                                        'detail': 'no active navigation; stationary=true'}
                            return {'accepted': True, 'run_id': 'original'}
                        if tool == 'status':
                            return {'state': 'PAUSED', 'detail': 'stationary=true'}
                        return {'accepted': True, 'detail': 'resume queued'}

                    def greet(event):
                        """Advance the clock through the slower parallel SDK call."""
                        order.append('greet')
                        clock.now += call_sec
                        return {'accepted': True, 'speech_ok': True, 'action_ok': True,
                                'playback_wait_sec': total_sec, 'detail': 'ok'}

                    def wait(seconds):
                        order.append('wait')
                        clock.now += seconds

                    with patch.object(main, '_navigation_call', side_effect=nav) as calls, \
                            patch.object(main, '_greet_stationary', side_effect=greet), \
                            patch.object(main.time, 'monotonic', side_effect=lambda: clock.now), \
                            patch.object(main._stop, 'wait', side_effect=wait) as waited, \
                            patch.object(main, '_release_arm', side_effect=lambda: order.append('release') or {'ok': True, 'detail': 'confirmed'}):
                        result = main._greet(self.event)
                    self.assertTrue(result['accepted'])
                    waited.assert_called_once()
                    self.assertAlmostEqual(waited.call_args.args[0], max(0.0, total_sec - call_sec))
                    self.assertAlmostEqual(clock.now - 100.0, max(call_sec, total_sec))
                    expected = (['pause', 'status', 'greet', 'wait', 'release', 'status', 'resume']
                                if active else ['pause', 'greet', 'wait', 'release'])
                    self.assertEqual(order, expected)
                    if active:
                        self.assertEqual(calls.call_args.args, ('resume', {'run_id': 'original'}))

    def test_idle_requires_explicit_stationary_confirmation(self):
        """Legacy, moving, or ambiguous rejection cannot authorize arm movement."""
        for response in ({'accepted': False, 'detail': 'no active navigation'},
                         {'accepted': False, 'detail': 'no active navigation; stationary=false'},
                         {'accepted': False, 'run_id': 'r', 'detail': 'no active navigation; stationary=true'}):
            with patch.object(main, '_navigation_call', return_value=response), patch.object(main, '_greet_stationary') as greet, patch.object(main, '_release_arm') as release:
                self.assertFalse(main._greet(self.event)['accepted'])
            greet.assert_not_called()
            release.assert_not_called()

    def test_idle_exception_still_releases_arm(self):
        """Unexpected idle greeting failure still releases and reports the original error."""
        idle = {'accepted': False, 'run_id': '', 'detail': 'no active navigation; stationary=true'}
        with patch.object(main, '_navigation_call', return_value=idle) as nav, patch.object(main, '_greet_stationary', side_effect=RuntimeError('idle greeting failed')), patch.object(main, '_release_arm', return_value={'ok': True, 'detail': 'confirmed'}) as release:
            result = main._greet(self.event)
        release.assert_called_once()
        self.assertEqual(nav.call_count, 1)
        self.assertIn('idle greeting failed', result['detail'])

    def test_idle_shutdown_during_wait_still_releases_arm(self):
        """Shutdown interrupts the idle wait but preserves cleanup without motion requests."""
        idle = {'accepted': False, 'run_id': '', 'detail': 'no active navigation; stationary=true'}
        greeting = {'accepted': True, 'speech_ok': True, 'action_ok': True, 'detail': 'ok'}
        with patch.object(main, '_navigation_call', return_value=idle) as nav, patch.object(main, '_greet_stationary', return_value=greeting), patch.object(main._stop, 'wait', side_effect=lambda _: main._stop.set()), patch.object(main, '_release_arm', return_value={'ok': True, 'detail': 'confirmed'}) as release:
            main._greet(self.event)
        main._stop.clear()
        release.assert_called_once()
        self.assertEqual(nav.call_count, 1)

    def test_pause_stop_greet_wait_resume_order(self):
        """No speech/arm is started before a confirmed stop, and resume uses the same ID."""
        order = []
        def nav(tool, payload):
            order.append(tool)
            if tool == 'status':
                return {'state': 'PAUSED', 'detail': 'navigation paused; stationary=true'}
            return {'accepted': True, 'run_id': 'original', 'detail': 'ok'}
        def greeting(event):
            order.append('greet')
            return {'speech_ok': True, 'action_ok': True, 'accepted': True, 'detail': 'ok'}
        with patch.object(main, '_navigation_call', side_effect=nav) as calls, patch.object(main, '_greet_stationary', side_effect=greeting), patch.object(main, '_release_arm', side_effect=lambda: order.append('release') or {'ok': True, 'detail': 'confirmed'}), patch.object(main._stop, 'wait', return_value=False):
            result = main._greet(self.event)
        self.assertTrue(result['accepted'])
        self.assertEqual(order, ['pause', 'status', 'greet', 'release', 'status', 'resume'])
        self.assertEqual(calls.call_args.args[1], {'run_id': 'original'})

    def test_rejected_pause_and_cancelled_run_do_not_greet(self):
        """Pause rejection or cancellation before stopping suppresses physical greeting."""
        for responses in ([{'accepted': False, 'detail': 'busy'}],
                          [{'accepted': True, 'run_id': 'r'}, {'state': 'CANCELED'}]):
            with patch.object(main, '_navigation_call', side_effect=responses), patch.object(main, '_greet_stationary') as greet:
                result = main._greet(self.event)
            self.assertFalse(result['accepted'])
            greet.assert_not_called()

    def test_greeting_failures_resume_after_confirmed_release(self):
        """Speech or preset failure does not block a verified cleanup and resume."""
        for speech_ok, action_ok in ((False, True), (True, False), (False, False)):
            with self.subTest(speech_ok=speech_ok, action_ok=action_ok):
                responses = [{'accepted': True, 'run_id': 'r'},
                             {'state': 'PAUSED', 'detail': 'stationary=true'},
                             {'state': 'PAUSED', 'detail': 'stationary=true'},
                             {'accepted': True, 'detail': 'resume queued'}]
                greeting = {'accepted': True, 'speech_ok': speech_ok, 'action_ok': action_ok, 'detail': 'original failure'}
                with patch.object(main, '_navigation_call', side_effect=responses) as nav, patch.object(main, '_greet_stationary', return_value=greeting), patch.object(main, '_release_arm', return_value={'ok': True, 'detail': 'confirmed'}) as release, patch.object(main._stop, 'wait', return_value=False):
                    result = main._greet(self.event)
                release.assert_called_once()
                self.assertEqual(nav.call_args.args, ('resume', {'run_id': 'r'}))
                self.assertEqual(result['speech_ok'], speech_ok)
                self.assertEqual(result['action_ok'], action_ok)
                self.assertIn('original failure', result['detail'])

    def test_unconfirmed_release_prevents_resume(self):
        """Missing or rejected release feedback retains the navigation pause."""
        responses = [{'accepted': True, 'run_id': 'r'}, {'state': 'PAUSED', 'detail': 'stationary=true'}]
        with patch.object(main, '_navigation_call', side_effect=responses) as nav, patch.object(main, '_greet_stationary', return_value={'accepted': True, 'speech_ok': True, 'action_ok': True, 'detail': 'ok'}), patch.object(main, '_release_arm', return_value={'ok': False, 'detail': 'release timeout'}), patch.object(main._stop, 'wait', return_value=False):
            result = main._greet(self.event)
        self.assertEqual(nav.call_count, 2)
        self.assertIn('remains paused', result['detail'])
        self.assertIn('release timeout', result['detail'])

    def test_cancel_or_stale_motion_after_release_prevents_resume(self):
        """A successful arm release cannot override cancellation or stale motion."""
        for state in ({'state': 'CANCELED'}, {'state': 'PAUSED', 'detail': 'stationary=false'}):
            responses = [{'accepted': True, 'run_id': 'r'}, {'state': 'PAUSED', 'detail': 'stationary=true'}, state]
            with patch.object(main, '_navigation_call', side_effect=responses) as nav, patch.object(main, '_greet_stationary', return_value={'accepted': True, 'speech_ok': True, 'action_ok': True, 'detail': 'ok'}), patch.object(main, '_release_arm', return_value={'ok': True, 'detail': 'confirmed'}), patch.object(main._stop, 'wait', return_value=False):
                result = main._greet(self.event)
            self.assertEqual(nav.call_count, 3)
            self.assertIn('not resumed', result['detail'])

    def test_greeting_exception_still_releases_without_resuming(self):
        """Unexpected failures still run cleanup but do not infer safe completion."""
        responses = [{'accepted': True, 'run_id': 'r'}, {'state': 'PAUSED', 'detail': 'stationary=true'}]
        with patch.object(main, '_navigation_call', side_effect=responses) as nav, patch.object(main, '_greet_stationary', side_effect=RuntimeError('unexpected failure')), patch.object(main, '_release_arm', return_value={'ok': True, 'detail': 'confirmed'}) as release:
            result = main._greet(self.event)
        release.assert_called_once()
        self.assertEqual(nav.call_count, 2)
        self.assertIn('unexpected failure', result['detail'])

    def test_shutdown_still_releases_without_resuming(self):
        """Shutdown during the greeting wait cleans up but never restarts motion."""
        responses = [{'accepted': True, 'run_id': 'r'}, {'state': 'PAUSED', 'detail': 'stationary=true'}]
        with patch.object(main, '_navigation_call', side_effect=responses) as nav, patch.object(main, '_greet_stationary', return_value={'accepted': True, 'speech_ok': True, 'action_ok': True, 'detail': 'ok'}), patch.object(main, '_release_arm', return_value={'ok': True, 'detail': 'confirmed'}) as release, patch.object(main._stop, 'wait', side_effect=lambda _: main._stop.set()):
            result = main._greet(self.event)
        main._stop.clear()
        release.assert_called_once()
        self.assertEqual(nav.call_count, 2)
        self.assertIn('skill stopped', result['detail'])

    def test_release_requires_feedback_marker_and_keeps_errors(self):
        """Old helpers returning acceptance alone cannot authorize navigation resume."""
        for response, expected in (({'accepted': True, 'detail': 'SDK2 action accepted: release_arm'}, False),
                                   ({'accepted': True, 'detail': 'release_arm confirmed by action state: id=0 holding=false name=""; idle confirmed for >=0.2s'}, True),
                                   ({'accepted': True, 'detail': 'Arm action feedback after release request: id=0 holding=false'}, False),
                                   ({'accepted': False, 'detail': 'release_arm confirmed by action state: id=0 holding=false name=""; idle confirmed for >=0.2s'}, False),
                                   ({'accepted': True, 'detail': 'release_arm confirmed by action state: id=99 holding=false'}, True)):
            with patch.object(main, '_timed_call', return_value=response):
                self.assertEqual(main._release_arm()['ok'], expected)
        with patch.object(main, '_timed_call', side_effect=TimeoutError('release transport timeout')):
            result = main._release_arm()
        self.assertFalse(result['ok'])
        self.assertIn('release transport timeout', result['detail'])

    def test_resume_exception_preserves_both_greeting_results(self):
        """A resume failure must not erase successful speech and arm feedback."""
        responses = [{'accepted': True, 'run_id': 'r'}, {'state': 'PAUSED', 'detail': 'stationary=true'}, {'state': 'PAUSED', 'detail': 'stationary=true'}, TimeoutError('resume RPC timeout')]
        with patch.object(main, '_navigation_call', side_effect=responses), patch.object(main, '_greet_stationary', return_value={'accepted': True, 'speech_ok': True, 'action_ok': True, 'detail': 'speech=ok; action=ok'}), patch.object(main, '_release_arm', return_value={'ok': True, 'detail': 'confirmed'}), patch.object(main._stop, 'wait', return_value=False):
            result = main._greet(self.event)
        self.assertTrue(result['speech_ok'])
        self.assertTrue(result['action_ok'])
        self.assertIn('TimeoutError: resume RPC timeout', result['detail'])

    def test_stop_timeout_does_not_greet(self):
        """Navigation cancellation alone is insufficient without fresh stationary motion."""
        with patch.object(main, '_navigation_call', return_value={'accepted': True, 'run_id': 'r'}), patch.object(main.time, 'monotonic', side_effect=[100, 100, 109]), patch.object(main, '_greet_stationary') as greet:
            result = main._greet(self.event)
        self.assertFalse(result['accepted'])
        greet.assert_not_called()


class ArmSdkIsolationTests(unittest.TestCase):
    def setUp(self):
        """Exercise speech-only navigation with no arm provider or robot required."""
        arm_mode = patch.object(main, 'ARM_GREETING_ENABLED', False)
        arm_mode.start()
        self.addCleanup(arm_mode.stop)
        main._settings = main.Settings.parse({'navigation_enabled': True})
        main._speech_connection = SimpleNamespace(endpoint='speech')
        main._arm_connection = None
        main._last_greet.clear()
        main._seen_events.clear()
        main._stop.clear()
        self.addCleanup(main._stop.clear)
        self.event = {'person_id': 12, 'event_id': 'isolation', 'intimacy_level': 4}

    def test_greeting_sends_only_speech_request(self):
        """Speech-only isolation sends no arm action request."""
        with patch.object(main, '_timed_call', return_value={'ok': True, 'detail': 'spoken'}) as call:
            result = main._greet_stationary(self.event)

        self.assertTrue(result['speech_ok'])
        self.assertFalse(result['action_ok'])
        self.assertEqual(result['action'], '')
        self.assertIn('arm action skipped for SDK isolation test', result['detail'])
        call.assert_called_once()
        self.assertEqual(call.call_args.args[:3], ('speech', main._speech_connection, 'speak'))

    def test_navigation_resumes_without_arm_release_request(self):
        """A stopped run resumes after speech wait without connecting to arm SDK."""
        order = []

        def nav(tool, payload):
            """Keep the original paused run ready for its resume request."""
            order.append(tool)
            if tool == 'pause':
                return {'accepted': True, 'run_id': 'original'}
            if tool == 'status':
                return {'state': 'PAUSED', 'detail': 'stationary=true'}
            if tool == 'resume':
                return {'accepted': True, 'detail': 'resume queued'}
            self.fail(f'unexpected navigation operation: {tool}')

        def speak(label, connection, tool, payload, timeout):
            self.assertEqual((label, connection, tool), ('speech', main._speech_connection, 'speak'))
            order.append('speech')
            return {'ok': True, 'detail': 'text accepted'}

        with patch.object(main, '_navigation_call', side_effect=nav), \
                patch.object(main, '_timed_call', side_effect=speak) as call, \
                patch.object(main.time, 'monotonic', return_value=100.0), \
                patch.object(main._stop, 'wait', side_effect=lambda _: order.append('wait')) as wait, \
                patch.object(main, '_release_arm') as release:
            result = main._greet(self.event)

        self.assertTrue(result['accepted'])
        self.assertTrue(result['speech_ok'])
        self.assertEqual(order, ['pause', 'status', 'speech', 'wait', 'status', 'resume'])
        call.assert_called_once()
        wait.assert_called_once_with(12.0)
        release.assert_not_called()

    def test_activation_does_not_connect_to_arm_provider(self):
        """The trial does not open the arm SDK connection when the skill starts."""
        class Connection:
            def close(self):
                pass

        class Worker:
            def start(self):
                pass

            def join(self, timeout):
                pass

            def is_alive(self):
                return False

        main._worker = None
        main._settings = main.Settings.parse({'navigation_enabled': True})
        connected_contracts = []

        def connect(_capability, contract, _transport):
            connected_contracts.append(contract)
            return Connection()

        with patch.object(main, 'MessageStore', return_value=object()), \
                patch.object(main, '_find', side_effect=lambda contract, _provider: contract), \
                patch.object(main, '_find_optional', return_value=None), \
                patch.object(main.skill, 'connect_capability', side_effect=connect), \
                patch.object(main.threading, 'Thread', return_value=Worker()):
            self.assertIsInstance(main.activate(), main.Ok)
            main.deactivate()

        self.assertNotIn(main.ARM, connected_contracts)

    def test_idle_speech_only_greeting_never_releases_arm_or_resumes(self):
        """An already stationary robot also sends only the speech request."""
        idle = {'accepted': False, 'run_id': '', 'detail': 'no active navigation; stationary=true'}
        with patch.object(main, '_navigation_call', return_value=idle) as nav, \
                patch.object(main, '_timed_call', return_value={'ok': True}) as call, \
                patch.object(main._stop, 'wait', return_value=False), \
                patch.object(main, '_release_arm') as release:
            result = main._greet(self.event)
        self.assertTrue(result['speech_ok'])
        self.assertIn('no navigation to resume', result['detail'])
        nav.assert_called_once_with('pause', {'run_id': ''})
        call.assert_called_once()
        release.assert_not_called()

    def test_cancellation_or_stale_odometry_prevents_speech_only_resume(self):
        """Skipping the arm SDK does not bypass navigation's stop checks."""
        for state in ({'state': 'CANCELED'}, {'state': 'PAUSED', 'detail': 'stationary=false'}):
            with self.subTest(state=state):
                main._last_greet.clear()
                main._seen_events.clear()
                responses = [{'accepted': True, 'run_id': 'r'},
                             {'state': 'PAUSED', 'detail': 'stationary=true'}, state]
                with patch.object(main, '_navigation_call', side_effect=responses) as nav, \
                        patch.object(main, '_timed_call', return_value={'ok': True}), \
                        patch.object(main._stop, 'wait', return_value=False), \
                        patch.object(main, '_release_arm') as release:
                    result = main._greet(self.event)
                self.assertIn('not resumed', result['detail'])
                self.assertEqual(nav.call_count, 3)
                release.assert_not_called()

    def test_shutdown_prevents_speech_only_resume(self):
        """Stopping this skill during its playback wait never restarts navigation."""
        responses = [{'accepted': True, 'run_id': 'r'},
                     {'state': 'PAUSED', 'detail': 'stationary=true'}]
        with patch.object(main, '_navigation_call', side_effect=responses) as nav, \
                patch.object(main, '_timed_call', return_value={'ok': True}), \
                patch.object(main._stop, 'wait', side_effect=lambda _: main._stop.set()), \
                patch.object(main, '_release_arm') as release:
            result = main._greet(self.event)
        self.assertIn('skill stopped', result['detail'])
        self.assertEqual(nav.call_count, 2)
        release.assert_not_called()


if __name__ == "__main__":
    unittest.main()
