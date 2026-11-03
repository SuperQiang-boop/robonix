"""Persistent G1 text playback checks without compiling SDK code or moving G1."""
import asyncio
import json
import os
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from speech_service.unitree_tts import UnitreeTTSBackend
from speech_service import service


# Emulate only the private helper protocol; record requests independently so
# tests detect accidental process replacement or replay after a lost reply.
_HELPER = r'''
import json
import os
import socket
import struct
import sys
import time

sock = socket.socket(fileno=int(sys.argv[2]))

def record(value):
    with open(os.environ['FAKE_TTS_LOG'], 'a') as output:
        output.write(json.dumps(value) + '\n')

def receive(size):
    """Accumulate a complete request, stopping when Speech closes the socket."""
    result = b''
    while len(result) < size:
        chunk = sock.recv(size - len(result))
        if not chunk:
            return None
        result += chunk
    return result

record({'started': os.getpid(), 'interface': sys.argv[1]})
ready = b'WRONG\n' if os.environ.get('FAKE_TTS_BAD_READY') else b'READY\n'
for byte in ready:
    sock.sendall(bytes([byte]))
while True:
    header = receive(4)
    if header is None:
        break
    size, = struct.unpack('!I', header)
    text = receive(size).decode('utf-8')
    record({'text': text})
    if text == 'DROP':
        break
    if text == 'TIMEOUT':
        time.sleep(30)
        break
    status = 3102 if text == 'SDK_FAIL' else 0
    for byte in struct.pack('!i', status):
        sock.sendall(bytes([byte]))
sock.close()
'''


class UnitreeTTSTests(unittest.TestCase):
    """Exercise the actual child-process IPC and Speech dispatch offline."""

    def setUp(self):
        """Create an executable fake helper in the normal deployment layout."""
        self.directory = tempfile.TemporaryDirectory()
        root = Path(self.directory.name)
        helper = root / 'primitives/g1_audio/rbnx-build/bin/g1_audio_tts_helper'
        helper.parent.mkdir(parents=True)
        helper.write_text(f'#!{sys.executable}\n' + _HELPER)
        helper.chmod(0o755)
        self.log = root / 'requests.jsonl'
        self.env = patch.dict(os.environ, {
            'ROBONIX_DEPLOY_DIR': str(root), 'G1_NETWORK_INTERFACE': 'eth-test',
            'FAKE_TTS_LOG': str(self.log),
        })
        self.env.start()
        self.backend = UnitreeTTSBackend()

    def tearDown(self):
        self.backend.close()
        self.env.stop()
        self.directory.cleanup()

    def records(self):
        """Read independent evidence of helper starts and delivered text."""
        if not self.log.exists():
            return []
        return [json.loads(line) for line in self.log.read_text().splitlines()]

    def test_two_phrases_reuse_live_client_and_preserve_utf8(self):
        """A successful reply must not destroy the client or split multiline text."""
        first = '您好！\n这是第二行。'
        detail = self.backend.speak_native(first, 'g1_audio')
        process = self.backend._process
        self.assertIsNone(process.poll())
        self.backend.speak_native('再见', 'g1_audio')
        self.assertIs(self.backend._process, process)
        self.assertIn('completion not confirmed', detail)
        self.assertEqual(self.records(), [
            {'started': process.pid, 'interface': 'eth-test'},
            {'text': first}, {'text': '再见'},
        ])
        self.backend.close()
        self.assertEqual(process.poll(), 0)
        self.backend.close()
        with self.assertRaisesRegex(RuntimeError, 'closed'):
            self.backend.speak_native('关闭后', 'g1_audio')

    def test_sdk_failure_is_reported_without_replay(self):
        """An SDK rejection is visible and keeps the existing client alive."""
        with self.assertRaisesRegex(RuntimeError, '3102'):
            self.backend.speak_native('SDK_FAIL', 'g1_audio')
        process = self.backend._process
        self.backend.speak_native('恢复', 'g1_audio')
        self.assertIs(self.backend._process, process)
        self.assertEqual([r['text'] for r in self.records() if 'text' in r],
                         ['SDK_FAIL', '恢复'])

    def test_disconnect_is_not_replayed_and_next_request_reconnects(self):
        """A lost reply fails once; a later independent request can reconnect."""
        with self.assertRaisesRegex(RuntimeError, 'not retried'):
            self.backend.speak_native('DROP', 'g1_audio')
        self.assertIsNone(self.backend._process)
        self.backend.speak_native('下一句', 'g1_audio')
        records = self.records()
        self.assertEqual(len([r for r in records if 'started' in r]), 2)
        self.assertEqual([r['text'] for r in records if 'text' in r], ['DROP', '下一句'])

    def test_timeout_reaps_helper_without_replay(self):
        """A hung response is bounded and does not resend possibly accepted text."""
        receive = self.backend._receive
        def bounded_receive(size, deadline):
            return receive(size, min(deadline, time.monotonic() + 0.1))
        with patch.object(self.backend, '_receive', side_effect=bounded_receive):
            with self.assertRaisesRegex(RuntimeError, 'not retried'):
                self.backend.speak_native('TIMEOUT', 'g1_audio')
        self.assertIsNone(self.backend._process)
        self.assertEqual(len(self.records()), 2)
        self.assertEqual(self.records()[1], {'text': 'TIMEOUT'})

    def test_invalid_readiness_reaps_helper(self):
        """An incompatible helper fails startup instead of accepting speech."""
        with patch.dict(os.environ, {'FAKE_TTS_BAD_READY': '1'}):
            with self.assertRaisesRegex(RuntimeError, 'readiness failed'):
                self.backend.speak_native('您好', 'g1_audio')
        self.assertIsNone(self.backend._process)
        self.assertEqual(len(self.records()), 1)

    def test_failed_spawn_closes_ipc(self):
        """A spawn error releases the parent socket without retrying the request."""
        with patch('speech_service.unitree_tts.subprocess.Popen', side_effect=OSError('spawn failed')) as spawn:
            with self.assertRaisesRegex(RuntimeError, 'spawn failed'):
                self.backend.speak_native('您好', 'g1_audio')
        spawn.assert_called_once()
        self.assertIsNone(self.backend._socket)
        self.assertIsNone(self.backend._process)

    def test_invalid_text_or_target_never_starts_helper(self):
        """Reject unsupported targets and malformed or oversized text locally."""
        for text, target in [('您好', 'other'), ('您' * 1366, 'g1_audio'),
                             ('\0', 'g1_audio'), ('  ', 'g1_audio')]:
            with self.subTest(text=text[:10], target=target):
                with self.assertRaises(ValueError):
                    self.backend.speak_native(text, target)
        self.assertEqual(self.records(), [])

    def test_pcm_export_is_explicitly_unsupported(self):
        with self.assertRaisesRegex(RuntimeError, 'use speech/speak'):
            asyncio.run(self.backend.synthesize('您好'))

    def test_speak_uses_native_backend_without_pcm_or_edge(self):
        """Dispatch speech/speak directly to the native backend with its target."""
        native = SimpleNamespace(speak_native=Mock(return_value='SDK accepted'), synthesize=Mock())
        with patch.object(service._tts_servicer, 'tts_backend', native), \
             patch.object(service, '_default_speaker_provider_id', 'g1_audio'), \
             patch.object(service.ATLAS, 'find_capability', return_value=[SimpleNamespace(provider_id='g1_audio')]), \
             patch.object(service.speech, 'connect_capability') as connect, \
             patch.object(service, 'EdgeTTSBackend') as edge:
            response = service.speak(SimpleNamespace(text='您好', target=''))
        self.assertTrue(response.ok)
        native.speak_native.assert_called_once_with('您好', 'g1_audio')
        native.synthesize.assert_not_called()
        connect.assert_not_called()
        edge.assert_not_called()

    def test_replacement_and_shutdown_release_backend(self):
        """Replacing a backend and repeated lifecycle shutdown reap owned clients."""
        self.backend.speak_native('您好', 'g1_audio')
        process = self.backend._process
        servicer = service.SpeechTtsServicer(self.backend)
        replacement = SimpleNamespace(close=Mock())
        servicer.configure_backend(replacement)
        self.assertIsNotNone(process.poll())
        servicer.configure_backend(replacement)
        replacement.close.assert_not_called()
        with patch.object(service, '_tts_servicer', servicer), \
             patch.object(service._tts_stream_servicer, 'tts_backend', replacement):
            service.shutdown()
            service.shutdown()
            self.assertIsNone(service._tts_stream_servicer.tts_backend)
        replacement.close.assert_called_once_with()


if __name__ == '__main__':
    unittest.main()
