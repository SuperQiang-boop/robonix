"""Regressions for upstream failure and a caller disconnecting during speech."""
import sys
from pathlib import Path
import unittest
from unittest.mock import Mock
from types import SimpleNamespace
from io import BytesIO

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[5] / 'pylib/robonix-api'))
from face_recognition.main import _SpeechHandler


class ResponseTests(unittest.TestCase):
    def handler(self):
        """Create a request handler without opening a real socket."""
        handler = object.__new__(_SpeechHandler)
        handler.path = '/speak'
        handler.headers = {'Content-Length': '16'}
        handler.rfile = BytesIO(b'{"text":"hello"}')
        handler.server = SimpleNamespace(proxy=Mock())
        handler.send_response = Mock()
        handler.send_header = Mock()
        handler.end_headers = Mock()
        handler.wfile = Mock()
        return handler

    def test_disconnected_client_does_not_retry_or_send_second_response(self):
        """Speech succeeds but the caller is gone; send exactly one response."""
        handler = self.handler()
        handler.server.proxy.speak.return_value = {'ok': True}
        handler.wfile.write.side_effect = BrokenPipeError()
        handler.do_POST()
        handler.server.proxy.speak.assert_called_once_with('hello')
        handler.send_response.assert_called_once_with(200)

    def test_timeout_is_gateway_timeout(self):
        handler = self.handler()
        handler.server.proxy.speak.side_effect = TimeoutError()
        handler.do_POST()
        handler.send_response.assert_called_once_with(504)

    def test_upstream_failure_is_not_bad_request(self):
        handler = self.handler()
        handler.server.proxy.speak.side_effect = RuntimeError('speaker unavailable')
        handler.do_POST()
        handler.send_response.assert_called_once_with(502)
