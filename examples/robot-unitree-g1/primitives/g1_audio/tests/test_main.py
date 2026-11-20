from __future__ import annotations

import os
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

try:
    from g1_audio import main
except ImportError as exc:
    pytest.skip(f"Robonix runtime dependencies are not installed: {exc}", allow_module_level=True)


class Context:
    def __init__(self):
        self.code = None
        self.detail = None

    def abort(self, code, detail):
        self.code = code
        self.detail = detail
        raise RuntimeError(detail)


def test_init_validates_network_and_stream_bound():
    assert isinstance(
        main.init({"network_interface": "eth0", "max_audio_bytes": 4096}), main.Ok
    )
    assert main._NETWORK_INTERFACE == "eth0"
    assert main._MAX_AUDIO_BYTES == 4096
    assert isinstance(
        main.init({"network_interface": "", "max_audio_bytes": 4096}), main.Err
    )
    assert isinstance(
        main.init({"network_interface": "eth0", "max_audio_bytes": 1}), main.Err
    )


def test_init_validates_microphone_configuration():
    assert isinstance(
        main.init(
            {
                "network_interface": "eth0",
                "mic_multicast_group": "239.168.1.10",
                "mic_multicast_port": 5555,
                "mic_sample_rate": 16000,
            }
        ),
        main.Ok,
    )
    assert main._MIC_MULTICAST_GROUP == "239.168.1.10"
    assert isinstance(
        main.init({"network_interface": "eth0", "mic_multicast_port": 65536}),
        main.Err,
    )
    assert isinstance(
        main.init({"network_interface": "eth0", "mic_multicast_group": "not-an-ip"}),
        main.Err,
    )


def test_speaker_forwards_pcm_to_sdk_helper(monkeypatch):
    monkeypatch.setattr(main, "_SDK_HELPER", SimpleNamespace(poll=lambda: None))
    monkeypatch.setattr(main, "_IPC_SOCKET", object())
    playback = Mock()
    monkeypatch.setattr(main, "_play_pcm", playback)
    context = Context()

    result = main.speaker(
        [SimpleNamespace(data=b"\x01\x00"), SimpleNamespace(data=b"\x02\x00")],
        context,
    )

    assert result is not None
    playback.assert_called_once_with(b"\x01\x00\x02\x00")


@pytest.mark.parametrize(
    ("chunks", "status", "message"),
    [
        ([], "INVALID_ARGUMENT", "empty"),
        ([SimpleNamespace(data=b"\x01")], "INVALID_ARGUMENT", "even number"),
    ],
)
def test_speaker_rejects_invalid_pcm(chunks, status, message, monkeypatch):
    monkeypatch.setattr(main, "_SDK_HELPER", SimpleNamespace(poll=lambda: None))
    monkeypatch.setattr(main, "_IPC_SOCKET", object())
    context = Context()

    with pytest.raises(RuntimeError, match=message):
        main.speaker(chunks, context)

    assert context.code.name == status


def test_speaker_rejects_inactive_helper(monkeypatch):
    monkeypatch.setattr(main, "_SDK_HELPER", None)
    monkeypatch.setattr(main, "_IPC_SOCKET", None)
    context = Context()

    with pytest.raises(RuntimeError, match="not active"):
        main.speaker([], context)

    assert context.code.name == "UNAVAILABLE"


def test_activate_is_idempotent_for_running_helper(monkeypatch):
    helper = SimpleNamespace(poll=lambda: None)
    monkeypatch.setattr(main, "_SDK_HELPER", helper)
    monkeypatch.setattr(main, "_IPC_SOCKET", object())
    starter = Mock(side_effect=AssertionError("must not start a second helper"))
    monkeypatch.setattr(main, "_start_helper", starter)

    assert isinstance(main.activate(), main.Ok)
    starter.assert_not_called()


class MicSocket:
    def __init__(self):
        self.closed = False
        self.recv_count = 0

    def recvfrom(self, _size):
        self.recv_count += 1
        if self.recv_count == 1:
            return b"\x01\x00" * 800, ("239.168.1.10", 5555)
        raise AssertionError("mic stream should stop after client cancellation")

    def close(self):
        self.closed = True


class MicContext(Context):
    def __init__(self):
        super().__init__()
        self.active_checks = 0

    def is_active(self):
        self.active_checks += 1
        return self.active_checks == 1


def test_mic_stream_enables_voice_and_yields_audio_chunk(monkeypatch):
    controller = Mock()
    mic_socket = MicSocket()
    monkeypatch.setattr(main, "_MIC_CONTROLLER", controller)
    monkeypatch.setattr(main, "_open_mic_socket", lambda: mic_socket)
    monkeypatch.setattr(main, "_close_mic_socket", lambda sock: sock.close())
    context = MicContext()

    chunks = list(main.mic(None, context))

    assert len(chunks) == 1
    assert chunks[0].data == b"\x01\x00" * 800
    assert chunks[0].sequence == 0
    assert chunks[0].duration_s == pytest.approx(0.05)
    controller.set_mode.assert_called_once_with(1)
    controller.stop.assert_not_called()
    assert mic_socket.closed


def test_mic_stream_rejects_concurrent_client(monkeypatch):
    monkeypatch.setattr(main, "_MIC_LOCK", __import__("threading").Lock())
    main._MIC_LOCK.acquire()
    try:
        context = Context()
        with pytest.raises(RuntimeError, match="already in use"):
            list(main.mic(None, context))
        assert context.code.name == "RESOURCE_EXHAUSTED"
    finally:
        main._MIC_LOCK.release()
