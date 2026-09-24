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
