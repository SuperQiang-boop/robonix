#!/usr/bin/env python3
"""Robonix speaker primitive for Unitree G1 SDK2 audio playback."""

from __future__ import annotations

import logging
import os
import socket
import struct
import subprocess
import threading
from pathlib import Path

import audio_pb2
import grpc
from google.protobuf.empty_pb2 import Empty
from robonix_api import Err, Ok, Primitive


g1_audio = Primitive(id="g1_audio", namespace="robonix/primitive/audio")
log = logging.getLogger("g1_audio")

_SDK_HELPER: subprocess.Popen | None = None
_IPC_SOCKET: socket.socket | None = None
_NETWORK_INTERFACE = "enp7s0"
_MAX_AUDIO_BYTES = 32 * 1024 * 1024
_STARTUP_TIMEOUT_S = 15.0
_SPEAKER_LOCK = threading.Lock()


def _close_helper() -> None:
    global _SDK_HELPER, _IPC_SOCKET
    helper, ipc_socket = _SDK_HELPER, _IPC_SOCKET
    _SDK_HELPER = None
    _IPC_SOCKET = None
    if ipc_socket is not None:
        try:
            if helper is not None and helper.poll() is None:
                ipc_socket.sendall(struct.pack("!Q", 0))
        except OSError:
            pass
        try:
            ipc_socket.close()
        except OSError:
            pass
    if helper is not None:
        try:
            helper.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            helper.terminate()
            try:
                helper.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                helper.kill()
                helper.wait()


def _read_line(sock: socket.socket, limit: int = 512) -> str:
    result = bytearray()
    while len(result) < limit:
        byte = sock.recv(1)
        if not byte:
            break
        if byte == b"\n":
            break
        result.extend(byte)
    return result.decode("utf-8", errors="replace")


def _start_helper() -> None:
    global _SDK_HELPER, _IPC_SOCKET
    if _SDK_HELPER is not None and _IPC_SOCKET is not None:
        if _SDK_HELPER.poll() is None:
            return
        _close_helper()
    helper_path = Path(
        os.environ.get(
            "G1_AUDIO_SDK_HELPER",
            str(Path(__file__).resolve().parent.parent / "rbnx-build/bin/g1_audio_sdk_helper"),
        )
    )
    if not helper_path.is_file():
        raise RuntimeError(f"SDK2 audio helper not found: {helper_path}; rebuild g1_audio")

    parent_socket, child_socket = socket.socketpair()
    parent_socket.settimeout(_STARTUP_TIMEOUT_S)
    try:
        helper = subprocess.Popen(
            [str(helper_path), _NETWORK_INTERFACE, str(child_socket.fileno())],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            pass_fds=(child_socket.fileno(),),
            close_fds=True,
        )
    except Exception:
        parent_socket.close()
        child_socket.close()
        raise
    child_socket.close()

    try:
        readiness = _read_line(parent_socket)
        if readiness != "READY":
            raise RuntimeError(readiness or f"SDK2 helper exited ({helper.poll()})")
    except Exception:
        parent_socket.close()
        helper.terminate()
        try:
            helper.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            helper.kill()
            helper.wait()
        raise

    parent_socket.settimeout(None)
    _SDK_HELPER = helper
    _IPC_SOCKET = parent_socket
    log.info("Unitree SDK2 audio client ready on interface %s", _NETWORK_INTERFACE)


def _play_pcm(pcm: bytes) -> None:
    helper, ipc_socket = _SDK_HELPER, _IPC_SOCKET
    if helper is None or ipc_socket is None or helper.poll() is not None:
        raise RuntimeError("Unitree SDK2 audio helper is not running")
    ipc_socket.sendall(struct.pack("!Q", len(pcm)) + pcm)
    response = bytearray()
    while len(response) < 4:
        part = ipc_socket.recv(4 - len(response))
        if not part:
            raise RuntimeError("SDK2 audio helper disconnected during playback")
        response.extend(part)
    status = struct.unpack("!i", response)[0]
    if status != 0:
        raise RuntimeError(f"Unitree SDK2 PlayStream failed with status {status}")


@g1_audio.on_init
def init(config: dict):
    global _NETWORK_INTERFACE, _MAX_AUDIO_BYTES, _STARTUP_TIMEOUT_S
    _NETWORK_INTERFACE = str(config.get("network_interface", "enp7s0")).strip()
    try:
        _MAX_AUDIO_BYTES = int(config.get("max_audio_bytes", 32 * 1024 * 1024))
        _STARTUP_TIMEOUT_S = float(config.get("startup_timeout_s", 15.0))
    except (TypeError, ValueError) as exc:
        return Err(f"invalid g1_audio config: {exc}")
    if not _NETWORK_INTERFACE:
        return Err("network_interface must not be empty")
    if not 2 <= _MAX_AUDIO_BYTES <= 32 * 1024 * 1024:
        return Err("max_audio_bytes must be between 2 and 33554432")
    if not 1.0 <= _STARTUP_TIMEOUT_S <= 60.0:
        return Err("startup_timeout_s must be between 1 and 60")
    return Ok()


@g1_audio.on_activate
def activate():
    with _SPEAKER_LOCK:
        try:
            _start_helper()
        except Exception as exc:  # noqa: BLE001
            log.exception("failed to initialize Unitree SDK2 audio client")
            return Err(f"failed to initialize G1 speaker: {exc}")
    return Ok()


@g1_audio.on_deactivate
def deactivate():
    with _SPEAKER_LOCK:
        _close_helper()
    return Ok()


@g1_audio.on_shutdown
def shutdown():
    with _SPEAKER_LOCK:
        _close_helper()
    return Ok()


@g1_audio.grpc(
    "robonix/primitive/audio/speaker",
    description="Play a 16 kHz mono pcm_s16le stream through the Unitree G1 speaker.",
)
def speaker(chunks, context):
    """Play one bounded 16 kHz mono s16le stream on the G1 speaker."""
    with _SPEAKER_LOCK:
        if _SDK_HELPER is None or _IPC_SOCKET is None:
            context.abort(grpc.StatusCode.UNAVAILABLE, "G1 speaker is not active")

        pcm = bytearray()
        try:
            for chunk in chunks:
                if not chunk.data:
                    continue
                pcm.extend(chunk.data)
                if len(pcm) > _MAX_AUDIO_BYTES:
                    context.abort(
                        grpc.StatusCode.RESOURCE_EXHAUSTED,
                        f"audio stream exceeds {_MAX_AUDIO_BYTES} bytes",
                    )
            if not pcm:
                context.abort(grpc.StatusCode.INVALID_ARGUMENT, "audio stream is empty")
            if len(pcm) % 2:
                context.abort(
                    grpc.StatusCode.INVALID_ARGUMENT,
                    "pcm_s16le stream must contain an even number of bytes",
                )
            _play_pcm(bytes(pcm))
        except grpc.RpcError:
            raise
        except Exception as exc:  # noqa: BLE001
            log.exception("G1 speaker playback failed")
            context.abort(grpc.StatusCode.UNAVAILABLE, str(exc))
    return Empty()


if __name__ == "__main__":
    g1_audio.run()
