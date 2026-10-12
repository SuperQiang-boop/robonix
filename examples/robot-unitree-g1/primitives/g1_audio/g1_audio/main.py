#!/usr/bin/env python3
"""Robonix speaker primitive for Unitree G1 SDK2 audio playback."""

from __future__ import annotations

import logging
import os
import fcntl
import socket
import struct
import subprocess
import time
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
_MIC_MULTICAST_GROUP = "239.168.123.161"
_MIC_MULTICAST_PORT = 5555
_MIC_BIND_ADDRESS = ""
_MIC_SAMPLE_RATE = 16000
_MIC_CHANNELS = 1
_MIC_BITS_PER_SAMPLE = 16
_MIC_MAX_PACKET_BYTES = 65535
_MIC_SOCKET_TIMEOUT_S = 0.5
_SPEAKER_LOCK = threading.Lock()
_MIC_LOCK = threading.Lock()
_MIC_SOCKETS: set[socket.socket] = set()
_MIC_CONTROLLER = None
_MIC_PACKET_LOG_COUNT = 0


def _interface_ipv4_address() -> str:
    if _MIC_BIND_ADDRESS:
        return _MIC_BIND_ADDRESS
    request = struct.pack("256s", _NETWORK_INTERFACE.encode()[:15])
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        address = fcntl.ioctl(probe.fileno(), 0x8915, request)[20:24]
    return socket.inet_ntoa(address)


def _multicast_membership() -> bytes:
    try:
        interface_address = _interface_ipv4_address()
    except OSError:
        interface_address = "0.0.0.0"
    return struct.pack(
        "=4s4s",
        socket.inet_aton(_MIC_MULTICAST_GROUP),
        socket.inet_aton(interface_address),
    )


class _VoiceController:
    def __init__(self, interface: str, timeout_s: float):
        self._interface = interface
        self._timeout_s = timeout_s
        self._helper: subprocess.Popen | None = None
        self._socket: socket.socket | None = None
        self._lock = threading.Lock()

    def _start(self) -> None:
        if self._helper is not None and self._socket is not None:
            if self._helper.poll() is None:
                return
            self.stop()
        helper_path = Path(
            os.environ.get(
                "G1_AUDIO_VOICE_HELPER",
                str(Path(__file__).resolve().parent.parent / "rbnx-build/bin/g1_audio_voice_helper"),
            )
        )
        if not helper_path.is_file():
            raise RuntimeError(
                f"SDK2 voice helper not found: {helper_path}; rebuild g1_audio"
            )
        parent_socket, child_socket = socket.socketpair()
        parent_socket.settimeout(self._timeout_s)
        try:
            helper = subprocess.Popen(
                [str(helper_path), self._interface, str(child_socket.fileno())],
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
                raise RuntimeError(readiness or f"SDK2 voice helper exited ({helper.poll()})")
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
        self._helper = helper
        self._socket = parent_socket

    def _read_status(self, voice_socket: socket.socket | None = None) -> int:
        voice_socket = voice_socket or self._socket
        if voice_socket is None:
            raise RuntimeError("Unitree SDK2 voice helper is not running")
        response = bytearray()
        while len(response) < 4:
            part = voice_socket.recv(4 - len(response))
            if not part:
                raise RuntimeError("SDK2 voice helper disconnected")
            response.extend(part)
        return struct.unpack("!i", response)[0]

    def set_mode(self, mode: int) -> None:
        with self._lock:
            self._start()
            if self._socket is None:
                raise RuntimeError("Unitree SDK2 voice helper is not running")
            self._socket.sendall(struct.pack("!i", mode))
            code = self._read_status()
            if code != 0:
                raise RuntimeError(f"G1 voice mode {mode} failed with status {code}")

    def stop(self) -> None:
        with self._lock:
            helper, voice_socket = self._helper, self._socket
            self._helper = None
            self._socket = None
            if voice_socket is not None:
                try:
                    if helper is not None and helper.poll() is None:
                        voice_socket.sendall(struct.pack("!i", 0))
                except OSError:
                    pass
                finally:
                    try:
                        voice_socket.close()
                    except OSError:
                        pass
            if helper is None:
                return
            try:
                helper.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                helper.terminate()
                try:
                    helper.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    helper.kill()
                    helper.wait()


def _open_mic_socket() -> socket.socket:
    mic_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    try:
        mic_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        mic_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 262144)
        mic_socket.bind(("", _MIC_MULTICAST_PORT))
        membership = _multicast_membership()
        interface_address = membership[4:8]
        mic_socket.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, interface_address)
        mic_socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, membership)
        mic_socket.settimeout(_MIC_SOCKET_TIMEOUT_S)
        return mic_socket
    except Exception:
        mic_socket.close()
        raise


def _close_mic_socket(mic_socket: socket.socket) -> None:
    try:
        mic_socket.setsockopt(socket.IPPROTO_IP, socket.IP_DROP_MEMBERSHIP, _multicast_membership())
    except OSError:
        pass
    try:
        mic_socket.close()
    finally:
        _MIC_SOCKETS.discard(mic_socket)


def _stop_active_mic() -> None:
    for mic_socket in tuple(_MIC_SOCKETS):
        _close_mic_socket(mic_socket)
    if _MIC_CONTROLLER is not None:
        _MIC_CONTROLLER.stop()


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
    global _MIC_MULTICAST_GROUP, _MIC_MULTICAST_PORT, _MIC_BIND_ADDRESS
    global _MIC_SAMPLE_RATE, _MIC_CHANNELS, _MIC_BITS_PER_SAMPLE
    global _MIC_MAX_PACKET_BYTES, _MIC_SOCKET_TIMEOUT_S
    global _MIC_CONTROLLER
    _NETWORK_INTERFACE = str(config.get("network_interface", "enp7s0")).strip()
    try:
        _MAX_AUDIO_BYTES = int(config.get("max_audio_bytes", 32 * 1024 * 1024))
        _STARTUP_TIMEOUT_S = float(config.get("startup_timeout_s", 15.0))
        _MIC_MULTICAST_GROUP = str(
            config.get("mic_multicast_group", "239.168.123.161")
        ).strip()
        _MIC_MULTICAST_PORT = int(config.get("mic_multicast_port", 5555))
        _MIC_BIND_ADDRESS = str(config.get("mic_bind_address", "")).strip()
        _MIC_SAMPLE_RATE = int(config.get("mic_sample_rate", 16000))
        _MIC_CHANNELS = int(config.get("mic_channels", 1))
        _MIC_BITS_PER_SAMPLE = int(config.get("mic_bits_per_sample", 16))
        _MIC_MAX_PACKET_BYTES = int(config.get("mic_max_packet_bytes", 65535))
        _MIC_SOCKET_TIMEOUT_S = float(config.get("mic_socket_timeout_s", 0.5))
    except (TypeError, ValueError) as exc:
        return Err(f"invalid g1_audio config: {exc}")
    if not _NETWORK_INTERFACE:
        return Err("network_interface must not be empty")
    if not 2 <= _MAX_AUDIO_BYTES <= 32 * 1024 * 1024:
        return Err("max_audio_bytes must be between 2 and 33554432")
    if not 1.0 <= _STARTUP_TIMEOUT_S <= 60.0:
        return Err("startup_timeout_s must be between 1 and 60")
    try:
        socket.inet_aton(_MIC_MULTICAST_GROUP)
        if _MIC_BIND_ADDRESS:
            socket.inet_aton(_MIC_BIND_ADDRESS)
    except OSError:
        return Err("mic multicast and bind addresses must be valid IPv4 addresses")
    if not 1 <= _MIC_MULTICAST_PORT <= 65535:
        return Err("mic_multicast_port must be between 1 and 65535")
    if _MIC_SAMPLE_RATE <= 0 or _MIC_CHANNELS <= 0 or _MIC_BITS_PER_SAMPLE not in (8, 16, 24, 32):
        return Err("mic audio format must have positive rate/channels and supported bit depth")
    if not 1 <= _MIC_MAX_PACKET_BYTES <= 65535:
        return Err("mic_max_packet_bytes must be between 1 and 65535")
    if not 0.05 <= _MIC_SOCKET_TIMEOUT_S <= 10.0:
        return Err("mic_socket_timeout_s must be between 0.05 and 10")
    _MIC_CONTROLLER = _VoiceController(_NETWORK_INTERFACE, _STARTUP_TIMEOUT_S)
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
    _stop_active_mic()
    with _SPEAKER_LOCK:
        _close_helper()
    return Ok()


@g1_audio.on_shutdown
def shutdown():
    _stop_active_mic()
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


@g1_audio.grpc(
    "robonix/primitive/audio/mic",
    description="Stream 16 kHz mono pcm_s16le audio from the Unitree G1 microphone.",
)
def mic(request, context):
    """Stream microphone PCM from the G1 voice service multicast channel."""
    global _MIC_PACKET_LOG_COUNT
    del request
    if not _MIC_LOCK.acquire(blocking=False):
        context.abort(
            grpc.StatusCode.RESOURCE_EXHAUSTED,
            "G1 microphone is already in use by another stream",
        )
    mic_socket = None
    controller = _MIC_CONTROLLER or _VoiceController(_NETWORK_INTERFACE, _STARTUP_TIMEOUT_S)
    try:
        controller.set_mode(1)
        mic_socket = _open_mic_socket()
        _MIC_SOCKETS.add(mic_socket)
        log.info(
            "G1 microphone stream started on %s:%d via %s (%s)",
            _MIC_MULTICAST_GROUP,
            _MIC_MULTICAST_PORT,
            _NETWORK_INTERFACE,
            _interface_ipv4_address(),
        )
        sequence = 0
        while context.is_active():
            try:
                data, _ = mic_socket.recvfrom(_MIC_MAX_PACKET_BYTES)
            except socket.timeout:
                continue
            if not data:
                continue
            _MIC_PACKET_LOG_COUNT += 1
            if _MIC_PACKET_LOG_COUNT <= 3 or _MIC_PACKET_LOG_COUNT % 100 == 0:
                sample_width = max(1, _MIC_BITS_PER_SAMPLE // 8)
                nonzero_bytes = sum(byte != 0 for byte in data)
                log.info(
                    "G1 microphone packet #%d: bytes=%d nonzero_bytes=%d first16=%s",
                    _MIC_PACKET_LOG_COUNT,
                    len(data),
                    nonzero_bytes,
                    data[:16].hex(),
                )
            duration_s = len(data) / (
                _MIC_SAMPLE_RATE * _MIC_CHANNELS * (_MIC_BITS_PER_SAMPLE / 8)
            )
            yield audio_pb2.AudioChunk(
                timestamp_ns=time.time_ns(),
                data=data,
                sequence=sequence,
                duration_s=duration_s,
            )
            sequence = (sequence + 1) & 0xFFFFFFFF
    except grpc.RpcError:
        raise
    except Exception as exc:  # noqa: BLE001
        log.exception("G1 microphone capture failed")
        context.abort(grpc.StatusCode.UNAVAILABLE, str(exc))
    finally:
        if mic_socket is not None:
            _close_mic_socket(mic_socket)
        _MIC_LOCK.release()
        log.info("G1 microphone stream stopped")


if __name__ == "__main__":
    g1_audio.run()
