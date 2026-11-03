"""G1 built-in text playback; the robot owns synthesis and speaker output."""
import os
from pathlib import Path
import subprocess
import socket
import struct
import threading
import time


class UnitreeTTSBackend:
    """Accept text playback without producing or downloading PCM audio."""

    def __init__(self):
        """Validate the SDK helper and network selection before activation."""
        self.interface = os.environ.get('G1_NETWORK_INTERFACE', '').strip()
        deploy = os.environ.get('ROBONIX_DEPLOY_DIR', '')
        self.helper = Path(deploy) / 'primitives/g1_audio/rbnx-build/bin/g1_audio_tts_helper'
        if not self.interface:
            raise ValueError('G1_NETWORK_INTERFACE is required for G1 text playback')
        if not deploy or not self.helper.is_file() or not os.access(self.helper, os.X_OK):
            raise ValueError(f'G1 TTS helper is not executable: {self.helper}; rebuild g1_audio')
        self._lock = threading.Lock()
        self._process = None
        self._socket = None
        self._closed = False

    def _receive(self, size: int, deadline: float) -> bytes:
        """Read one bounded reply, including fragmented IPC responses."""
        result = bytearray()
        while len(result) < size:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError('G1 TTS helper response timed out')
            self._socket.settimeout(remaining)
            chunk = self._socket.recv(size - len(result))
            if not chunk:
                raise ConnectionError('G1 TTS helper disconnected; rebuild g1_audio if outdated')
            result.extend(chunk)
        return bytes(result)

    def _stop_helper(self) -> None:
        """Close IPC and reap only the owned helper; caller holds the lock."""
        if self._socket is not None:
            self._socket.close()
            self._socket = None
        process, self._process = self._process, None
        if process is not None:
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2)

    def _start_helper(self) -> None:
        """Lazily start one SDK client and retain it across successful requests."""
        if self._closed:
            raise RuntimeError('G1 TTS backend is closed')
        if self._process is not None and self._process.poll() is None:
            return
        self._stop_helper()
        parent, child = socket.socketpair()
        self._socket = parent
        try:
            self._process = subprocess.Popen(
                [str(self.helper), self.interface, str(child.fileno())],
                pass_fds=(child.fileno(),), stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
            )
        finally:
            child.close()
        if self._receive(6, time.monotonic() + 20) != b'READY\n':
            raise RuntimeError('G1 TTS helper readiness failed; rebuild g1_audio')

    def close(self) -> None:
        """Release the persistent SDK client at backend replacement or shutdown."""
        with self._lock:
            self._closed = True
            self._stop_helper()

    def speak_native(self, text: str, target: str) -> str:
        """Send Chinese text once; report SDK rejection without retrying playback."""
        if target != 'g1_audio':
            raise ValueError('G1 text playback requires target g1_audio')
        if not text.strip() or '\0' in text or len(text.encode('utf-8')) > 4096:
            raise ValueError('G1 speech text must contain 1–4096 UTF-8 bytes without NUL')
        payload = text.encode('utf-8')
        with self._lock:
            try:
                self._start_helper()
                self._socket.settimeout(20)
                self._socket.sendall(struct.pack('!I', len(payload)) + payload)
                status, = struct.unpack('!i', self._receive(4, time.monotonic() + 20))
            except (OSError, RuntimeError) as exc:
                self._stop_helper()
                # A lost reply may follow acceptance: never replay this request.
                raise RuntimeError(f'G1 TTS helper failed; playback not retried: {exc}') from exc
            if status:
                raise RuntimeError(f'G1 TtsMaker failed: status={status}')
        return 'G1 accepted text playback; completion not confirmed: G1 TtsMaker status=0'

    async def synthesize(self, text: str, voice: str = '', speed: float = 1.0) -> bytes:
        """G1 accepts text playback only; no PCM synthesis response is available."""
        raise RuntimeError('G1 built-in TTS does not return audio; use speech/speak')

    async def synthesize_stream(self, text: str, voice: str = '', speed: float = 1.0):
        """Reject audio export because synthesis and playback happen on the robot."""
        raise RuntimeError('G1 built-in TTS does not return audio; use speech/speak')
        yield  # Keep the streaming backend protocol without inventing audio.
