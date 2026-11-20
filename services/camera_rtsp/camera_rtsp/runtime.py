"""Bounded ROS image publication with an owned MediaMTX server and FFmpeg."""
from __future__ import annotations

import json
import math
import logging
import os
import select
import shutil
import subprocess
import tempfile
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from urllib.parse import urlsplit


@dataclass(frozen=True)
class Settings:
    camera_provider_id: str
    rtsp_url: str
    manage_server: bool = True
    fps: int = 15
    startup_timeout: float = 25.0
    frame_timeout: float = 5.0

    @classmethod
    def parse(cls, config):
        """Reject unknown fields and invalid values before acquiring resources."""
        unknown = set(config) - set(cls.__dataclass_fields__)
        if unknown:
            raise ValueError(f"unknown configuration keys: {sorted(unknown)}")
        result = cls(**config)
        if not isinstance(result.camera_provider_id, str) or not result.camera_provider_id.strip():
            raise ValueError("camera_provider_id must be a nonempty string")
        url = urlsplit(result.rtsp_url)
        if url.scheme != "rtsp" or not url.hostname or not url.path.strip("/") or url.query or url.fragment:
            raise ValueError("rtsp_url must contain an RTSP host and stream path")
        if url.username or url.password:
            raise ValueError("credentials in rtsp_url are not supported")
        if type(result.manage_server) is not bool:
            raise ValueError("manage_server must be boolean")
        if result.manage_server and url.hostname not in {"localhost", "127.0.0.1"}:
            raise ValueError("managed server requires a loopback rtsp_url; use an external server for remote access")
        if url.port is not None and not 1 <= url.port <= 65535:
            raise ValueError("invalid RTSP port")
        if type(result.fps) is not int or not 1 <= result.fps <= 60:
            raise ValueError("fps must be an integer between 1 and 60")
        for value in (result.startup_timeout, result.frame_timeout):
            if type(value) not in (int, float) or not math.isfinite(value) or not 0 < value <= 120:
                raise ValueError("timeouts must be finite numbers in (0, 120]")
        return result


def rgb_frame(msg):
    """Remove row padding and normalize RGB/BGR/RGBA/BGRA/mono8 to RGB24."""
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}
    count = channels.get(msg.encoding)
    if count is None or not 0 < msg.width <= 8192 or not 0 < msg.height <= 8192:
        raise ValueError("unsupported image encoding or dimensions")
    if msg.step < msg.width * count or len(msg.data) != msg.height * msg.step:
        raise ValueError("invalid image stride or data length")
    source = memoryview(bytes(msg.data))
    packed = b"".join(source[y * msg.step:y * msg.step + msg.width * count] for y in range(msg.height))
    if msg.encoding == "rgb8":
        return packed
    out = bytearray(msg.width * msg.height * 3)
    if count == 1:
        out[0::3] = out[1::3] = out[2::3] = packed
    else:
        red, blue = (2, 0) if msg.encoding.startswith("bgr") else (0, 2)
        out[0::3], out[1::3], out[2::3] = packed[red::count], packed[1::count], packed[blue::count]
    return bytes(out)


def terminate(process):
    """Reap an owned process, escalating only after a bounded graceful wait."""
    if process is None:
        return
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
    process.wait(timeout=3)
    if process.stdin:
        process.stdin.close()


class Pipeline:
    """Own one ROS context, channel, thread, encoder and optional RTSP server."""

    def __init__(self, settings, channel):
        """Keep cold handles; start() owns all subsequent acquisitions."""
        self.settings, self.channel = settings, channel
        self.stop = threading.Event()
        self.thread = self.node = self.context = self.encoder = self.server = None
        self.executor = None
        self.directory = None
        self.error = ""
        self.ready = False
        self.last_frame = 0.0
        self.dimensions = None
        self.latest = None

    def start(self):
        """Wait for a real decoded RTSP frame; rollbacks belong to the caller."""
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        for name in ("ffmpeg", "mediamtx") if self.settings.manage_server else ("ffmpeg",):
            if not shutil.which(name):
                raise RuntimeError(f"required executable missing: {name}")
        if self.settings.manage_server:
            self.directory = tempfile.TemporaryDirectory(prefix="camera-rtsp-")
            config = Path(self.directory.name) / "mediamtx.json"
            url = urlsplit(self.settings.rtsp_url)
            config.write_text(json.dumps({"rtspAddress": f"127.0.0.1:{url.port or 554}",
                "rtspTransports": ["tcp"], "rtmp": False, "hls": False, "webrtc": False,
                "srt": False, "paths": {url.path.lstrip("/"): {"source": "publisher"}}}))
            self.server = subprocess.Popen(["mediamtx", str(config)])
        self.context = Context()
        rclpy.init(args=[], context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.node = rclpy.create_node("camera_rtsp_bridge", context=self.context)
        self.executor.add_node(self.node)
        self.node.create_subscription(Image, self.channel.endpoint, self.receive, qos_profile_sensor_data)
        self.thread = threading.Thread(target=self.run, name="camera-rtsp")
        self.thread.start()
        deadline = time.monotonic() + self.settings.startup_timeout
        while time.monotonic() < deadline:
            if self.error:
                raise RuntimeError(self.error)
            if self.encoder is not None:
                try:
                    probe = subprocess.run(["ffmpeg", "-nostdin", "-loglevel", "error",
                        "-rtsp_transport", "tcp", "-i", self.settings.rtsp_url,
                        "-map", "0:v:0", "-frames:v", "1", "-f", "null", "-"],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                        timeout=min(3, max(0.01, deadline - time.monotonic())))
                    if probe.returncode == 0 and self.healthy():
                        self.ready = True
                        return
                except subprocess.TimeoutExpired:
                    pass
            self.stop.wait(0.1)
        raise TimeoutError("RTSP activation timed out before decoding a frame")

    def receive(self, msg):
        """Keep only the latest frame; fail explicitly on malformed or resized data."""
        try:
            dimensions = (msg.width, msg.height)
            frame = rgb_frame(msg)
            if self.dimensions is not None and dimensions != self.dimensions:
                raise ValueError("camera dimensions changed; deactivate and activate again")
            self.dimensions = dimensions
            self.latest = frame
            self.last_frame = time.monotonic()
        except Exception as exc:
            self.error = f"invalid camera frame: {exc}"
            self.stop.set()

    def healthy(self):
        """Check input freshness and both owned processes without false success."""
        return (not self.error and not self.stop.is_set() and self.encoder is not None
                and self.encoder.poll() is None
                and (self.server is None or self.server.poll() is None)
                and time.monotonic() - self.last_frame < self.settings.frame_timeout)

    def run(self):
        """Spin ROS and write frames with bounded pipe waits and no unbounded queue."""
        started = next_frame = time.monotonic()
        try:
            while not self.stop.is_set():
                self.executor.spin_once(timeout_sec=0.02)
                now = time.monotonic()
                if self.server is not None and self.server.poll() is not None:
                    raise RuntimeError("MediaMTX exited")
                if self.encoder is not None and self.encoder.poll() is not None:
                    raise RuntimeError(f"FFmpeg exited with code {self.encoder.returncode}")
                if now - (self.last_frame or started) > self.settings.frame_timeout:
                    raise TimeoutError("camera RGB stream timed out")
                if self.latest is None or now < next_frame:
                    continue
                if self.encoder is None:
                    width, height = self.dimensions
                    self.encoder = subprocess.Popen(["ffmpeg", "-nostdin", "-hide_banner",
                        "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
                        "-s", f"{width}x{height}", "-r", str(self.settings.fps), "-i", "pipe:0",
                        "-an", "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2", "-c:v", "libx264",
                        "-pix_fmt", "yuv420p", "-preset", "ultrafast", "-tune", "zerolatency",
                        "-g", str(self.settings.fps), "-f", "rtsp", "-rtsp_transport", "tcp",
                        self.settings.rtsp_url], stdin=subprocess.PIPE, bufsize=0)
                    os.set_blocking(self.encoder.stdin.fileno(), False)
                data = memoryview(self.latest)
                self.latest = None
                deadline = now + self.settings.frame_timeout
                while data and not self.stop.is_set():
                    if time.monotonic() > deadline:
                        raise TimeoutError("FFmpeg input blocked")
                    fd = self.encoder.stdin.fileno()
                    if select.select([], [fd], [], 0.1)[1]:
                        try:
                            data = data[os.write(fd, data):]
                        except BlockingIOError:
                            continue
                next_frame = now + 1 / self.settings.fps
        except Exception as exc:
            logging.getLogger(__name__).exception("camera RTSP worker failed: %s", exc)
            self.error = f"{type(exc).__name__}: {exc}"
            self.stop.set()
        finally:
            self.ready = False
            terminate(self.encoder)
            terminate(self.server)

    def close(self):
        """Join the worker before destroying ROS resources and close the Atlas channel."""
        self.ready = False
        self.stop.set()
        if self.thread is not None:
            self.thread.join(timeout=10)
            if self.thread.is_alive():
                raise RuntimeError("camera RTSP worker failed to stop")
        terminate(self.encoder)
        terminate(self.server)
        if self.executor is not None:
            if not self.executor.shutdown(timeout_sec=3):
                raise RuntimeError("camera RTSP executor failed to stop")
            self.executor = None
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        if self.context is not None and self.context.ok():
            self.context.shutdown()
        if self.directory is not None:
            self.directory.cleanup()
        self.channel.close()
