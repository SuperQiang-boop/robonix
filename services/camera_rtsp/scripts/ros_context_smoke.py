"""Run with the package Python after sourcing ROS; no camera or media tools needed."""
import time
from unittest.mock import Mock, patch

import rclpy
from rclpy.utilities import get_default_context
from sensor_msgs.msg import Image
from camera_rtsp.runtime import Pipeline, Settings


def main():
    """Exercise the real worker with an isolated context across repeated activation failures."""
    assert not get_default_context().ok(), 'Run in a fresh Python process'
    for attempt in range(2):
        channel = Mock(endpoint='/camera_rtsp_context_smoke')
        pipeline = Pipeline(Settings('camera', 'rtsp://127.0.0.1:8554/test',
                                     manage_server=False, frame_timeout=0.2,
                                     startup_timeout=2), channel)
        received = []
        original_receive = pipeline.receive
        # Invoke the actual image conversion but do not start a media subprocess.
        def receive(msg):
            """Record a real ROS callback and let the worker reach its no-input timeout."""
            original_receive(msg)
            received.append(pipeline.latest)
            pipeline.latest = None
        pipeline.receive = receive
        original_create = rclpy.create_node
        def create_node(*args, **kwargs):
            """Publish a known RGB frame from the same real ROS node using a one-shot timer."""
            node = original_create(*args, **kwargs)
            publisher = node.create_publisher(Image, channel.endpoint, 10)
            def publish():
                message = Image(width=1, height=1, step=3, encoding='rgb8', data=[1, 2, 3])
                publisher.publish(message)
                timer.cancel()
            timer = node.create_timer(0.05, publish)
            return node
        try:
            with patch('shutil.which', return_value='/unused'), patch('rclpy.create_node', create_node):
                try:
                    pipeline.start()
                except RuntimeError as exc:
                    assert 'camera RGB stream timed out' in str(exc), str(exc)
                else:
                    raise AssertionError('activation unexpectedly succeeded without media output')
            assert received == [bytes([1, 2, 3])], received
        finally:
            pipeline.close()
            pipeline.close()
        assert not pipeline.thread.is_alive()
        assert pipeline.executor is None and pipeline.node is None
        assert not pipeline.context.ok()
        assert not get_default_context().ok()
    print('ROS context, real image callback, worker timeout and repeated cleanup OK')


if __name__ == '__main__':
    main()
