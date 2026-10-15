"""Verify the real ROS safety-cancel path in an isolated graph without a chassis."""

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import uuid

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from rclpy.qos import DurabilityPolicy, QoSProfile


def main():
    """Exercise production guard discovery, UUID isolation, cached failure and zero latching."""
    if os.environ.get("ROS_DOMAIN_ID") != "217" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError("Use isolated ROS_DOMAIN_ID=217 ROS_LOCALHOST_ONLY=1")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("service_root")
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("g1_guard_smoke")
    process = None
    commands = []
    cancellations = []
    status_pub = node.create_publisher(GoalStatusArray, "/navigate_to_pose/_action/status", 10)
    report_pub = node.create_publisher(
        DiagnosticArray, "/robonix/navigation/terminal",
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    cmd_pub = node.create_publisher(Twist, "/cmd_vel_guard_input", 10)
    subscription = node.create_subscription(
        Twist, "/g1_guard_test/cmd_vel", commands.append, 100)
    with tempfile.TemporaryDirectory(prefix="g1-guard-smoke-") as directory:
        root = Path(directory)

        def cancel(request, response):
            """Assert the failure record already exists when the cancellation reaches ROS."""
            goal_uuid = bytes(request.goal_info.goal_id.uuid).hex()
            record = json.loads((root / f"guard-{goal_uuid}.json").read_text())
            assert record["goal_uuid"] == goal_uuid
            assert "correction" in record["reason"]
            cancellations.append(goal_uuid)
            response.return_code = CancelGoal.Response.ERROR_NONE
            response.goals_canceling.append(request.goal_info)
            return response

        service = node.create_service(CancelGoal, "/navigate_to_pose/_action/cancel_goal", cancel)

        def wait(predicate, seconds=5.0):
            """Spin only this fake navigator and require the owned guard to remain alive."""
            deadline = time.monotonic() + seconds
            while not predicate():
                if process.poll() is not None or time.monotonic() >= deadline:
                    raise RuntimeError((root / "guard.log").read_text())
                rclpy.spin_once(node, timeout_sec=0.02)

        def observe(seconds):
            deadline = time.monotonic() + seconds
            wait(lambda: time.monotonic() >= deadline, seconds + 1.0)

        def begin(goal_uuid):
            """Publish synthetic NavigateToPose status, never a physical navigation goal."""
            status = GoalStatus()
            status.goal_info.goal_id.uuid = list(bytes.fromhex(goal_uuid))
            status.goal_info.stamp = node.get_clock().now().to_msg()
            status.status = GoalStatus.STATUS_EXECUTING
            status_pub.publish(GoalStatusArray(status_list=[status]))
            observe(0.2)

        def report(goal_uuid, level, active=False):
            """Reproduce a controller diagnostic with exact UUID and phase information."""
            status = DiagnosticStatus(name="robonix/navigation/terminal",
                hardware_id=goal_uuid, level=level,
                message="terminal correction distance/time budget exhausted",
                values=[KeyValue(key="correction_active", value=str(active).lower())])
            report_pub.publish(DiagnosticArray(status=[status]))
            observe(0.2)

        python_path = os.pathsep.join(filter(None, (
            str(Path(args.service_root).resolve()), os.environ.get("PYTHONPATH"))))
        environment = dict(os.environ, PYTHONPATH=python_path,
            ROBONIX_VELOCITY_OUTPUT_TOPIC="/g1_guard_test/cmd_vel",
            ROBONIX_NAV_MAX_LINEAR_SPEED_MPS="0.35",
            ROBONIX_NAV_DEFAULT_SPEED_PERCENTAGE="100",
            ROBONIX_NAV_GUARD_SPEED_LIMIT_TOPIC="/g1_guard_test/speed_limit",
            ROBONIX_NAV_TRACE_DIR=directory)
        with (root / "guard.log").open("w") as log:
            try:
                process = subprocess.Popen([sys.executable, "-m", "nav2_wrapper.velocity_guard"],
                    env=environment, stdout=log, stderr=log)
                wait(lambda: status_pub.get_subscription_count() > 0 and
                     report_pub.get_subscription_count() > 0 and cmd_pub.get_subscription_count() > 0)
                first, second = uuid.uuid4().hex, uuid.uuid4().hex
                begin(first)
                report(second, DiagnosticStatus.ERROR)
                assert not cancellations, "unrelated UUID canceled the active navigation"
                report(first, DiagnosticStatus.OK, active=True)
                command = Twist()
                command.linear.x = 0.1
                cmd_pub.publish(command)
                wait(lambda: any(command.linear.x > 0.09 for command in commands))
                report(first, DiagnosticStatus.ERROR)
                wait(lambda: cancellations == [first])
                commands.clear()
                cmd_pub.publish(command)
                observe(0.3)
                assert commands and all(abs(item.linear.x) < 1e-6 and abs(item.angular.z) < 1e-6
                                        for item in commands)
                begin(second)
                wait(lambda: cancellations == [first, second])
                print("PASS: real ROS guard UUID isolation, correction phase, persist-before-cancel, "
                      "cached report race and latched zero; no chassis/SDK")
            finally:
                if process is not None:
                    process.terminate()
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                node.destroy_subscription(subscription)
                node.destroy_service(service)
                node.destroy_node()
                rclpy.shutdown()


if __name__ == "__main__":
    main()
