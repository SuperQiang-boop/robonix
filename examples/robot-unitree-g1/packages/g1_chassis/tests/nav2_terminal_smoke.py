"""Exercise real Nav2 terminal rotation in an isolated graph, with no chassis or SDK."""

import argparse
import math
import os
from pathlib import Path
import subprocess
import tempfile
import time
import uuid

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import FollowPath
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import TransformBroadcaster
import yaml


class ControllerCheck:
    """Publish synthetic odometry/TF and observe an isolated controller's commands."""

    def __init__(self, process, log):
        self.node = rclpy.create_node("g1_terminal_smoke")
        self.process = process
        self.log = log
        self.yaw = 0.0
        self.position_x = 0.0
        self.position_y = 0.0
        self.measured_angular = 0.0
        self.measured_linear = 0.0
        self.commands = []
        self.reports = []
        self.broadcaster = TransformBroadcaster(self.node)
        self.odom = self.node.create_publisher(Odometry, "/g1_terminal_test/odom", 10)
        self.map_pub = self.node.create_publisher(
            OccupancyGrid, "/g1_terminal_test/map",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.subscription = self.node.create_subscription(
            Twist, "/g1_terminal_test/cmd_vel", self.commands.append, 100)
        self.report_sub = self.node.create_subscription(
            DiagnosticArray, "/robonix/navigation/terminal",
            lambda message: self.reports.extend(message.status),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.status_pub = self.node.create_publisher(
            GoalStatusArray, "/navigate_to_pose/_action/status", 10)
        self.speed_pub = self.node.create_publisher(SpeedLimit, "/speed_limit", 10)
        self.timer = self.node.create_timer(0.02, self.publish_pose)
        self.lifecycle = self.node.create_client(ChangeState, "/controller_server/change_state")
        self.action = ActionClient(self.node, FollowPath, "/follow_path")

    def publish_pose(self):
        """Supply only synthetic reception-stamped pose; never publish motion commands."""
        transform = TransformStamped()
        transform.header.stamp = self.node.get_clock().now().to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_footprint"
        transform.transform.translation.x = self.position_x
        transform.transform.translation.y = self.position_y
        transform.transform.rotation.z = math.sin(self.yaw / 2)
        transform.transform.rotation.w = math.cos(self.yaw / 2)
        self.broadcaster.sendTransform(transform)
        odom = Odometry()
        odom.header = transform.header
        odom.child_frame_id = transform.child_frame_id
        odom.pose.pose.orientation = transform.transform.rotation
        odom.pose.pose.position.x = self.position_x
        odom.pose.pose.position.y = self.position_y
        odom.twist.twist.angular.z = self.measured_angular
        odom.twist.twist.linear.x = self.measured_linear
        self.odom.publish(odom)

    def publish_collision_map(self, blocked=True):
        """Seed only an isolated static layer with an obstacle intersecting the footprint."""
        grid = OccupancyGrid()
        grid.header.frame_id = "odom"
        grid.header.stamp = self.node.get_clock().now().to_msg()
        grid.info.width = grid.info.height = 80
        grid.info.resolution = 0.05
        grid.info.origin.position.x = grid.info.origin.position.y = -2.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * 6400
        for column in range(43, 47):
            for row in range(41, 45):
                grid.data[row * 80 + column] = 100 if blocked else 0
        self.map_pub.publish(grid)

    def wait(self, predicate, seconds=8.0):
        """Spin fake pose publication while awaiting a bounded controller result."""
        deadline = time.monotonic() + seconds
        while not predicate():
            if self.process.poll() is not None or time.monotonic() >= deadline:
                self.log.seek(0)
                raise RuntimeError(f"isolated controller check failed:\n{self.log.read()}")
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def spin_for(self, seconds):
        """Keep the fake robot stationary while observing requested controller velocities."""
        deadline = time.monotonic() + seconds
        self.wait(lambda: time.monotonic() >= deadline, seconds + 1.0)

    def transition(self, transition_id):
        """Configure or activate only this isolated Nav2 controller."""
        self.wait(lambda: self.lifecycle.service_is_ready())
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = self.lifecycle.call_async(request)
        self.wait(future.done)
        assert future.result().success

    def goal(self, position_x, yaw, replan=False):
        """Submit a synthetic path to FollowPath, not to the physical robot's navigator."""
        self.wait(self.action.server_is_ready)
        if not replan:
            status = GoalStatus()
            self.navigation_uuid = uuid.uuid4().hex
            status.goal_info.goal_id.uuid = list(bytes.fromhex(self.navigation_uuid))
            status.goal_info.stamp = self.node.get_clock().now().to_msg()
            status.status = GoalStatus.STATUS_EXECUTING
            self.status_pub.publish(GoalStatusArray(status_list=[status]))
            self.spin_for(0.15)
        path = NavPath()
        path.header.frame_id = "odom"
        path.header.stamp = self.node.get_clock().now().to_msg()
        for index in range(21):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = position_x * index / 20
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        path.poses[-1].pose.orientation.z = math.sin(yaw / 2)
        path.poses[-1].pose.orientation.w = math.cos(yaw / 2)
        request = FollowPath.Goal()
        request.path = path
        request.controller_id = "FollowPath"
        request.goal_checker_id = "general_goal_checker"
        future = self.action.send_goal_async(request)
        self.wait(future.done)
        handle = future.result()
        assert handle.accepted
        return handle

    def check_heading(self, yaw, replan=False):
        """Require pure rotation, including across replans, and a real successful result."""
        self.yaw = 0.0
        self.position_y = 0.0
        self.spin_for(0.15)
        self.commands.clear()
        handle = self.goal(0.10, yaw)
        self.spin_for(0.2)
        assert all(abs(command.angular.z) < 1e-6 for command in self.commands), \
            "terminal rotation must wait for the measured standstill interval"
        self.wait(lambda: any(command.angular.z * yaw > 0.10 * abs(yaw)
                              for command in self.commands))
        if replan:
            for _ in range(3):
                self.spin_for(0.25)
                handle = self.goal(0.10, yaw, replan=True)
        self.spin_for(0.3)
        assert self.commands
        assert all(abs(command.linear.x) < 1e-6 and abs(command.linear.y) < 1e-6
                   for command in self.commands), "terminal heading requested translation"
        assert any(command.angular.z * yaw > 0.0 for command in self.commands)
        assert all(abs(command.angular.z) <= 0.3001 for command in self.commands)
        result = handle.get_result_async()
        assert not result.done(), "wrong heading must not be declared successful"
        self.yaw = yaw
        self.wait(result.done)
        assert result.result().status == GoalStatus.STATUS_SUCCEEDED
        self.spin_for(0.25)
        assert abs(self.commands[-1].linear.x) < 1e-6
        assert abs(self.commands[-1].angular.z) < 1e-6

    def check_correction(self, repeat_drift=False, budget_arrival=False):
        """Allow one bounded correction across replans, then succeed or reject a second drift."""
        self.yaw = 0.0
        self.position_x = 0.0
        self.position_y = 0.0
        self.spin_for(0.15)
        self.commands.clear()
        self.reports.clear()
        handle = self.goal(0.1, 1.0)
        self.wait(lambda: any(command.angular.z > 0.1 for command in self.commands))
        self.position_x = -0.20
        self.measured_angular = 0.15
        self.wait(lambda: any("bounded terminal" in report.message for report in self.reports))
        self.commands.clear()
        handle = self.goal(0.1, 1.0, replan=True)
        self.spin_for(0.5)
        assert self.commands
        assert all(abs(command.linear.x) < 1e-6 and abs(command.angular.z) < 1e-6
                   for command in self.commands), "correction did not wait for measured braking"
        self.measured_angular = 0.0
        self.speed_pub.publish(SpeedLimit(speed_limit=0.12, percentage=False))
        self.commands.clear()
        self.wait(lambda: any(command.linear.x > 0.08 for command in self.commands))
        assert all(0.0 <= command.linear.x <= 0.1201 and abs(command.linear.y) < 1e-6
                   for command in self.commands), "correction ignored speed cap or moved laterally"
        self.speed_pub.publish(SpeedLimit(speed_limit=50.0, percentage=True))
        self.spin_for(0.3)
        self.commands.clear()
        self.spin_for(0.3)
        assert any(command.linear.x > 0.12 for command in self.commands)
        assert all(command.linear.x <= 0.1751 for command in self.commands)
        self.position_x = -0.08 if budget_arrival else 0.04
        self.commands.clear()
        self.wait(lambda: any("correction finished" in report.message for report in self.reports))
        if budget_arrival:
            assert any("arrival XY window at budget" in report.message for report in self.reports)
        self.wait(lambda: any(command.angular.z > 0.1 for command in self.commands))
        result = handle.get_result_async()
        assert not result.done(), "position correction must not skip the final heading"
        self.yaw = 1.0
        if repeat_drift:
            self.position_x = -0.14
            self.wait(lambda: any("after one correction" in report.message for report in self.reports))
            handle = self.goal(0.1, 1.0, replan=True)
            self.commands.clear()
            self.spin_for(0.5)
            assert all(abs(command.linear.x) < 1e-6 and abs(command.angular.z) < 1e-6
                       for command in self.commands), "same-goal replan reset a failed correction"
            previous_uuid = self.navigation_uuid
            next_status = GoalStatus()
            self.navigation_uuid = uuid.uuid4().hex
            next_status.goal_info.goal_id.uuid = list(bytes.fromhex(self.navigation_uuid))
            next_status.goal_info.stamp = self.node.get_clock().now().to_msg()
            next_status.status = GoalStatus.STATUS_EXECUTING
            self.status_pub.publish(GoalStatusArray(status_list=[next_status]))
            self.reports.clear()
            self.spin_for(0.6)
            assert any(report.hardware_id == previous_uuid and report.level >= DiagnosticStatus.ERROR
                       for report in self.reports), "did not exercise the old controller's repeat report"
            assert all(report.hardware_id != self.navigation_uuid for report in self.reports
                       if report.level >= DiagnosticStatus.ERROR), \
                "old correction failure was relabeled with a new navigation UUID"
            assert previous_uuid != self.navigation_uuid
            future = handle.cancel_goal_async()
            self.wait(future.done)
        else:
            self.wait(result.done)
            assert result.result().status == GoalStatus.STATUS_SUCCEEDED
        self.speed_pub.publish(SpeedLimit(speed_limit=0.0))
        self.position_x = 0.0
        self.spin_for(0.2)

    def check_correction_budget(self, distance_budget=False):
        """A stationary synthetic robot exhausts travel or alignment time without any fallback."""
        self.position_x = self.position_y = self.yaw = 0.0
        self.spin_for(0.2)
        self.commands.clear()
        self.reports.clear()
        handle = self.goal(0.1, 1.0)
        self.wait(lambda: any(command.angular.z > 0.1 for command in self.commands))
        if distance_budget:
            self.position_x = -0.20
        else:
            self.position_y = 0.23
            self.yaw = 1.0
        self.wait(lambda: any("bounded terminal" in report.message for report in self.reports))
        started = time.monotonic()
        for _ in range(3):
            handle = self.goal(0.1, 1.0, replan=True)
            self.spin_for(0.25)
        self.wait(lambda: any(report.level >= DiagnosticStatus.ERROR and
                              "budget exhausted outside arrival" in report.message for report in self.reports))
        assert time.monotonic() - started < 6.8, "replan reset the correction deadline"
        if distance_budget:
            assert any(command.linear.x >= 0.19 for command in self.commands)
        else:
            assert all(abs(command.linear.x) < 1e-6 for command in self.commands)
        assert all(command.linear.x <= 0.2001 and abs(command.linear.y) < 1e-6
                   for command in self.commands)
        self.commands.clear()
        self.spin_for(0.5)
        assert all(abs(command.linear.x) < 1e-6 and abs(command.angular.z) < 1e-6
                   for command in self.commands), "exhausted correction did not latch zero"
        result = handle.get_result_async()
        assert not result.done(), "an exhausted correction must not report success"
        future = handle.cancel_goal_async()
        self.wait(future.done)
        self.wait(result.done)
        self.position_x = self.position_y = self.yaw = 0.0
        self.spin_for(0.2)

    def check_stopped_arrival(self):
        """Even an aligned, in-tolerance goal must wait for measured rotation to stop."""
        self.yaw = 0.0
        self.position_y = 0.0
        self.measured_angular = 0.15
        self.spin_for(0.2)
        handle = self.goal(0.1, 0.0)
        result = handle.get_result_async()
        self.spin_for(0.65)
        assert not result.done(), "moving robot must not report arrival"
        self.measured_angular = 0.0
        self.spin_for(0.25)
        assert not result.done(), "arrival must retain the full stopped dwell"
        self.wait(result.done)
        assert result.result().status == GoalStatus.STATUS_SUCCEEDED

    def check_measured_correction_limit(self):
        """Measured correction displacement remains a hard stop even inside the arrival window."""
        self.position_x = self.position_y = self.yaw = 0.0
        self.spin_for(0.2)
        self.commands.clear()
        self.reports.clear()
        handle = self.goal(0.1, 1.0)
        self.wait(lambda: any(command.angular.z > 0.1 for command in self.commands))
        self.position_x = -0.20
        self.wait(lambda: any("bounded terminal" in report.message for report in self.reports))
        self.position_x = 0.11
        self.wait(lambda: any(report.level >= DiagnosticStatus.ERROR and
                              "measured displacement budget" in report.message for report in self.reports))
        self.commands.clear()
        self.spin_for(0.5)
        assert self.commands and all(abs(command.linear.x) < 1e-6 and abs(command.angular.z) < 1e-6
                                     for command in self.commands)
        result = handle.get_result_async()
        assert not result.done(), "wrong heading and a measured travel fault must not report arrival"
        future = handle.cancel_goal_async()
        self.wait(future.done)
        self.wait(result.done)
        self.position_x = 0.0
        self.spin_for(0.2)

    def check_braking_before_turn(self):
        """Nonzero measured translation must postpone turning, not just goal success."""
        self.yaw = 0.0
        self.measured_linear = 0.15
        self.spin_for(0.2)
        self.commands.clear()
        handle = self.goal(0.1, 0.8)
        self.spin_for(0.6)
        assert self.commands
        assert all(abs(command.linear.x) < 1e-6 and abs(command.angular.z) < 1e-6
                   for command in self.commands), "turning began before translation stopped"
        self.measured_linear = 0.0
        self.spin_for(0.2)
        assert all(abs(command.angular.z) < 1e-6 for command in self.commands), \
            "turning ignored the full measured standstill interval"
        self.wait(lambda: any(command.angular.z > 0.1 for command in self.commands))
        future = handle.cancel_goal_async()
        self.wait(future.done)
        self.spin_for(0.2)

    def check_collision_stop(self):
        """A blocked small-angle terminal turn must hold zero rather than invoke MPPI."""
        self.spin_for(0.4)
        self.commands.clear()
        handle = self.goal(0.1, 0.4)
        self.spin_for(1.5)
        assert self.commands
        assert all(abs(command.linear.x) < 1e-6 and abs(command.angular.z) < 1e-6
                   for command in self.commands), "blocked terminal rotation requested motion"
        self.log.flush()
        self.log.seek(0)
        assert "terminal rotation blocked" in self.log.read(), \
            "collision test did not actually exercise the terminal collision checker"
        result = handle.get_result_async()
        assert not result.done(), "collision must not become a successful arrival"
        future = handle.cancel_goal_async()
        self.wait(future.done)
        self.wait(result.done)

    def check_correction_collision(self):
        """A newly observed obstacle must stop forward correction without MPPI fallback."""
        self.spin_for(0.4)
        self.commands.clear()
        self.reports.clear()
        handle = self.goal(0.1, 1.0)
        self.wait(lambda: any(command.angular.z > 0.1 for command in self.commands))
        self.position_x = -0.20
        self.wait(lambda: any(command.linear.x > 0.1 for command in self.commands))
        self.publish_collision_map()
        self.wait(lambda: any(report.level >= DiagnosticStatus.ERROR and
                              "correction blocked" in report.message for report in self.reports))
        self.commands.clear()
        self.spin_for(0.4)
        assert self.commands and all(abs(command.linear.x) < 1e-6 and abs(command.angular.z) < 1e-6
                                     for command in self.commands)
        result = handle.get_result_async()
        assert not result.done()
        future = handle.cancel_goal_async()
        self.wait(future.done)
        self.wait(result.done)

    def check_transit(self):
        """Ensure MPPI still produces forward motion outside the terminal window."""
        self.yaw = 0.0
        self.spin_for(0.15)
        self.commands.clear()
        handle = self.goal(1.5, 0.0)
        self.wait(lambda: any(command.linear.x > 0.15 for command in self.commands))
        assert all(command.linear.x <= 0.3501 for command in self.commands)
        future = handle.cancel_goal_async()
        self.wait(future.done)
        result = handle.get_result_async()
        self.wait(result.done)
        self.spin_for(0.25)
        assert abs(self.commands[-1].linear.x) < 1e-6
        assert abs(self.commands[-1].angular.z) < 1e-6


def main():
    """Run with ROS domain 217 and localhost-only transport, then stop owned processes."""
    if os.environ.get("ROS_DOMAIN_ID") != "217" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError("Use isolated ROS_DOMAIN_ID=217 ROS_LOCALHOST_ONLY=1")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("params_file")
    parser.add_argument("--collision", action="store_true")
    parser.add_argument("--correction-collision", action="store_true")
    args = parser.parse_args()
    params = yaml.safe_load(Path(args.params_file).read_text())
    params["controller_server"]["ros__parameters"]["odom_topic"] = "/g1_terminal_test/odom"
    costmap = params["local_costmap"]["local_costmap"]["ros__parameters"]
    costmap["plugins"] = ["obstacle_layer"]
    costmap["obstacle_layer"]["enabled"] = False
    costmap["obstacle_layer"]["observation_sources"] = ""
    costmap["track_unknown_space"] = False
    if args.collision or args.correction_collision:
        costmap["plugins"] = ["static_layer"]
        costmap["static_layer"] = {
            "plugin": "nav2_costmap_2d::StaticLayer",
            "map_topic": "/g1_terminal_test/map",
            "map_subscribe_transient_local": True,
        }
    rclpy.init()
    check = None
    with tempfile.TemporaryDirectory(prefix="g1-terminal-smoke-") as directory:
        params_path = Path(directory) / "params.yaml"
        params_path.write_text(yaml.safe_dump(params))
        with (Path(directory) / "controller.log").open("w+") as log:
            process = subprocess.Popen([
                "/opt/ros/humble/lib/nav2_controller/controller_server", "--ros-args",
                "--params-file", str(params_path), "-r", "cmd_vel:=/g1_terminal_test/cmd_vel",
            ], stdout=log, stderr=log)
            try:
                check = ControllerCheck(process, log)
                if args.collision or args.correction_collision:
                    check.publish_collision_map(blocked=args.collision)
                check.transition(1)
                check.transition(3)
                if args.correction_collision:
                    check.check_correction_collision()
                    print("PASS: obstacle blocks bounded forward correction; no chassis/SDK")
                elif args.collision:
                    check.check_collision_stop()
                    print("PASS: blocked terminal turn holds zero without MPPI fallback; no chassis/SDK")
                else:
                    check.check_heading(1.0, replan=True)
                    check.check_heading(-1.0)
                    check.check_correction()
                    check.check_correction(budget_arrival=True)
                    check.check_correction(repeat_drift=True)
                    check.check_correction_budget()
                    check.check_correction_budget(distance_budget=True)
                    check.check_measured_correction_limit()
                    check.check_braking_before_turn()
                    check.check_stopped_arrival()
                    check.check_transit()
                    print("PASS: pure terminal rotation in both directions, replan persistence, "
                          "one bounded correction, arrival-window handoff at command budget, "
                          "speed caps, time/travel limits, measured displacement hard stop, "
                          "no second correction, measured braking dwell, "
                          "strict stopped arrival, goal success, MPPI transit and zero on stop; "
                          "no chassis/SDK")
            finally:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
                if check:
                    check.node.destroy_node()
                rclpy.shutdown()


if __name__ == "__main__":
    main()
