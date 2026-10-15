#include "robonix_nav2_terminal/settled_goal_checker.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include "angles/angles.h"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace robonix_nav2_terminal {

/** Configure strict, simultaneous pose and velocity tolerances with a dwell time. */
void SettledGoalChecker::initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, const std::string &name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) {
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("SettledGoalChecker cannot lock lifecycle node");
  }
  clock_ = node->get_clock();
  const auto parameter = [&](const char *suffix, double &value) {
    nav2_util::declare_parameter_if_not_declared(
        node, name + "." + suffix, rclcpp::ParameterValue(value));
    node->get_parameter(name + "." + suffix, value);
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::runtime_error(name + "." + suffix + " must be finite and positive");
    }
  };
  parameter("xy_goal_tolerance", xy_tolerance_);
  parameter("yaw_goal_tolerance", yaw_tolerance_);
  parameter("trans_stopped_velocity", stopped_linear_);
  parameter("rot_stopped_velocity", stopped_angular_);
  parameter("settle_time", settle_time_);
  goal_status_sub_ = node->create_subscription<action_msgs::msg::GoalStatusArray>(
      "/navigate_to_pose/_action/status", rclcpp::QoS(10),
      [this](const action_msgs::msg::GoalStatusArray::SharedPtr status) {
        goal_epoch_.observe(*status);
      });
}

/** Preserve dwell across replans; isGoalReached resets it on violation or a new goal. */
void SettledGoalChecker::reset() {}

/** Require XY, yaw and measured stopped speed together for the entire dwell interval. */
bool SettledGoalChecker::isGoalReached(
    const geometry_msgs::msg::Pose &pose, const geometry_msgs::msg::Pose &goal,
    const geometry_msgs::msg::Twist &velocity) {
  const auto epoch = goal_epoch_.value();
  const bool changed = previous_goal_ &&
      (std::hypot(goal.position.x - previous_goal_->position.x,
                  goal.position.y - previous_goal_->position.y) > 0.02 ||
       std::abs(angles::shortest_angular_distance(
           tf2::getYaw(previous_goal_->orientation), tf2::getYaw(goal.orientation))) > 0.03);
  if (shouldStartGoal(previous_goal_.has_value(), epoch, seen_goal_epoch_, changed)) {
    settled_since_.reset();
  }
  previous_goal_ = goal;
  if (epoch != 0) {
    seen_goal_epoch_ = epoch;
  }
  const double distance = std::hypot(pose.position.x - goal.position.x,
                                     pose.position.y - goal.position.y);
  const double yaw_error = std::abs(angles::shortest_angular_distance(
      tf2::getYaw(pose.orientation), tf2::getYaw(goal.orientation)));
  const double speed = std::hypot(velocity.linear.x, velocity.linear.y);
  if (!std::isfinite(distance) || !std::isfinite(yaw_error) ||
      !std::isfinite(speed) || !std::isfinite(velocity.angular.z) ||
      distance > xy_tolerance_ || yaw_error > yaw_tolerance_ ||
      speed > stopped_linear_ || std::abs(velocity.angular.z) > stopped_angular_) {
    settled_since_.reset();
    return false;
  }
  const auto now = clock_->now();
  if (!settled_since_ || now < *settled_since_) {
    settled_since_ = now;
  }
  return (now - *settled_since_).seconds() >= settle_time_;
}

/** Report original arrival bounds, never the wider terminal safety exit radius. */
bool SettledGoalChecker::getTolerances(
    geometry_msgs::msg::Pose &pose, geometry_msgs::msg::Twist &velocity) {
  const double invalid = std::numeric_limits<double>::lowest();
  pose.position.x = xy_tolerance_;
  pose.position.y = xy_tolerance_;
  pose.position.z = invalid;
  pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw_tolerance_);
  velocity.linear.x = stopped_linear_;
  velocity.linear.y = stopped_linear_;
  velocity.linear.z = invalid;
  velocity.angular.x = invalid;
  velocity.angular.y = invalid;
  velocity.angular.z = stopped_angular_;
  return true;
}

}

PLUGINLIB_EXPORT_CLASS(robonix_nav2_terminal::SettledGoalChecker,
                       nav2_core::GoalChecker)
