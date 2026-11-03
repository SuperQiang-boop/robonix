#include "robonix_nav2_terminal/persistent_rotation_shim.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "angles/angles.h"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace robonix_nav2_terminal {

/** Configure the upstream shim plus bounded, stop-before-turn terminal control. */
void PersistentRotationShim::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) {
  RotationShimController::configure(parent, name, tf, costmap);
  auto node = parent.lock();
  const auto parameter = [&](const char *suffix, double &value) {
    nav2_util::declare_parameter_if_not_declared(
        node, name + "." + suffix, rclcpp::ParameterValue(value));
    node->get_parameter(name + "." + suffix, value);
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::runtime_error(name + "." + suffix + " must be finite and positive");
    }
  };
  parameter("terminal_settle_time", settle_time_);
  parameter("terminal_stopped_linear_velocity", stopped_linear_);
  parameter("terminal_stopped_angular_velocity", stopped_angular_);
  parameter("terminal_yaw_target_tolerance", yaw_target_tolerance_);
  parameter("terminal_xy_exit_tolerance", xy_exit_tolerance_);
  parameter("terminal_timeout", terminal_timeout_);
  parameter("terminal_correction_trigger", correction_trigger_);
  parameter("terminal_correction_xy_tolerance", correction_xy_tolerance_);
  parameter("terminal_correction_max_speed", correction_max_speed_);
  parameter("terminal_correction_linear_accel", correction_linear_accel_);
  parameter("terminal_correction_max_distance", correction_max_distance_);
  parameter("terminal_correction_timeout", correction_timeout_);
  parameter("terminal_correction_forward_min_speed", correction_forward_min_speed_);
  parameter("terminal_correction_max_commanded_distance", correction_max_commanded_distance_);
  parameter("terminal_correction_startup_grace", correction_startup_grace_);
  parameter("terminal_correction_no_progress_timeout", correction_no_progress_timeout_);
  parameter("terminal_correction_progress_distance", correction_progress_distance_);
  node->get_parameter(name + ".vx_max", configured_max_speed_);
  if (correction_trigger_ >= xy_exit_tolerance_ ||
      correction_xy_tolerance_ >= correction_trigger_ ||
      correction_max_speed_ > configured_max_speed_ ||
      correction_forward_min_speed_ > correction_max_speed_ ||
      correction_max_commanded_distance_ < correction_max_distance_ ||
      correction_startup_grace_ + correction_no_progress_timeout_ >= correction_timeout_) {
    throw std::runtime_error("Terminal correction bounds must remain inside the chassis envelope");
  }
  terminal_report_pub_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/robonix/navigation/terminal", rclcpp::QoS(1).reliable().transient_local());
  goal_status_sub_ = node->create_subscription<action_msgs::msg::GoalStatusArray>(
      "/navigate_to_pose/_action/status", rclcpp::QoS(10),
      [this](const action_msgs::msg::GoalStatusArray::SharedPtr status) {
        goal_epoch_.observe(*status);
      });
}

/** Clear terminal state on lifecycle activation, not ordinary path updates. */
void PersistentRotationShim::activate() {
  RotationShimController::activate();
  have_goal_ = false;
  seen_goal_epoch_ = 0;
  resetTerminal();
}

void PersistentRotationShim::cleanup() {
  goal_status_sub_.reset();
  terminal_report_pub_.reset();
  RotationShimController::cleanup();
}

/** Reset every terminal latch and open-loop angular history for a new goal. */
void PersistentRotationShim::resetTerminal() {
  terminal_active_ = false;
  turning_ = false;
  heading_reached_ = false;
  terminal_failed_ = false;
  terminal_failure_reason_.clear();
  correction_active_ = false;
  correction_attempts_ = 0;
  correction_last_speed_ = 0.0;
  correction_last_velocity_ = geometry_msgs::msg::Twist();
  stopped_since_.reset();
  last_angular_vel_ = 0.0;
}

/** Preserve externally requested speed caps for both MPPI transit and bounded correction. */
void PersistentRotationShim::setSpeedLimit(const double &speed_limit,
                                          const bool &percentage) {
  requested_speed_limit_ = speed_limit;
  speed_limit_percentage_ = percentage;
  RotationShimController::setSpeedLimit(speed_limit, percentage);
}

/** Send UUID-scoped phase/failure information; reports never grant navigation success. */
void PersistentRotationShim::reportTerminal(const std::string &reason, bool failed) {
  diagnostic_msgs::msg::DiagnosticArray report;
  report.header.stamp = clock_->now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "robonix/navigation/terminal";
  status.hardware_id = plan_goal_uuid_;
  status.level = failed ? diagnostic_msgs::msg::DiagnosticStatus::ERROR :
                         diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = reason;
  diagnostic_msgs::msg::KeyValue value;
  value.key = "correction_active";
  value.value = correction_active_ ? "true" : "false";
  status.values.push_back(value);
  report.status.push_back(status);
  terminal_report_pub_->publish(report);
  last_report_at_ = clock_->now();
}

/** Latch zero and notify the final guard to stop/cancel with an actionable failure reason. */
void PersistentRotationShim::failTerminal(const std::string &reason) {
  terminal_failed_ = true;
  terminal_failure_reason_ = reason;
  correction_active_ = false;
  correction_last_speed_ = 0.0;
  correction_last_velocity_ = geometry_msgs::msg::Twist();
  last_angular_vel_ = 0.0;
  RCLCPP_ERROR(logger_, "terminal failed: %s", reason.c_str());
  reportTerminal(reason, true);
}

/** Admit only one correction inside the original terminal radius and deadline. */
void PersistentRotationShim::startCorrection(
    const geometry_msgs::msg::PoseStamped &pose) {
  if (correction_attempts_ != 0) {
    failTerminal("terminal position still outside tolerance after one correction");
    return;
  }
  ++correction_attempts_;
  correction_active_ = true;
  correction_ready_ = false;
  correction_origin_ = pose.pose;
  correction_started_at_ = clock_->now();
  correction_last_at_ = correction_started_at_;
  correction_commanded_distance_ = 0.0;
  correction_last_speed_ = 0.0;
  correction_last_velocity_ = geometry_msgs::msg::Twist();
  stopped_since_.reset();
  last_angular_vel_ = 0.0;
  RCLCPP_INFO(logger_, "terminal correction started: one attempt, max %.2f m / %.1f s",
              correction_max_distance_, correction_timeout_);
  reportTerminal("bounded terminal position correction");
}

/** Stop correction and return ownership to final-heading control, never declare arrival. */
void PersistentRotationShim::finishCorrection(const std::string &reason) {
  correction_active_ = false;
  heading_reached_ = false;
  turning_ = false;
  correction_last_speed_ = 0.0;
  correction_last_velocity_ = geometry_msgs::msg::Twist();
  stopped_since_.reset();
  last_angular_vel_ = 0.0;
  RCLCPP_INFO(logger_, "%s", reason.c_str());
  reportTerminal(reason);
}

/** Check the actual short correction command, not a differently clamped trajectory. */
void PersistentRotationShim::checkCorrectionSweep(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &command) {
  auto *costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap->getMutex());
  double position_x = pose.pose.position.x;
  double position_y = pose.pose.position.y;
  double yaw = tf2::getYaw(pose.pose.orientation);
  const auto footprint = costmap_ros_->getRobotFootprint();
  for (double elapsed = 0.0; elapsed <= simulate_ahead_time_; elapsed += control_duration_) {
    const double cost = collision_checker_->footprintCostAtPose(position_x, position_y, yaw, footprint);
    if (cost < 0.0 || cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
      throw nav2_core::PlannerException("terminal correction blocked by local costmap");
    }
    position_x += (command.linear.x * std::cos(yaw) - command.linear.y * std::sin(yaw)) * control_duration_;
    position_y += (command.linear.x * std::sin(yaw) + command.linear.y * std::cos(yaw)) * control_duration_;
    yaw += command.angular.z * control_duration_;
  }
}

/** Correct body-frame XY and final yaw together under collision and motion budgets. */
geometry_msgs::msg::TwistStamped PersistentRotationShim::correctPosition(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::PoseStamped &goal,
    const geometry_msgs::msg::Twist &velocity, double arrival_xy_tolerance) {
  geometry_msgs::msg::TwistStamped command;
  command.header = pose.header;
  const auto now = clock_->now();
  const double elapsed = (now - correction_last_at_).seconds();
  const auto previous_velocity = correction_last_velocity_;
  correction_commanded_distance_ += correction_last_speed_ * std::max(0.0, elapsed);
  correction_last_at_ = now;
  correction_last_speed_ = 0.0;
  correction_last_velocity_ = geometry_msgs::msg::Twist();
  const double measured_distance = std::hypot(pose.pose.position.x - correction_origin_.position.x,
                                              pose.pose.position.y - correction_origin_.position.y);
  if (measured_distance >= correction_max_distance_) {
    failTerminal("terminal correction measured displacement budget exhausted");
    return command;
  }
  const double delta_x = goal.pose.position.x - pose.pose.position.x;
  const double delta_y = goal.pose.position.y - pose.pose.position.y;
  const double distance = std::hypot(delta_x, delta_y);
  if (distance <= correction_xy_tolerance_) {
    finishCorrection("terminal correction finished in inner XY window; stop before final heading");
    return command;
  }
  const bool timed_out = (now - correction_started_at_).seconds() > correction_timeout_;
  const bool travel_exhausted = correction_commanded_distance_ >= correction_max_commanded_distance_;
  if (timed_out || travel_exhausted) {
    if (distance <= arrival_xy_tolerance) {
      finishCorrection("terminal correction finished in arrival XY window at budget; stop before final heading");
    } else {
      failTerminal(timed_out ? "terminal correction time budget exhausted outside arrival XY window" :
          "terminal correction commanded travel budget exhausted outside arrival XY window");
    }
    return command;
  }
  if (!correction_ready_) {
    if (!stoppedForInterval(velocity)) {
      return command;
    }
    correction_ready_ = true;
    correction_progress_reference_ = distance;
    correction_progress_at_ = now + rclcpp::Duration::from_seconds(correction_startup_grace_);
    stopped_since_.reset();
  }
  // Give the gait time to start, then require cumulative progress toward XY.
  // Replans cannot reset this clock; moving sideways without closing in is not progress.
  if (now >= correction_progress_at_) {
    if (correction_progress_reference_ - distance >= correction_progress_distance_) {
      correction_progress_reference_ = distance;
      correction_progress_at_ = now;
    } else if ((now - correction_progress_at_).seconds() >= correction_no_progress_timeout_) {
      if (distance <= arrival_xy_tolerance) {
        finishCorrection("terminal correction finished in arrival XY window without further progress");
      } else {
        failTerminal("terminal correction no XY progress after startup grace");
      }
      return command;
    }
  }
  const double yaw = tf2::getYaw(pose.pose.orientation);
  const double forward_error = delta_x * std::cos(yaw) + delta_y * std::sin(yaw);
  const double lateral_error = -delta_x * std::sin(yaw) + delta_y * std::cos(yaw);
  const double yaw_error = angles::shortest_angular_distance(
      yaw, tf2::getYaw(goal.pose.orientation));
  double speed_cap = correction_max_speed_;
  if (requested_speed_limit_ > 0.0) {
    speed_cap = std::min(speed_cap, speed_limit_percentage_ ?
        configured_max_speed_ * requested_speed_limit_ / 100.0 : requested_speed_limit_);
  }
  double target_x = 0.35 * forward_error;
  double target_y = 0.30 * lateral_error;
  // Only boost predominantly forward corrections, preserving their vector direction.
  // Reverse and lateral gait thresholds have not been established on this robot.
  const double proportional_speed = std::hypot(target_x, target_y);
  if (forward_error > 0.0 && std::abs(lateral_error) <= forward_error * std::tan(0.25) &&
      proportional_speed > 0.0 && proportional_speed < correction_forward_min_speed_) {
    target_x *= correction_forward_min_speed_ / proportional_speed;
    target_y *= correction_forward_min_speed_ / proportional_speed;
  }
  const double target_speed = std::hypot(target_x, target_y);
  if (target_speed > speed_cap) {
    target_x *= speed_cap / target_speed;
    target_y *= speed_cap / target_speed;
  }
  const double change_x = target_x - previous_velocity.linear.x;
  const double change_y = target_y - previous_velocity.linear.y;
  const double change = std::hypot(change_x, change_y);
  const double scale = change > 0.0 ?
      std::min(1.0, correction_linear_accel_ * control_duration_ / change) : 1.0;
  command.twist.linear.x = previous_velocity.linear.x + scale * change_x;
  command.twist.linear.y = previous_velocity.linear.y + scale * change_y;
  const double speed = std::hypot(command.twist.linear.x, command.twist.linear.y);
  if (speed > speed_cap) {
    command.twist.linear.x *= speed_cap / speed;
    command.twist.linear.y *= speed_cap / speed;
  }
  const double angular_target = std::clamp(0.35 * yaw_error, -rotate_to_heading_angular_vel_,
                                          rotate_to_heading_angular_vel_);
  command.twist.angular.z = std::clamp(angular_target,
      last_angular_vel_ - max_angular_accel_ * control_duration_,
      last_angular_vel_ + max_angular_accel_ * control_duration_);
  checkCorrectionSweep(pose, command.twist);
  correction_last_speed_ = std::hypot(command.twist.linear.x, command.twist.linear.y);
  correction_last_velocity_ = command.twist;
  last_angular_vel_ = command.twist.angular.z;
  return command;
}

/** Keep terminal ownership across replans; use the navigation UUID when available. */
void PersistentRotationShim::setPlan(const nav_msgs::msg::Path &path) {
  if (path.poses.empty()) {
    throw nav2_core::PlannerException("PersistentRotationShim received an empty path");
  }
  const auto [epoch, uuid] = goal_epoch_.snapshot();
  bool changed = !have_goal_;
  if (have_goal_) {
    const auto &previous = current_path_.poses.back().pose;
    const auto &next = path.poses.back().pose;
    changed = current_path_.header.frame_id != path.header.frame_id ||
              std::hypot(previous.position.x - next.position.x,
                         previous.position.y - next.position.y) > 0.02 ||
              std::abs(angles::shortest_angular_distance(
                  tf2::getYaw(previous.orientation), tf2::getYaw(next.orientation))) > 0.03;
  }
  const bool new_goal = shouldStartGoal(have_goal_, epoch, seen_goal_epoch_, changed);
  if (new_goal) {
    resetTerminal();
  }
  if (new_goal || plan_goal_uuid_.empty()) {
    plan_goal_uuid_ = uuid;
  }
  if (epoch != 0) {
    seen_goal_epoch_ = epoch;
  }
  have_goal_ = true;
  RotationShimController::setPlan(path);
}

/** Require continuously low measured speed before releasing a stop phase. */
bool PersistentRotationShim::stoppedForInterval(
    const geometry_msgs::msg::Twist &velocity) {
  if (!std::isfinite(velocity.linear.x) || !std::isfinite(velocity.linear.y) ||
      !std::isfinite(velocity.angular.z) ||
      std::hypot(velocity.linear.x, velocity.linear.y) > stopped_linear_ ||
      std::abs(velocity.angular.z) > stopped_angular_) {
    stopped_since_.reset();
    return false;
  }
  const auto now = clock_->now();
  if (!stopped_since_ || now < *stopped_since_) {
    stopped_since_ = now;
  }
  return (now - *stopped_since_).seconds() >= settle_time_;
}

/** Collision-check the terminal sweep even below the upstream path-heading threshold. */
void PersistentRotationShim::checkTerminalRotation(
    const geometry_msgs::msg::PoseStamped &pose, double angular_speed,
    double yaw_error) {
  auto *costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap->getMutex());
  const double initial_yaw = tf2::getYaw(pose.pose.orientation);
  const auto footprint = costmap_ros_->getRobotFootprint();
  for (double elapsed = 0.0; elapsed <= simulate_ahead_time_;
       elapsed += control_duration_) {
    const double rotation = std::clamp(angular_speed * elapsed,
                                       -std::abs(yaw_error), std::abs(yaw_error));
    const double cost = collision_checker_->footprintCostAtPose(
        pose.pose.position.x, pose.pose.position.y, initial_yaw + rotation, footprint);
    if (cost < 0.0 || cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
      throw nav2_core::PlannerException("Terminal rotation collision; MPPI fallback forbidden");
    }
  }
}

/** Own terminal rotation until completion or a guarded stop, without MPPI fallback. */
geometry_msgs::msg::TwistStamped PersistentRotationShim::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &velocity,
    nav2_core::GoalChecker *goal_checker) {
  if (plan_goal_uuid_.empty()) {
    plan_goal_uuid_ = goal_epoch_.uuid();
  }
  auto goal = getSampledPathGoal();
  if (!nav2_util::transformPoseInTargetFrame(goal, goal, *tf_, pose.header.frame_id)) {
    throw nav2_core::PlannerException("Terminal goal transform failed");
  }
  geometry_msgs::msg::Pose tolerance;
  geometry_msgs::msg::Twist velocity_tolerance;
  if (!goal_checker->getTolerances(tolerance, velocity_tolerance)) {
    throw nav2_core::PlannerException("Terminal goal tolerances unavailable");
  }
  const double distance = std::hypot(pose.pose.position.x - goal.pose.position.x,
                                     pose.pose.position.y - goal.pose.position.y);
  if (!terminal_active_ && distance <= tolerance.position.x) {
    terminal_active_ = true;
    terminal_started_at_ = clock_->now();
    stopped_since_.reset();
    last_angular_vel_ = 0.0;
    RCLCPP_INFO(logger_, "terminal stop-before-turn entered at %.3f m", distance);
  }
  if (!terminal_active_) {
    return RotationShimController::computeVelocityCommands(pose, velocity, goal_checker);
  }
  geometry_msgs::msg::TwistStamped stop;
  stop.header = pose.header;
  if (terminal_failed_) {
    if ((clock_->now() - last_report_at_).seconds() >= 0.5) {
      reportTerminal(terminal_failure_reason_, true);
    }
    return stop;
  }
  if (distance > xy_exit_tolerance_ ||
      (clock_->now() - terminal_started_at_).seconds() > terminal_timeout_) {
    failTerminal("terminal XY drift or overall time limit exceeded");
    return stop;
  }
  // XY entry is latched for this goal. Only turn here; never chase the point.
  const double yaw_error = angles::shortest_angular_distance(
      tf2::getYaw(pose.pose.orientation), tf2::getYaw(goal.pose.orientation));
  const double yaw_tolerance = std::abs(tf2::getYaw(tolerance.orientation));
  if (std::abs(yaw_error) <= yaw_tolerance) {
    if (!heading_reached_) {
      RCLCPP_INFO(logger_, "terminal heading accepted at %.3f m; waiting for measured standstill", distance);
    }
    heading_reached_ = true;
    last_angular_vel_ = 0.0;
    return stop;
  }
  if (heading_reached_) {
    if (!stoppedForInterval(velocity)) {
      return stop;
    }
    heading_reached_ = false;
    turning_ = false;
    stopped_since_.reset();
    last_angular_vel_ = 0.0;
    return stop;
  }
  if (!turning_) {
    if (!stoppedForInterval(velocity)) {
      return stop;
    }
    turning_ = true;
    stopped_since_.reset();
    RCLCPP_INFO(logger_, "terminal pure rotation started, yaw error %.3f rad", yaw_error);
  }
  try {
    auto command = computeRotateToHeadingCommand(yaw_error, pose, velocity);
    checkTerminalRotation(pose, command.twist.angular.z, yaw_error);
    last_angular_vel_ = command.twist.angular.z;
    return command;
  } catch (const std::runtime_error &error) {
    failTerminal(std::string("terminal rotation blocked: ") + error.what());
    return stop;
  }
}

}

PLUGINLIB_EXPORT_CLASS(robonix_nav2_terminal::PersistentRotationShim,
                       nav2_core::Controller)
