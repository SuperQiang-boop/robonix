#pragma once

#include <optional>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav2_rotation_shim_controller/nav2_rotation_shim_controller.hpp"
#include "robonix_nav2_terminal/navigate_goal_epoch.hpp"

namespace robonix_nav2_terminal {

class PersistentRotationShim
    : public nav2_rotation_shim_controller::RotationShimController {
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
                 std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override;
  void activate() override;
  void cleanup() override;
  void setPlan(const nav_msgs::msg::Path &path) override;
  void setSpeedLimit(const double &speed_limit, const bool &percentage) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped &pose,
      const geometry_msgs::msg::Twist &velocity,
      nav2_core::GoalChecker *goal_checker) override;

private:
  void resetTerminal();
  bool stoppedForInterval(const geometry_msgs::msg::Twist &velocity);
  void checkTerminalRotation(const geometry_msgs::msg::PoseStamped &pose,
                             double angular_speed, double yaw_error);
  void reportTerminal(const std::string &reason, bool failed = false);
  void failTerminal(const std::string &reason);
  void startCorrection(const geometry_msgs::msg::PoseStamped &pose);
  void finishCorrection(const std::string &reason);
  geometry_msgs::msg::TwistStamped correctPosition(
      const geometry_msgs::msg::PoseStamped &pose,
      const geometry_msgs::msg::PoseStamped &goal,
      const geometry_msgs::msg::Twist &velocity, double arrival_xy_tolerance);
  void checkCorrectionSweep(const geometry_msgs::msg::PoseStamped &pose,
                            const geometry_msgs::msg::Twist &command);

  NavigateGoalEpoch goal_epoch_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr goal_status_sub_;
  uint64_t seen_goal_epoch_{0};
  std::string plan_goal_uuid_;
  bool have_goal_{false};
  bool terminal_active_{false};
  bool turning_{false};
  bool heading_reached_{false};
  bool terminal_failed_{false};
  std::string terminal_failure_reason_;
  bool correction_active_{false};
  bool correction_aligning_{true};
  bool correction_ready_{false};
  unsigned correction_attempts_{0};
  double correction_trigger_{0.28};
  double correction_xy_tolerance_{0.10};
  double correction_max_speed_{0.20};
  double correction_linear_accel_{0.40};
  double correction_max_distance_{0.30};
  double correction_timeout_{6.0};
  double correction_commanded_distance_{0.0};
  double correction_last_speed_{0.0};
  double requested_speed_limit_{0.0};
  bool speed_limit_percentage_{false};
  double configured_max_speed_{0.35};
  geometry_msgs::msg::Pose correction_origin_;
  rclcpp::Time correction_started_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time correction_last_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_report_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr terminal_report_pub_;
  double settle_time_{0.4};
  double stopped_linear_{0.04};
  double stopped_angular_{0.06};
  double yaw_target_tolerance_{0.12};
  double xy_exit_tolerance_{0.40};
  double terminal_timeout_{15.0};
  std::optional<rclcpp::Time> stopped_since_;
  rclcpp::Time terminal_started_at_{0, 0, RCL_ROS_TIME};
};

}
