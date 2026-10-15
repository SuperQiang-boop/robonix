#include <limits>

#include "gtest/gtest.h"
#include "nav2_util/geometry_utils.hpp"
#include "rcl/time.h"
#include "robonix_nav2_terminal/settled_goal_checker.hpp"

namespace {

geometry_msgs::msg::Pose pose(double position_x, double yaw = 0.0) {
  geometry_msgs::msg::Pose result;
  result.position.x = position_x;
  result.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
  return result;
}

class SettledCheck : public ::testing::Test {
protected:
  /** Give each case a deterministic ROS clock without an executor or real robot. */
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("settled_checker_test");
    ASSERT_EQ(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()), RCL_RET_OK);
    advance(1.0);
    checker.initialize(node, "checker", nullptr);
  }

  void advance(double seconds) {
    ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),
                                       static_cast<int64_t>(seconds * 1e9)), RCL_RET_OK);
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  robonix_nav2_terminal::SettledGoalChecker checker;
  geometry_msgs::msg::Twist velocity;
};

TEST_F(SettledCheck, RequiresDwellAndKeepsItAcrossReplans) {
  EXPECT_FALSE(checker.isGoalReached(pose(0.1), pose(0.0), velocity));
  advance(1.2);
  checker.reset();
  EXPECT_FALSE(checker.isGoalReached(pose(0.1), pose(0.0), velocity));
  advance(1.41);
  EXPECT_TRUE(checker.isGoalReached(pose(0.1), pose(0.0), velocity));
}

TEST_F(SettledCheck, PositionAndHeadingMustBeValidAtTheSameTime) {
  EXPECT_FALSE(checker.isGoalReached(pose(0.1, 0.8), pose(0.0), velocity));
  advance(2.0);
  checker.reset();
  EXPECT_FALSE(checker.isGoalReached(pose(0.21), pose(0.0), velocity));
  advance(3.0);
  EXPECT_FALSE(checker.isGoalReached(pose(0.1), pose(0.0), velocity));
  advance(3.41);
  EXPECT_TRUE(checker.isGoalReached(pose(0.1), pose(0.0), velocity));
}

TEST_F(SettledCheck, MotionResetsTheFullDwellInterval) {
  EXPECT_FALSE(checker.isGoalReached(pose(0.1), pose(0.0), velocity));
  advance(1.3);
  velocity.angular.z = 0.1;
  EXPECT_FALSE(checker.isGoalReached(pose(0.1), pose(0.0), velocity));
  advance(1.5);
  velocity.angular.z = 0.0;
  EXPECT_FALSE(checker.isGoalReached(pose(0.1), pose(0.0), velocity));
  advance(1.8);
  EXPECT_FALSE(checker.isGoalReached(pose(0.1), pose(0.0), velocity));
  advance(1.91);
  EXPECT_TRUE(checker.isGoalReached(pose(0.1), pose(0.0), velocity));
}

TEST_F(SettledCheck, NewGoalAndClockRewindResetDwell) {
  EXPECT_FALSE(checker.isGoalReached(pose(0.0), pose(0.0), velocity));
  advance(2.0);
  EXPECT_FALSE(checker.isGoalReached(pose(0.1), pose(0.1), velocity));
  advance(1.0);
  EXPECT_FALSE(checker.isGoalReached(pose(0.1), pose(0.1), velocity));
  advance(1.41);
  EXPECT_TRUE(checker.isGoalReached(pose(0.1), pose(0.1), velocity));
}

TEST_F(SettledCheck, RejectsInvalidMeasurementsAndPreservesArrivalBounds) {
  velocity.linear.y = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(checker.isGoalReached(pose(0.0), pose(0.0), velocity));
  geometry_msgs::msg::Pose tolerance;
  geometry_msgs::msg::Twist speed;
  ASSERT_TRUE(checker.getTolerances(tolerance, speed));
  EXPECT_DOUBLE_EQ(tolerance.position.x, 0.20);
  EXPECT_DOUBLE_EQ(speed.linear.x, 0.04);
  EXPECT_DOUBLE_EQ(speed.angular.z, 0.06);
}

}
