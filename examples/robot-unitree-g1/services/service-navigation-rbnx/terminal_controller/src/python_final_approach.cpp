// Nav2 owns actions, TF and velocity publication; Python owns final-approach math.
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "angles/angles.h"
#include "nav2_core/controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "robonix_nav2_terminal/navigate_goal_epoch.hpp"
#include "tf2/utils.h"

extern char **environ;

namespace robonix_nav2_terminal {

class PythonFinalApproachGoalChecker : public nav2_core::GoalChecker {
public:
  /** Use the Python completion flag; never let the ordinary XY checker finish early. */
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
                  const std::string &name,
                  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {
    auto node = parent.lock();
    if (!node) { throw std::runtime_error("Python goal checker node unavailable"); }
    nav2_util::declare_parameter_if_not_declared(node, name + ".xy_goal_tolerance", rclcpp::ParameterValue(0.08));
    nav2_util::declare_parameter_if_not_declared(node, name + ".yaw_goal_tolerance", rclcpp::ParameterValue(0.12));
    node->get_parameter(name + ".xy_goal_tolerance", xy_);
    node->get_parameter(name + ".yaw_goal_tolerance", yaw_);
    status_ = node->create_subscription<action_msgs::msg::GoalStatusArray>(
        "/navigate_to_pose/_action/status", rclcpp::QoS(10),
        [this](action_msgs::msg::GoalStatusArray::SharedPtr msg) { epoch_.observe(*msg); });
  }
  void reset() override {}  // Same-goal replans must not reset the Python dwell.
  /** Reset completion only for a new navigation action or a changed fallback goal. */
  void sync(const geometry_msgs::msg::Pose &goal) {
    const auto epoch = epoch_.value();
    const bool changed = have_goal_ &&
        (std::hypot(goal.position.x - goal_.position.x, goal.position.y - goal_.position.y) > 0.02 ||
         std::abs(angles::shortest_angular_distance(tf2::getYaw(goal_.orientation), tf2::getYaw(goal.orientation))) > 0.03);
    if (shouldStartGoal(have_goal_, epoch, seen_, changed)) { done_ = false; }
    goal_ = goal;
    have_goal_ = true;
    if (epoch != 0) { seen_ = epoch; }
  }
  bool isGoalReached(const geometry_msgs::msg::Pose &, const geometry_msgs::msg::Pose &goal,
                     const geometry_msgs::msg::Twist &) override {
    sync(goal);
    return done_;
  }
  void setDone(bool done) { done_ = done; }
  /** Provide entry tolerances for Nav2; success itself comes from the Python dwell. */
  bool getTolerances(geometry_msgs::msg::Pose &pose, geometry_msgs::msg::Twist &velocity) override {
    pose.position.x = pose.position.y = xy_;
    pose.position.z = std::numeric_limits<double>::lowest();
    pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw_);
    velocity = geometry_msgs::msg::Twist();
    return true;
  }
private:
  NavigateGoalEpoch epoch_;
  uint64_t seen_{0};
  bool have_goal_{false}, done_{false};
  double xy_{0.08}, yaw_{0.12};
  geometry_msgs::msg::Pose goal_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_;
};

class PythonFinalApproachController : public nav2_core::Controller {
public:
  PythonFinalApproachController() : loader_("nav2_core", "nav2_core::Controller") {}
  ~PythonFinalApproachController() override { stopWorker(); }

  /** Configure ordinary MPPI transit and the independent Python terminal algorithm. */
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
                 std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    node_ = parent.lock();
    if (!node_) { throw std::runtime_error("Python controller node unavailable"); }
    tf_ = tf;
    const auto parameter = [&](const std::string &key, double value) {
      nav2_util::declare_parameter_if_not_declared(node_, name + "." + key, rclcpp::ParameterValue(value));
      node_->get_parameter(name + "." + key, value);
      if (!std::isfinite(value) || value < 0) { throw std::runtime_error("Invalid " + name + "." + key); }
      return value;
    };
    takeover_ = parameter("takeover_dist", 0.60);
    configured_cap_ = parameter("vx_max", 0.35);
    cap_ = configured_cap_;
    // The original script's per-cycle slew limits assume precisely 20 Hz.
    double frequency = 0.0;
    node_->get_parameter("controller_frequency", frequency);
    if (std::abs(frequency - 20.0) > 1e-6) { throw std::runtime_error("Python final approach requires controller_frequency=20"); }
    args_ = {"python3", "-u", ament_index_cpp::get_package_share_directory("robonix_nav2_terminal") + "/python/final_approach_g1.py"};
    const std::vector<std::pair<std::string, double>> defaults = {
        {"Kp_linear", .35}, {"Kp_lateral", .30}, {"Kp_angular", .35},
        {"max_linear_vel", .35}, {"max_lateral_vel", .20}, {"max_angular_vel", .50},
        {"enable_gait_min_vel", 1}, {"gait_min_linear_vel", .20}, {"gait_min_lateral_vel", .20},
        {"gait_min_angular_vel", .17453292519943295}, {"dist_tolerance_enter", .08},
        {"dist_tolerance_exit", .10}, {"angle_tolerance", .12}, {"max_dv", .025},
        {"max_dvy", .020}, {"max_dw", .040}, {"stable_time_required", 1},
        {"filter_alpha", .30}, {"align_speed_scale", .50}};
    for (const auto &item : defaults) {
      args_.push_back("--" + item.first);
      args_.push_back(std::to_string(parameter(item.first, item.second)));
    }
    primary_ = loader_.createSharedInstance("nav2_mppi_controller::MPPIController");
    primary_->configure(parent, name, tf, costmap);
    status_ = node_->create_subscription<action_msgs::msg::GoalStatusArray>(
        "/navigate_to_pose/_action/status", rclcpp::QoS(10),
        [this](action_msgs::msg::GoalStatusArray::SharedPtr msg) { epoch_.observe(*msg); });
  }

  /** Start a fresh private worker on lifecycle activation, without publishing velocity. */
  void activate() override {
    primary_->activate();
    have_goal_ = false;
    seen_ = 0;
    terminal_ = false;
    reset_ = true;
    startWorker();
  }
  void deactivate() override { stopWorker(); primary_->deactivate(); }
  void cleanup() override { stopWorker(); primary_->cleanup(); status_.reset(); }
  /** Pass replans to MPPI without resetting the terminal algorithm for the same goal. */
  void setPlan(const nav_msgs::msg::Path &path) override {
    if (path.poses.empty()) { throw nav2_core::PlannerException("Python controller received empty path"); }
    goal_ = path.poses.back();
    if (goal_.header.frame_id.empty()) { goal_.header.frame_id = path.header.frame_id; }
    primary_->setPlan(path);
  }
  void setSpeedLimit(const double &limit, const bool &percentage) override {
    primary_->setSpeedLimit(limit, percentage);
    cap_ = limit == 0.0 ? configured_cap_ : std::min(configured_cap_, percentage ? configured_cap_ * limit / 100.0 : limit);
  }

  /** Use MPPI until takeover, then return Python commands through Nav2's usual output. */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped &pose, const geometry_msgs::msg::Twist &velocity,
      nav2_core::GoalChecker *goal_checker) override {
    auto *checker = dynamic_cast<PythonFinalApproachGoalChecker *>(goal_checker);
    if (!checker) { throw nav2_core::PlannerException("Python controller requires PythonFinalApproachGoalChecker"); }
    if (goal_.header.frame_id.empty()) { throw nav2_core::PlannerException("Python controller has no goal frame"); }
    auto current = pose;
    if (!nav2_util::transformPoseInTargetFrame(pose, current, *tf_, goal_.header.frame_id)) {
      throw nav2_core::PlannerException("Python controller pose transform failed");
    }
    const auto epoch = epoch_.value();
    const bool changed = have_goal_ &&
        (std::hypot(goal_.pose.position.x - previous_.position.x, goal_.pose.position.y - previous_.position.y) > .02 ||
         std::abs(angles::shortest_angular_distance(tf2::getYaw(previous_.orientation), tf2::getYaw(goal_.pose.orientation))) > .03);
    if (shouldStartGoal(have_goal_, epoch, seen_, changed)) {
      terminal_ = false;
      reset_ = true;
      checker->setDone(false);
    }
    have_goal_ = true;
    previous_ = goal_.pose;
    if (epoch != 0) { seen_ = epoch; }
    const double distance = std::hypot(current.pose.position.x - goal_.pose.position.x,
                                        current.pose.position.y - goal_.pose.position.y);
    if (!terminal_ && distance > takeover_) {
      return primary_->computeVelocityCommands(pose, velocity, goal_checker);
    }
    if (!terminal_) {
      RCLCPP_INFO(node_->get_logger(), "Python final approach takeover at %.3f m", distance);
      terminal_ = true;
    }
    std::ostringstream request;
    request << std::setprecision(17) << current.pose.position.x << ' ' << current.pose.position.y << ' '
            << tf2::getYaw(current.pose.orientation) << ' ' << goal_.pose.position.x << ' '
            << goal_.pose.position.y << ' ' << tf2::getYaw(goal_.pose.orientation) << ' '
            << node_->now().seconds() << ' ' << cap_ << ' ' << (reset_ ? 1 : 0) << '\n';
    const auto response = exchange(request.str());
    reset_ = false;
    geometry_msgs::msg::TwistStamped result;
    result.header = pose.header;
    int done = 0;
    std::istringstream parsed(response);
    if (!(parsed >> result.twist.linear.x >> result.twist.linear.y >> result.twist.angular.z >> done) ||
        !std::isfinite(result.twist.linear.x) || !std::isfinite(result.twist.linear.y) ||
        !std::isfinite(result.twist.angular.z) || (done != 0 && done != 1)) {
      stopWorker();
      throw nav2_core::PlannerException("Invalid Python final approach response");
    }
    checker->setDone(done == 1);
    return result;
  }

private:
  /** Spawn Python without a shell; socketpair gives a single private request/reply pipe. */
  void startWorker() {
    stopWorker();
    int sockets[2];
    if (socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0, sockets) != 0) {
      throw std::runtime_error("Python final approach socketpair failed");
    }
    posix_spawn_file_actions_t actions;
    const int init_error = posix_spawn_file_actions_init(&actions);
    if (init_error != 0) {
      close(sockets[0]);
      close(sockets[1]);
      throw std::runtime_error("Python worker spawn initialization failed");
    }
    int action_error = posix_spawn_file_actions_adddup2(&actions, sockets[1], STDIN_FILENO);
    if (action_error == 0) { action_error = posix_spawn_file_actions_adddup2(&actions, sockets[1], STDOUT_FILENO); }
    if (action_error == 0) { action_error = posix_spawn_file_actions_addclose(&actions, sockets[0]); }
    if (action_error == 0) { action_error = posix_spawn_file_actions_addclose(&actions, sockets[1]); }
    if (action_error != 0) {
      posix_spawn_file_actions_destroy(&actions);
      close(sockets[0]);
      close(sockets[1]);
      throw std::runtime_error("Python worker stdio setup failed");
    }
    std::vector<char *> argv;
    for (auto &arg : args_) { argv.push_back(arg.data()); }
    argv.push_back(nullptr);
    const int error = posix_spawnp(&pid_, "python3", &actions, nullptr, argv.data(), environ);
    posix_spawn_file_actions_destroy(&actions);
    close(sockets[1]);
    if (error != 0) {
      close(sockets[0]);
      pid_ = -1;
      throw std::runtime_error("Could not start Python final approach: " + std::to_string(error));
    }
    socket_ = sockets[0];
  }
  /** Terminate/reap the compute-only worker on lifecycle stop or communication failure. */
  void stopWorker() {
    if (socket_ >= 0) { close(socket_); socket_ = -1; }
    if (pid_ > 0) {
      kill(pid_, SIGKILL);
      while (waitpid(pid_, nullptr, 0) < 0 && errno == EINTR) {}
      pid_ = -1;
    }
  }
  /** Bound every exchange; a missing worker throws so Nav2 stops instead of reusing a command. */
  std::string exchange(const std::string &request) {
    if (socket_ < 0 || send(socket_, request.data(), request.size(), MSG_NOSIGNAL | MSG_DONTWAIT) != static_cast<ssize_t>(request.size())) {
      stopWorker();
      throw nav2_core::PlannerException("Python final approach worker unavailable");
    }
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(150);
    std::string response;
    while (response.find('\n') == std::string::npos && response.size() < 256) {
      const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now()).count();
      pollfd descriptor{socket_, POLLIN, 0};
      if (remaining <= 0 || poll(&descriptor, 1, static_cast<int>(remaining)) <= 0) { break; }
      char buffer[256];
      const auto size = recv(socket_, buffer, sizeof(buffer), MSG_DONTWAIT);
      if (size <= 0) { break; }
      response.append(buffer, static_cast<size_t>(size));
    }
    if (response.find('\n') == std::string::npos) {
      stopWorker();
      throw nav2_core::PlannerException("Python final approach worker timed out or exited");
    }
    return response;
  }
  pluginlib::ClassLoader<nav2_core::Controller> loader_;
  std::shared_ptr<nav2_core::Controller> primary_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  NavigateGoalEpoch epoch_;
  uint64_t seen_{0};
  bool have_goal_{false}, terminal_{false}, reset_{true};
  double takeover_{.60}, configured_cap_{.35}, cap_{.35};
  geometry_msgs::msg::PoseStamped goal_;
  geometry_msgs::msg::Pose previous_;
  std::vector<std::string> args_;
  pid_t pid_{-1};
  int socket_{-1};
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_;
};

} // namespace robonix_nav2_terminal

PLUGINLIB_EXPORT_CLASS(robonix_nav2_terminal::PythonFinalApproachController, nav2_core::Controller)
PLUGINLIB_EXPORT_CLASS(robonix_nav2_terminal::PythonFinalApproachGoalChecker, nav2_core::GoalChecker)
