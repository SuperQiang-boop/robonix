#include <cstring>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <sys/un.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <unistd.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "g1_chassis/protocol.hpp"
#include "g1_chassis/odometry.hpp"
using g1_chassis::CommandPacket;
using g1_chassis::PacketType;

namespace g1_chassis_adapter {

class AdapterNode : public rclcpp::Node {
 public:
  /** Connect to the daemon and publish measured planar state at the configured rate. */
  AdapterNode() : Node("g1_chassis_adapter") {
    // Resolve IPC socket path from environment.
    const char *socket_env = std::getenv("G1_IPC_SOCKET");
    if (!socket_env) {
      RCLCPP_FATAL(this->get_logger(), "G1_IPC_SOCKET environment variable not set");
      std::exit(1);
    }
    socket_path_ = std::string(socket_env);
    RCLCPP_INFO(this->get_logger(), "IPC socket: %s", socket_path_.c_str());

    // Open a non-blocking UNIX socket to the daemon.
    socket_fd_ = ::socket(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (socket_fd_ < 0) {
      RCLCPP_FATAL(this->get_logger(), "socket() failed: %s", strerror(errno));
      std::exit(1);
    }

    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    std::strncpy(address.sun_path, socket_path_.c_str(),
                 sizeof(address.sun_path) - 1U);

    // Connect in non-blocking mode; wait up to 2s for the daemon.
    int flags = ::fcntl(socket_fd_, F_GETFL, 0);
    ::fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);
    int ret = ::connect(socket_fd_, reinterpret_cast<const sockaddr *>(&address),
                        sizeof(address));
    if (ret < 0 && errno == EINPROGRESS) {
      // Wait for connection to complete.
      fd_set writefds;
      FD_ZERO(&writefds);
      FD_SET(socket_fd_, &writefds);
      timeval tv{};
      tv.tv_sec = 2;
      tv.tv_usec = 0;
      if (select(socket_fd_ + 1, nullptr, &writefds, nullptr, &tv) > 0) {
        int err = 0;
        socklen_t len = sizeof(err);
        getsockopt(socket_fd_, SOL_SOCKET, SO_ERROR, &err, &len);
        if (err == 0) {
          ret = 0;
        } else {
          ret = -1;
        }
      } else {
        ret = -1;
      }
    }
    if (ret < 0) {
      RCLCPP_FATAL(this->get_logger(),
                   "Failed to connect to daemon at %s: %s",
                   socket_path_.c_str(), strerror(errno));
      std::exit(1);
    }
    RCLCPP_INFO(this->get_logger(), "Connected to daemon");

    // Parameters.
    twist_in_topic_ = declare_parameter("twist_in_topic", "/cmd_vel");
    odom_topic_ = declare_parameter("odom_topic", "/odom");
    publish_odom_tf_ = declare_parameter("publish_odom_tf", true);
    odom_frame_ = declare_parameter("odom_frame", "odom");
    base_frame_ = declare_parameter("base_frame", "base_footprint");
    joint_state_topic_ = declare_parameter("joint_state_topic", "/joint_states");
    const auto publish_rate = declare_parameter("odom_publish_rate_hz", 50.0);
    state_timeout_s_ = declare_parameter("state_timeout_s", 0.15);
    if (!std::isfinite(publish_rate) || publish_rate < 10.0 || publish_rate > 100.0 ||
        !std::isfinite(state_timeout_s_) || state_timeout_s_ < 0.05 || state_timeout_s_ > 0.30)
      throw std::runtime_error("invalid odom_publish_rate_hz or state_timeout_s");

    // Twist subscriber.
    twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        twist_in_topic_, 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          OnTwist(msg);
        });

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    joint_state_pub_ =
        create_publisher<sensor_msgs::msg::JointState>(joint_state_topic_, 10);

    // Broadcast odom -> base_footprint TF so mapping and nav2 can resolve the chain.
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Periodic timer to keep the ROS graph alive and forward stale-free cmd_vel.
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / publish_rate)),
        [this]() { TimerTick(); });

    sequence_ = 0;
  }

  ~AdapterNode() override {
    if (socket_fd_ >= 0) ::close(socket_fd_);
  }

 private:
  CommandPacket TwistToPacket(const geometry_msgs::msg::Twist &msg) {
    CommandPacket pkt{};
    pkt.type = static_cast<uint8_t>(PacketType::kCmd);
    pkt.sequence = sequence_++;
    pkt.vx = static_cast<int32_t>(msg.linear.x * 10000);
    pkt.vy = static_cast<int32_t>(msg.linear.y * 10000);
    pkt.omega = static_cast<int32_t>(msg.angular.z * 10000);
    return pkt;
  }

  void OnTwist(const geometry_msgs::msg::Twist::SharedPtr msg) {
    last_twist_ = TwistToPacket(*msg);
    last_twist_received_ = true;
    stale_stop_sent_ = false;
    last_twist_time_ = std::chrono::steady_clock::now();
  }

  /** Publish only new valid state, and stop command refresh when state or commands expire. */
  void TimerTick() {
    DrainReplies();
    PublishNeutralJointState(get_clock()->now());
    const auto now_ns = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
    const bool fresh = state_received_ && now_ns >= latest_state_.received_monotonic_ns &&
        (now_ns - latest_state_.received_monotonic_ns) / 1e9 <= state_timeout_s_;
    if (fresh && latest_state_.sequence != processed_sequence_) {
      processed_sequence_ = latest_state_.sequence;
      std::string reason;
      const auto wall_now = static_cast<uint64_t>(get_clock()->now().nanoseconds());
      const bool timestamp_valid = latest_state_.received_unix_ns > last_stamp_ns_ &&
          latest_state_.received_unix_ns <= wall_now &&
          wall_now - latest_state_.received_unix_ns <= static_cast<uint64_t>(state_timeout_s_ * 1e9);
      if (!timestamp_valid) reason = "non-increasing or stale local reception timestamp";
      odometry_valid_ = timestamp_valid &&
          projector_.Update(latest_state_, &reason);
      if (odometry_valid_) PublishOdometry();
      else RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "SDK odometry rejected: %s", reason.c_str());
    }
    if (!fresh || !odometry_valid_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No fresh valid G1 odometry; TF publication and motion refresh inhibited");
    }

    // Keep streaming the latest command through short ROS scheduling gaps.
    // Stop refreshing after 250 ms so the daemon's 300 ms watchdog still
    // faults safely when the navigation publisher actually disappears.
    const auto command_age =
        std::chrono::steady_clock::now() - last_twist_time_;
    if (fresh && odometry_valid_ && last_twist_received_ &&
        command_age < std::chrono::milliseconds(250)) {
      send_command(last_twist_);
    } else if (last_twist_received_ && !stale_stop_sent_) {
      // End a stale command stream explicitly. This keeps the daemon in its
      // safe disarmed/stopped state without tripping a latched watchdog fault.
      CommandPacket stop{};
      stop.type = static_cast<uint8_t>(PacketType::kCmd);
      stop.sequence = sequence_++;
      send_command(stop);
      stale_stop_sent_ = true;
    }

  }

  /** Publish planar ground-projected pose; velocity comes from pose differences in the child frame. */
  void PublishOdometry() {
    const auto &pose = projector_.pose();
    auto odom = std::make_unique<nav_msgs::msg::Odometry>();
    const auto stamp = rclcpp::Time(static_cast<int64_t>(latest_state_.received_unix_ns), get_clock()->get_clock_type());
    odom->header.stamp = stamp;
    odom->header.frame_id = odom_frame_;
    odom->child_frame_id = base_frame_;
    odom->pose.pose.position.x = pose.x;
    odom->pose.pose.position.y = pose.y;
    odom->pose.pose.orientation.z = std::sin(pose.yaw / 2);
    odom->pose.pose.orientation.w = std::cos(pose.yaw / 2);
    odom->twist.twist.linear.x = pose.vx;
    odom->twist.twist.linear.y = pose.vy;
    odom->twist.twist.angular.z = pose.wz;
    odom->pose.covariance[0] = odom->pose.covariance[7] = 0.01;
    odom->pose.covariance[35] = 0.02;
    odom->twist.covariance[0] = odom->twist.covariance[7] = 0.04;
    odom->twist.covariance[35] = 0.04;
    for (const auto index : {14, 21, 28}) {
      odom->pose.covariance[index] = 1e6;
      odom->twist.covariance[index] = 1e6;
    }
    odom_pub_->publish(std::move(odom));

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = pose.x;
    tf.transform.translation.y = pose.y;
    tf.transform.rotation.z = std::sin(pose.yaw / 2);
    tf.transform.rotation.w = std::cos(pose.yaw / 2);
    if (publish_odom_tf_) tf_broadcaster_->sendTransform(tf);
    last_stamp_ns_ = latest_state_.received_unix_ns;
  }

  // Publish the G1 29-DOF neutral visualization pose while SDK joint states
  // are absent, so robot_state_publisher can connect every limb and sensor.
  // These are placeholder positions, not measured robot joint states.
  void PublishNeutralJointState(const rclcpp::Time &stamp) {
    auto joint_state = std::make_unique<sensor_msgs::msg::JointState>();
    joint_state->header.stamp = stamp;
    joint_state->name = {
        "left_hip_pitch_joint",
        "left_hip_roll_joint",
        "left_hip_yaw_joint",
        "left_knee_joint",
        "left_ankle_pitch_joint",
        "left_ankle_roll_joint",
        "right_hip_pitch_joint",
        "right_hip_roll_joint",
        "right_hip_yaw_joint",
        "right_knee_joint",
        "right_ankle_pitch_joint",
        "right_ankle_roll_joint",
        "waist_yaw_joint",
        "waist_roll_joint",
        "waist_pitch_joint",
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "left_wrist_pitch_joint",
        "left_wrist_yaw_joint",
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_roll_joint",
        "right_wrist_pitch_joint",
        "right_wrist_yaw_joint",
    };
    joint_state->position.assign(joint_state->name.size(), 0.0);
    joint_state_pub_->publish(std::move(joint_state));
  }

  void send_command(const CommandPacket &cmd) {
    // Non-blocking send — if EAGAIN, drop this tick and the next one will
    // retry with updated velocity.
    ssize_t sent = ::send(socket_fd_, &cmd, sizeof(cmd), MSG_DONTWAIT | MSG_NOSIGNAL);
    if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      RCLCPP_WARN(this->get_logger(), "send() error: %s", strerror(errno));
    }
  }

  /** Drain bounded mixed reply/state packets; ignore truncated or incompatible IPC data. */
  void DrainReplies() {
    for (int attempt = 0; attempt < 256; ++attempt) {
      uint8_t packet[sizeof(g1_chassis::StatePacket)]{};
      const ssize_t received =
          ::recv(socket_fd_, packet, sizeof(packet), MSG_DONTWAIT | MSG_TRUNC);
      if (received < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return;
        if (errno != EINTR) {
          RCLCPP_WARN(this->get_logger(), "recv() error: %s", strerror(errno));
        }
        continue;
      }
      if (received == 0) return;
      if (received == static_cast<ssize_t>(sizeof(g1_chassis::StatePacket)) &&
          packet[0] == static_cast<uint8_t>(PacketType::kState)) {
        g1_chassis::StatePacket state;
        std::memcpy(&state, packet, sizeof(state));
        if (state.version == 1 && state.sequence > latest_state_.sequence) {
          latest_state_ = state;
          state_received_ = true;
        }
        continue;
      }
      g1_chassis::ReplyPacket reply{};
      if (received != static_cast<ssize_t>(sizeof(reply)) ||
          packet[0] != static_cast<uint8_t>(PacketType::kReply)) {
        RCLCPP_WARN(this->get_logger(), "discarding malformed daemon reply");
        continue;
      }
      std::memcpy(&reply, packet, sizeof(reply));
      if (reply.code !=
                 static_cast<uint8_t>(g1_chassis::ReplyCode::kOk)) {
        RCLCPP_ERROR(this->get_logger(),
                     "daemon rejected cmd_vel: code=%u armed=%u faulted=%u",
                     reply.code, reply.armed, reply.faulted);
      }
    }
  }

  std::string socket_path_;
  int socket_fd_{-1};

  std::string twist_in_topic_;
  std::string odom_topic_;
  std::string joint_state_topic_;
  bool publish_odom_tf_{true};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  g1_chassis::StatePacket latest_state_;
  g1_chassis::OdometryProjector projector_;
  double state_timeout_s_{0.15};
  bool state_received_{false};
  bool odometry_valid_{false};
  uint64_t processed_sequence_{0};
  uint64_t last_stamp_ns_{0};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  CommandPacket last_twist_{};
  bool last_twist_received_{false};
  bool stale_stop_sent_{false};
  std::chrono::steady_clock::time_point last_twist_time_{};
  uint8_t sequence_{0};
};

}  // namespace g1_chassis_adapter

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<g1_chassis_adapter::AdapterNode>());
  rclcpp::shutdown();
  return 0;
}
