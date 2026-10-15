#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

#include "g1_chassis/protocol.hpp"

namespace g1_chassis {

struct PlanarOdometry {
  double x{0};
  double y{0};
  double yaw{0};
  double vx{0};
  double vy{0};
  double wz{0};
};

class OdometryProjector {
 public:
  /** Validate and project consecutive real states; latch discontinuities until restart. */
  bool Update(const StatePacket &state, std::string *reason) {
    if (faulted_) {
      *reason = "SDK pose discontinuity; restart after checking localization";
      return false;
    }
    if (state.type != static_cast<uint8_t>(PacketType::kState) ||
        state.version != 1 || state.sequence == 0 ||
        state.received_unix_ns == 0 || state.received_monotonic_ns == 0 ||
        state.error_code != 0) {
      *reason = "invalid SDK state metadata or error_code";
      return false;
    }
    if (initialized_ && state.sequence <= sequence_) {
      *reason = "duplicate or out-of-order SDK state";
      return false;
    }
    double norm_squared = 0;
    for (const auto value : state.quaternion_wxyz) norm_squared += value * value;
    if (!std::isfinite(norm_squared) || norm_squared < 0.25 || norm_squared > 2.25 ||
        !std::isfinite(state.position[0]) || !std::isfinite(state.position[1]) ||
        !std::isfinite(state.position[2])) {
      *reason = "invalid SDK position or quaternion";
      return false;
    }
    const double norm = std::sqrt(norm_squared);
    const double quaternion_w = state.quaternion_wxyz[0] / norm;
    const double quaternion_x = state.quaternion_wxyz[1] / norm;
    const double quaternion_y = state.quaternion_wxyz[2] / norm;
    const double quaternion_z = state.quaternion_wxyz[3] / norm;
    PlanarOdometry next;
    next.x = state.position[0];
    next.y = state.position[1];
    next.yaw = std::atan2(2 * (quaternion_w * quaternion_z + quaternion_x * quaternion_y),
                          1 - 2 * (quaternion_y * quaternion_y + quaternion_z * quaternion_z));
    if (initialized_) {
      if (state.received_monotonic_ns <= monotonic_ns_) {
        *reason = "non-increasing SDK reception time";
        return false;
      }
      const double elapsed = (state.received_monotonic_ns - monotonic_ns_) / 1e9;
      const double delta_x = next.x - pose_.x;
      const double delta_y = next.y - pose_.y;
      const double delta_yaw = std::atan2(std::sin(next.yaw - pose_.yaw), std::cos(next.yaw - pose_.yaw));
      if (std::hypot(delta_x, delta_y) > 0.30 || std::abs(delta_yaw) > 0.50) {
        faulted_ = true;
        *reason = "SDK pose jump exceeds 0.30 m or 0.50 rad: dt=" +
            std::to_string(elapsed) + " s, dx=" + std::to_string(delta_x) +
            " m, dy=" + std::to_string(delta_y) + " m, dyaw=" +
            std::to_string(delta_yaw) + " rad; previous=(" +
            std::to_string(pose_.x) + "," + std::to_string(pose_.y) + "," +
            std::to_string(pose_.yaw) + "), current=(" +
            std::to_string(next.x) + "," + std::to_string(next.y) + "," +
            std::to_string(next.yaw) + ")";
        return false;
      }
      if (elapsed <= 0.15) {
        const double world_vx = delta_x / elapsed;
        const double world_vy = delta_y / elapsed;
        const double alpha = 1 - std::exp(-elapsed / 0.10);
        const double prior_vx = std::cos(delta_yaw) * pose_.vx + std::sin(delta_yaw) * pose_.vy;
        const double prior_vy = -std::sin(delta_yaw) * pose_.vx + std::cos(delta_yaw) * pose_.vy;
        next.vx = prior_vx + alpha * (std::cos(next.yaw) * world_vx + std::sin(next.yaw) * world_vy - prior_vx);
        next.vy = prior_vy + alpha * (-std::sin(next.yaw) * world_vx + std::cos(next.yaw) * world_vy - prior_vy);
        next.wz = pose_.wz + alpha * (delta_yaw / elapsed - pose_.wz);
      }
    }
    pose_ = next;
    sequence_ = state.sequence;
    monotonic_ns_ = state.received_monotonic_ns;
    initialized_ = true;
    return true;
  }

  const PlanarOdometry &pose() const { return pose_; }
  bool initialized() const { return initialized_; }
  bool faulted() const { return faulted_; }

 private:
  PlanarOdometry pose_;
  uint64_t sequence_{0};
  uint64_t monotonic_ns_{0};
  bool initialized_{false};
  bool faulted_{false};
};

}
