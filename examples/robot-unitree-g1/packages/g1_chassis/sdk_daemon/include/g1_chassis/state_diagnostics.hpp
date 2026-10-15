#pragma once

#include <cmath>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

#include "g1_chassis/protocol.hpp"

namespace g1_chassis {

class StateDiagnostics {
 public:
  /** Summarize reception and pose deltas once per second without SDK calls or state mutation. */
  std::string Update(const StatePacket *state, uint64_t now_ns,
                     uint64_t sent_sequence) {
    if (initialized_ && now_ns - last_log_ns_ < 1'000'000'000ULL) return {};
    std::ostringstream message;
    message << std::fixed << std::setprecision(6)
            << "state available=" << (state != nullptr)
            << " tx_seq=" << sent_sequence;
    if (state) {
      const bool comparable = have_previous_state_ &&
          state->sequence >= previous_state_.sequence;
      const double window_s = initialized_
          ? static_cast<double>(now_ns - last_log_ns_) / 1e9 : 0.0;
      const double rx_hz = comparable && window_s > 0
          ? static_cast<double>(state->sequence - previous_state_.sequence) / window_s
          : 0.0;
      const double age_ms = now_ns >= state->received_monotonic_ns
          ? static_cast<double>(now_ns - state->received_monotonic_ns) / 1e6 : -1.0;
      message << " rx_seq=" << state->sequence << " rx_hz=" << rx_hz
              << " age_ms=" << age_ms << " window_s=" << window_s
              << " error=" << state->error_code
              << " x=" << state->position[0] << " y=" << state->position[1]
              << " delta_valid=" << comparable
              << " dxy=" << (comparable ? std::hypot(
                  state->position[0] - previous_state_.position[0],
                  state->position[1] - previous_state_.position[1]) : 0.0)
              << " qw=" << state->quaternion_wxyz[0]
              << " qx=" << state->quaternion_wxyz[1]
              << " qy=" << state->quaternion_wxyz[2]
              << " qz=" << state->quaternion_wxyz[3]
              << " raw_vx=" << state->velocity[0]
              << " raw_vy=" << state->velocity[1]
              << " raw_wz=" << state->yaw_speed;
      previous_state_ = *state;
    }
    have_previous_state_ = state != nullptr;
    initialized_ = true;
    last_log_ns_ = now_ns;
    return message.str();
  }

 private:
  bool initialized_{false};
  bool have_previous_state_{false};
  uint64_t last_log_ns_{0};
  StatePacket previous_state_;
};

}
