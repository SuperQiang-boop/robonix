#include "g1_chassis/daemon_core.hpp"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <mutex>
#include <sstream>

#include "g1_chassis/loco_client_interface.hpp"

namespace g1_chassis {

// ---- Deployment eligibility helper ----

bool MotionWatchdogDeploymentEligible(bool allow_motion, uint64_t watchdog_ms) {
  return allow_motion && watchdog_ms == 300U;
}

// ---- DaemonCore ----

DaemonCore::DaemonCore(const DaemonConfig &config, ILocoClient &client)
    : config_(config), client_(client) {}

// Send a single SDK stop request and remember whether its reply was confirmed.
int32_t DaemonCore::IssueStop(const char *reason) {
  stop_pending_ = true;
  const auto started = std::chrono::steady_clock::now();
  int32_t result = client_.StopMove();
  const auto elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
  const auto wall_time = std::chrono::duration<double>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  std::ostringstream message;
  message << std::fixed << std::setprecision(6)
          << "[g1-daemon] t=" << wall_time << " StopMove reason=" << reason
          << " rpc=" << result << " elapsed_ms=" << elapsed_ms
          << " velocity_calls=" << velocity_calls_ << "\n";
  std::cerr << message.str();
  stop_pending_ = result != 0;
  return result;
}

// Clamp a requested velocity to deployment limits before forwarding it to SDK2.
int32_t DaemonCore::IssueVelocity(float vx, float vy, float omega,
                                uint8_t command_sequence, uint64_t now_ns) {
  // Clamp to configured limits
  if (std::abs(vx) > config_.max_vx) {
    vx = std::copysign(config_.max_vx, vx);
  }
  if (std::abs(vy) > config_.max_vy) {
    vy = std::copysign(config_.max_vy, vy);
  }
  if (std::abs(omega) > config_.max_wz) {
    omega = std::copysign(config_.max_wz, omega);
  }
  const float duration = static_cast<float>(config_.watchdog_ns / 1e9);
  const auto started = std::chrono::steady_clock::now();
  const int32_t result = client_.SetVelocity(vx, vy, omega, duration);
  const auto elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
  ++velocity_calls_;
  const double gap_ms = velocity_calls_ > 1
      ? static_cast<double>(now_ns - last_velocity_time_ns_) / 1e6 : 0.0;
  last_velocity_time_ns_ = now_ns;
  if (velocity_calls_ == 1 || !moving_ || result != 0 || elapsed_ms >= 50.0 ||
      now_ns - last_velocity_log_ns_ >= 1'000'000'000ULL) {
    const auto wall_time = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    std::ostringstream message;
    message << std::fixed << std::setprecision(6)
            << "[g1-daemon] t=" << wall_time
            << " SetVelocity calls=" << velocity_calls_
            << " cmd_seq=" << static_cast<unsigned>(command_sequence)
            << " rpc=" << result << " elapsed_ms=" << elapsed_ms
            << " gap_ms=" << gap_ms << " vx=" << vx << " vy=" << vy
            << " wz=" << omega << " duration=" << duration << "\n";
    std::cerr << message.str();
    last_velocity_log_ns_ = now_ns;
  }
  return result;
}

ReplyPacket DaemonCore::Handle(const CommandPacket &cmd, uint64_t now_ns) {
  std::lock_guard<std::mutex> lock(mutex_);

  // Always accept commands even when disarmed — the watchdog needs the stream.
  // But only act on them when armed.
  if (cmd.type != static_cast<uint8_t>(PacketType::kCmd)) {
    sequence_ = (sequence_ + 1) & 0xFF;
    return MakeReply(cmd.sequence, ReplyCode::kMalformed,
                     armed_, faulted_);
  }

  if (faulted_) {
    return MakeReply(cmd.sequence, ReplyCode::kFaulted, armed_, true);
  }

  // Decode fixed-point velocity.
  double vx = cmd.vx / 10000.0;
  double vy = cmd.vy / 10000.0;
  double omega = cmd.omega / 10000.0;

  // Check if this is a zero-velocity command (adapter → daemon "stop" signal).
  bool is_zero = (std::abs(vx) < 0.0001F) && (std::abs(vy) < 0.0001F) &&
                 (std::abs(omega) < 0.0001F);

  if (config_.allow_motion) {
    // First valid command arms the daemon (zero-preamble safety).
    if (!armed_) {
      // Check SDK client is ready.
      int32_t arm_result = client_.PrepareArm();
      if (arm_result != 0) {
        sequence_ = (sequence_ + 1) & 0xFF;
        return MakeReply(cmd.sequence, ReplyCode::kSdkError, false, false);
      }
      // The operator establishes the G1 locomotion mode before motion is
      // enabled. Do not overwrite it here with Start or BalanceStand.
      armed_ = true;
      last_arm_time_ns_ = now_ns;
      std::cerr << "[g1-daemon] ARMED (first valid cmd_vel)\n";
    }

    // Stop promptly on the first zero command after motion. Nav2 commonly
    // publishes zero commands at a high rate, so suppress duplicate RPCs.
    if (is_zero) {
      if (moving_ && IssueStop("zero_command") != 0) {
        faulted_ = true;
        armed_ = false;
        sequence_ = (sequence_ + 1) & 0xFF;
        return MakeReply(cmd.sequence, ReplyCode::kSdkError, armed_, faulted_);
      }
      moving_ = false;
      last_motion_time_ns_ = now_ns;
      sequence_ = (sequence_ + 1) & 0xFF;
      return MakeReply(cmd.sequence, ReplyCode::kOk, armed_, faulted_);
    }

    // Non-zero command: issue velocity.
    if (IssueVelocity(static_cast<float>(vx), static_cast<float>(vy),
                      static_cast<float>(omega), cmd.sequence, now_ns) != 0) {
      faulted_ = true;
      armed_ = false;
      IssueStop("velocity_rpc_error");
      moving_ = false;
      sequence_ = (sequence_ + 1) & 0xFF;
      return MakeReply(cmd.sequence, ReplyCode::kSdkError, armed_, faulted_);
    }
    moving_ = true;
    last_motion_time_ns_ = now_ns;
  }

  sequence_ = (sequence_ + 1) & 0xFF;
  return MakeReply(cmd.sequence, ReplyCode::kOk, armed_, faulted_);
}

bool DaemonCore::CheckWatchdog(uint64_t now_ns) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (stop_pending_) {
    IssueStop("retry_unconfirmed");
    return true;
  }
  if (!armed_ || faulted_ || !moving_) return false;

  // No watchdog when motion is disabled.
  if (!config_.allow_motion) return false;

  // If we're beyond the watchdog window without a valid command, fault.
  if (now_ns - last_motion_time_ns_ > config_.watchdog_ns) {
    if (!stop_pending_) {
      std::cerr << "[g1-daemon] WATCHDOG EXPIRED — issuing StopMove\n";
      faulted_ = true;
      armed_ = false;
      IssueStop("watchdog_expired");
      moving_ = false;
      return true;
    }
  }
  return false;
}

void DaemonCore::OnDisconnect() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (config_.allow_motion && (armed_ || moving_ || stop_pending_)) {
    std::cerr << "[g1-daemon] ADAPTER DISCONNECTED — issuing StopMove\n";
    IssueStop("adapter_disconnect");
    moving_ = false;
    faulted_ = true;
  }
  armed_ = false;
}

bool DaemonCore::stop_unconfirmed() {
  std::lock_guard<std::mutex> lock(mutex_);
  return stop_pending_;
}

}  // namespace g1_chassis
