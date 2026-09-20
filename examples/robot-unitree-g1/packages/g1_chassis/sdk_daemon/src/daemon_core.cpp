#include "g1_chassis/daemon_core.hpp"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <mutex>

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
int32_t DaemonCore::IssueStop() {
  if (stop_pending_) return -1;
  stop_pending_ = true;
  int32_t result = client_.StopMove();
  if (result != 0) {
    std::cerr << "[g1-daemon] StopMove RPC returned " << result << "\n";
  } else {
    std::cerr << "[g1-daemon] StopMove acknowledged\n";
    stop_pending_ = false;
  }
  return result;
}

// Clamp a requested velocity to deployment limits before forwarding it to SDK2.
int32_t DaemonCore::IssueVelocity(float vx, float vy, float omega) {
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
  // Send a continuous move (duration = infinity for streaming).
  const int32_t result = client_.SetVelocity(vx, vy, omega, 86400.0F);
  std::cerr << "[g1-daemon] SetVelocity RPC returned " << result << "\n";
  return result;
}

// Log SDK-reported locomotion values to diagnose operator mode mismatches.
void DaemonCore::LogLocoState() {
  std::int32_t fsm_id = 0;
  std::int32_t fsm_mode = 0;
  std::int32_t balance_mode = 0;
  const auto fsm_id_result = client_.GetFsmId(&fsm_id);
  const auto fsm_mode_result = client_.GetFsmMode(&fsm_mode);
  const auto balance_mode_result = client_.GetBalanceMode(&balance_mode);
  std::cerr << "[g1-daemon] locomotion state: fsm_id=" << fsm_id
            << " (rpc=" << fsm_id_result << "), fsm_mode=" << fsm_mode
            << " (rpc=" << fsm_mode_result << "), balance_mode="
            << balance_mode << " (rpc=" << balance_mode_result << ")\n";
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
      LogLocoState();
      // The operator establishes the G1 locomotion mode before motion is
      // enabled. Do not overwrite it here with Start or BalanceStand.
      armed_ = true;
      last_arm_time_ns_ = now_ns;
      std::cerr << "[g1-daemon] ARMED (first valid cmd_vel)\n";
    }

    // Stop promptly on the first zero command after motion. Nav2 commonly
    // publishes zero commands at a high rate, so suppress duplicate RPCs.
    if (is_zero) {
      if (moving_ && IssueStop() != 0) {
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
                      static_cast<float>(omega)) != 0) {
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

  if (!armed_ || faulted_ || !moving_) return false;

  // No watchdog when motion is disabled.
  if (!config_.allow_motion) return false;

  // If we're beyond the watchdog window without a valid command, fault.
  if (now_ns - last_motion_time_ns_ > config_.watchdog_ns) {
    if (!stop_pending_) {
      std::cerr << "[g1-daemon] WATCHDOG EXPIRED — issuing StopMove\n";
      IssueStop();
      faulted_ = true;
      return true;
    }
  }
  return false;
}

void DaemonCore::OnDisconnect() {
  std::lock_guard<std::mutex> lock(mutex_);
  armed_ = false;
  if (!stop_pending_) {
    std::cerr << "[g1-daemon] ADAPTER DISCONNECTED — issuing StopMove\n";
    stop_pending_ = client_.StopMove() != 0;
    moving_ = false;
    faulted_ = true;
  }
}

bool DaemonCore::stop_unconfirmed() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (stop_pending_) {
    return true;  // Still unconfirmed
  }
  // Retry stop if it was previously unconfirmed but client is ready.
  if (armed_ || stop_pending_) {
    client_.StopMove();
    return true;
  }
  return false;
}

}  // namespace g1_chassis
