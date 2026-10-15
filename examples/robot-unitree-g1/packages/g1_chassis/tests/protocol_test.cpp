#include <catch2/catch.hpp>
#include <iostream>
#include <sstream>
#include <type_traits>
#include "g1_chassis/daemon_core.hpp"
#include "g1_chassis/loco_client_interface.hpp"
#include "g1_chassis/protocol.hpp"

using namespace g1_chassis;

namespace {

class CaptureDaemonLog {
 public:
  CaptureDaemonLog() : previous_(std::cerr.rdbuf(output_.rdbuf())) {}
  ~CaptureDaemonLog() { std::cerr.rdbuf(previous_); }
  std::string text() const { return output_.str(); }

 private:
  std::ostringstream output_;
  std::streambuf *previous_;
};

class FakeLocoClient final : public ILocoClient {
 public:
  bool Initialize(const std::string &, std::string *) override { return true; }
  int32_t PrepareArm() override { return prepare_result; }
  int32_t GetFsmId(int32_t *value) override {
    ++fsm_id_calls;
    *value = fsm_id;
    return fsm_id_result;
  }
  int32_t GetFsmMode(int32_t *value) override {
    ++fsm_mode_calls;
    *value = fsm_mode;
    return fsm_mode_result;
  }
  int32_t GetBalanceMode(int32_t *value) override {
    ++balance_mode_calls;
    *value = balance_mode;
    return balance_mode_result;
  }
  // Record the exact velocity passed by the daemon without contacting hardware.
  int32_t SetVelocity(float vx, float vy, float omega, float duration) override {
    ++velocity_calls;
    last_vx = vx;
    last_vy = vy;
    last_omega = omega;
    last_duration = duration;
    return velocity_result;
  }
  int32_t StopMove() override {
    ++stop_calls;
    if (stop_failures_remaining > 0) {
      --stop_failures_remaining;
      return stop_failure_code;
    }
    return stop_result;
  }
  int32_t BalanceStand() override {
    ++balance_stand_calls;
    return 0;
  }
  int32_t Start() override {
    ++start_calls;
    return 0;
  }
  int32_t Damp() override { return 0; }
  int32_t StandUp() override { return 0; }

  int32_t prepare_result{0};
  int32_t velocity_result{0};
  int32_t stop_result{0};
  int32_t stop_failure_code{3104};
  int stop_failures_remaining{0};
  int32_t fsm_id_result{0};
  int32_t fsm_mode_result{0};
  int32_t balance_mode_result{0};
  int velocity_calls{0};
  int stop_calls{0};
  int balance_stand_calls{0};
  int start_calls{0};
  int fsm_id_calls{0};
  int fsm_mode_calls{0};
  int balance_mode_calls{0};
  int32_t fsm_id{500};
  int32_t fsm_mode{2};
  int32_t balance_mode{1};
  float last_vx{0.0F};
  float last_vy{0.0F};
  float last_omega{0.0F};
  float last_duration{0.0F};
};

// Build a fixed-point IPC command using the same scaling as the ROS adapter.
CommandPacket VelocityCommand(float vx, float vy, float omega) {
  CommandPacket command{};
  command.type = static_cast<uint8_t>(PacketType::kCmd);
  command.vx = static_cast<int32_t>(vx * 10000.0F);
  command.vy = static_cast<int32_t>(vy * 10000.0F);
  command.omega = static_cast<int32_t>(omega * 10000.0F);
  return command;
}

// Enable motion for daemon-core tests without requiring a real SDK connection.
DaemonConfig MotionConfig() {
  DaemonConfig config;
  config.allow_motion = true;
  return config;
}

}  // namespace

TEST_CASE("CommandPacket struct layout is compact", "[ipc]") {
  // Expected: 1(type) + 1(seq) + 4(vx) + 4(vy) + 4(omega) + 10(reserved) = 24
  // The struct should have no padding between fields.
  static_assert(std::is_trivially_copyable<CommandPacket>::value,
                "CommandPacket must be trivially copyable for IPC");
  REQUIRE(sizeof(CommandPacket) == 24);
}

TEST_CASE("ReplyPacket struct layout is compact", "[ipc]") {
  // Expected: 1(type) + 1(seq) + 1(code) + 9(reserved) + 1(armed) + 1(faulted) + 2(reserved2) = 16
  static_assert(std::is_trivially_copyable<ReplyPacket>::value,
                "ReplyPacket must be trivially copyable for IPC");
  REQUIRE(sizeof(ReplyPacket) == 16);
}

TEST_CASE("Reply code values", "[ipc]") {
  REQUIRE(static_cast<uint8_t>(ReplyCode::kOk) == 0);
  REQUIRE(static_cast<uint8_t>(ReplyCode::kDisabled) == 1);
  REQUIRE(static_cast<uint8_t>(ReplyCode::kDisarmed) == 2);
  REQUIRE(static_cast<uint8_t>(ReplyCode::kFaulted) == 3);
  REQUIRE(static_cast<uint8_t>(ReplyCode::kMalformed) == 4);
  REQUIRE(static_cast<uint8_t>(ReplyCode::kSdkError) == 5);
}

TEST_CASE("PacketType values", "[ipc]") {
  REQUIRE(static_cast<uint8_t>(PacketType::kCmd) == 1);
  REQUIRE(static_cast<uint8_t>(PacketType::kReply) == 2);
}

TEST_CASE("velocity forwarding leaves operator locomotion mode untouched", "[daemon]") {
  FakeLocoClient client;
  DaemonCore core(MotionConfig(), client);

  const ReplyPacket reply = core.Handle(VelocityCommand(0.2F, 0.0F, 0.3F), 1);

  REQUIRE(reply.code == static_cast<uint8_t>(ReplyCode::kOk));
  REQUIRE(client.start_calls == 0);
  REQUIRE(client.balance_stand_calls == 0);
  REQUIRE(client.fsm_id_calls == 0);
  REQUIRE(client.fsm_mode_calls == 0);
  REQUIRE(client.balance_mode_calls == 0);
  REQUIRE(client.velocity_calls == 1);
  REQUIRE(client.last_vx == 0.2F);
  REQUIRE(client.last_omega == 0.3F);
}

TEST_CASE("zero velocity issues one stop after movement", "[daemon]") {
  FakeLocoClient client;
  DaemonCore core(MotionConfig(), client);

  core.Handle(VelocityCommand(0.2F, 0.0F, 0.0F), 1);
  const ReplyPacket first_zero = core.Handle(VelocityCommand(0.0F, 0.0F, 0.0F), 2);
  const ReplyPacket second_zero = core.Handle(VelocityCommand(0.0F, 0.0F, 0.0F), 3);

  REQUIRE(first_zero.code == static_cast<uint8_t>(ReplyCode::kOk));
  REQUIRE(second_zero.code == static_cast<uint8_t>(ReplyCode::kOk));
  REQUIRE(client.stop_calls == 1);
}

TEST_CASE("successful SDK command diagnostics are bounded and report actual velocity", "[daemon]") {
  FakeLocoClient client;
  auto config = MotionConfig();
  config.max_vx = 0.1F;
  DaemonCore core(config, client);
  CaptureDaemonLog captured;
  const auto original_flags = std::cerr.flags();
  auto command = VelocityCommand(0.2F, 0.0F, 0.3F);
  command.sequence = 255;
  core.Handle(command, 1'000'000'000ULL);
  const auto initial = captured.text();
  REQUIRE(initial.find("SetVelocity calls=1 cmd_seq=255 rpc=0") != std::string::npos);
  REQUIRE(initial.find("vx=0.100000") != std::string::npos);
  REQUIRE(initial.find("duration=0.300000") != std::string::npos);
  core.Handle(command, 1'020'000'000ULL);
  REQUIRE(captured.text() == initial);
  command.sequence = 0;
  core.Handle(command, 2'000'000'000ULL);
  REQUIRE(captured.text().find("SetVelocity calls=3 cmd_seq=0 rpc=0") != std::string::npos);
  REQUIRE(captured.text().find("gap_ms=980.000000") != std::string::npos);
  REQUIRE(std::cerr.flags() == original_flags);
  REQUIRE(client.fsm_id_calls == 0);
}

TEST_CASE("SDK errors and stop causes are logged without waiting for the summary", "[daemon]") {
  FakeLocoClient client;
  DaemonCore core(MotionConfig(), client);
  CaptureDaemonLog captured;
  core.Handle(VelocityCommand(0.2F, 0.0F, 0.0F), 1);
  client.velocity_result = 3104;
  core.Handle(VelocityCommand(0.2F, 0.0F, 0.0F), 2);
  REQUIRE(captured.text().find("SetVelocity calls=2 cmd_seq=") != std::string::npos);
  REQUIRE(captured.text().find("rpc=3104") != std::string::npos);
  REQUIRE(captured.text().find("StopMove reason=velocity_rpc_error rpc=0") != std::string::npos);
  REQUIRE(core.faulted());
}

TEST_CASE("zero stop and motion resumption have immediate diagnostics", "[daemon]") {
  FakeLocoClient client;
  DaemonCore core(MotionConfig(), client);
  CaptureDaemonLog captured;
  core.Handle(VelocityCommand(0.2F, 0.0F, 0.0F), 1);
  core.Handle(VelocityCommand(0.0F, 0.0F, 0.0F), 2);
  REQUIRE(captured.text().find("StopMove reason=zero_command rpc=0") != std::string::npos);
  core.Handle(VelocityCommand(0.2F, 0.0F, 0.0F), 3);
  REQUIRE(captured.text().find("SetVelocity calls=2 cmd_seq=") != std::string::npos);
}

TEST_CASE("SDK velocity failure rejects the command", "[daemon]") {
  FakeLocoClient client;
  client.velocity_result = 3104;
  DaemonCore core(MotionConfig(), client);

  const ReplyPacket reply = core.Handle(VelocityCommand(0.2F, 0.0F, 0.0F), 1);

  REQUIRE(reply.code == static_cast<uint8_t>(ReplyCode::kSdkError));
  REQUIRE(client.velocity_calls == 1);
  REQUIRE(client.stop_calls == 1);
  REQUIRE(core.faulted());
  REQUIRE_FALSE(core.armed());
}

TEST_CASE("SDK velocity duration stays within the motion watchdog", "[daemon]") {
  FakeLocoClient client;
  DaemonConfig config = MotionConfig();
  config.watchdog_ns = 300'000'000ULL;
  DaemonCore core(config, client);

  REQUIRE(core.Handle(VelocityCommand(0.2F, 0.0F, 0.0F), 1).code ==
          static_cast<uint8_t>(ReplyCode::kOk));
  REQUIRE(client.last_duration == Approx(0.3F));
}

TEST_CASE("watchdog fault rejects later motion commands", "[daemon]") {
  FakeLocoClient client;
  DaemonCore core(MotionConfig(), client);

  core.Handle(VelocityCommand(0.2F, 0.0F, 0.0F), 1);
  REQUIRE(core.CheckWatchdog(400'000'002ULL));
  const ReplyPacket reply = core.Handle(VelocityCommand(0.2F, 0.0F, 0.0F), 400'000'003ULL);

  REQUIRE(reply.code == static_cast<uint8_t>(ReplyCode::kFaulted));
  REQUIRE(client.velocity_calls == 1);
}

TEST_CASE("watchdog retries an unconfirmed StopMove", "[daemon]") {
  FakeLocoClient client;
  client.stop_failures_remaining = 1;
  DaemonCore core(MotionConfig(), client);

  core.Handle(VelocityCommand(0.2F, 0.0F, 0.0F), 1);
  REQUIRE(core.CheckWatchdog(400'000'002ULL));
  REQUIRE(core.stop_unconfirmed());
  REQUIRE(core.CheckWatchdog(410'000'002ULL));
  REQUIRE_FALSE(core.stop_unconfirmed());
  REQUIRE(client.stop_calls == 2);
  REQUIRE(core.faulted());
  REQUIRE_FALSE(core.armed());
}
