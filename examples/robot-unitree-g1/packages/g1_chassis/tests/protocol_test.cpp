#include <catch2/catch.hpp>
#include <type_traits>
#include "g1_chassis/daemon_core.hpp"
#include "g1_chassis/loco_client_interface.hpp"
#include "g1_chassis/protocol.hpp"

using namespace g1_chassis;

namespace {

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
  int32_t SetVelocity(float vx, float vy, float omega, float) override {
    ++velocity_calls;
    last_vx = vx;
    last_vy = vy;
    last_omega = omega;
    return velocity_result;
  }
  int32_t StopMove() override {
    ++stop_calls;
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

TEST_CASE("velocity forwarding preserves operator locomotion mode", "[daemon]") {
  FakeLocoClient client;
  DaemonCore core(MotionConfig(), client);

  const ReplyPacket reply = core.Handle(VelocityCommand(0.2F, 0.0F, 0.3F), 1);

  REQUIRE(reply.code == static_cast<uint8_t>(ReplyCode::kOk));
  REQUIRE(client.start_calls == 0);
  REQUIRE(client.balance_stand_calls == 0);
  REQUIRE(client.fsm_id_calls == 1);
  REQUIRE(client.fsm_mode_calls == 1);
  REQUIRE(client.balance_mode_calls == 1);
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

TEST_CASE("SDK velocity failure rejects the command", "[daemon]") {
  FakeLocoClient client;
  client.velocity_result = 3104;
  DaemonCore core(MotionConfig(), client);

  const ReplyPacket reply = core.Handle(VelocityCommand(0.2F, 0.0F, 0.0F), 1);

  REQUIRE(reply.code == static_cast<uint8_t>(ReplyCode::kSdkError));
  REQUIRE(client.velocity_calls == 1);
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
