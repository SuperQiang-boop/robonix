#include <catch2/catch.hpp>
#include <limits>
#include "g1_chassis/odometry.hpp"

namespace {

/** Build valid, deterministic IPC samples without a robot or DDS participant. */
g1_chassis::StatePacket State(uint64_t sequence, double yaw = 0) {
  g1_chassis::StatePacket state;
  state.sequence = sequence;
  state.received_monotonic_ns = 1000000000ULL + sequence * 20000000ULL;
  state.received_unix_ns = 1791880000000000000ULL + sequence * 20000000ULL;
  state.quaternion_wxyz[0] = std::cos(yaw / 2);
  state.quaternion_wxyz[3] = std::sin(yaw / 2);
  return state;
}

}

TEST_CASE("SDK state packet has a separate versioned wire layout", "[odom]") {
  REQUIRE(sizeof(g1_chassis::StatePacket) == 76);
  REQUIRE(sizeof(g1_chassis::CommandPacket) == 24);
  REQUIRE(sizeof(g1_chassis::ReplyPacket) == 16);
}

TEST_CASE("SDK pose is normalized and projected to planar navigation", "[odom]") {
  g1_chassis::OdometryProjector projector;
  std::string reason;
  auto state = State(1, 0.7);
  state.position[0] = -1.57;
  state.position[1] = -0.87;
  state.position[2] = 0.73;
  state.velocity[0] = 99;
  REQUIRE(projector.Update(state, &reason));
  REQUIRE(projector.pose().x == Approx(-1.57));
  REQUIRE(projector.pose().yaw == Approx(0.7));
  REQUIRE(projector.pose().vx == 0);
}

TEST_CASE("Pose differences produce child-frame velocity without assuming SDK velocity frame", "[odom]") {
  g1_chassis::OdometryProjector projector;
  std::string reason;
  const double yaw = std::acos(-1.0) / 2;
  for (uint64_t sequence = 1; sequence <= 50; ++sequence) {
    auto state = State(sequence, yaw);
    state.position[1] = static_cast<float>(sequence * 0.004);
    state.velocity[0] = 99;
    REQUIRE(projector.Update(state, &reason));
  }
  REQUIRE(projector.pose().vx == Approx(0.2).margin(0.001));
  REQUIRE(projector.pose().vy == Approx(0).margin(0.001));
}

TEST_CASE("Repeated samples do not refresh odometry", "[odom]") {
  g1_chassis::OdometryProjector projector;
  std::string reason;
  REQUIRE(projector.Update(State(1), &reason));
  REQUIRE_FALSE(projector.Update(State(1), &reason));
  REQUIRE_FALSE(projector.faulted());
  REQUIRE(projector.Update(State(2), &reason));
}

TEST_CASE("Invalid SDK state is rejected", "[odom]") {
  g1_chassis::OdometryProjector projector;
  std::string reason;
  auto state = State(1);
  SECTION("error code") { state.error_code = 1; }
  SECTION("version") { state.version = 2; }
  SECTION("packet type") { state.type = 1; }
  SECTION("zero reception time") { state.received_unix_ns = 0; }
  SECTION("zero quaternion") { state.quaternion_wxyz[0] = 0; }
  SECTION("NaN position") { state.position[0] = std::numeric_limits<float>::quiet_NaN(); }
  REQUIRE_FALSE(projector.Update(state, &reason));
  REQUIRE_FALSE(projector.initialized());
}

TEST_CASE("SDK localization jumps latch a fault rather than teleport navigation TF", "[odom]") {
  g1_chassis::OdometryProjector projector;
  std::string reason;
  REQUIRE(projector.Update(State(1), &reason));
  auto jumped = State(2);
  SECTION("translation") { jumped.position[0] = 0.5; }
  SECTION("rotation") { jumped = State(2, 0.7); }
  REQUIRE_FALSE(projector.Update(jumped, &reason));
  REQUIRE(projector.faulted());
  REQUIRE_FALSE(projector.Update(State(3), &reason));
}

TEST_CASE("Non-increasing local reception times are rejected", "[odom]") {
  g1_chassis::OdometryProjector projector;
  std::string reason;
  auto first = State(1);
  REQUIRE(projector.Update(first, &reason));
  auto next = State(2);
  next.received_monotonic_ns = first.received_monotonic_ns;
  REQUIRE_FALSE(projector.Update(next, &reason));
}

TEST_CASE("Velocity is not extrapolated across a state outage", "[odom]") {
  g1_chassis::OdometryProjector projector;
  std::string reason;
  REQUIRE(projector.Update(State(1), &reason));
  auto resumed = State(20);
  resumed.position[0] = 0.05;
  REQUIRE(projector.Update(resumed, &reason));
  REQUIRE(projector.pose().vx == 0);
}

TEST_CASE("Yaw wrap does not create a false reset", "[odom]") {
  g1_chassis::OdometryProjector projector;
  std::string reason;
  REQUIRE(projector.Update(State(1, 3.13), &reason));
  REQUIRE(projector.Update(State(2, -3.13), &reason));
  REQUIRE_FALSE(projector.faulted());
}
