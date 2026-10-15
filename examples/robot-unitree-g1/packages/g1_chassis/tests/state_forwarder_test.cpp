#include <catch2/catch.hpp>

#include <atomic>
#include <chrono>
#include <poll.h>
#include <thread>

#include <sys/socket.h>
#include <unistd.h>

#include "g1_chassis/state_forwarder.hpp"
#include "g1_chassis/state_diagnostics.hpp"

TEST_CASE("state diagnostics separate fresh stationary samples from missing reception", "[state_forwarder]") {
  g1_chassis::StateDiagnostics diagnostics;
  g1_chassis::StatePacket state;
  state.sequence = 1;
  state.received_monotonic_ns = 1'000'000'000ULL;
  state.position[0] = 2.0F;
  const auto first = diagnostics.Update(&state, 1'000'000'000ULL, 1);
  REQUIRE(first.find("delta_valid=0") != std::string::npos);
  REQUIRE(diagnostics.Update(&state, 1'020'000'000ULL, 1).empty());
  state.sequence = 501;
  state.received_monotonic_ns = 2'000'000'000ULL;
  const auto stationary = diagnostics.Update(&state, 2'000'000'000ULL, 501);
  REQUIRE(stationary.find("rx_hz=500.000000") != std::string::npos);
  REQUIRE(stationary.find("age_ms=0.000000") != std::string::npos);
  REQUIRE(stationary.find("delta_valid=1 dxy=0.000000") != std::string::npos);
  const auto stale = diagnostics.Update(&state, 3'000'000'000ULL, 501);
  REQUIRE(stale.find("rx_hz=0.000000") != std::string::npos);
  REQUIRE(stale.find("age_ms=1000.000000") != std::string::npos);
}

TEST_CASE("state diagnostics report displacement and send backlog without modifying samples", "[state_forwarder]") {
  g1_chassis::StateDiagnostics diagnostics;
  g1_chassis::StatePacket state;
  state.sequence = 100;
  state.received_monotonic_ns = 1'000'000'000ULL;
  diagnostics.Update(&state, 1'000'000'000ULL, 100);
  state.sequence = 600;
  state.received_monotonic_ns = 1'990'000'000ULL;
  state.position[0] = 0.3F;
  state.position[1] = 0.4F;
  state.velocity[0] = 0.2F;
  state.error_code = 7;
  const auto summary = diagnostics.Update(&state, 2'000'000'000ULL, 100);
  REQUIRE(summary.find("tx_seq=100 rx_seq=600") != std::string::npos);
  REQUIRE(summary.find("age_ms=10.000000") != std::string::npos);
  REQUIRE(summary.find("error=7") != std::string::npos);
  REQUIRE(summary.find("dxy=0.500000") != std::string::npos);
  REQUIRE(summary.find("raw_vx=0.200000") != std::string::npos);
  REQUIRE(state.received_monotonic_ns == 1'990'000'000ULL);
  REQUIRE(state.sequence == 600);
}

TEST_CASE("state diagnostics do not invent deltas after absent or reset samples", "[state_forwarder]") {
  g1_chassis::StateDiagnostics diagnostics;
  REQUIRE(diagnostics.Update(nullptr, 0, 0).find("available=0") != std::string::npos);
  g1_chassis::StatePacket state;
  state.sequence = 500;
  state.received_monotonic_ns = 1'000'000'000ULL;
  REQUIRE(diagnostics.Update(&state, 1'000'000'000ULL, 500).find("delta_valid=0") != std::string::npos);
  state.sequence = 1;
  state.received_monotonic_ns = 2'000'000'000ULL;
  const auto reset = diagnostics.Update(&state, 2'000'000'000ULL, 1);
  REQUIRE(reset.find("delta_valid=0") != std::string::npos);
  REQUIRE(reset.find("rx_hz=0.000000") != std::string::npos);
}

TEST_CASE("state delivery continues while a motion RPC is blocked", "[state_forwarder]") {
  int sockets[2]{};
  REQUIRE(::socketpair(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK, 0, sockets) == 0);
  std::atomic<bool> sdk_call_blocked{false};
  g1_chassis::StatePacket latest;
  latest.sequence = 42;
  latest.received_monotonic_ns = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count());
  g1_chassis::StateForwarder forwarder(
      sockets[0], [&latest](g1_chassis::StatePacket *state) {
        *state = latest;
        return true;
      });
  std::thread blocked_rpc([&sdk_call_blocked]() {
    sdk_call_blocked.store(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  });
  while (!sdk_call_blocked.load()) std::this_thread::yield();

  pollfd descriptor{sockets[1], POLLIN, 0};
  const int ready = ::poll(&descriptor, 1, 150);
  g1_chassis::StatePacket received;
  const auto received_size = ready == 1
      ? ::recv(sockets[1], &received, sizeof(received), 0)
      : static_cast<ssize_t>(-1);
  const bool call_is_still_blocked = sdk_call_blocked.load();

  blocked_rpc.join();
  REQUIRE(ready == 1);
  REQUIRE(received_size == sizeof(received));
  REQUIRE(received.sequence == 42);
  REQUIRE(call_is_still_blocked);
  ::close(sockets[0]);
  ::close(sockets[1]);
}

TEST_CASE("state forwarder drops stale SDK samples", "[state_forwarder]") {
  int sockets[2]{};
  REQUIRE(::socketpair(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK, 0, sockets) == 0);
  g1_chassis::StatePacket stale;
  stale.sequence = 1;
  stale.received_monotonic_ns = 1;
  g1_chassis::StateForwarder forwarder(
      sockets[0], [&stale](g1_chassis::StatePacket *state) {
        *state = stale;
        return true;
      });

  pollfd descriptor{sockets[1], POLLIN, 0};
  REQUIRE(::poll(&descriptor, 1, 80) == 0);
  ::close(sockets[0]);
  ::close(sockets[1]);
}
