#include <catch2/catch.hpp>

#include <sys/socket.h>
#include <unistd.h>

#include "g1_chassis/command_stream.hpp"

namespace {

g1_chassis::CommandPacket Command(int32_t vx) {
  g1_chassis::CommandPacket command{};
  command.type = static_cast<uint8_t>(g1_chassis::PacketType::kCmd);
  command.vx = vx;
  return command;
}

}

TEST_CASE("command drain keeps the latest queued velocity", "[command_stream]") {
  int sockets[2]{};
  REQUIRE(::socketpair(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK, 0, sockets) == 0);
  const auto first = Command(100);
  const auto latest = Command(300);
  REQUIRE(::send(sockets[0], &first, sizeof(first), 0) == sizeof(first));
  REQUIRE(::send(sockets[0], &latest, sizeof(latest), 0) == sizeof(latest));

  const auto result = g1_chassis::ReadLatestCommand(sockets[1]);

  REQUIRE(result.size == sizeof(latest));
  REQUIRE(result.command.vx == 300);
  ::close(sockets[0]);
  ::close(sockets[1]);
}

TEST_CASE("queued stop takes priority over later queued motion", "[command_stream]") {
  int sockets[2]{};
  REQUIRE(::socketpair(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK, 0, sockets) == 0);
  const auto motion = Command(100);
  const auto stop = Command(0);
  const auto later_motion = Command(300);
  REQUIRE(::send(sockets[0], &motion, sizeof(motion), 0) == sizeof(motion));
  REQUIRE(::send(sockets[0], &stop, sizeof(stop), 0) == sizeof(stop));
  REQUIRE(::send(sockets[0], &later_motion, sizeof(later_motion), 0) == sizeof(later_motion));

  const auto result = g1_chassis::ReadLatestCommand(sockets[1]);

  REQUIRE(result.size == sizeof(stop));
  REQUIRE(result.command.vx == 0);
  ::close(sockets[0]);
  ::close(sockets[1]);
}

TEST_CASE("oversized command backlog is drained and replaced by stop", "[command_stream]") {
  int sockets[2]{};
  REQUIRE(::socketpair(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK, 0, sockets) == 0);
  int send_buffer = 1 << 20;
  REQUIRE(::setsockopt(sockets[0], SOL_SOCKET, SO_SNDBUF, &send_buffer,
                       sizeof(send_buffer)) == 0);
  const auto motion = Command(100);
  for (int index = 0; index < 300; ++index) {
    REQUIRE(::send(sockets[0], &motion, sizeof(motion), 0) == sizeof(motion));
  }

  const auto result = g1_chassis::ReadLatestCommand(sockets[1]);

  REQUIRE(result.size == sizeof(motion));
  REQUIRE(result.command.vx == 0);
  const auto drained = g1_chassis::ReadLatestCommand(sockets[1]);
  REQUIRE(drained.size < 0);
  ::close(sockets[0]);
  ::close(sockets[1]);
}
