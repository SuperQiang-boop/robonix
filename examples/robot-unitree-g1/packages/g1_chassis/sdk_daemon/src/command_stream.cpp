#include "g1_chassis/command_stream.hpp"

#include <sys/socket.h>

namespace g1_chassis {

/** Drain a bounded command batch without replaying queued motion; a queued stop takes priority. */
CommandReadResult ReadLatestCommand(int descriptor) {
  CommandReadResult latest;
  bool stop_received = false;
  int drained = 0;
  for (; drained < 256; ++drained) {
    CommandReadResult next;
    next.size = ::recv(descriptor, &next.command, sizeof(next.command),
                       MSG_DONTWAIT | MSG_TRUNC);
    next.error = next.size < 0 ? errno : 0;
    if (next.size < 0) {
      if (next.error == EINTR) continue;
      if (next.error == EAGAIN || next.error == EWOULDBLOCK) return latest;
      return next;
    }
    if (next.size != static_cast<ssize_t>(sizeof(next.command)) ||
        next.command.type != static_cast<uint8_t>(PacketType::kCmd)) {
      return next;
    }
    const bool is_stop = next.command.vx == 0 && next.command.vy == 0 &&
                         next.command.omega == 0;
    if (!stop_received || is_stop) latest = next;
    stop_received = stop_received || is_stop;
  }
  for (int attempt = 0; attempt < 4096; ++attempt) {
    CommandPacket discarded{};
    const auto size = ::recv(descriptor, &discarded, sizeof(discarded),
                             MSG_DONTWAIT | MSG_TRUNC);
    if (size < 0) {
      if (errno == EINTR) {
        --attempt;
        continue;
      }
      if (errno == EAGAIN || errno == EWOULDBLOCK) break;
      CommandReadResult failed;
      failed.size = size;
      failed.error = errno;
      return failed;
    }
    if (size == 0) return {latest.command, 0, 0};
    if (attempt == 4095) {
      CommandReadResult overflow;
      overflow.size = 0;
      overflow.error = 0;
      return overflow;
    }
  }
  latest.command.vx = 0;
  latest.command.vy = 0;
  latest.command.omega = 0;
  return latest;
}

}
