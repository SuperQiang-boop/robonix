#include "g1_chassis/state_forwarder.hpp"
#include "g1_chassis/state_diagnostics.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace g1_chassis {

/** Own a duplicate peer descriptor so command RPCs and peer cleanup cannot stall or reuse it. */
StateForwarder::StateForwarder(int peer, StateReader reader)
    : reader_(std::move(reader)) {
  descriptor_ = ::fcntl(peer, F_DUPFD_CLOEXEC, 0);
  if (descriptor_ < 0) {
    throw std::runtime_error("cannot duplicate state IPC socket: " +
                             std::string(std::strerror(errno)));
  }
  try {
    worker_ = std::thread(&StateForwarder::Run, this);
  } catch (...) {
    ::close(descriptor_);
    throw;
  }
}

/** Join state delivery before closing its descriptor; never make SDK motion calls. */
StateForwarder::~StateForwarder() {
  stopping_.store(true);
  if (worker_.joinable()) worker_.join();
  ::close(descriptor_);
}

/** Forward only new, fresh reception-stamped samples independently of synchronous command RPCs. */
void StateForwarder::Run() {
  uint64_t last_sequence = 0;
  StateDiagnostics diagnostics;
  try {
    while (!stopping_.load()) {
      StatePacket state;
      const bool available = reader_(&state);
      const auto now_ns = static_cast<uint64_t>(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::steady_clock::now().time_since_epoch()).count());
      if (available && state.sequence > last_sequence &&
          now_ns >= state.received_monotonic_ns &&
          now_ns - state.received_monotonic_ns <= 150'000'000ULL) {
        const auto sent = ::send(descriptor_, &state, sizeof(state),
                                 MSG_DONTWAIT | MSG_NOSIGNAL);
        if (sent == static_cast<ssize_t>(sizeof(state))) {
          last_sequence = state.sequence;
        } else if (sent >= 0) {
          throw std::runtime_error("incomplete state IPC send");
        } else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
          throw std::runtime_error("state IPC send failed: " +
                                   std::string(std::strerror(errno)));
        }
      }
      const auto summary = diagnostics.Update(available ? &state : nullptr,
                                               now_ns, last_sequence);
      if (!summary.empty()) {
        const auto wall_time = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        std::ostringstream message;
        message << std::fixed << std::setprecision(6)
                << "[g1-daemon] t=" << wall_time << " " << summary << "\n";
        std::cerr << message.str();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  } catch (const std::exception &exception) {
    std::cerr << "[g1-daemon] state forwarding failed: " << exception.what() << "\n";
    failed_.store(true);
  } catch (...) {
    std::cerr << "[g1-daemon] state forwarding failed with an unknown exception\n";
    failed_.store(true);
  }
}

}
