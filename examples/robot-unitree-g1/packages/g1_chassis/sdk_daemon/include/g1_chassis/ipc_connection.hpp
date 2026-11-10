#pragma once

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <string>
#include <sys/socket.h>
#include <sys/un.h>
#include <thread>
#include <unistd.h>

namespace g1_chassis {

/** Connect once ready within a fixed deadline; return an owned nonblocking fd or preserve errno.
 * Retry only startup errors with fresh sockets, without sending packets or probing and disconnecting.
 * An established connection is never retried, so runtime disconnect safety remains unchanged.
 */
inline int ConnectDaemonSocket(const std::string &path,
                               std::chrono::milliseconds timeout) {
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  if (path.empty() || path.front() != '/' || path.find('\0') != std::string::npos) {
    errno = EINVAL;
    return -1;
  }
  if (path.size() >= sizeof(address.sun_path)) {
    errno = ENAMETOOLONG;
    return -1;
  }
  std::memcpy(address.sun_path, path.c_str(), path.size() + 1U);
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (true) {
    const int descriptor =
        ::socket(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (descriptor < 0) return -1;
    if (::connect(descriptor, reinterpret_cast<const sockaddr *>(&address),
                  sizeof(address)) == 0) {
      return descriptor;
    }
    const int error = errno;
    ::close(descriptor);
    const auto now = std::chrono::steady_clock::now();
    if ((error != ENOENT && error != ECONNREFUSED && error != EAGAIN &&
         error != EINTR) || now >= deadline) {
      errno = error;
      return -1;
    }
    std::this_thread::sleep_until(
        std::min(deadline, now + std::chrono::milliseconds(50)));
  }
}

}
