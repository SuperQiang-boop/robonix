#include <catch2/catch.hpp>

#include <cstdlib>
#include <fcntl.h>
#include <stdexcept>

#include "g1_chassis/ipc_connection.hpp"

namespace {

class FakeDaemonSocket {
 public:
  /** Reserve a private temporary path without starting an SDK process. */
  FakeDaemonSocket() {
    char directory[] = "/tmp/g1-ipc-test-XXXXXX";
    if (::mkdtemp(directory) == nullptr) {
      throw std::runtime_error("cannot create test directory");
    }
    directory_ = directory;
    path = directory_ + "/daemon.sock";
  }

  /** Close only this test's descriptor and remove its temporary socket path. */
  ~FakeDaemonSocket() {
    Close();
    ::unlink(path.c_str());
    ::rmdir(directory_.c_str());
  }

  /** Bind a fake seqpacket socket, optionally leaving it unlistened as a stale path. */
  int Bind(bool listen, int type = SOCK_SEQPACKET) {
    descriptor = ::socket(AF_UNIX, type | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
    if (descriptor < 0) return errno;
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    std::memcpy(address.sun_path, path.c_str(), path.size() + 1U);
    if (::bind(descriptor, reinterpret_cast<const sockaddr *>(&address),
               sizeof(address)) != 0) return errno;
    if (listen && ::listen(descriptor, 1) != 0) return errno;
    return 0;
  }

  void Close() {
    if (descriptor >= 0) ::close(descriptor);
    descriptor = -1;
  }

  std::string path;
  int descriptor{-1};

 private:
  std::string directory_;
};

}

TEST_CASE("IPC connects only once a real listener replaces a missing or stale path", "[ipc_connection]") {
  FakeDaemonSocket server;
  SECTION("missing path") {}
  SECTION("stale socket path") {
    REQUIRE(server.Bind(false) == 0);
    server.Close();
  }
  int server_error = 0;
  std::thread listener([&]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    ::unlink(server.path.c_str());
    server_error = server.Bind(true);
  });
  const int client = g1_chassis::ConnectDaemonSocket(
      server.path, std::chrono::milliseconds(1000));
  listener.join();
  const int peer = ::accept4(server.descriptor, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
  const int duplicate = ::accept4(server.descriptor, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
  const int descriptor_flags = ::fcntl(client, F_GETFD);
  const int status_flags = ::fcntl(client, F_GETFL);
  if (client >= 0) ::close(client);
  if (peer >= 0) ::close(peer);
  if (duplicate >= 0) ::close(duplicate);
  REQUIRE(server_error == 0);
  REQUIRE(client >= 0);
  REQUIRE(peer >= 0);
  REQUIRE(duplicate == -1);
  REQUIRE((descriptor_flags & FD_CLOEXEC) != 0);
  REQUIRE((status_flags & O_NONBLOCK) != 0);
}

TEST_CASE("IPC startup errors respect one total deadline and preserve errno", "[ipc_connection]") {
  FakeDaemonSocket server;
  int expected_error = ENOENT;
  SECTION("missing daemon") {}
  SECTION("dead daemon leaves a stale socket") {
    REQUIRE(server.Bind(false) == 0);
    server.Close();
    expected_error = ECONNREFUSED;
  }
  const auto started = std::chrono::steady_clock::now();
  const int client = g1_chassis::ConnectDaemonSocket(
      server.path, std::chrono::milliseconds(150));
  const int error = errno;
  const auto elapsed = std::chrono::steady_clock::now() - started;
  if (client >= 0) ::close(client);
  REQUIRE(client == -1);
  REQUIRE(error == expected_error);
  REQUIRE(elapsed >= std::chrono::milliseconds(150));
  REQUIRE(elapsed < std::chrono::seconds(1));
}

TEST_CASE("IPC rejects invalid socket paths without truncation", "[ipc_connection]") {
  REQUIRE(g1_chassis::ConnectDaemonSocket("relative.sock", std::chrono::seconds(1)) == -1);
  REQUIRE(errno == EINVAL);
  REQUIRE(g1_chassis::ConnectDaemonSocket("/" + std::string(108, 'x'), std::chrono::seconds(1)) == -1);
  REQUIRE(errno == ENAMETOOLONG);
}

TEST_CASE("IPC does not retry incompatible socket types", "[ipc_connection]") {
  FakeDaemonSocket server;
  REQUIRE(server.Bind(true, SOCK_STREAM) == 0);
  const auto started = std::chrono::steady_clock::now();
  const int client = g1_chassis::ConnectDaemonSocket(server.path, std::chrono::seconds(3));
  const int error = errno;
  const auto elapsed = std::chrono::steady_clock::now() - started;
  if (client >= 0) ::close(client);
  REQUIRE(client == -1);
  REQUIRE(error == EPROTOTYPE);
  REQUIRE(elapsed < std::chrono::seconds(1));
}
