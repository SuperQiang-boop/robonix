#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <iostream>
#include <string>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/g1/audio/g1_audio_client.hpp>

namespace {

// Read a complete frame or stop when the owning Speech process disconnects.
bool read_all(int fd, void* data, std::size_t size) {
  auto* cursor = static_cast<char*>(data);
  while (size > 0) {
    const ssize_t count = ::recv(fd, cursor, size, 0);
    if (count == 0) return false;
    if (count < 0) {
      if (errno == EINTR) continue;
      return false;
    }
    cursor += count;
    size -= static_cast<std::size_t>(count);
  }
  return true;
}

// Send a complete response without SIGPIPE if Speech has already stopped.
bool write_all(int fd, const void* data, std::size_t size) {
  const auto* cursor = static_cast<const char*>(data);
  while (size > 0) {
    const ssize_t count = ::send(fd, cursor, size, MSG_NOSIGNAL);
    if (count == 0) return false;
    if (count < 0) {
      if (errno == EINTR) continue;
      return false;
    }
    cursor += count;
    size -= static_cast<std::size_t>(count);
  }
  return true;
}

// Keep AudioClient alive between phrases: TtsMaker returns before audible
// completion. Destroying the client after each request can interrupt playback.
void serve(int fd) {
  unitree::robot::g1::AudioClient client;
  client.Init();
  client.SetTimeout(10.0f);
  const std::string ready = "READY\n";
  if (!write_all(fd, ready.data(), ready.size())) return;
  while (true) {
    std::uint32_t encoded_size = 0;
    if (!read_all(fd, &encoded_size, sizeof(encoded_size))) break;
    const auto size = ntohl(encoded_size);
    if (size == 0 || size > 4096) break;
    std::string text(size, '\0');
    if (!read_all(fd, text.data(), text.size())) break;
    if (text.find('\0') != std::string::npos) break;
    const auto status = client.TtsMaker(text, 0);
    const std::uint32_t encoded_status = htonl(static_cast<std::uint32_t>(status));
    if (!write_all(fd, &encoded_status, sizeof(encoded_status))) break;
  }
}

}  // namespace

// Own the DDS connection until Speech disconnects, including after acceptance.
int main(int argc, char** argv) {
  if (argc != 3 || std::string(argv[1]).empty()) {
    std::cerr << "usage: g1_audio_tts_helper <interface> <ipc-fd>\n";
    return 2;
  }
  bool initialized = false;
  int result = 0;
  int fd = -1;
  try {
    fd = std::stoi(argv[2]);
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    initialized = true;
    serve(fd);
  } catch (const std::exception& error) {
    std::cerr << "G1 TTS helper failed: " << error.what() << "\n";
    result = 1;
  }
  if (initialized) unitree::robot::ChannelFactory::Instance()->Release();
  if (fd >= 0) ::close(fd);
  return result;
}
