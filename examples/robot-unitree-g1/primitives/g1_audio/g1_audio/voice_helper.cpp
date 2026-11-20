#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <iostream>
#include <string>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/client/client.hpp>

namespace {

constexpr std::int32_t kSetVoiceModeApi = 1008;
constexpr char kVoiceService[] = "voice";
constexpr char kVoiceApiVersion[] = "1.0.0.0";

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

bool write_all(int fd, const void* data, std::size_t size) {
  const auto* cursor = static_cast<const char*>(data);
  while (size > 0) {
    const ssize_t count = ::send(fd, cursor, size, MSG_NOSIGNAL);
    if (count < 0) {
      if (errno == EINTR) continue;
      return false;
    }
    cursor += count;
    size -= static_cast<std::size_t>(count);
  }
  return true;
}

bool send_status(int fd, std::int32_t status) {
  const std::uint32_t encoded = htonl(static_cast<std::uint32_t>(status));
  return write_all(fd, &encoded, sizeof(encoded));
}

class VoiceClient final : public unitree::robot::Client {
 public:
  VoiceClient() : Client(kVoiceService, false) {}

  void Init() override {
    SetApiVersion(kVoiceApiVersion);
    RegistApi(kSetVoiceModeApi, 0);
  }

  std::int32_t SetMode(std::int32_t mode) {
    std::string response;
    return Call(kSetVoiceModeApi, "{\"mode\": " + std::to_string(mode) + "}",
                response);
  }
};

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "usage: g1_audio_voice_helper <interface> <ipc-fd>\n";
    return 2;
  }

  const std::string network_interface = argv[1];
  const int ipc_fd = std::stoi(argv[2]);

  bool dds_initialized = false;
  try {
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
    dds_initialized = true;
    {
      VoiceClient client;
      client.Init();
      client.SetTimeout(10.0f);

      const std::string ready = "READY\n";
      if (!write_all(ipc_fd, ready.data(), ready.size())) return 1;

      while (true) {
        std::uint32_t encoded_mode = 0;
        if (!read_all(ipc_fd, &encoded_mode, sizeof(encoded_mode))) break;
        const std::int32_t mode = static_cast<std::int32_t>(ntohl(encoded_mode));
        if (mode == 0) break;
        if (mode != 1 && mode != 2) {
          if (!send_status(ipc_fd, -2)) break;
          continue;
        }
        if (!send_status(ipc_fd, client.SetMode(mode))) break;
      }
    }
    unitree::robot::ChannelFactory::Instance()->Release();
    dds_initialized = false;
  } catch (const std::exception& error) {
    const std::string message = "ERROR:SDK2 voice initialization failed: " +
                                std::string(error.what()) + "\n";
    write_all(ipc_fd, message.data(), message.size());
    if (dds_initialized) {
      unitree::robot::ChannelFactory::Instance()->Release();
    }
    ::close(ipc_fd);
    return 1;
  }

  ::close(ipc_fd);
  return 0;
}
