#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <unitree/common/time/time_tool.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/g1/audio/g1_audio_client.hpp>

namespace {

constexpr std::size_t kSdkChunkBytes = 96000;
constexpr std::uint64_t kMaxRequestBytes = 32ULL * 1024ULL * 1024ULL;
constexpr char kAppName[] = "robonix_g1_audio";

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

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "[g1-audio-sdk] usage: g1_audio_sdk_helper <interface> <ipc-fd>\n";
    return 2;
  }
  const std::string network_interface = argv[1];
  const int ipc_fd = std::stoi(argv[2]);
  std::cerr << "[g1-audio-sdk] starting interface=" << network_interface
            << " ipc_fd=" << ipc_fd << "\n";

  try {
    std::cerr << "[g1-audio-sdk] initializing ChannelFactory\n";
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
    std::cerr << "[g1-audio-sdk] constructing AudioClient\n";
    auto client = std::make_unique<unitree::robot::g1::AudioClient>();
    std::cerr << "[g1-audio-sdk] initializing AudioClient\n";
    client->Init();
    std::cerr << "[g1-audio-sdk] setting timeout\n";
    client->SetTimeout(10.0f);
    std::cerr << "[g1-audio-sdk] probing volume\n";
    std::uint8_t volume = 0;
    const int result = client->GetVolume(volume);
    std::cerr << "[g1-audio-sdk] volume probe result=" << result
              << " volume=" << static_cast<unsigned>(volume) << "\n";
    if (result != 0) {
      const std::string message = "ERROR:SDK2 audio volume probe failed: " +
                                  std::to_string(result) + "\n";
      write_all(ipc_fd, message.data(), message.size());
      return 1;
    }
    const std::string ready = "READY\n";
    if (!write_all(ipc_fd, ready.data(), ready.size())) return 1;

    while (true) {
      std::uint64_t network_size = 0;
      if (!read_all(ipc_fd, &network_size, sizeof(network_size))) break;
      const std::uint64_t request_size = be64toh(network_size);
      if (request_size == 0) break;
      if (request_size > kMaxRequestBytes || request_size % 2 != 0) {
        if (!send_status(ipc_fd, -2)) break;
        if (request_size > kMaxRequestBytes) break;
        continue;
      }

      std::vector<std::uint8_t> pcm(static_cast<std::size_t>(request_size));
      if (!read_all(ipc_fd, pcm.data(), pcm.size())) break;

      std::int32_t status = 0;
      const std::string stream_id = std::to_string(
          unitree::common::GetCurrentTimeMillisecond());
      for (std::size_t offset = 0; offset < pcm.size(); offset += kSdkChunkBytes) {
        const std::size_t end = std::min(offset + kSdkChunkBytes, pcm.size());
        std::vector<std::uint8_t> chunk(pcm.begin() + offset, pcm.begin() + end);
        status = client->PlayStream(kAppName, stream_id, std::move(chunk));
        if (status != 0) break;
        if (end < pcm.size()) unitree::common::Sleep(1);
      }
      client->PlayStop(stream_id);
      if (!send_status(ipc_fd, status)) break;
    }
  } catch (const std::exception& error) {
    const std::string message = "ERROR:SDK2 audio initialization failed: " +
                                std::string(error.what()) + "\n";
    write_all(ipc_fd, message.data(), message.size());
    ::close(ipc_fd);
    return 1;
  }

  ::close(ipc_fd);
  return 0;
}
