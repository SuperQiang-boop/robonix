#pragma once

#include <cerrno>
#include <sys/types.h>

#include "g1_chassis/protocol.hpp"

namespace g1_chassis {

struct CommandReadResult {
  CommandPacket command{};
  ssize_t size{-1};
  int error{EAGAIN};
};

CommandReadResult ReadLatestCommand(int descriptor);

}
