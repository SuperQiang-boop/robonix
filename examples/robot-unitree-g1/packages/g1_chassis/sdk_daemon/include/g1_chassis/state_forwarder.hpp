#pragma once

#include <atomic>
#include <functional>
#include <thread>

#include "g1_chassis/protocol.hpp"

namespace g1_chassis {

class StateForwarder {
 public:
  using StateReader = std::function<bool(StatePacket *)>;

  StateForwarder(int peer, StateReader reader);
  ~StateForwarder();
  StateForwarder(const StateForwarder &) = delete;
  StateForwarder &operator=(const StateForwarder &) = delete;

  bool failed() const { return failed_.load(); }

 private:
  void Run();

  int descriptor_{-1};
  StateReader reader_;
  std::atomic<bool> stopping_{false};
  std::atomic<bool> failed_{false};
  std::thread worker_;
};

}
