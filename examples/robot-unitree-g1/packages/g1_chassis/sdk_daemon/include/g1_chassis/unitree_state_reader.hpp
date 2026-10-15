#pragma once

#include <memory>
#include <string>

#include "g1_chassis/protocol.hpp"

namespace g1_chassis {

class UnitreeStateReader {
 public:
  UnitreeStateReader();
  ~UnitreeStateReader();
  void Start(const std::string &interface, const std::string &topic,
             bool initialize_factory);
  bool Latest(StatePacket *packet);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}
