#pragma once

#include <cstdlib>
#include <chrono>
#include <functional>
#include <string>
#include <thread>
#include <unitree/idl/ros2/String_.hpp>

namespace arm_test {
inline std::function<void(const void *)> handler;
inline std::thread publisher;

// Deliver real SDK String messages without opening DDS or contacting a robot.
inline void Emit(const std::string &json) {
  std_msgs::msg::dds_::String_ message;
  message.data(json);
  handler(&message);
}

inline std::string Scenario() {
  return std::getenv("ARM_TEST_SCENARIO");
}

// Reproduce this firmware's 10 Hz no-preset state for a sustained idle interval.
inline void EmitIdle() {
  for (int index = 0; index < 3; ++index) {
    if (index) std::this_thread::sleep_for(std::chrono::milliseconds(120));
    Emit(R"({"id":0,"holding":false,"name":""})");
  }
}
}  // namespace arm_test

namespace unitree::robot {
class ChannelFactory {
 public:
  static ChannelFactory *Instance() {
    static ChannelFactory instance;
    return &instance;
  }
  void Init(int, const std::string &) {}
};

template <typename Message>
class ChannelSubscriber {
 public:
  explicit ChannelSubscriber(const std::string &) {}
  ~ChannelSubscriber() {
    if (arm_test::publisher.joinable()) arm_test::publisher.join();
  }

  // A sample delivered on subscription precedes the release request.
  void InitChannel(const std::function<void(const void *)> &callback) {
    arm_test::handler = callback;
    if (arm_test::Scenario() == "old_only") {
      arm_test::Emit(R"({"id":99,"holding":false})");
    } else if (arm_test::Scenario() == "old_idle") {
      arm_test::EmitIdle();
    }
  }
};
}  // namespace unitree::robot

namespace unitree::robot::g1 {
class G1ArmActionClient {
 public:
  void Init() {}
  void SetTimeout(float) {}
  int ExecuteAction(const std::string &) { return 0; }

  // Reproduce reply ordering, failures and invalid feedback around the real CLI.
  int ExecuteAction(int id) {
    if (id != 99) return 0;
    const auto scenario = arm_test::Scenario();
    const std::string released = R"({"id":99,"holding":false})";
    if (scenario == "idle_before_reply" || scenario == "idle_rpc_failure" ||
        scenario == "stale_idle" || scenario == "idle_superseded") {
      arm_test::EmitIdle();
    }
    if (scenario == "stale_idle") {
      std::this_thread::sleep_for(std::chrono::milliseconds(400));
    } else if (scenario == "idle_superseded") {
      arm_test::Emit(R"({"id":25,"holding":false})");
    } else if (scenario == "idle_after_reply") {
      arm_test::publisher = std::thread([] {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        arm_test::EmitIdle();
      });
    } else if (scenario == "idle_single") {
      arm_test::Emit(R"({"id":0,"holding":false,"name":""})");
    } else if (scenario == "idle_holding") {
      arm_test::Emit(R"({"id":0,"holding":true,"name":""})");
    } else if (scenario == "idle_named") {
      arm_test::Emit(R"({"id":0,"holding":false,"name":"wave"})");
    } else if (scenario == "idle_missing_name") {
      arm_test::Emit(R"({"id":0,"holding":false})");
    }
    if (scenario == "before_reply" || scenario == "rpc_failure" || scenario == "superseded") {
      arm_test::Emit(released);
    }
    if (scenario == "after_reply") {
      arm_test::publisher = std::thread([released] {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        arm_test::Emit(released);
      });
    } else if (scenario == "holding" || scenario == "superseded") {
      arm_test::Emit(R"({"id":99,"holding":true})");
    } else if (scenario == "wrong_id") {
      arm_test::Emit(R"({"id":25,"holding":false})");
    } else if (scenario == "malformed") {
      arm_test::Emit(R"({"id":99})");
    }
    return scenario == "rpc_failure" || scenario == "idle_rpc_failure" ? 7401 : 0;
  }
};
}  // namespace unitree::robot::g1
