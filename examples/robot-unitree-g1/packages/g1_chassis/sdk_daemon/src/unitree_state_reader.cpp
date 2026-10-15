#include "g1_chassis/unitree_state_reader.hpp"

#include <chrono>
#include <mutex>

#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace g1_chassis {

struct UnitreeStateReader::Impl {
  std::mutex mutex;
  StatePacket state;
  std::unique_ptr<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>> subscriber;
};

UnitreeStateReader::UnitreeStateReader() : impl_(std::make_unique<Impl>()) {}
UnitreeStateReader::~UnitreeStateReader() {
  if (impl_->subscriber) impl_->subscriber->CloseChannel();
}

/** Receive only SDK state; wall time denotes local reception, not acquisition. */
void UnitreeStateReader::Start(const std::string &interface, const std::string &topic,
                             bool initialize_factory) {
  if (initialize_factory) unitree::robot::ChannelFactory::Instance()->Init(0, interface);
  impl_->subscriber = std::make_unique<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>(topic);
  impl_->subscriber->InitChannel([this](const void *message) {
    const auto &source = *static_cast<const unitree_go::msg::dds_::SportModeState_ *>(message);
    StatePacket state;
    state.received_unix_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    state.received_monotonic_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    state.error_code = source.error_code();
    for (std::size_t index = 0; index < 3; ++index) {
      state.position[index] = source.position()[index];
      state.velocity[index] = source.velocity()[index];
    }
    for (std::size_t index = 0; index < 4; ++index)
      state.quaternion_wxyz[index] = source.imu_state().quaternion()[index];
    state.yaw_speed = source.yaw_speed();
    std::lock_guard<std::mutex> lock(impl_->mutex);
    state.sequence = impl_->state.sequence + 1;
    impl_->state = state;
  });
}

/** Copy the latest real sample without queuing the 500 Hz input stream. */
bool UnitreeStateReader::Latest(StatePacket *packet) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->state.sequence == 0) return false;
  *packet = impl_->state;
  return true;
}

}
