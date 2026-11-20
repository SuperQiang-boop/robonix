#ifndef INSPIRECV_TASK_ACCELERATION_H_
#define INSPIRECV_TASK_ACCELERATION_H_

#include <cstdint>

#include <inspirecv/core/define.h>

namespace inspirecv {
namespace task {

enum class ExecutionBackend : uint8_t {
    kCpu = 0,
    kCuda = 1,
};

// Host-side backend selection. kDefault is valid for per-pipeline options and
// inherits the process-wide preference. Device-to-device CUDA APIs are not
// affected by this preference.
enum class BackendPreference : uint8_t {
    kDefault = 0,
    kAuto = 1,
    kCpu = 2,
    kCuda = 3,
};

// CUDA acceleration is opt-in and process-wide. Enabling returns false and
// leaves CUDA disabled when this build has no CUDA backend or no usable device.
bool INSPIRECV_API SetCudaEnabled(bool enabled) noexcept;
bool INSPIRECV_API IsCudaEnabled() noexcept;
bool INSPIRECV_API IsCudaAvailable() noexcept;

// Controls host-side Pipeline dispatch when CUDA is enabled. The default is
// kAuto. kDefault is rejected because it would recursively inherit itself.
bool INSPIRECV_API SetDefaultBackendPreference(
  BackendPreference preference) noexcept;
BackendPreference INSPIRECV_API GetDefaultBackendPreference() noexcept;

}  // namespace task
}  // namespace inspirecv

#endif  // INSPIRECV_TASK_ACCELERATION_H_
