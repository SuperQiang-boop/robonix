#ifndef INSPIRECV_ACCELERATION_H_
#define INSPIRECV_ACCELERATION_H_

#include <cstdint>

#include <inspirecv/core/define.h>

namespace inspirecv {

enum class AccelerationBackend : uint8_t {
    kCpu = 0,
    kCuda = 1,
};

enum class AccelerationPreference : uint8_t {
    kAuto = 0,
    kCpu = 1,
    kCuda = 2,
};

// CUDA is disabled by default. Enabling it fails cleanly when this build has
// no CUDA support or the process cannot access a compatible device.
bool INSPIRECV_API SetCudaAccelerationEnabled(bool enabled) noexcept;
bool INSPIRECV_API IsCudaAccelerationEnabled() noexcept;
bool INSPIRECV_API IsCudaAccelerationAvailable() noexcept;

// Process-wide preference used by APIs without a per-operation override.
// Auto applies conservative default thresholds benchmarked on an RTX 3060.
// The thresholds are used on every compatible CUDA device; unprofiled
// operations remain on CPU.
void INSPIRECV_API SetAccelerationPreference(
  AccelerationPreference preference) noexcept;
AccelerationPreference INSPIRECV_API GetAccelerationPreference() noexcept;

// Reports the backend used by the most recent Image operation on this thread.
AccelerationBackend INSPIRECV_API GetLastImageExecutionBackend() noexcept;

}  // namespace inspirecv

#endif  // INSPIRECV_ACCELERATION_H_
