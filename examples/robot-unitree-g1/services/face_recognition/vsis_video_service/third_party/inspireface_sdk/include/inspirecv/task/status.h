#ifndef INSPIRECV_TASK_STATUS_H_
#define INSPIRECV_TASK_STATUS_H_

#include <cstdint>

#include <inspirecv/core/define.h>

namespace inspirecv {
namespace task {

enum class Status : uint8_t {
    kOk = 0,
    kInvalidArgument = 1,
    kUnsupportedSampling = 2,
    kUnsupportedConversion = 3,
    kUnsupportedElementType = 4,
    kAccelerationDisabled = 5,
    kAccelerationUnavailable = 6,
    kAccelerationFailure = 7,
    kNonInvertibleTransform = 8,
};

const char* INSPIRECV_API StatusMessage(Status status);

}  // namespace task
}  // namespace inspirecv

#endif  // INSPIRECV_TASK_STATUS_H_
