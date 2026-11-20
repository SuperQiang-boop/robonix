#ifndef INSPIRECV_TASK_CORE_PREPROCESS_TYPES_H_
#define INSPIRECV_TASK_CORE_PREPROCESS_TYPES_H_

#include <cstddef>

#include <inspirecv/task/core/st_types.h>

namespace inspirecv {
namespace task {

enum StreamFormat {
    RGBA = 0,
    RGB = 1,
    BGR = 2,
    GRAY = 3,
    BGRA = 4,
    YCrCb = 5,
    YUV = 6,
    HSV = 7,
    XYZ = 8,
    BGR555 = 9,
    BGR565 = 10,
    YUV_NV21 = 11,
    YUV_NV12 = 12,
    YUV_I420 = 13,
    HSV_FULL = 14,
};

enum Filter { NEAREST = 0, BILINEAR = 1, BICUBIC = 2 };

enum Wrap { CLAMP_TO_EDGE = 0, ZERO = 1, REPEAT = 2 };

enum class TensorLayout {
    NHWC = 0,
    NCHW = 1,
    NC4HW4 = 2,
};

// Non-owning description of caller-provided tensor memory. A zero stride
// requests the tightly packed default for the selected layout.
struct TensorView {
    void* data = nullptr;
    int width = 0;
    int height = 0;
    int channels = 0;
    halide_type_t type = halide_type_of<float>();
    TensorLayout layout = TensorLayout::NHWC;
    size_t rowStride = 0;
    size_t channelStride = 0;
};

}  // namespace task
}  // namespace inspirecv

#endif  // INSPIRECV_TASK_CORE_PREPROCESS_TYPES_H_
