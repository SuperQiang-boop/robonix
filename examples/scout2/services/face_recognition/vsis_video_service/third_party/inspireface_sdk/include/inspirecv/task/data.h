#ifndef INSPIRECV_TASK_DATA_H_
#define INSPIRECV_TASK_DATA_H_

#include <cstddef>
#include <cstdint>

namespace inspirecv {
namespace task {

enum class ElementType : uint8_t {
    kUInt8,
    kFloat32,
};

enum class TensorOrder : uint8_t {
    kHwc,
    kChw,
    kChannelPacked4,
};

enum class BorderMode : uint8_t {
    kReplicate = 0,
    kConstant = 1,
    kRepeat = 2,
};

enum class SamplingMode : uint8_t {
    kNearest = 0,
    kLinear = 1,
    kCubic = 2,
};

enum class PixelFormat : uint8_t {
    kRgba = 0,
    kRgb = 1,
    kBgr = 2,
    kGray = 3,
    kBgra = 4,
    kYCrCb = 5,
    kYuv = 6,
    kHsv = 7,
    kXyz = 8,
    kBgr555 = 9,
    kBgr565 = 10,
    kNv21 = 11,
    kNv12 = 12,
    kI420 = 13,
    kHsvFull = 14,
};

// Non-owning destination descriptor. Zero strides select tightly packed
// defaults for the chosen tensor order.
struct TensorBuffer {
    void* data = nullptr;
    size_t row_stride_bytes = 0;
    size_t channel_stride_bytes = 0;
    int width = 0;
    int height = 0;
    int channels = 0;
    ElementType element_type = ElementType::kFloat32;
    TensorOrder order = TensorOrder::kHwc;
};

// Non-owning input descriptor. A zero row_stride_bytes selects the natural
// stride for the configured pixel format, including semi-planar YUV formats.
struct RawImageView {
    const uint8_t* data = nullptr;
    size_t row_stride_bytes = 0;
    int width = 0;
    int height = 0;
};

}  // namespace task
}  // namespace inspirecv

#endif  // INSPIRECV_TASK_DATA_H_
