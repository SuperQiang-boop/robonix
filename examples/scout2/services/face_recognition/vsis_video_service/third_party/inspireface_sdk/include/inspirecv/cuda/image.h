#ifndef INSPIRECV_CUDA_IMAGE_H_
#define INSPIRECV_CUDA_IMAGE_H_

#include <cstddef>
#include <cstdint>
#include <memory>

#include <inspirecv/core/define.h>
#include <inspirecv/core/image.h>
#include <inspirecv/core/transform_matrix.h>

namespace inspirecv {
namespace cuda {

enum class Status : uint8_t {
    kOk = 0,
    kInvalidArgument = 1,
    kAccelerationDisabled = 2,
    kAccelerationUnavailable = 3,
    kUnsupportedOperation = 4,
    kAllocationFailure = 5,
    kExecutionFailure = 6,
};

enum class ElementType : uint8_t {
    kUInt8 = 0,
    kFloat32 = 1,
};

// Non-owning description for interoperability with CUDA-aware consumers.
// The address refers to tightly packed device memory owned elsewhere.
struct DeviceImageView {
    std::uintptr_t data = 0;
    size_t row_stride_bytes = 0;
    int width = 0;
    int height = 0;
    int channels = 0;
    ElementType element_type = ElementType::kUInt8;
};

// Owning device-resident image. Upload and Download are synchronization
// boundaries; geometry operations are enqueued on the supplied stream and can
// be chained without intermediate host copies. A null stream selects CUDA's
// default stream. CUDA headers are intentionally absent from this interface.
class INSPIRECV_API DeviceImage final {
public:
    DeviceImage();
    ~DeviceImage();

    DeviceImage(DeviceImage&& other) noexcept;
    DeviceImage& operator=(DeviceImage&& other) noexcept;
    DeviceImage(const DeviceImage&) = delete;
    DeviceImage& operator=(const DeviceImage&) = delete;

    static Status Upload(const Image& source, DeviceImage* destination);
    static Status Upload(const ImageT<float>& source,
                         DeviceImage* destination);

    Status Download(Image* destination, void* stream = nullptr) const;
    Status Download(ImageT<float>* destination,
                    void* stream = nullptr) const;

    Status Resize(int width, int height, bool use_linear,
                  DeviceImage* destination, void* stream = nullptr) const;
    Status WarpAffine(const TransformMatrix& matrix, int width, int height,
                      DeviceImage* destination,
                      void* stream = nullptr) const;
    Status Rotate90(DeviceImage* destination,
                    void* stream = nullptr) const;
    Status Rotate180(DeviceImage* destination,
                     void* stream = nullptr) const;
    Status Rotate270(DeviceImage* destination,
                     void* stream = nullptr) const;

    bool Empty() const noexcept;
    int Width() const noexcept;
    int Height() const noexcept;
    int Channels() const noexcept;
    ElementType Type() const noexcept;
    DeviceImageView View() const noexcept;

private:
    static Status UploadBytes(const void* source, int width, int height,
                              int channels, ElementType type,
                              DeviceImage* destination);
    Status EnqueueGeometry(int operation, int width, int height,
                           const float* affine, DeviceImage* destination,
                           void* stream) const;
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

Status INSPIRECV_API Synchronize(void* stream = nullptr) noexcept;
INSPIRECV_API const char* StatusMessage(Status status) noexcept;

}  // namespace cuda
}  // namespace inspirecv

#endif  // INSPIRECV_CUDA_IMAGE_H_
