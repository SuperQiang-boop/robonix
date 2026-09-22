#ifndef INSPIRECV_TASK_CUDA_H_
#define INSPIRECV_TASK_CUDA_H_

#include <cstddef>
#include <cstdint>
#include <memory>

#include <inspirecv/cuda/image.h>
#include <inspirecv/task/pipeline.h>

namespace inspirecv {
namespace task {
namespace cuda {

// CUDA headers are intentionally absent from the public API. Device addresses
// and streams can be passed by CUDA/TensorRT callers without making ordinary
// InspireCV consumers depend on a CUDA Toolkit installation.
struct DeviceImageView {
    std::uintptr_t data = 0;
    size_t row_stride_bytes = 0;
    int width = 0;
    int height = 0;
};

struct DeviceTensorBuffer {
    std::uintptr_t data = 0;
    size_t row_stride_bytes = 0;
    size_t channel_stride_bytes = 0;
    int width = 0;
    int height = 0;
    int channels = 0;
    ElementType element_type = ElementType::kFloat32;
    TensorOrder order = TensorOrder::kChw;
};

// Device-to-device counterpart of task::Pipeline. Run is asynchronous with
// respect to the supplied CUDA stream. A null stream selects CUDA's default
// stream. The caller owns all device memory and stream synchronization.
class INSPIRECV_API Pipeline final {
   public:
    explicit Pipeline(const PipelineOptions& options);
    ~Pipeline();

    Pipeline(Pipeline&& other) noexcept;
    Pipeline& operator=(Pipeline&& other) noexcept;
    Pipeline(const Pipeline&) = delete;
    Pipeline& operator=(const Pipeline&) = delete;

    Status SetTransform(const TransformMatrix& transform);
    Status ConfigurationStatus() const noexcept;
    Status Run(const DeviceImageView& source,
               const DeviceTensorBuffer& destination,
               void* stream = nullptr);

    // Direct bridge from the owning device-resident Image API. The source
    // must be a three-channel uint8 image matching PipelineOptions.
    Status Run(const ::inspirecv::cuda::DeviceImage& source,
               const DeviceTensorBuffer& destination,
               void* stream = nullptr);

    // Enqueues a batch on one stream without synchronizing between images.
    // All descriptors are validated before the first kernel is launched.
    // Same-shaped items reuse the cached geometry and are the preferred path.
    Status RunBatch(const DeviceImageView* sources,
                    const DeviceTensorBuffer* destinations,
                    size_t count, void* stream = nullptr);

   private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace cuda
}  // namespace task
}  // namespace inspirecv

#endif  // INSPIRECV_TASK_CUDA_H_
