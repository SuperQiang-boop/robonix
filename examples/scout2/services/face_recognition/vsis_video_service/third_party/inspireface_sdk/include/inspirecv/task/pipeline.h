#ifndef INSPIRECV_TASK_PIPELINE_H_
#define INSPIRECV_TASK_PIPELINE_H_

#include <array>
#include <cstdint>
#include <memory>

#include <inspirecv/core/define.h>
#include <inspirecv/core/image.h>
#include <inspirecv/core/transform_matrix.h>
#include <inspirecv/task/acceleration.h>
#include <inspirecv/task/data.h>
#include <inspirecv/task/status.h>

namespace inspirecv {
namespace task {

struct PipelineOptions {
    PixelFormat input_format = PixelFormat::kRgba;
    PixelFormat output_format = PixelFormat::kRgba;
    SamplingMode sampling = SamplingMode::kNearest;
    BorderMode border = BorderMode::kReplicate;
    std::array<float, 4> mean{{0.0f, 0.0f, 0.0f, 0.0f}};
    std::array<float, 4> scale{{1.0f, 1.0f, 1.0f, 1.0f}};

    // Reproduces the historical identity-float ordering required by a small
    // set of already-converted model inputs. New pipelines should leave this
    // disabled.
    bool preserve_identity_float_order = false;

    // Host-side execution preference. kDefault inherits the process-wide
    // value; explicit CUDA device pipelines are unaffected.
    BackendPreference backend_preference = BackendPreference::kDefault;
};

// Reusable image/tensor preprocessing pipeline. This is the only Task API
// layer that depends on Image; the execution engine remains raw-memory based.
class INSPIRECV_API Pipeline final {
   public:
    explicit Pipeline(const PipelineOptions& options);
    ~Pipeline();

    Pipeline(Pipeline&& other) noexcept;
    Pipeline& operator=(Pipeline&& other) noexcept;
    Pipeline(const Pipeline&) = delete;
    Pipeline& operator=(const Pipeline&) = delete;

    // Replaces the destination-to-source transform only when it is invertible.
    // A rejected transform leaves the previous pipeline state unchanged.
    Status SetTransform(const TransformMatrix& transform);
    void SetBorderValue(uint8_t value);

    // Reports invalid enum values immediately after construction rather than
    // delaying all configuration errors until the first Run call.
    Status ConfigurationStatus() const noexcept;

    // Returns the interleaved pixel format produced by Image output methods.
    PixelFormat OutputFormat() const noexcept;

    // Reports the backend used by the most recent successful conversion.
    // Unsupported CUDA requests deliberately fall back to the CPU backend.
    ExecutionBackend LastExecutionBackend() const noexcept;

    // Image output is transactional: destination is modified only on success.
    Status Run(const Image& source, int output_width, int output_height,
               Image* destination);

    // Converts a raw input (including NV12/NV21/I420) directly into one owned,
    // tightly packed HWC Image allocation. No intermediate tensor or output
    // copy is created. Image color-sensitive operations use BGR conventions;
    // callers producing kRgb must preserve that format information explicitly.
    Status Run(const RawImageView& source, int output_width,
               int output_height, Image* destination);

    // Reuses caller-provided output storage in frame loops without allocating or
    // replacing the destination Image buffer. Its dimensions select the output
    // shape, and its channel count must match the configured output format.
    // Source and destination storage must not overlap. Unlike Run, an execution
    // failure may leave destination partially modified.
    Status RunInto(const Image& source, Image* destination);
    Status RunInto(const RawImageView& source, Image* destination);

    Status Run(const Image& source, const TensorBuffer& destination);
    Status Run(const RawImageView& source, const TensorBuffer& destination);

   private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace task
}  // namespace inspirecv

#endif  // INSPIRECV_TASK_PIPELINE_H_
