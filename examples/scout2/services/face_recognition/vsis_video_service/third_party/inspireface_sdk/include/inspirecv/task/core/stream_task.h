#ifndef INSPIRECV_STREAMTASK_CORE_STREAM_TASK_H_
#define INSPIRECV_STREAMTASK_CORE_STREAM_TASK_H_

#include <inspirecv/task/task_status.h>
#include <inspirecv/task/core/preprocess_types.h>
#include "matrix.h"
#include "st_defs.h"
#include <inspirecv/core/transform_matrix.h>
#include <cstddef>

namespace inspirecv {
class TransformMatrix; // forward declaration for adapter overloads
namespace task {
    

class INSPIRECV_TASK_PUBLIC StreamTask {
public:
    struct Inside;
    struct Config {
        Filter filterType = NEAREST;
        StreamFormat sourceFormat = RGBA;
        StreamFormat destFormat = RGBA;
        float mean[4]   = {0.0f, 0.0f, 0.0f, 0.0f};
        float normal[4] = {1.0f, 1.0f, 1.0f, 1.0f};
        Wrap wrap = CLAMP_TO_EDGE;
        // Preserve the historical source-channel order for identity float
        // conversion. New code should use the normal destination-format order.
        bool preserveIdentityFloatOrder = false;
    };

public:
    // MixedCase API
    static StreamTask* Create(const Config& config);
    ~StreamTask();
    static void Destroy(StreamTask* streamTask);

    inline const Matrix& GetMatrix() const { return transform_; }
    void SetMatrix(const Matrix& matrix);

    // Convert to raw memory (uint8_t* or float* by halide type)
    TaskStatus Convert(const uint8_t* source, int iw, int ih, int stride,
                        void* dest, int ow, int oh, int outputBpp,
                        int outputStride, halide_type_t type);

    // Convert directly into caller-owned tensor memory. NCHW and NC4HW4
    // currently accept float32 output with one or three logical channels; no
    // intermediate image tensor is allocated. The original Convert overload
    // remains NHWC and unchanged.
    TaskStatus Convert(const uint8_t* source, int iw, int ih, int stride,
                       const TensorView& output);

    void SetPadding(uint8_t value) { padding_value_ = value; }

    void SetDraw();
    void Draw(uint8_t* img, int w, int h, int c, const int* regions, int num, const uint8_t* color);

    // Compatibility wrappers (will be removed later)
    static inline StreamTask* create(const Config& config) { return Create(config); }
    static inline void destroy(StreamTask* streamTask) { Destroy(streamTask); }
    inline const Matrix& matrix() const { return GetMatrix(); }
    inline void setMatrix(const Matrix& m) { SetMatrix(m); }
    inline void SetMatrix(const ::inspirecv::TransformMatrix& tm) {
        float affine[6];
        affine[Matrix::kAScaleX] = tm[0]; // sx
        affine[Matrix::kASkewY ] = tm[3]; // ky
        affine[Matrix::kASkewX ] = tm[1]; // kx
        affine[Matrix::kAScaleY] = tm[4]; // sy
        affine[Matrix::kATransX] = tm[2]; // tx
        affine[Matrix::kATransY] = tm[5]; // ty
        Matrix m; m.setAffine(affine);
        SetMatrix(m);
    }
    inline void setMatrix(const ::inspirecv::TransformMatrix& tm) { SetMatrix(tm); }
    inline TaskStatus convert(const uint8_t* s, int iw, int ih, int stride,
                                void* d, int ow, int oh, int outputBpp,
                                int outputStride, halide_type_t type) {
        return Convert(s, iw, ih, stride, d, ow, oh, outputBpp, outputStride, type);
    }
    inline TaskStatus convert(const uint8_t* s, int iw, int ih, int stride,
                              const TensorView& output) {
        return Convert(s, iw, ih, stride, output);
    }
    inline void setPadding(uint8_t v) { SetPadding(v); }
    inline void setDraw() { SetDraw(); }
    inline void draw(uint8_t* img, int w, int h, int c, const int* regions, int num, const uint8_t* color) {
        Draw(img, w, h, c, regions, num, color);
    }

private:
    explicit StreamTask(const Config& config);
    StreamTask(const StreamTask&) = delete;
    StreamTask& operator=(const StreamTask&) = delete;
    Matrix transform_;
    Matrix transform_invert_;
    Inside* inside_;
    uint8_t padding_value_ = 0;
};

// No alias; legacy name removed by project policy
    
} // namespace task
} // namespace inspirecv

#endif // INSPIRECV_STREAMTASK_CORE_STREAM_TASK_H_
