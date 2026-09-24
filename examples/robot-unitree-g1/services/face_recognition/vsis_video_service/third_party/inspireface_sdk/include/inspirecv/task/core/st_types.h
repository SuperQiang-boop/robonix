#ifndef INSPIRECV_STREAMTASK_CORE_ST_TYPES_H_
#define INSPIRECV_STREAMTASK_CORE_ST_TYPES_H_

#include <stdint.h>

extern "C" {
typedef enum halide_type_code_t
{
    halide_type_int = 0,
    halide_type_uint = 1,
    halide_type_float = 2,
    halide_type_handle = 3,
    halide_type_bfloat = 4
} halide_type_code_t;

typedef struct halide_type_t {
    uint8_t code;
    uint8_t bits;
    uint16_t lanes;
} halide_type_t;
}

static inline int halide_type_bytes(const halide_type_t& t) { return (t.bits + 7) / 8; }

template<typename T>
inline halide_type_t halide_type_of();

template<>
inline halide_type_t halide_type_of<float>() {
    halide_type_t t = { (uint8_t)halide_type_float, 32, 1 };
    return t;
}

template<>
inline halide_type_t halide_type_of<uint8_t>() {
    halide_type_t t = { (uint8_t)halide_type_uint, 8, 1 };
    return t;
}

#endif // INSPIRECV_STREAMTASK_CORE_ST_TYPES_H_



