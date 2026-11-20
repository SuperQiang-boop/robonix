#ifndef INSPIRECV_STREAMTASK_MATRIX_H_
#define INSPIRECV_STREAMTASK_MATRIX_H_

#include <inspirecv/task/core/matrix.h>
#include <inspirecv/core/transform_matrix.h>

namespace inspirecv { namespace task {
    using Matrix = inspirecv::task::Matrix;

    // Convert inspirecv::TransformMatrix (2x3 affine) to task::Matrix (3x3)
    inline Matrix FromTransformMatrix(const ::inspirecv::TransformMatrix& tm) {
        float affine[6];
        affine[Matrix::kAScaleX] = tm[0]; // sx
        affine[Matrix::kASkewY ] = tm[3]; // ky
        affine[Matrix::kASkewX ] = tm[1]; // kx
        affine[Matrix::kAScaleY] = tm[4]; // sy
        affine[Matrix::kATransX] = tm[2]; // tx
        affine[Matrix::kATransY] = tm[5]; // ty
        Matrix m; m.setAffine(affine);
        return m;
    }
}} // namespace inspirecv::task

#endif // INSPIRECV_STREAMTASK_MATRIX_H_


