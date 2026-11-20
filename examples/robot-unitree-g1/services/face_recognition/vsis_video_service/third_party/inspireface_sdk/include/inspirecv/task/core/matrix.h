#ifndef INSPIRECV_STREAMTASK_CORE_MATRIX_H_
#define INSPIRECV_STREAMTASK_CORE_MATRIX_H_

#include <string.h>
#include <cstddef>
#include <cstdint>
#include "st_defs.h"
#include "rect.h"

namespace inspirecv { namespace task {

class INSPIRECV_TASK_PUBLIC Matrix {
public:
    Matrix() { setIdentity(); }

    enum TypeMask {
        kIdentity_Mask    = 0,
        kTranslate_Mask   = 0x01,
        kScale_Mask       = 0x02,
        kAffine_Mask      = 0x04,
        kPerspective_Mask = 0x08,
    };

    TypeMask getType() const {
        if (fTypeMask & kUnknown_Mask) {
            fTypeMask = this->computeTypeMask();
        }
        return (TypeMask)(fTypeMask & 0xF);
    }

    bool isIdentity() const { return this->getType() == 0; }
    bool isScaleTranslate() const { return !(this->getType() & ~(kScale_Mask | kTranslate_Mask)); }
    bool isTranslate() const { return !(this->getType() & ~(kTranslate_Mask)); }

    static constexpr int kMScaleX = 0;
    static constexpr int kMSkewX  = 1;
    static constexpr int kMTransX = 2;
    static constexpr int kMSkewY  = 3;
    static constexpr int kMScaleY = 4;
    static constexpr int kMTransY = 5;
    static constexpr int kMPersp0 = 6;
    static constexpr int kMPersp1 = 7;
    static constexpr int kMPersp2 = 8;

    static constexpr int kAScaleX = 0;
    static constexpr int kASkewY  = 1;
    static constexpr int kASkewX  = 2;
    static constexpr int kAScaleY = 3;
    static constexpr int kATransX = 4;
    static constexpr int kATransY = 5;

    float operator[](int index) const {
        INSPIRECV_TASK_ASSERT((unsigned)index < 9);
        return fMat[index];
    }
    float get(int index) const {
        INSPIRECV_TASK_ASSERT((unsigned)index < 9);
        return fMat[index];
    }
    float getScaleX() const { return fMat[kMScaleX]; }
    float getScaleY() const { return fMat[kMScaleY]; }
    float getSkewY()  const { return fMat[kMSkewY]; }
    float getSkewX()  const { return fMat[kMSkewX]; }
    float getTranslateX() const { return fMat[kMTransX]; }
    float getTranslateY() const { return fMat[kMTransY]; }
    float getPerspX() const { return fMat[kMPersp0]; }
    float getPerspY() const { return fMat[kMPersp1]; }

    float& operator[](int index) {
        INSPIRECV_TASK_ASSERT((unsigned)index < 9);
        this->setTypeMask(kUnknown_Mask);
        return fMat[index];
    }
    void set(int index, float value) {
        INSPIRECV_TASK_ASSERT((unsigned)index < 9);
        fMat[index] = value;
        this->setTypeMask(kUnknown_Mask);
    }

    void setAll(float scaleX, float skewX, float transX,
                float skewY, float scaleY, float transY,
                float persp0, float persp1, float persp2) {
        fMat[kMScaleX] = scaleX;
        fMat[kMSkewX]  = skewX;
        fMat[kMTransX] = transX;
        fMat[kMSkewY]  = skewY;
        fMat[kMScaleY] = scaleY;
        fMat[kMTransY] = transY;
        fMat[kMPersp0] = persp0;
        fMat[kMPersp1] = persp1;
        fMat[kMPersp2] = persp2;
        this->setTypeMask(kUnknown_Mask);
    }

    void get9(float buffer[9]) const { memcpy(buffer, fMat, 9 * sizeof(float)); }
    void set9(const float buffer[9]);

    void reset();
    void setIdentity() { this->reset(); }

    void setTranslate(float dx, float dy);
    void setScale(float sx, float sy, float px, float py);
    void setScale(float sx, float sy);
    void setRotate(float degrees, float px, float py);
    void setRotate(float degrees);
    void setSinCos(float sinValue, float cosValue, float px, float py);
    void setSinCos(float sinValue, float cosValue);
    void setSkew(float kx, float ky, float px, float py);
    void setSkew(float kx, float ky);

    void setConcat(const Matrix& a, const Matrix& b);
    void preTranslate(float dx, float dy);
    void preScale(float sx, float sy, float px, float py);
    void preScale(float sx, float sy);
    void preRotate(float degrees, float px, float py);
    void preRotate(float degrees);
    void preSkew(float kx, float ky, float px, float py);
    void preSkew(float kx, float ky);
    void preConcat(const Matrix& other);

    void postTranslate(float dx, float dy);
    void postScale(float sx, float sy, float px, float py);
    void postScale(float sx, float sy);
    bool postIDiv(int divx, int divy);
    void postRotate(float degrees, float px, float py);
    void postRotate(float degrees);
    void postSkew(float kx, float ky, float px, float py);
    void postSkew(float kx, float ky);
    void postConcat(const Matrix& other);

    enum ScaleToFit {
        kFill_ScaleToFit,
        kStart_ScaleToFit,
        kCenter_ScaleToFit,
        kEnd_ScaleToFit,
    };
    bool setRectToRect(const Rect& src, const Rect& dst, ScaleToFit stf);

    bool setPolyToPoly(const Point src[], const Point dst[], int count);

    bool invert(Matrix* inverse) const {
        if (this->isIdentity()) {
            if (inverse) { inverse->reset(); }
            return true;
        }
        return this->invertNonIdentity(inverse);
    }

    static void SetAffineIdentity(float affine[6]);
    bool asAffine(float affine[6]) const;
    void setAffine(const float affine[6]);

    void mapPoints(Point dst[], const Point src[], int count) const {
        INSPIRECV_TASK_ASSERT((dst && src && count > 0) || count == 0);
        INSPIRECV_TASK_ASSERT(src == dst || &dst[count] <= &src[0] || &src[count] <= &dst[0]);
        if (isTriviallyIdentity()) {
            if (dst != src && count > 0) {
                memcpy(dst, src, static_cast<std::size_t>(count) * sizeof(Point));
            }
            return;
        }
        mapPointsGeneral(dst, src, count);
    }
    void mapPoints(Point pts[], int count) const { mapPoints(pts, pts, count); }
    void mapXY(float x, float y, Point* result) const {
        if (isTriviallyIdentity()) {
            result->fX = x;
            result->fY = y;
            return;
        }
        mapXYGeneral(x, y, result);
    }
    Point mapXY(float x, float y) const {
        Point result;
        mapXY(x, y, &result);
        return result;
    }

    void mapRectScaleTranslate(Rect* dst, const Rect& src) const;
    bool mapRect(Rect* dst, const Rect& src) const;
    bool mapRect(Rect* rect) const { return this->mapRect(rect, *rect); }

    bool cheapEqualTo(const Matrix& m) const { return 0 == memcmp(fMat, m.fMat, sizeof(fMat)); }
    friend INSPIRECV_TASK_PUBLIC bool operator==(const Matrix& a, const Matrix& b);
    friend INSPIRECV_TASK_PUBLIC bool operator!=(const Matrix& a, const Matrix& b) { return !(a == b); }

    void setScaleTranslate(float sx, float sy, float tx, float ty) {
        fMat[kMScaleX] = sx; fMat[kMSkewX]  = 0;  fMat[kMTransX] = tx;
        fMat[kMSkewY]  = 0;  fMat[kMScaleY] = sy; fMat[kMTransY] = ty;
        fMat[kMPersp0] = 0;  fMat[kMPersp1] = 0;  fMat[kMPersp2] = 1;
        unsigned mask = 0;
        if (sx != 1 || sy != 1) { mask |= kScale_Mask; }
        if (tx || ty) { mask |= kTranslate_Mask; }
        this->setTypeMask(mask | kRectStaysRect_Mask);
    }

private:
    static constexpr int kRectStaysRect_Mask = 0x10;
    static constexpr int kOnlyPerspectiveValid_Mask = 0x40;
    static constexpr int kUnknown_Mask = 0x80;
    static constexpr int kORableMasks = kTranslate_Mask | kScale_Mask | kAffine_Mask | kPerspective_Mask;
    static constexpr int kAllMasks =
        kTranslate_Mask | kScale_Mask | kAffine_Mask | kPerspective_Mask | kRectStaysRect_Mask;

    float fMat[9];
    mutable uint32_t fTypeMask = kIdentity_Mask | kRectStaysRect_Mask;

    uint8_t computeTypeMask() const;
    uint8_t computePerspectiveTypeMask() const;

    void setTypeMask(int mask) { fTypeMask = (uint8_t)(mask); }
    void orTypeMask(int mask) { fTypeMask = (uint8_t)(fTypeMask | mask); }
    void clearTypeMask(int mask) { fTypeMask = fTypeMask & ~mask; }

    TypeMask getPerspectiveTypeMaskOnly() const {
        if ((fTypeMask & kUnknown_Mask) && !(fTypeMask & kOnlyPerspectiveValid_Mask)) {
            fTypeMask = this->computePerspectiveTypeMask();
        }
        return (TypeMask)(fTypeMask & 0xF);
    }
    bool isTriviallyIdentity() const {
        if (fTypeMask & kUnknown_Mask) { return false; }
        return ((fTypeMask & 0xF) == 0);
    }
    inline void updateTranslateMask() {
        if ((fMat[kMTransX] != 0) | (fMat[kMTransY] != 0)) { fTypeMask |= kTranslate_Mask; }
        else { fTypeMask &= ~kTranslate_Mask; }
    }

    bool invertNonIdentity(Matrix* inverse) const;
    void mapPointsGeneral(Point destination[], const Point source[], int count) const;
    void mapXYGeneral(float x, float y, Point* result) const;

};

}} // namespace inspirecv::task

#endif // INSPIRECV_STREAMTASK_CORE_MATRIX_H_
