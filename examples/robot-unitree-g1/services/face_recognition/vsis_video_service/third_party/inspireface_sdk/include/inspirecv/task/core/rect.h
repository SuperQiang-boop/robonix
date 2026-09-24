#ifndef INSPIRECV_STREAMTASK_CORE_RECT_H_
#define INSPIRECV_STREAMTASK_CORE_RECT_H_

#include <algorithm>
#include <utility>

#include "st_defs.h"

namespace inspirecv {
namespace task {

struct Point {
    float fX;
    float fY;

    void set(float x, float y) { *this = Point{x, y}; }
};

struct INSPIRECV_TASK_PUBLIC Rect {
    float fLeft;    //!< smaller x-axis bounds
    float fTop;     //!< smaller y-axis bounds
    float fRight;   //!< larger x-axis bounds
    float fBottom;  //!< larger y-axis bounds

    static constexpr Rect MakeEmpty() { return {0.0f, 0.0f, 0.0f, 0.0f}; }
    static constexpr Rect MakeWH(float width, float height) {
        return {0.0f, 0.0f, width, height};
    }
    static Rect MakeIWH(int width, int height) {
        return {0.0f, 0.0f, static_cast<float>(width), static_cast<float>(height)};
    }
    static constexpr Rect MakeLTRB(float left, float top, float right, float bottom) {
        return {left, top, right, bottom};
    }
    static constexpr Rect MakeXYWH(float x, float y, float width, float height) {
        return {x, y, x + width, y + height};
    }

    bool isEmpty() const { return !(fLeft < fRight && fTop < fBottom); }
    bool isSorted() const { return fLeft <= fRight && fTop <= fBottom; }

    float x() const { return fLeft; }
    float y() const { return fTop; }
    float left() const { return fLeft; }
    float top() const { return fTop; }
    float right() const { return fRight; }
    float bottom() const { return fBottom; }
    float width() const { return fRight - fLeft; }
    float height() const { return fBottom - fTop; }
    float centerX() const { return 0.5f * fLeft + 0.5f * fRight; }
    float centerY() const { return 0.5f * fTop + 0.5f * fBottom; }

    void setEmpty() { *this = MakeEmpty(); }
    void set(float left, float top, float right, float bottom) {
        *this = MakeLTRB(left, top, right, bottom);
    }
    void setLTRB(float left, float top, float right, float bottom) {
        set(left, top, right, bottom);
    }
    void iset(int left, int top, int right, int bottom) {
        set(static_cast<float>(left), static_cast<float>(top),
            static_cast<float>(right), static_cast<float>(bottom));
    }
    void isetWH(int width, int height) {
        set(0.0f, 0.0f, static_cast<float>(width), static_cast<float>(height));
    }
    void setXYWH(float x, float y, float width, float height) {
        set(x, y, x + width, y + height);
    }
    void setWH(float width, float height) { set(0.0f, 0.0f, width, height); }

    Rect makeOffset(float horizontal, float vertical) const {
        Rect result = *this;
        result.offset(horizontal, vertical);
        return result;
    }
    Rect makeInset(float horizontal, float vertical) const {
        Rect result = *this;
        result.inset(horizontal, vertical);
        return result;
    }
    Rect makeOutset(float horizontal, float vertical) const {
        return MakeLTRB(fLeft - horizontal, fTop - vertical,
                        fRight + horizontal, fBottom + vertical);
    }

    void offset(float horizontal, float vertical) {
        fLeft += horizontal;
        fTop += vertical;
        fRight += horizontal;
        fBottom += vertical;
    }
    void offsetTo(float x, float y) {
        const float horizontal = x - fLeft;
        const float vertical = y - fTop;
        fRight += horizontal;
        fBottom += vertical;
        fLeft = x;
        fTop = y;
    }
    void inset(float horizontal, float vertical) {
        fLeft += horizontal;
        fTop += vertical;
        fRight -= horizontal;
        fBottom -= vertical;
    }
    void outset(float horizontal, float vertical) { inset(-horizontal, -vertical); }

private:
    static bool HasPositiveOverlap(float leftA, float topA, float rightA, float bottomA,
                                   float leftB, float topB, float rightB, float bottomB) {
        const float overlapLeft = std::max(leftA, leftB);
        const float overlapTop = std::max(topA, topB);
        const float overlapRight = std::min(rightA, rightB);
        const float overlapBottom = std::min(bottomA, bottomB);
        return overlapLeft < overlapRight && overlapTop < overlapBottom;
    }

public:
    bool intersects(float left, float top, float right, float bottom) const {
        return HasPositiveOverlap(fLeft, fTop, fRight, fBottom,
                                  left, top, right, bottom);
    }
    bool intersects(const Rect& other) const {
        return intersects(other.fLeft, other.fTop, other.fRight, other.fBottom);
    }
    static bool Intersects(const Rect& first, const Rect& second) {
        return HasPositiveOverlap(first.fLeft, first.fTop, first.fRight, first.fBottom,
                                  second.fLeft, second.fTop, second.fRight, second.fBottom);
    }

    void joinNonEmptyArg(const Rect& other) {
        INSPIRECV_TASK_ASSERT(!other.isEmpty());
        if (fLeft >= fRight || fTop >= fBottom) {
            *this = other;
            return;
        }
        joinPossiblyEmptyRect(other);
    }
    void joinPossiblyEmptyRect(const Rect& other) {
        fLeft = std::min(fLeft, other.fLeft);
        fTop = std::min(fTop, other.fTop);
        fRight = std::max(fRight, other.fRight);
        fBottom = std::max(fBottom, other.fBottom);
    }

    bool contains(float x, float y) const {
        return x >= fLeft && x < fRight && y >= fTop && y < fBottom;
    }

    void sort() {
        using std::swap;
        if (fLeft > fRight) {
            swap(fLeft, fRight);
        }
        if (fTop > fBottom) {
            swap(fTop, fBottom);
        }
    }
    Rect makeSorted() const {
        const float minimumX = std::min(fLeft, fRight);
        const float minimumY = std::min(fTop, fBottom);
        const float maximumX = std::max(fLeft, fRight);
        const float maximumY = std::max(fTop, fBottom);
        return MakeLTRB(minimumX, minimumY, maximumX, maximumY);
    }

    const float* asScalars() const { return &fLeft; }
};

}  // namespace task
}  // namespace inspirecv

#endif  // INSPIRECV_STREAMTASK_CORE_RECT_H_
