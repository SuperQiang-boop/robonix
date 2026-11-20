#ifndef INSPIRECV_IMAGE_H
#define INSPIRECV_IMAGE_H

#include <memory>
#include <string>
#include <type_traits>
#include "define.h"
#include "rect.h"
#include "point.h"
#include "size.h"
#include "transform_matrix.h"

namespace inspirecv {

template<typename Pixel = uint8_t>
class INSPIRECV_API ImageT {
public:
    // Lightweight image class (templated by Pixel) with IO, transforms and basic ops.
    ImageT(const ImageT&) = delete;
    ImageT& operator=(const ImageT&) = delete;

    // Default empty image (W=H=C=0).
    ImageT();
    ~ImageT();

    ImageT(ImageT&&) noexcept;
    ImageT& operator=(ImageT&&) noexcept;

    // Construct with size and optional external data (interleaved W×H×C).
    ImageT(int width, int height, int channels, const Pixel* data = nullptr,
          bool copy_data = true);

    // Reset size and optional data.
    void Reset(int width, int height, int channels, const Pixel* data = nullptr);

    // Deep copy image.
    ImageT Clone() const;

    // Get width in pixels.
    int Width() const;
    // Get height in pixels.
    int Height() const;
    // Get number of channels.
    int Channels() const;
    // Check if image is empty.
    bool Empty() const;

    // Read-only data pointer (interleaved W×H×C).
    const Pixel* Data() const;
    // Backend handle pointer (advanced use).
    void* GetInternalImage() const;

    // Read from file (channels=1 or 3); float path converts u8->float in [0..255].
    bool Read(const std::string& filename, int channels = 3);
    // Write to file; float is saturated/rounded to 8U for output.
    bool Write(const std::string& filename) const;
    // Show image; float is visualized via temporary 8U.
    void Show(const std::string& window_name = std::string("win"), int delay = 0) const;

    // Fill all pixels with a scalar.
    void Fill(double value);
    // Multiply by scalar and return new image.
    ImageT Mul(double scale) const;
    // Add scalar and return new image.
    ImageT Add(double value) const;

    // Resize (bilinear or nearest).
    ImageT Resize(int width, int height, bool use_linear = true) const;
    // Crop by rectangle (origin at top-left).
    ImageT Crop(const Rect<int>& rect) const;
    // Apply affine transform (2x3 matrix).
    ImageT WarpAffine(const TransformMatrix& matrix, int width, int height) const;

    // Rotate 90 degrees clockwise.
    ImageT Rotate90() const;
    // Rotate 180 degrees.
    ImageT Rotate180() const;
    // Rotate 270 degrees clockwise.
    ImageT Rotate270() const;
    // Swap R and B channels (3/4 channels).
    ImageT SwapRB() const;
    // Flip horizontally.
    ImageT FlipHorizontal() const;
    // Flip vertically.
    ImageT FlipVertical() const;

    // Pad borders; color uses RGB order for 3/4 channels.
    ImageT Pad(int top, int bottom, int left, int right, const std::vector<double>& color) const;

    // Gaussian blur (odd ksize is preferred; sigma<=0 is auto).
    ImageT GaussianBlur(int kernel_size, double sigma) const;
    // Erode (single-channel only).
    ImageT Erode(int kernel_size, int iterations = 1) const;
    // Dilate (single-channel only).
    ImageT Dilate(int kernel_size, int iterations = 1) const;
    // Threshold (binary type supported).
    ImageT Threshold(double thresh, double maxval, int type) const;
    // Convert to grayscale.
    ImageT ToGray() const;
    // Per-pixel absolute difference (same size/channels).
    ImageT AbsDiff(const ImageT& other) const;
    // Mean across channels to single-channel.
    ImageT MeanChannels() const;
    // Alpha blend with 8U mask: out = m*this + (1-m)*other, m in [0..1].
    ImageT Blend(const ImageT& other, const ImageT<uint8_t>& mask) const;

    // Draw line; color uses RGB order for 3/4 channels.
    void DrawLine(const Point<int>& p1, const Point<int>& p2, const std::vector<double>& color,
                  int thickness = 1);
    // Draw rectangle.
    void DrawRect(const Rect<int>& rect, const std::vector<double>& color, int thickness = 1);
    // Draw circle (filled point with radius).
    void DrawCircle(const Point<int>& center, int radius, const std::vector<double>& color,
                    int thickness = 1);
    // Fill rectangle with color.
    void Fill(const Rect<int>& rect, const std::vector<double>& color);

    // Create image (optional external data; copy_data follows ctor semantics).
    static ImageT Create(int width, int height, int channels, const Pixel* data = nullptr,
                        bool copy_data = true);
    // Create empty image.
    static ImageT Create();
    // Create from file (same semantics as Read).
    static ImageT Create(const std::string& filename, int channels = 3);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;

    ImageT(std::unique_ptr<Impl> impl);

    friend class Impl;
    template<typename P>
    friend std::ostream& operator<<(std::ostream& os, const ImageT<P>& image);
    template<typename> friend class ImageT;  // allow cross-Pixel access to impl_
};

template<typename Pixel>
INSPIRECV_API std::ostream& operator<<(std::ostream& os, const ImageT<Pixel>& image);

// Default image type alias equal to ImageT<uint8_t>.
using Image = ImageT<uint8_t>;

}  // namespace inspirecv

#endif  // INSPIRECV_IMAGE_H
