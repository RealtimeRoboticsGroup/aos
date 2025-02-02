#ifndef FRC_ORIN_THRESHOLD_H_
#define FRC_ORIN_THRESHOLD_H_

#include <stdint.h>

#include "frc/orin/cuda.h"
#include "frc/vision/vision_generated.h"

namespace frc::apriltag {

// Returns the number of bytes per pixel.
constexpr __device__ uint32_t
BytesPerPixel(const vision::ImageFormat image_format) {
  switch (image_format) {
    case vision::ImageFormat::MONO8:
      return 1;
    case vision::ImageFormat::MONO16:
      return 2;
    case vision::ImageFormat::YUYV422:
      return 2;
    case vision::ImageFormat::BGR8:
      return 3;
    case vision::ImageFormat::BGRA8:
      return 4;
    case vision::ImageFormat::MJPEG:
      asm("trap;");
  }
  asm("trap;");
  return 0;
}

class Threshold {
 public:
  // Create a full-size grayscale image from a color image on the provided
  // stream.
  virtual void CudaToGreyscale(const uint8_t *color_image, uint8_t *gray_image,
                               uint32_t width, uint32_t height,
                               CudaStream *stream) = 0;

  // Converts to grayscale, decimates, and thresholds an image on the provided
  // stream.
  virtual void CudaThresholdAndDecimate(
      const uint8_t *color_image, uint8_t *decimated_image,
      uint8_t *unfiltered_minmax_image, uint8_t *minmax_image,
      uint8_t *thresholded_image, uint32_t width, uint32_t height,
      uint32_t min_white_black_diff, CudaStream *stream) = 0;

  virtual ~Threshold() = default;
};

// Constructs an optimized threshold object for the provided image format.
std::unique_ptr<Threshold> MakeThreshold(vision::ImageFormat image_format);

}  // namespace frc::apriltag

#endif  // FRC_ORIN_THRESHOLD_H_
