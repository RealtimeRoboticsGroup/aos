#ifndef FRC_ORIN_THRESHOLD_H_
#define FRC_ORIN_THRESHOLD_H_

#include <stdint.h>

#include "frc/orin/cuda.h"
#include "frc/vision/vision_generated.h"

namespace frc::apriltag {

class BaseThreshold {
 public:
  // Create a full-size grayscale image from a color image on the provided
  // stream.
  virtual void CudaToGreyscale(const uint8_t *color_image, uint8_t *gray_image,
                               uint32_t width, uint32_t height,
                               CudaStream *stream) = 0;

  // Converts to grayscale, decimates, and thresholds an image on the provided
  // stream.
  virtual void CudaToGreyscaleAndDecimateHalide(
      const uint8_t *color_image, uint8_t *decimated_image,
      uint8_t *unfiltered_minmax_image, uint8_t *minmax_image,
      uint8_t *thresholded_image, uint32_t width, uint32_t height,
      uint32_t min_white_black_diff, CudaStream *stream) = 0;

  virtual ~BaseThreshold() = default;
};

template <vision::ImageFormat IMAGE_FORMAT>
class Threshold : public BaseThreshold {
 public:
  // Create a full-size grayscale image from a color image on the provided
  // stream.
  void CudaToGreyscale(const uint8_t *color_image, uint8_t *gray_image,
                       uint32_t width, uint32_t height,
                       CudaStream *stream) override;

  // Converts to grayscale, decimates, and thresholds an image on the provided
  // stream.
  void CudaToGreyscaleAndDecimateHalide(
      const uint8_t *color_image, uint8_t *decimated_image,
      uint8_t *unfiltered_minmax_image, uint8_t *minmax_image,
      uint8_t *thresholded_image, uint32_t width, uint32_t height,
      uint32_t min_white_black_diff, CudaStream *stream) override;

  virtual ~Threshold() = default;
};

}  // namespace frc::apriltag

#endif  // FRC_ORIN_THRESHOLD_H_
