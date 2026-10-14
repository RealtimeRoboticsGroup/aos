#ifndef FRC_VISION_GRAY_IMAGE_DIFF_H_
#define FRC_VISION_GRAY_IMAGE_DIFF_H_

#include <cstddef>
#include <cstdint>

#include "absl/types/span.h"

namespace frc::vision {

// Mean absolute difference, in grey levels, under which two decodes of 8-bit
// greyscale images count as the same image.  The NVJPG and turbojpeg decodes
// of one JPEG agree to well under 1; two different camera frames differ by
// 20+.
inline constexpr double kSameImage = 2.0;

// Mean absolute difference, in grey levels, between two 8-bit greyscale
// images of the same size.  With stride > 1 only every stride-th pixel is
// compared, which is still plenty to tell camera frames apart.
double MeanAbsDiff(absl::Span<const uint8_t> a, absl::Span<const uint8_t> b,
                   size_t stride = 1);

}  // namespace frc::vision

#endif  // FRC_VISION_GRAY_IMAGE_DIFF_H_
