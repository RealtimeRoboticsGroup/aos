#include "frc/vision/gray_image_diff.h"

#include <cstdlib>

#include "absl/log/absl_check.h"

namespace frc::vision {

double MeanAbsDiff(absl::Span<const uint8_t> a, absl::Span<const uint8_t> b,
                   size_t stride) {
  ABSL_CHECK_EQ(a.size(), b.size());
  ABSL_CHECK(!a.empty());
  ABSL_CHECK_GT(stride, 0u);
  uint64_t sum = 0;
  if (stride == 1) {
    // Kept apart from the strided loop below so the compiler vectorizes it.
    // nvjpeg_decoder_verify compares every pixel of every frame and has
    // about 8 ms per frame at 120 Hz; the scalar loop costs it a few ms.
    for (size_t i = 0; i < a.size(); ++i) {
      sum += std::abs(static_cast<int>(a[i]) - static_cast<int>(b[i]));
    }
    return static_cast<double>(sum) / a.size();
  }
  size_t count = 0;
  for (size_t i = 0; i < a.size(); i += stride) {
    sum += std::abs(static_cast<int>(a[i]) - static_cast<int>(b[i]));
    ++count;
  }
  return static_cast<double>(sum) / count;
}

}  // namespace frc::vision
