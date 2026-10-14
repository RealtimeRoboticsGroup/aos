#include "frc/vision/gray_image_diff.h"

#include <cstdint>
#include <vector>

#include "gtest/gtest.h"

namespace frc::vision::testing {

TEST(GrayImageDiffTest, IdenticalImagesDiffByZero) {
  const std::vector<uint8_t> a = {0, 17, 128, 255};
  EXPECT_EQ(MeanAbsDiff(a, a), 0.0);
}

TEST(GrayImageDiffTest, AveragesAbsoluteDifferences) {
  const std::vector<uint8_t> a = {0, 10, 255, 100};
  const std::vector<uint8_t> b = {4, 0, 251, 100};
  // |0-4| + |10-0| + |255-251| + |100-100| = 18, over 4 pixels.
  EXPECT_EQ(MeanAbsDiff(a, b), 4.5);
}

TEST(GrayImageDiffTest, StrideOnlyLooksAtEveryNthPixel) {
  const std::vector<uint8_t> a = {0, 0, 0, 0, 0, 0, 0};
  const std::vector<uint8_t> b = {6, 200, 200, 3, 200, 200, 0};
  // Pixels 0, 3 and 6: (6 + 3 + 0) / 3.
  EXPECT_EQ(MeanAbsDiff(a, b, 3), 3.0);
}

TEST(GrayImageDiffDeathTest, SizesMustMatch) {
  const std::vector<uint8_t> a(4);
  const std::vector<uint8_t> b(5);
  EXPECT_DEATH(MeanAbsDiff(a, b), "a.size\\(\\) == b.size\\(\\)");
}

}  // namespace frc::vision::testing
