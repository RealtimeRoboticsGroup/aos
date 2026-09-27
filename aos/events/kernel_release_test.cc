#include "aos/events/kernel_release.h"

#include "gtest/gtest.h"

namespace aos::internal::testing {

// Exactly 5.9 and 5.10 have the bug, whatever follows the version number.
TEST(KernelReleaseTest, SpuriousNowaitZeroOnlyOnFiveNineAndFiveTen) {
  EXPECT_TRUE(KernelHasSpuriousNowaitZero("5.9.0"));
  EXPECT_TRUE(KernelHasSpuriousNowaitZero("5.9.16-arch1-1"));
  EXPECT_TRUE(KernelHasSpuriousNowaitZero("5.10"));
  EXPECT_TRUE(KernelHasSpuriousNowaitZero("5.10.120-tegra"));

  EXPECT_FALSE(KernelHasSpuriousNowaitZero("5.8.18"));
  EXPECT_FALSE(KernelHasSpuriousNowaitZero("5.1.0"));
  EXPECT_FALSE(KernelHasSpuriousNowaitZero("5.11.0"));
  EXPECT_FALSE(KernelHasSpuriousNowaitZero("5.15.148-l4t-r36.4"));
  EXPECT_FALSE(KernelHasSpuriousNowaitZero("5.90.0"));
  EXPECT_FALSE(KernelHasSpuriousNowaitZero("6.10.0"));
  EXPECT_FALSE(KernelHasSpuriousNowaitZero("6.12.77-rt"));
  EXPECT_FALSE(KernelHasSpuriousNowaitZero("4.9.0"));
  EXPECT_FALSE(KernelHasSpuriousNowaitZero("15.10.0"));
}

// Anything that does not parse as major.minor is treated as unaffected.
TEST(KernelReleaseTest, UnparseableReleaseIsNotAffected) {
  EXPECT_FALSE(KernelHasSpuriousNowaitZero(""));
  EXPECT_FALSE(KernelHasSpuriousNowaitZero("5"));
  EXPECT_FALSE(KernelHasSpuriousNowaitZero("5."));
  EXPECT_FALSE(KernelHasSpuriousNowaitZero("5-10"));
  EXPECT_FALSE(KernelHasSpuriousNowaitZero("v5.10"));
}

}  // namespace aos::internal::testing
