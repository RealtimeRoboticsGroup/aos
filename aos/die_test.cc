#include "aos/die.h"

#include "gtest/gtest.h"

namespace aos::testing {

TEST(DieDeathTest, Works) {
#ifdef _WIN32
  EXPECT_EXIT(Die("str=%s num=%d\n", "hi", 5), ::testing::ExitedWithCode(3),
              ".*str=hi num=5\n");
#else
  EXPECT_EXIT(Die("str=%s num=%d\n", "hi", 5),
              ::testing::KilledBySignal(SIGABRT), ".*str=hi num=5\n");
#endif
}

}  // namespace aos::testing
