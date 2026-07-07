#include "aos/libc/aos_strerror.h"

#include <string.h>

#include <cerrno>
#include <string>

#include "gtest/gtest.h"

namespace aos::libc::testing {

// Tries a couple of easy ones.
TEST(StrerrorTest, Basic) {
#ifdef _WIN32
  EXPECT_STREQ("Arg list too long", aos_strerror(E2BIG));
#else
  EXPECT_STREQ("Argument list too long", aos_strerror(E2BIG));
#endif
  EXPECT_STREQ("Bad file descriptor", aos_strerror(EBADF));
#ifdef __APPLE__
  EXPECT_STREQ("Unknown error: 4021", aos_strerror(4021));
#else
  EXPECT_STREQ("Unknown error 4021", aos_strerror(4021));
#endif
}

// Runs through all errno values and makes sure it gives the same result as
// strerror(3).
TEST(StrerrorTest, All) {
  for (int i = 0; i < 4095; ++i) {
    SCOPED_TRACE("iteration " + ::std::to_string(i));
#ifdef _WIN32
    std::string expected = strerror(i);
    std::string actual = aos_strerror(i);
    if (expected == "Unknown error") {
      EXPECT_EQ("Unknown error " + std::to_string(i), actual);
    } else {
      EXPECT_EQ(expected, actual);
    }
#else
    EXPECT_STREQ(strerror(i), aos_strerror(i));
#endif
  }
}

}  // namespace aos::libc::testing
