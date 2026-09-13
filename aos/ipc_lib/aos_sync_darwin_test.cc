// macOS-specific tests for aos_sync.  aos_sync_test.cc holds the rest.

#include "aos/ipc_lib/aos_sync.h"

#include <chrono>
#include <ctime>

#include "gtest/gtest.h"

namespace aos::testing {
namespace {

namespace chrono = std::chrono;

constexpr chrono::milliseconds kTimeout{100};
// futex_wait_timeout() can return 0 spuriously, or 1 on a signal.
constexpr int kMaxAttempts = 3;

// The timeout reaches os_sync_wait_on_address_with_timeout() as nanoseconds.
// Converting it to mach ticks would divide it by 41.67 on Apple Silicon and
// return early; converting the other way overshoots by the same factor.  Both
// miss the bounds below by more than 4x.
TEST(AosSyncDarwinTest, WaitTimesOutAfterRoughlyTheTimeout) {
  aos_futex futex{};  // Zeroed is unset, so only the timeout ends the wait.

  struct timespec timeout;
  timeout.tv_sec = 0;
  timeout.tv_nsec = chrono::nanoseconds(kTimeout).count();

  for (int attempt = 0; attempt < kMaxAttempts; ++attempt) {
    const chrono::steady_clock::time_point start = chrono::steady_clock::now();
    const int ret = futex_wait_timeout(&futex, &timeout);
    const chrono::steady_clock::duration elapsed =
        chrono::steady_clock::now() - start;

    if (ret == 0 || ret == 1) continue;
    ASSERT_EQ(2, ret) << ": Failed with errno " << errno;

    EXPECT_GE(elapsed, kTimeout * 9 / 10) << ": Returned early.";
    EXPECT_LT(elapsed, kTimeout * 10) << ": Waited far too long.";
    return;
  }
  FAIL() << ": Never reached the timeout in " << kMaxAttempts << " attempts.";
}

}  // namespace
}  // namespace aos::testing
