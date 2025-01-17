#include "aos/ipc_lib/signalfd.h"

#include <memory>
#include <thread>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "gtest/gtest.h"

#include "aos/testing/test_logging.h"

namespace aos::ipc_lib::testing {

// Tests in this file use separate threads to isolate all manipulation of signal
// masks between test cases.

// Verify that SignalFd will leave signals unblocked if we ask it to.
TEST(SignalFdTest, LeaveSignalBlocked) {
  ::aos::testing::EnableTestLogging();
  std::thread thread([]() {
    {
      sigset_t test_mask;
      CHECK_EQ(0, sigemptyset(&test_mask));
      CHECK_EQ(0, sigaddset(&test_mask, SIGUSR1));
      PCHECK(sigprocmask(SIG_BLOCK, &test_mask, nullptr) == 0);
    }
    SignalFd({SIGUSR1});
    {
      sigset_t blocked_now;
      PCHECK(sigprocmask(SIG_BLOCK, nullptr, &blocked_now) == 0);
      ASSERT_TRUE(sigismember(&blocked_now, SIGUSR1));
    }
  });
  thread.join();
}

// Verify that SignalFd actually blocks the requested signals, and unblocks them
// afterwards.
TEST(SignalFdTest, BlockSignal) {
  ::aos::testing::EnableTestLogging();
  std::thread thread([]() {
    {
      sigset_t blocked_now;
      PCHECK(sigprocmask(SIG_BLOCK, nullptr, &blocked_now) == 0);
      ASSERT_FALSE(sigismember(&blocked_now, SIGUSR1));
    }
    {
      SignalFd signalfd({SIGUSR1});
      sigset_t blocked_now;
      PCHECK(sigprocmask(SIG_BLOCK, nullptr, &blocked_now) == 0);
      ASSERT_TRUE(sigismember(&blocked_now, SIGUSR1));
    }
    {
      sigset_t blocked_now;
      PCHECK(sigprocmask(SIG_BLOCK, nullptr, &blocked_now) == 0);
      ASSERT_FALSE(sigismember(&blocked_now, SIGUSR1));
    }
  });
  thread.join();
}

// Verify that SignalFd responds correctly when some other code unblocks one of
// its signals.
TEST(SignalFdDeathTest, ExternalUnblockSignal) {
  ::aos::testing::EnableTestLogging();
  auto t = []() {
    SignalFd signalfd({SIGUSR1});
    sigset_t test_mask;
    CHECK_EQ(0, sigemptyset(&test_mask));
    CHECK_EQ(0, sigaddset(&test_mask, SIGUSR1));
    PCHECK(sigprocmask(SIG_UNBLOCK, &test_mask, nullptr) == 0);
  };
  EXPECT_DEATH(
      { t(); }, "Some other code unblocked one or more of our signals");
}

}  // namespace aos::ipc_lib::testing
