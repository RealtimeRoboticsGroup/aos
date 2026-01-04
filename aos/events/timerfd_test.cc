#include "aos/events/epoll.h"
#include "gtest/gtest.h"
#include <unistd.h>
#include <sys/wait.h>

namespace aos {
namespace internal {
namespace testing {

class TimerFdTest : public ::testing::Test {
 protected:
  void SetUp() override {
  }
};

TEST_F(TimerFdTest, ForkSafety) {
  // Test that TimerFd works in a forked child.
  // This mimics how ASSERT_EXIT might use it (forking and then running code).
  
  // We use ASSERT_EXIT to simulate the crash environment.
  ASSERT_EXIT({
    // Create TimerFd in the child.
    TimerFd timer;
    timer.SetTime(monotonic_clock::now() + std::chrono::milliseconds(10), std::chrono::milliseconds(0));
    
    // We ideally want to wait for it.
    // If we use EPoll, it might crash.
    EPoll loop;
    loop.OnReadable(timer.fd(), [&]() {
      timer.Read();
      loop.Quit();
    });
    
    loop.Run();
    
    // If we get here, we survived.
    exit(0);
  }, ::testing::ExitedWithCode(0), "");
}

} // namespace testing
} // namespace internal
} // namespace aos
