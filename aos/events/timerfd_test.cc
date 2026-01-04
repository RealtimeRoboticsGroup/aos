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

TEST_F(TimerFdTest, EpollOutlivesFork) {
  // Test that EPoll works in a forked child even if created in parent.
  // This verifies that kqueue is correctly re-created after fork.

  int pipefd[2];
  ASSERT_EQ(pipe(pipefd), 0);
  
  EPoll loop;
  bool read_called = false;
  loop.OnReadable(pipefd[0], [&]() {
    read_called = true;
    char buf[1];
    EXPECT_EQ(read(pipefd[0], buf, 1), 1);
    loop.Quit();
  });

  // Write something to the pipe so it's readable immediately.
  char c = 'a';
  ASSERT_EQ(write(pipefd[1], &c, 1), 1);

  ASSERT_EXIT({
    // In child process.
    // loop.Run() should detect fork, rebuild kqueue, and fire the event.
    loop.Run();
    if (read_called) exit(0);
    else exit(1);
  }, ::testing::ExitedWithCode(0), "");
  
  loop.DeleteFd(pipefd[0]);
  close(pipefd[0]);
  close(pipefd[1]);
}

} // namespace testing
} // namespace internal
} // namespace aos
