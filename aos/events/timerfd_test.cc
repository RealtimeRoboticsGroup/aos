#ifndef _WIN32
#include <sys/wait.h>
#include <unistd.h>
#endif
#include <thread>

#include "absl/flags/declare.h"
#include "absl/flags/flag.h"
#include "gtest/gtest.h"

#include "aos/events/epoll.h"

ABSL_DECLARE_FLAG(bool, use_io_uring);

namespace aos {
namespace internal {
namespace testing {

class TimerFdTest : public ::testing::TestWithParam<bool> {
 protected:
  void SetUp() override { absl::SetFlag(&FLAGS_use_io_uring, GetParam()); }
};

#ifndef _WIN32
TEST_P(TimerFdTest, ForkSafety) {
  // Test that TimerFd works in a forked child.
  // This mimics how ASSERT_EXIT might use it (forking and then running code).
  // On macOS, kqueue/kevent handles are not inherited across fork, so any
  // existing event-loop state must be rebuilt in the child.

  // We use ASSERT_EXIT to simulate the crash environment.
  ASSERT_EXIT(
      {
        // Create TimerFd in the child.
        TimerFd timer;
        timer.SetTime(monotonic_clock::now() + std::chrono::milliseconds(10),
                      std::chrono::milliseconds(0));

        // Wait via EPoll to exercise the timer/event-loop path used in
        // forked death-test children. Regressions in fork handling can show up
        // here as loop.Run() failures (for example, invalid kqueue state).
        EPoll loop;
        loop.OnReadable(timer.fd(), [&]() {
          timer.Read();
          loop.Quit();
        });

        loop.Run();

        // If we get here, we survived.
        exit(0);
      },
      ::testing::ExitedWithCode(0), "");
}

TEST_P(TimerFdTest, EpollOutlivesFork) {
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

  ASSERT_EXIT(
      {
        // In child process.
        // loop.Run() should detect fork, rebuild kqueue, and fire the event.
        loop.Run();
        if (read_called) {
          exit(0);
        } else {
          exit(1);
        }
      },
      ::testing::ExitedWithCode(0), "");

  loop.DeleteFd(pipefd[0]);
  close(pipefd[0]);
  close(pipefd[1]);
}
#endif

TEST_P(TimerFdTest, MultipleExpirations) {
  TimerFd timer;
  timer.SetTime(monotonic_clock::now() + std::chrono::milliseconds(100),
                std::chrono::milliseconds(100));

  // Sleep for 3.5x interval.
  std::this_thread::sleep_for(std::chrono::milliseconds(350));

  uint64_t expirations = timer.Read();
  // Should be at least 3.
  EXPECT_GE(expirations, 3);
}

TEST_P(TimerFdTest, EarlyWakeup) {
  // Set the timer in the past so expiration is immediate and deterministic.
  // We use EPoll to wait for readiness and then verify Read() reports >= 1.
  EPoll loop;
  TimerFd timer;
  int count = 0;
  loop.OnReadable(timer.fd(), [&] {
    count += timer.Read();
    loop.Quit();
  });

  timer.SetTime(monotonic_clock::now() - std::chrono::milliseconds(1),
                std::chrono::nanoseconds(0));
  loop.Run();
  EXPECT_GE(count, 1);
  loop.DeleteFd(timer.fd());
}

INSTANTIATE_TEST_SUITE_P(TimerFdTestBackends, TimerFdTest,
                         ::testing::Values(true, false));

}  // namespace testing
}  // namespace internal
}  // namespace aos
