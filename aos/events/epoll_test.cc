#include "aos/events/epoll.h"

#include <fcntl.h>
#include <unistd.h>

#include "absl/flags/declare.h"
#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "gtest/gtest.h"

#include "aos/events/pipe.h"

ABSL_DECLARE_FLAG(bool, use_io_uring);

namespace aos::testing {

class EPollTest : public ::testing::TestWithParam<bool> {
 public:
  void SetUp() override {
    absl::SetFlag(&FLAGS_use_io_uring, GetParam());
    epoll_ptr_ = std::make_unique<EPoll>();
  }

  void RunFor(std::chrono::nanoseconds duration) {
    internal::TimerFd timerfd;
    bool did_quit = false;
    epoll_ptr_->OnReadable(timerfd.fd(), [this, &timerfd, &did_quit]() {
      CHECK(!did_quit);
      epoll_ptr_->Quit();
      did_quit = true;
      timerfd.Read();
    });
    timerfd.SetTime(monotonic_clock::now() + duration,
                    monotonic_clock::duration::zero());
    epoll_ptr_->Run();
    CHECK(did_quit);
    epoll_ptr_->DeleteFd(timerfd.fd());
  }

  // Tests should avoid relying on ordering for events closer in time than this,
  // or waiting for longer than this to ensure events happen in order.
  static constexpr std::chrono::nanoseconds tick_duration() {
    return std::chrono::milliseconds(50);
  }

  std::unique_ptr<EPoll> epoll_ptr_;
};

#define epoll_ (*epoll_ptr_)

// Test that the basics of OnReadable work.
TEST_P(EPollTest, BasicReadable) {
  Pipe pipe;
  bool got_data = false;
  epoll_.OnReadable(pipe.read_fd(), [&]() {
    ASSERT_FALSE(got_data);
    ASSERT_EQ("some", pipe.Read(4));
    got_data = true;
  });
  RunFor(tick_duration());
  EXPECT_FALSE(got_data);

  pipe.Write("some");
  RunFor(tick_duration());
  EXPECT_TRUE(got_data);

  epoll_.DeleteFd(pipe.read_fd());
}

// Test that the basics of OnWritable work.
TEST_P(EPollTest, BasicWritable) {
  Pipe pipe;
  int number_writes = 0;
  epoll_.OnWritable(pipe.write_fd(), [&]() {
    pipe.Write(" ");
    ++number_writes;
  });

  // First, fill up the pipe's write buffer.
  RunFor(tick_duration());
  EXPECT_GT(number_writes, 0);

  // Now, if we try again, we shouldn't do anything.
  const int bytes_in_pipe = number_writes;
  number_writes = 0;
  RunFor(tick_duration());
  EXPECT_EQ(number_writes, 0);

  // Empty the pipe, then fill it up again.
  for (int i = 0; i < bytes_in_pipe; ++i) {
    ASSERT_EQ(" ", pipe.Read(1));
  }
  number_writes = 0;
  RunFor(tick_duration());
  EXPECT_EQ(number_writes, bytes_in_pipe);

  epoll_.DeleteFd(pipe.write_fd());
}

// Test that the basics of OnError work.
TEST_P(EPollTest, BasicError) {
  // In order to trigger an error, close the read file descriptor (per the
  // epoll_ctl manpage, this should trigger an error).
  Pipe pipe;
  int number_errors = 0;
  epoll_.OnError(pipe.write_fd(), [&]() { ++number_errors; });

  // Sanity check that we *don't* get any errors before anything interesting has
  // happened.
  RunFor(tick_duration());
  EXPECT_EQ(number_errors, 0);

  pipe.close_read_fd();

  // For some reason, OnError doesn't seem to play nice with the timer setup we
  // have in this test, so just poll for a single event.
  while (number_errors == 0) {
    epoll_.Poll(true);
  }

  EXPECT_EQ(number_errors, 1);

  epoll_.DeleteFd(pipe.write_fd());
}

TEST_P(EPollTest, InvalidFd) {
  EPoll epoll;
  Pipe pipe;
  epoll.OnReadable(pipe.read_fd(), []() {});
  EXPECT_DEATH(epoll.OnReadable(pipe.read_fd(), []() {}),
               "Duplicate in functions");
  epoll.OnWritable(pipe.read_fd(), []() {});
  EXPECT_DEATH(epoll.OnWritable(pipe.read_fd(), []() {}),
               "Duplicate out functions");

  epoll.DeleteFd(pipe.read_fd());
  EXPECT_DEATH(epoll.DeleteFd(pipe.read_fd()), "fd [0-9]+ not found");
  EXPECT_DEATH(epoll.DeleteFd(pipe.write_fd()), "fd [0-9]+ not found");
}

// Tests that enabling/disabling a writable FD works.
TEST_P(EPollTest, WritableEnableDisable) {
  Pipe pipe;
  int number_writes = 0;
  epoll_.OnWritable(pipe.write_fd(), [&]() {
    pipe.Write(" ");
    ++number_writes;
  });

  // First, fill up the pipe's write buffer.
  RunFor(tick_duration());
  EXPECT_GT(number_writes, 0);

  // Empty the pipe.
  const int bytes_in_pipe = number_writes;
  for (int i = 0; i < bytes_in_pipe; ++i) {
    ASSERT_EQ(" ", pipe.Read(1));
  }

  // If we disable writable checking, then nothing should happen.
  epoll_.DisableWritable(pipe.write_fd());
  number_writes = 0;
  RunFor(tick_duration());
  EXPECT_EQ(number_writes, 0);

  // Disabling it again should be a NOP.
  epoll_.DisableWritable(pipe.write_fd());

  // And then when we re-enable, it should fill the pipe up again.
  epoll_.EnableWritable(pipe.write_fd());
  number_writes = 0;
  RunFor(tick_duration());
  EXPECT_EQ(number_writes, bytes_in_pipe);

  epoll_.DeleteFd(pipe.write_fd());
}

TEST_P(EPollTest, QuitInBeforeWait) {
  epoll_.BeforeWait([this]() { epoll_.Quit(); });
  epoll_.Run();
}

TEST_P(EPollTest, RemoveWithoutEvents) {
  Pipe pipe;
  epoll_.OnEvents(pipe.read_fd(), [](uint32_t) {});
  epoll_.DeleteFd(pipe.read_fd());
}

INSTANTIATE_TEST_SUITE_P(EPollTestBackends, EPollTest,
                         ::testing::Values(true, false));

}  // namespace aos::testing
