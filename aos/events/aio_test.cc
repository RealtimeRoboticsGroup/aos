#include "aos/events/aio.h"

#include <fcntl.h>
#include <signal.h>
#ifndef _WIN32
#include <sys/epoll.h>
#endif

#include <array>
#include <chrono>
#include <thread>

#include "absl/flags/declare.h"
#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "gtest/gtest.h"

#include "aos/events/pipe.h"
#include "aos/ipc_lib/signalfd.h"
#include "aos/ipc_lib/thread_signal.h"
#include "aos/realtime.h"

ABSL_DECLARE_FLAG(bool, use_io_uring);

namespace aos::testing {

class AioTest : public ::testing::TestWithParam<bool> {
 protected:
  void SetUp() override {
    ::absl::SetFlag(&FLAGS_use_io_uring, GetParam());
    ABSL_LOG(INFO) << "Testing Aio with " << (GetParam() ? "io_uring" : "epoll")
                   << " backend.";
  }
};

// Tests that we can push basic strings through a pipe with io_uring.
TEST_P(AioTest, BasicPipeReadWrite) {
  Aio aio;
  Pipe pipe;

  char write_buf[] = "Hello io_uring!";
  char read_buf[64] = {0};

  AsyncRequest write_req;
  write_req.callback = [](Completion completion, void *) {
    EXPECT_TRUE(aos::IsOk(completion.status));
    EXPECT_GT(completion.result, 0);
  };

  AsyncRequest read_req;
  read_req.callback = [](Completion completion, void *) {
    EXPECT_TRUE(aos::IsOk(completion.status));
    EXPECT_GT(completion.result, 0);
  };

  size_t count = 0;
  {
    ScopedRealtime rt;

    aio.AsyncWrite(pipe.write_fd(), write_buf, &write_req);
    aio.AsyncRead(pipe.read_fd(), read_buf, &read_req);

    while ((!write_req.done || !read_req.done) && aio.Poll(true)) {
      ++count;
    }
  }

  // The epoll backend processes exactly one event per Poll() call.  This aligns
  // with the behavior of epoll_linux.cc.
  EXPECT_EQ(count, GetParam() ? 1 : 2);
  EXPECT_STREQ(read_buf, "Hello io_uring!");
}

// Tests that we can have 2 timers going.
TEST_P(AioTest, AsyncTimerTest) {
  Aio aio;

  Aio::Timer timer1(&aio);
  Aio::Timer timer2(&aio);

  aos::monotonic_clock::time_point timer_fired1 =
      aos::monotonic_clock::min_time;
  aos::monotonic_clock::time_point timer_fired2 =
      aos::monotonic_clock::min_time;

  size_t count = 0;
  aos::monotonic_clock::time_point start_time = aos::monotonic_clock::now();
  {
    ScopedRealtime rt;

    timer1.Schedule(
        start_time + std::chrono::milliseconds(100),
        [](Completion completion, void *ctx) {
          EXPECT_TRUE(aos::IsOk(completion.status));
          *static_cast<aos::monotonic_clock::time_point *>(ctx) =
              aos::monotonic_clock::now();
        },
        &timer_fired1);

    timer2.Schedule(
        start_time + std::chrono::milliseconds(500),
        [](Completion completion, void *ctx) {
          EXPECT_TRUE(aos::IsOk(completion.status));
          *static_cast<aos::monotonic_clock::time_point *>(ctx) =
              aos::monotonic_clock::now();
        },
        &timer_fired2);

    while ((timer_fired1 == monotonic_clock::min_time ||
            timer_fired2 == monotonic_clock::min_time) &&
           aio.Poll(true)) {
      ++count;
    }
  }

  EXPECT_EQ(count, 2);
  EXPECT_GT(timer_fired1, start_time + std::chrono::milliseconds(100));
  EXPECT_LT(timer_fired1, start_time + std::chrono::milliseconds(100) +
                              std::chrono::milliseconds(100));
  EXPECT_GT(timer_fired2, start_time + std::chrono::milliseconds(500));
  EXPECT_LT(timer_fired2, start_time + std::chrono::milliseconds(500) +
                              std::chrono::milliseconds(100));
}

// Tests that ThreadSignal events trigger the registered SignalFd wakeup
// callback in the event loop.
TEST_P(AioTest, ThreadSignalTest) {
  Aio aio;

  aos::ipc_lib::SignalFd sfd({aos::ipc_lib::kWakeupSignal});

  bool signal_fired = false;
  aio.RegisterSignalFd(&sfd, [&signal_fired]() { signal_fired = true; });

  const auto pid = aos::GetProcessId();
  const auto tid = aos::GetThreadId();

  std::thread signaler([pid, tid]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    aos::ipc_lib::ThreadSignal signaler_signal;
    signaler_signal.Signal(pid, tid);
  });

  size_t count = 0;
  {
    ScopedRealtime rt;

    // Poll until the read request completes.
    while (!signal_fired && aio.Poll(true)) {
      ++count;
    }
  }

  EXPECT_GT(count, 0);
  EXPECT_TRUE(signal_fired);

  signaler.join();
  aio.UnregisterSignalFd(&sfd);
}

// Tests that a registered SignalFd callback is successfully invoked multiple
// times when multiple signals are sent sequentially, using multishot poll.
TEST_P(AioTest, MultiThreadSignalTest) {
  Aio aio;

  aos::ipc_lib::SignalFd sfd({aos::ipc_lib::kWakeupSignal});

  size_t signal_count = 0;
  std::vector<aos::monotonic_clock::time_point> callback_times;
  callback_times.reserve(3);
  aio.RegisterSignalFd(&sfd, [&signal_count, &callback_times]() {
    ++signal_count;
    callback_times.push_back(aos::monotonic_clock::now());
  });

  const auto pid = aos::GetProcessId();
  const auto tid = aos::GetThreadId();

  auto start = aos::monotonic_clock::now();
  std::thread signaler([pid, tid]() {
    for (size_t i = 0; i < 3; ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      aos::ipc_lib::ThreadSignal signaler_signal;
      signaler_signal.Signal(pid, tid);
    }
  });

  size_t count = 0;
  {
    ScopedRealtime rt;

    // Poll until we receive all 3 signals.
    while (signal_count < 3 && aio.Poll(true)) {
      ++count;
    }
  }

  EXPECT_EQ(count, 3);
  EXPECT_EQ(signal_count, 3);

  ASSERT_EQ(callback_times.size(), 3);
  // The first signal shouldn't show up until the thread finishes its first 50ms
  // wait. We use 40ms of slack to account for non-realtime OS scheduler
  // variations.
  EXPECT_GE(callback_times[0], start + std::chrono::milliseconds(40));
  // The subsequent callbacks must be received in order.
  EXPECT_GE(callback_times[1], callback_times[0]);
  EXPECT_GE(callback_times[2], callback_times[1]);
  EXPECT_LT(callback_times[2], start + std::chrono::seconds(1));

  signaler.join();
  aio.UnregisterSignalFd(&sfd);
}

// Tests that we can cancel a timer immediately after scheduling it.
TEST_P(AioTest, CancelTimerTest) {
  Aio aio;

  Aio::Timer timer(&aio);
  bool callback_invoked = false;

  auto start = aos::monotonic_clock::now();
  {
    ScopedRealtime rt;
    // Schedule a timer for 10 seconds in the future.
    timer.Schedule(
        start + std::chrono::seconds(10),
        [](Completion completion, void *ctx) {
          EXPECT_TRUE(aos::IsOk(completion.status));
          *static_cast<bool *>(ctx) = true;
        },
        &callback_invoked);

    // Immediately cancel the timer.
    timer.Cancel();

    // Poll to ensure it never fires.
    aio.Poll(false);
  }

  EXPECT_FALSE(callback_invoked);
  EXPECT_LT(aos::monotonic_clock::now(), start + std::chrono::seconds(1));
}

// Tests that canceling a pending AsyncRead request executes the callback
// asynchronously inside Poll(), never nested/synchronously inside Cancel().
TEST_P(AioTest, CancelAsyncReadTest) {
  Aio aio;
  Pipe pipe;

  AsyncRequest request;
  bool callback_invoked = false;
  std::optional<aos::Status> fired_status;

  char buf[10];

  {
    ScopedRealtime rt;
    request.callback = [](Completion completion, void *ctx) {
      auto *state =
          static_cast<std::pair<std::optional<aos::Status> *, bool *> *>(ctx);
      state->first->emplace(std::move(completion.status));
      *state->second = true;
    };
    std::pair<std::optional<aos::Status> *, bool *> ctx(&fired_status,
                                                        &callback_invoked);
    request.context = &ctx;

    aio.AsyncRead(pipe.read_fd(), buf, &request);

    // Verify it hasn't run yet.
    EXPECT_FALSE(callback_invoked);

    // Cancel the request.
    aio.Cancel(&request);

    // The callback must NOT run synchronously inside Cancel().
    EXPECT_FALSE(callback_invoked);

    // Now poll, which should execute the canceled callback.
    while (!callback_invoked && aio.Poll(true)) {
    }
  }

  EXPECT_TRUE(callback_invoked);
  ASSERT_TRUE(fired_status.has_value());
  EXPECT_FALSE(aos::IsOk(*fired_status));
  EXPECT_EQ(fired_status->error().message(), "Canceled");
}

// Tests that we can change a timer's deadline by canceling and rescheduling it.
TEST_P(AioTest, ChangeTimerDeadlineTest) {
  Aio aio;

  Aio::Timer timer(&aio);
  std::optional<aos::Status> fired_status;
  size_t invocations = 0;

  std::pair<std::optional<aos::Status> *, size_t *> ctx(&fired_status,
                                                        &invocations);

  size_t count = 0;
  auto start = aos::monotonic_clock::now();
  {
    ScopedRealtime rt;
    // Schedule a timer for 10 seconds in the future.
    timer.Schedule(
        start + std::chrono::seconds(10),
        [](Completion completion, void *ctx) {
          EXPECT_TRUE(aos::IsOk(completion.status));
          auto state =
              static_cast<std::pair<std::optional<aos::Status> *, size_t *> *>(
                  ctx);
          state->first->emplace(std::move(completion.status));
          (*state->second)++;
        },
        &ctx);

    // Re-schedule for 100 milliseconds (implicitly cancels the first).
    timer.Schedule(
        start + std::chrono::milliseconds(100),
        [](Completion completion, void *ctx) {
          EXPECT_TRUE(aos::IsOk(completion.status));
          auto state =
              static_cast<std::pair<std::optional<aos::Status> *, size_t *> *>(
                  ctx);
          state->first->emplace(std::move(completion.status));
          (*state->second)++;
        },
        &ctx);

    // Poll until the rescheduled timer fires (1 completion).
    while (invocations < 1 && aio.Poll(true)) {
      ++count;
    }
  }

  EXPECT_GT(count, 0);
  EXPECT_EQ(invocations, 1);
  ASSERT_TRUE(fired_status.has_value());
  EXPECT_TRUE(aos::IsOk(*fired_status));
  auto end = aos::monotonic_clock::now();
  EXPECT_GT(end, start + std::chrono::milliseconds(100));
  EXPECT_LT(end, start + std::chrono::seconds(1));
}

// Tests that scheduling a timer with a deadline in the past fires immediately
// on the next poll.
TEST_P(AioTest, PastTimerTest) {
  Aio aio;

  Aio::Timer timer(&aio);
  bool callback_invoked = false;

  size_t count;
  aos::monotonic_clock::time_point start;
  {
    ScopedRealtime rt;
    start = aos::monotonic_clock::now();
    // Schedule a timer with a deadline in the past.
    timer.Schedule(
        start - std::chrono::seconds(5),
        [](Completion completion, void *ctx) {
          EXPECT_TRUE(aos::IsOk(completion.status));
          *static_cast<bool *>(ctx) = true;
        },
        &callback_invoked);

    // Poll once. It should fire immediately.
    count = 0;
    while (!callback_invoked && aio.Poll(true)) {
      ++count;
    }
  }

  EXPECT_EQ(count, 1);
  EXPECT_TRUE(callback_invoked);
  EXPECT_LT(aos::monotonic_clock::now(),
            start + std::chrono::milliseconds(500));
}

// Tests that a repeating timer can be implemented by rescheduling from the
// completion callback.
TEST_P(AioTest, RepeatingTimerTest) {
  Aio aio;

  struct TimerContext {
    Aio::Timer timer;
    size_t count = 0;
    bool done = false;

    TimerContext(Aio *aio) : timer(aio) {}

    static void OnTimer(Completion completion, void *ctx) {
      auto state = static_cast<TimerContext *>(ctx);
      EXPECT_TRUE(aos::IsOk(completion.status));
      state->count++;
      if (state->count < 3) {
        // Re-schedule for 50 milliseconds in the future.
        state->timer.Schedule(
            aos::monotonic_clock::now() + std::chrono::milliseconds(50),
            &OnTimer, state);
      } else {
        state->done = true;
      }
    }
  };

  TimerContext timer_ctx(&aio);

  size_t count = 0;
  auto start = aos::monotonic_clock::now();
  {
    ScopedRealtime rt;
    timer_ctx.timer.Schedule(start + std::chrono::milliseconds(50),
                             &TimerContext::OnTimer, &timer_ctx);

    // Poll until repeating timer triggers all iterations.
    while (!timer_ctx.done && aio.Poll(true)) {
      ++count;
    }
  }

  EXPECT_GT(count, 0);
  EXPECT_EQ(timer_ctx.count, 3);
  auto end = aos::monotonic_clock::now();
  EXPECT_GT(end, start + std::chrono::milliseconds(150));
  EXPECT_LT(end, start + std::chrono::seconds(1));
}

// Tests that we can cancel a timer after it has been active for some time.
TEST_P(AioTest, CancelTimerAfterDelayTest) {
  Aio aio;

  Aio::Timer timer(&aio);
  bool callback_invoked = false;

  aos::monotonic_clock::time_point start;
  {
    ScopedRealtime rt;
    start = aos::monotonic_clock::now();
    // Schedule a timer for 10 seconds in the future.
    timer.Schedule(
        start + std::chrono::seconds(10),
        [](Completion completion, void *ctx) {
          EXPECT_TRUE(aos::IsOk(completion.status));
          *static_cast<bool *>(ctx) = true;
        },
        &callback_invoked);

    // Let it run for 100 milliseconds.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    aio.Poll(false);

    // Verify the callback has NOT been invoked yet.
    EXPECT_FALSE(callback_invoked);

    // Cancel the timer.
    timer.Cancel();

    // Poll to ensure it never fires.
    aio.Poll(false);
  }

  EXPECT_FALSE(callback_invoked);
  EXPECT_LT(aos::monotonic_clock::now(), start + std::chrono::seconds(1));
}

// Tests that duplicate registrations for legacy fds and signalfds die.
TEST_P(AioTest, DuplicateRegistrationDeathTest) {
  Aio aio;
  Pipe pipe;

  aio.OnReadable(pipe.read_fd(), []() {});
  EXPECT_DEATH(aio.OnReadable(pipe.read_fd(), []() {}),
               "Duplicate in functions for");

  aio.OnWritable(pipe.write_fd(), []() {});
  EXPECT_DEATH(aio.OnWritable(pipe.write_fd(), []() {}),
               "Duplicate out functions for");

  aio.OnError(pipe.write_fd(), []() {});
  EXPECT_DEATH(aio.OnError(pipe.write_fd(), []() {}),
               "Duplicate error functions for");

  aos::ipc_lib::SignalFd sfd({aos::ipc_lib::kWakeupSignal});
  aio.RegisterSignalFd(&sfd, []() {});
  EXPECT_DEATH(aio.RegisterSignalFd(&sfd, []() {}), "Duplicate.*");

  // Clean up registered resources before loop destruction.
  aio.DeleteFd(pipe.read_fd());
  aio.DeleteFd(pipe.write_fd());
  aio.UnregisterSignalFd(&sfd);
}

// Tests that unregistering untracked legacy fds or signalfds dies.
TEST_P(AioTest, UntrackedUnregistrationDeathTest) {
  Aio aio;

  EXPECT_DEATH(aio.DeleteFd(999), "fd 999 not found");
  aos::ipc_lib::SignalFd sfd2({aos::ipc_lib::kWakeupSignal});
  EXPECT_DEATH(aio.UnregisterSignalFd(&sfd2),
               "(SignalFd not found|fd .* not found)");
}

// Tests that mixing OnEvents and other legacy hooks or calling invalid methods
// triggers assertions.
TEST_P(AioTest, MixedRegistrationAndInvalidHookDeathTest) {
  Aio aio;
  Pipe pipe;

  // OnEvents, then OnReadable fails.
  aio.OnEvents(pipe.read_fd(), [](uint32_t) {});
  EXPECT_DEATH(aio.OnReadable(pipe.read_fd(), []() {}),
               "Cannot mix OnEvents and OnReadable");

  // OnEvents, then EnableWritable fails.
  EXPECT_DEATH(aio.EnableWritable(pipe.read_fd()),
               "EnableWritable is only for fds registered using OnWritable");

  // OnEvents, then DisableWritable fails.
  EXPECT_DEATH(aio.DisableWritable(pipe.read_fd()),
               "DisableWritable is only for fds registered using OnWritable");

  aio.DeleteFd(pipe.read_fd());

  // OnWritable, then SetEvents fails.
  aio.OnWritable(pipe.write_fd(), []() {});
  EXPECT_DEATH(aio.SetEvents(pipe.write_fd(), 0x04),
               "SetEvents is only for fds registered using OnEvents");

  aio.DeleteFd(pipe.write_fd());

  if (!GetParam()) {
    // Under epoll, verify that mixing AsyncRead/AsyncWrite and
    // OnReadable/OnWritable/OnEvents fails.
    AsyncRequest req;
    char buf[10];

    // AsyncRead, then OnReadable fails.
    aio.AsyncRead(pipe.read_fd(), buf, &req);
    EXPECT_DEATH(aio.OnReadable(pipe.read_fd(), []() {}),
                 "Cannot mix OnReadable and AsyncRead");
    aio.Cancel(&req);
    while (aio.Poll(false)) {
    }

    // OnReadable, then AsyncRead fails.
    aio.OnReadable(pipe.read_fd(), []() {});
    EXPECT_DEATH(aio.AsyncRead(pipe.read_fd(), buf, &req),
                 "Cannot mix OnReadable and AsyncRead");
    aio.DeleteFd(pipe.read_fd());

    // AsyncWrite, then OnWritable fails.
    aio.AsyncWrite(pipe.write_fd(), buf, &req);
    EXPECT_DEATH(aio.OnWritable(pipe.write_fd(), []() {}),
                 "Cannot mix OnWritable and AsyncWrite");
    aio.Cancel(&req);
    while (aio.Poll(false)) {
    }

    // OnWritable, then AsyncWrite fails.
    aio.OnWritable(pipe.write_fd(), []() {});
    EXPECT_DEATH(aio.AsyncWrite(pipe.write_fd(), buf, &req),
                 "Cannot mix OnWritable and AsyncWrite");
    aio.DeleteFd(pipe.write_fd());

    // AsyncRead, then OnEvents fails.
    aio.AsyncRead(pipe.read_fd(), buf, &req);
    EXPECT_DEATH(aio.OnEvents(pipe.read_fd(), [](uint32_t) {}),
                 "Cannot mix OnEvents and AsyncRead/AsyncWrite");
    aio.Cancel(&req);
    while (aio.Poll(false)) {
    }

    // OnEvents, then AsyncRead fails.
    aio.OnEvents(pipe.read_fd(), [](uint32_t) {});
    EXPECT_DEATH(aio.AsyncRead(pipe.read_fd(), buf, &req),
                 "Cannot mix OnEvents and AsyncRead/AsyncWrite");
    aio.DeleteFd(pipe.read_fd());
  }
}

// Tests that a failed I/O operation (like reading from an invalid fd)
// is correctly captured as an error status and the raw errno is populated.
TEST_P(AioTest, FailedIoErrorTest) {
  Aio aio;

  AsyncRequest read_req;
  std::optional<aos::Status> fired_status;
  int32_t result_code = 0;

  read_req.callback = [](Completion completion, void *ctx) {
    auto self_ctx =
        static_cast<std::pair<std::optional<aos::Status> *, int32_t *> *>(ctx);
    self_ctx->first->emplace(std::move(completion.status));
    *self_ctx->second = completion.result;
  };
  std::pair<std::optional<aos::Status> *, int32_t *> ctx(&fired_status,
                                                         &result_code);
  read_req.context = &ctx;

  char buf[8];
  size_t count = 0;
  {
    ScopedRealtime rt;
    // Schedule a read on an invalid file descriptor (-1).
    aio.AsyncRead(-1, buf, &read_req);

    // Poll until it executes.
    while (!read_req.done && aio.Poll(true)) {
      ++count;
    }
  }

  EXPECT_GT(count, 0);
  ASSERT_TRUE(fired_status.has_value());
  EXPECT_FALSE(aos::IsOk(*fired_status));
  EXPECT_EQ(fired_status->error().message(), "io_uring error");
  EXPECT_EQ(result_code, EBADF);
}

// Tests the normal behavior of OnEvents (scheduling, callback delivery,
// persistent one-shot re-submission, and unregistering).
TEST_P(AioTest, LegacyFdTest) {
  Aio aio;
  Pipe pipe;

  size_t callback_count = 0;
  uint32_t active_events = 0;

  // Register for input readiness (0x01 = POLLIN / Epoll In).
  aio.OnEvents(pipe.read_fd(),
               [&callback_count, &active_events](uint32_t events) {
                 ++callback_count;
                 active_events = events;
               });
  aio.SetEvents(pipe.read_fd(), 0x01);

  char write_buf[] = "a";
  char read_buf[1];
  size_t count = 0;

  {
    ScopedRealtime rt;
    // Verify that since the pipe is empty, no callback fires.
    aio.Poll(false);
  }
  EXPECT_EQ(callback_count, 0);

  // Write some data to make the pipe readable.
  ASSERT_EQ(write(pipe.write_fd(), write_buf, 1), 1);

  {
    ScopedRealtime rt;
    // Poll.  The callback should fire.
    while (callback_count == 0 && aio.Poll(true)) {
      ++count;
    }
  }

  EXPECT_GT(count, 0);
  EXPECT_EQ(callback_count, 1);
  EXPECT_TRUE(active_events & 0x01);

  // Read the data to consume it and clear readability.
  ASSERT_EQ(read(pipe.read_fd(), read_buf, 1), 1);

  {
    ScopedRealtime rt;
    // Verify that polling now does not trigger additional callbacks.
    aio.Poll(false);
  }
  EXPECT_EQ(callback_count, 1);

  // Write again.  Since it is persistently re-submitted, it should fire again.
  ASSERT_EQ(write(pipe.write_fd(), write_buf, 1), 1);
  count = 0;
  {
    ScopedRealtime rt;
    while (callback_count == 1 && aio.Poll(true)) {
      ++count;
    }
  }

  EXPECT_GT(count, 0);
  EXPECT_EQ(callback_count, 2);

  // Read data to clear it.
  ASSERT_EQ(read(pipe.read_fd(), read_buf, 1), 1);

  // Unregister the descriptor.
  aio.DeleteFd(pipe.read_fd());

  // Write more data.
  ASSERT_EQ(write(pipe.write_fd(), write_buf, 1), 1);

  {
    ScopedRealtime rt;
    // Poll.  The callback should NOT fire anymore.
    aio.Poll(false);
  }
  EXPECT_EQ(callback_count, 2);

  // Clean up.
  ASSERT_EQ(read(pipe.read_fd(), read_buf, 1), 1);
}

// Tests the writable readiness behavior of OnEvents (using 0x04 / POLLOUT).
TEST_P(AioTest, LegacyFdWritableTest) {
  Aio aio;
  Pipe pipe;

  size_t callback_count = 0;
  uint32_t active_events = 0;

  // Register for output readiness (0x04 = POLLOUT / Epoll Out).
  aio.OnEvents(pipe.write_fd(),
               [&callback_count, &active_events](uint32_t events) {
                 ++callback_count;
                 active_events = events;
               });
  aio.SetEvents(pipe.write_fd(), 0x04);

  size_t count = 0;
  {
    ScopedRealtime rt;
    // Poll.  Since the pipe is empty and has space, it should be writable
    // immediately.
    while (callback_count == 0 && aio.Poll(true)) {
      ++count;
    }
  }
  EXPECT_GT(count, 0);
  EXPECT_EQ(callback_count, 1);
  EXPECT_TRUE(active_events & 0x04);

  // Clean up.
  aio.DeleteFd(pipe.write_fd());
}

// Tests the error/hangup readiness behavior of OnEvents (using 0x08 / POLLERR |
// POLLHUP).
TEST_P(AioTest, LegacyFdErrorTest) {
  Aio aio;
  Pipe pipe;

  size_t callback_count = 0;
  uint32_t active_events = 0;

  // Register the read end for hangup/error events (0x08 = POLLERR | POLLHUP).
  aio.OnEvents(pipe.read_fd(),
               [&callback_count, &active_events](uint32_t events) {
                 ++callback_count;
                 active_events = events;
               });
  aio.SetEvents(pipe.read_fd(), 0x08);

  {
    ScopedRealtime rt;
    // Verify no callback fires initially.
    aio.Poll(false);
  }
  EXPECT_EQ(callback_count, 0);

  // Close the write end of the pipe to trigger POLLHUP.
  pipe.close_write_fd();

  size_t count = 0;
  {
    ScopedRealtime rt;
    // Poll.  The callback should fire with the hangup/error event.
    while (callback_count == 0 && aio.Poll(true)) {
      ++count;
    }
  }
  EXPECT_GT(count, 0);
  EXPECT_EQ(callback_count, 1);
  EXPECT_TRUE(active_events & 0x08);

  // Clean up.
  aio.DeleteFd(pipe.read_fd());
}

// Tests that we can schedule, cancel, and schedule again before polling,
// and it does not hang and correctly processes the rescheduled timer.
TEST_P(AioTest, ScheduleCancelScheduleTest) {
  Aio aio;

  Aio::Timer timer(&aio);
  size_t canceled_count = 0;
  size_t fired_count = 0;

  struct TestState {
    size_t *canceled_count;
    size_t *fired_count;
  };
  TestState state{&canceled_count, &fired_count};

  // 1. Schedule timer.
  timer.Schedule(
      aos::monotonic_clock::now() + std::chrono::seconds(10),
      [](Completion /*completion*/, void *ctx) {
        auto *s = static_cast<TestState *>(ctx);
        (*s->canceled_count)++;
      },
      &state);

  // 2. Cancel timer.
  timer.Cancel();
  EXPECT_EQ(canceled_count, 0);
  EXPECT_EQ(fired_count, 0);

  // 3. Schedule timer again.
  timer.Schedule(
      aos::monotonic_clock::now() + std::chrono::milliseconds(100),
      [](Completion completion, void *ctx) {
        auto *s = static_cast<TestState *>(ctx);
        EXPECT_TRUE(aos::IsOk(completion.status));
        (*s->fired_count)++;
      },
      &state);

  // The first callback (canceled) should NEVER run.
  EXPECT_EQ(canceled_count, 0);
  EXPECT_EQ(fired_count, 0);

  // 4. Poll. We expect the new timer to fire.
  while (fired_count == 0 && aio.Poll(true)) {
  }

  EXPECT_EQ(canceled_count, 0);
  EXPECT_EQ(fired_count, 1);
}

struct NoNestedCallbackState {
  bool timer1_fired = false;
  bool timer3_fired = false;
  bool nested_callback_detected = false;
  Aio::Timer *timer2 = nullptr;
};

// Tests that cancelling or rescheduling a timer does not trigger other
// pending completion callbacks nested inside the current callback context.
TEST_P(AioTest, NoNestedCallbackTest) {
  Aio aio;

  Aio::Timer timer1(&aio);
  Aio::Timer timer2(&aio);
  Aio::Timer timer3(&aio);

  NoNestedCallbackState test_state;
  test_state.timer2 = &timer2;

  timer1.Schedule(
      aos::monotonic_clock::now() + std::chrono::milliseconds(100),
      [](Completion, void *ctx) {
        auto *s = static_cast<NoNestedCallbackState *>(ctx);
        s->timer1_fired = true;
        // Reschedule timer2 (which was scheduled for 10s).
        // This must NOT execute timer3's callback nested inside here.
        s->timer2->Schedule(
            aos::monotonic_clock::now() + std::chrono::seconds(20),
            [](Completion, void *) {}, nullptr);
        if (s->timer3_fired) {
          s->nested_callback_detected = true;
        }
      },
      &test_state);

  // Timer 2 is scheduled for 10s.
  timer2.Schedule(
      aos::monotonic_clock::now() + std::chrono::seconds(10),
      [](Completion, void *) {}, nullptr);

  // Timer 3 is scheduled to expire at the same time as Timer 1.
  timer3.Schedule(
      aos::monotonic_clock::now() + std::chrono::milliseconds(100),
      [](Completion, void *ctx) {
        auto *s = static_cast<NoNestedCallbackState *>(ctx);
        s->timer3_fired = true;
      },
      &test_state);

  // Run the event loop until both Timer 1 and Timer 3 have fired.
  while ((!test_state.timer1_fired || !test_state.timer3_fired) &&
         aio.Poll(true)) {
  }

  EXPECT_TRUE(test_state.timer1_fired);
  EXPECT_TRUE(test_state.timer3_fired);
  EXPECT_FALSE(test_state.nested_callback_detected);
}

// Tests the OnReadable/OnWritable/OnError legacy EPoll-like APIs.
TEST_P(AioTest, EPollLikeLegacyFdTest) {
  Aio aio;
  Pipe pipe;

  size_t readable_count = 0;
  size_t writable_count = 0;

  aio.OnReadable(pipe.read_fd(), [&readable_count]() { ++readable_count; });
  aio.OnWritable(pipe.write_fd(), [&writable_count]() { ++writable_count; });

  // Initially, writable should fire because the pipe is empty.
  size_t count = 0;
  {
    ScopedRealtime rt;
    while (writable_count == 0 && aio.Poll(true)) {
      ++count;
    }
  }
  EXPECT_GT(count, 0);
  EXPECT_EQ(writable_count, 1);
  EXPECT_EQ(readable_count, 0);

  // Disable writability.
  {
    ScopedRealtime rt;
    aio.DisableWritable(pipe.write_fd());
  }

  // Write a byte to make readable fire.
  char buf[] = "a";
  ssize_t res = write(pipe.write_fd(), buf, 1);
  ASSERT_EQ(res, 1);

  // Poll; readable should fire. Writable should NOT fire again since disabled.
  count = 0;
  {
    ScopedRealtime rt;
    while (readable_count == 0 && aio.Poll(true)) {
      ++count;
    }
  }
  EXPECT_GT(count, 0);
  EXPECT_EQ(readable_count, 1);
  EXPECT_EQ(writable_count, 1);

  // Re-enable writability.
  {
    ScopedRealtime rt;
    aio.EnableWritable(pipe.write_fd());
  }
  count = 0;
  {
    ScopedRealtime rt;
    while (writable_count == 1 && aio.Poll(true)) {
      ++count;
    }
  }
  EXPECT_GT(count, 0);
  EXPECT_EQ(writable_count, 2);

  // Clean up.
  aio.DeleteFd(pipe.read_fd());
  aio.DeleteFd(pipe.write_fd());
}

// Tests the OnEvents/SetEvents legacy EPoll-like APIs.
TEST_P(AioTest, EPollLikeOnEventsTest) {
  Aio aio;
  Pipe pipe;

  size_t event_count = 0;
  uint32_t active_events = 0;

  aio.OnEvents(pipe.read_fd(), [&event_count, &active_events](uint32_t events) {
    ++event_count;
    active_events = events;
  });

  // Schedule events (0x01 = POLLIN / Epoll In).
  {
    ScopedRealtime rt;
    aio.SetEvents(pipe.read_fd(), 0x01);
  }

  // Write a byte.
  char buf[] = "x";
  ssize_t res = write(pipe.write_fd(), buf, 1);
  ASSERT_EQ(res, 1);

  // Poll; callback should fire.
  size_t count = 0;
  {
    ScopedRealtime rt;
    while (event_count == 0 && aio.Poll(true)) {
      ++count;
    }
  }
  EXPECT_GT(count, 0);
  EXPECT_EQ(event_count, 1);
  EXPECT_TRUE(active_events & 0x01);

  // Clean up.
  aio.DeleteFd(pipe.read_fd());
}

// Tests DeleteFd and ForgetClosedFd APIs.
TEST_P(AioTest, EPollLikeDeleteAndForgetTest) {
  Aio aio;
  Pipe pipe;

  size_t readable_count = 0;
  aio.OnReadable(pipe.read_fd(), [&readable_count]() { ++readable_count; });

  // Delete fd.
  aio.DeleteFd(pipe.read_fd());

  // Write a byte and verify callback is not called.
  char buf[] = "x";
  ssize_t res = write(pipe.write_fd(), buf, 1);
  ASSERT_EQ(res, 1);

  {
    ScopedRealtime rt;
    aio.Poll(false);
  }
  EXPECT_EQ(readable_count, 0);

  // Register write end, close it, and forget closed fd.
  size_t writable_count = 0;
  int w_fd = pipe.write_fd();
  aio.OnWritable(w_fd, [&writable_count]() { ++writable_count; });

  pipe.close_write_fd();
  aio.ForgetClosedFd(w_fd);
}

// Tests repeating timers schedule functionality.
TEST_P(AioTest, RepeatingTimerScheduleTest) {
  Aio aio;

  size_t timer_fires = 0;
  Aio::Timer timer(&aio);

  size_t count = 0;
  {
    ScopedRealtime rt;
    timer.Schedule(
        aos::monotonic_clock::now() + std::chrono::milliseconds(50),
        std::chrono::milliseconds(50),
        [](Completion completion, void *context) {
          if (aos::IsOk(completion.status)) {
            ++(*static_cast<size_t *>(context));
          }
        },
        &timer_fires);

    // Poll multiple times to let the timer fire twice automatically.
    while (timer_fires < 2 && aio.Poll(true)) {
      ++count;
    }
  }

  EXPECT_GT(count, 0);
  EXPECT_GE(timer_fires, 2);

  // Cancel repeating timer.
  {
    ScopedRealtime rt;
    timer.Cancel();
  }
}

// Helper function to poll Aio for a given duration.
void RunAioFor(Aio &aio, std::chrono::nanoseconds duration) {
  Aio::Timer timer(&aio);
  bool done = false;
  {
    ScopedRealtime rt;
    timer.Schedule(
        aos::monotonic_clock::now() + duration,
        [](Completion, void *ctx) { *static_cast<bool *>(ctx) = true; }, &done);
    while (!done && aio.Poll(true)) {
    }
  }
}

// Test that the basics of OnWritable work, filling the pipe's buffer.
TEST_P(AioTest, EPollLikeBasicWritable) {
  Aio aio;
  Pipe pipe;
  int number_writes = 0;
  aio.OnWritable(pipe.write_fd(), [&]() {
    pipe.Write(" ");
    ++number_writes;
  });

  // First, fill up the pipe's write buffer.
  RunAioFor(aio, std::chrono::milliseconds(50));
  EXPECT_GT(number_writes, 0);

  // Now, if we try again, we shouldn't do anything because buffer is full.
  const int bytes_in_pipe = number_writes;
  number_writes = 0;
  RunAioFor(aio, std::chrono::milliseconds(50));
  EXPECT_EQ(number_writes, 0);

  // Empty the pipe, then fill it up again.
  for (int i = 0; i < bytes_in_pipe; ++i) {
    ASSERT_EQ(" ", pipe.Read(1));
  }
  number_writes = 0;
  RunAioFor(aio, std::chrono::milliseconds(50));
  EXPECT_EQ(number_writes, bytes_in_pipe);

  aio.DeleteFd(pipe.write_fd());
}

// Test that the basics of OnError work by closing the read end.
TEST_P(AioTest, EPollLikeBasicError) {
  Aio aio;
  Pipe pipe;
  int number_errors = 0;
  aio.OnError(pipe.write_fd(), [&]() { ++number_errors; });

  // Sanity check that we don't get any errors before anything has happened.
  RunAioFor(aio, std::chrono::milliseconds(50));
  EXPECT_EQ(number_errors, 0);

  pipe.close_read_fd();

  // Poll once to consume the hangup/error.
  {
    ScopedRealtime rt;
    aio.Poll(false);
  }

  EXPECT_EQ(number_errors, 1);

  aio.DeleteFd(pipe.write_fd());
}

// Tests that removing an event before scheduling any events works.
TEST_P(AioTest, EPollLikeRemoveWithoutEvents) {
  Aio aio;
  Pipe pipe;
  aio.OnEvents(pipe.read_fd(), [](uint32_t) {});
  aio.DeleteFd(pipe.read_fd());
}

// Tests that calling Quit from a BeforeWait callback successfully stops the
// loop.
TEST_P(AioTest, QuitInBeforeWait) {
  Aio aio;
  aio.BeforeWait([&aio]() { aio.Quit(); });
  aio.Run();
}

// Tests that we can wake up the event loop multiple times consecutively.
// This ensures the wakeup eventfd read completion callback is correctly
// re-registered in both backends.
TEST_P(AioTest, MultipleWakeupsTest) {
  Aio aio;
  std::atomic<int> wakeups{0};

  std::thread signaler([&aio, &wakeups]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    wakeups = 1;
    aio.Wakeup();

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    wakeups = 2;
    aio.Wakeup();
  });

  // Wait for first wakeup.
  while (wakeups < 1) {
    aio.Poll(true);
  }
  EXPECT_EQ(wakeups, 1);

  // Wait for second wakeup.
  while (wakeups < 2) {
    aio.Poll(true);
  }
  EXPECT_EQ(wakeups, 2);

  signaler.join();
}

// Tests that unregistering a signalfd correctly cancels the underlying
// multishot poll request and prevents any use-after-free or extra callbacks.
TEST_P(AioTest, UnregisterSignalFdTest) {
  Aio aio;
  aos::ipc_lib::SignalFd sfd({aos::ipc_lib::kWakeupSignal});

  int count = 0;
  aio.RegisterSignalFd(&sfd, [&count]() { ++count; });

  // Send kWakeupSignal to our thread.
  pthread_kill(pthread_self(), aos::ipc_lib::kWakeupSignal);

  // Poll until the signal is handled.
  while (count == 0 && aio.Poll(true)) {
  }
  EXPECT_EQ(count, 1);

  // Unregister the signalfd.
  aio.UnregisterSignalFd(&sfd);

  // Send the signal again.
  pthread_kill(pthread_self(), aos::ipc_lib::kWakeupSignal);

  // Clean up signal by reading it manually so it doesn't stay pending.
  struct signalfd_siginfo siginfo;
  ssize_t res = read(sfd.fd(), &siginfo, sizeof(siginfo));
  EXPECT_EQ(res, sizeof(siginfo));

  // Poll non-blockingly a few times to ensure the callback is NOT invoked.
  for (int i = 0; i < 5; ++i) {
    aio.Poll(false);
  }
  EXPECT_EQ(count, 1);
}

TEST_P(AioTest, DuplicateEventOnCancel) {
  // Test that clearing events on an active file descriptor from its callback
  // does not result in duplicate events due to recursive cancel handling.
  Aio aio;
  Pipe pipe;
  int count = 0;

  aio.OnEvents(pipe.write_fd(), [&](uint32_t events) {
    EXPECT_TRUE(events & EPOLLOUT);
    ++count;
    aio.SetEvents(pipe.write_fd(), 0);
  });

  aio.SetEvents(pipe.write_fd(), EPOLLOUT);
  aio.Poll(true);
  aio.Poll(false);

  EXPECT_EQ(count, 1);

  aio.DeleteFd(pipe.write_fd());
}

TEST_P(AioTest, ForkDeathTest) {
  // Test that an Aio instance constructed in the parent process remains
  // fully functional inside a forked child process by successfully detecting
  // the fork and recreating the io_uring ring.  We register both a standard
  // file descriptor event and a SignalFd to exercise all re-registration loops
  // in HandleFork.
  Aio aio;
  Pipe pipe;
  aos::ipc_lib::SignalFd sfd({aos::ipc_lib::kWakeupSignal});

  int signal_count = 0;
  int fd_count = 0;

  aio.RegisterSignalFd(&sfd, [&]() { ++signal_count; });

  aio.OnEvents(pipe.write_fd(), [&](uint32_t events) {
    EXPECT_TRUE(events & EPOLLOUT);
    ++fd_count;
  });

  aio.SetEvents(pipe.write_fd(), EPOLLOUT);

  EXPECT_EXIT(
      {
        pthread_kill(pthread_self(), aos::ipc_lib::kWakeupSignal);
        while ((signal_count == 0 || fd_count == 0) && aio.Poll(true)) {
        }
        if (signal_count == 1 && fd_count == 1) {
          exit(42);
        }
        exit(1);
      },
      ::testing::ExitedWithCode(42), "");

  aio.UnregisterSignalFd(&sfd);
  aio.DeleteFd(pipe.write_fd());
}
TEST_P(AioTest, TimerForkTest) {
  // Test that an active timer scheduled in the parent process remains fully
  // functional and fires inside a forked child process.  This verifies that the
  // backend correctly re-registers pending timeouts when recreating the loop.
  Aio aio;
  Aio::Timer timer(&aio);

  int timer_count = 0;
  timer.Schedule(
      aos::monotonic_clock::now(),
      [](Completion, void *context) {
        auto *counter = static_cast<int *>(context);
        ++(*counter);
      },
      &timer_count);

  EXPECT_EXIT(
      {
        while (timer_count == 0 && aio.Poll(true)) {
        }
        if (timer_count == 1) {
          exit(42);
        }
        exit(1);
      },
      ::testing::ExitedWithCode(42), "");
}
TEST_P(AioTest, ShouldRunTest) {
  Aio aio;
  EXPECT_FALSE(aio.should_run());

  // Set a timer to check should_run() while running and then quit.
  Aio::Timer timer(&aio);
  struct Context {
    Aio *aio;
    bool checked_running;
  };
  Context context{&aio, false};

  timer.Schedule(
      aos::monotonic_clock::now(),
      [](Completion, void *ctx) {
        auto *c = static_cast<Context *>(ctx);
        EXPECT_TRUE(c->aio->should_run());
        c->checked_running = true;
        c->aio->Quit();
        EXPECT_FALSE(c->aio->should_run());
      },
      &context);

  aio.Run();
  EXPECT_FALSE(aio.should_run());
  EXPECT_TRUE(context.checked_running);
}
INSTANTIATE_TEST_SUITE_P(AioBackends, AioTest, ::testing::Values(true, false));

}  // namespace aos::testing
