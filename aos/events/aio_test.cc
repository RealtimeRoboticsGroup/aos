#include "aos/events/aio.h"

#include <fcntl.h>
#include <signal.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <sys/inotify.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/syscall.h>
#include <sys/uio.h>
#include <sys/utsname.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "absl/flags/commandlineflag.h"
#include "absl/flags/declare.h"
#include "absl/flags/flag.h"
#include "absl/flags/reflection.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "aos/events/pipe.h"
#include "aos/ipc_lib/thread_signal.h"
#include "aos/realtime.h"
#include "aos/testing/tmpdir.h"

ABSL_DECLARE_FLAG(std::string, aio_backend);
ABSL_DECLARE_FLAG(uint32_t, aio_queue_depth);
ABSL_DECLARE_FLAG(size_t, aio_pool_size);

namespace aos::testing {

// Scoped watchdog: arms a SIGALRM fuse so a wedged test dies loudly instead
// of hanging until bazel's timeout, and disarms it on scope exit so the fuse
// cannot leak into subsequent tests in this process (alarm(2) keeps exactly
// one pending fuse per process, and nothing else would ever clear it).
// SIGALRM's default disposition -- terminate -- is the whole watchdog; no
// handler is installed on purpose.  Note the fuse lives in the *parent*
// test process even when armed around a death test: alarm() timers are not
// inherited across fork(), so the child never sees it -- the parent's fuse
// covers waiting on a wedged child.
class ScopedDeathTestWatchdog {
 public:
  ScopedDeathTestWatchdog() { alarm(30); }
  ~ScopedDeathTestWatchdog() { alarm(0); }
};

namespace {

constexpr char kFatalStatementMarker[] =
    "aio_test: reached the statement expected to die";

// For a death test whose statement is a sequence: call this just before the
// step that is expected to die, and match the child's output with
// DiesAfterMarker().  A death message alone cannot say which step produced
// it, so without the marker an earlier step dying with a similar message
// would pass the test for the wrong reason.
//
// The marker travels over stderr because that is what gtest already reads
// back from the child, on every platform.  A Pipe would not: on Windows the
// child is a fresh process that inherits none of the parent's descriptors.
void MarkFatalStatement() { std::cerr << kFatalStatementMarker << std::endl; }

// Matches a death that happened after MarkFatalStatement() ran, with `regex`
// in the output.  `regex` is matched exactly as EXPECT_DEATH would have
// matched it on its own.
::testing::Matcher<const std::string &> DiesAfterMarker(
    const std::string &regex) {
  return ::testing::AllOf(::testing::HasSubstr(kFatalStatementMarker),
                          ::testing::ContainsRegex(regex));
}

}  // namespace

// Self-imposed backpressure for ring-churning loops.  Closing an io_uring fd
// is fire-and-forget: the kernel frees the ring asynchronously on a
// workqueue, and IORING_SETUP_DEFER_TASKRUN rings additionally block that
// work on a full RCU grace period each (io_ring_exit_work).  Teardown
// throughput is therefore capped -- ~2500 rings/s on an idle machine,
// collapsing under CPU load as grace periods stretch -- while creation is
// effectively unbounded.  A loop that creates rings faster than the kernel
// retires them accumulates gigabytes of unreclaimable slab (pinned ctx,
// request, and ring-buffer memory), which is exactly what a heavily parallel
// CI run did to the whole build cluster.  There is no API to wait for a
// specific ring's teardown, but /proc/meminfo's SUnreclaim tracks the
// backlog well: capture a baseline, and whenever growth exceeds a slack
// threshold, sleep until the kernel catches back up.  Backpressure only,
// never an assertion -- the counter is machine-global, so a noisy neighbor
// can only ever make this throttle extra, not pass wrongly.  Linux-only;
// no-op elsewhere.
inline void ThrottleOnKernelRingTeardown() {
#ifdef __linux__
  static const auto read_sunreclaim_kb = []() -> long {
    FILE *f = fopen("/proc/meminfo", "r");
    if (f == nullptr) return -1;
    char line[128];
    long kb = -1;
    while (fgets(line, sizeof(line), f) != nullptr) {
      if (sscanf(line, "SUnreclaim: %ld kB", &kb) == 1) break;
    }
    fclose(f);
    return kb;
  };
  static const long baseline_kb = read_sunreclaim_kb();
  if (baseline_kb < 0) return;
  // Small slack: with per-process baselines, every concurrently-running
  // process effectively grants itself this much headroom, so fleet-wide
  // growth scales with (slack x process count).  (An absolute threshold
  // would avoid that but can't be chosen portably -- idle SUnreclaim
  // varies by gigabytes across machines.)
  constexpr long kSlackKb = 64 * 1024;  // 64 MB over baseline
  // Whole-process throttle budget, not per-checkpoint: on a node where
  // *other* processes keep the backlog elevated, a per-checkpoint deadline
  // multiplies across a loop's checkpoints (observed: 20 x 10s = 200s of
  // stalling inside one test, blowing its 300s timeout).  The budget bounds
  // the total slowdown this helper can ever add to a process; when it's
  // spent, the loop runs unthrottled and the machine-level pressure is the
  // neighbors' to shed.
  static std::chrono::milliseconds budget{15000};
  while (budget > std::chrono::milliseconds(0) &&
         read_sunreclaim_kb() > baseline_kb + kSlackKb) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    budget -= std::chrono::milliseconds(20);
  }
#endif
}

// A repeating timer, built the way consumers have to build one now that
// Aio::Timer is deliberately one-shot (see Aio::Timer::Schedule()):
// re-schedule from the callback against an absolute grid the caller owns.
//
// This is ShmTimerHandler's pattern minus its policy choice -- it skips
// straight to the next *future* deadline when it falls behind, where this
// delivers every elapsed period in turn.  Tests want the latter, since
// "did every period get delivered" is a thing several of them measure.
// Having the policy be visibly the caller's is the point: it is why Aio
// itself does not implement repeating timers.
class RepeatingTimer {
 public:
  RepeatingTimer(Aio *aio, std::function<void(Completion)> callback)
      : timer_(aio), callback_(std::move(callback)) {}

  void Start(aos::monotonic_clock::time_point base,
             aos::monotonic_clock::duration period) {
    base_ = base;
    period_ = period;
    timer_.Schedule(base_, &OnFire, this);
  }

  // Retargets a running timer, exactly as an external Schedule() would.
  void Reschedule(aos::monotonic_clock::time_point base,
                  aos::monotonic_clock::duration period) {
    Start(base, period);
  }

  void Cancel() { timer_.Cancel(); }

 private:
  static void OnFire(Completion completion, void *ctx) {
    auto *self = static_cast<RepeatingTimer *>(ctx);
    // Re-arm before dispatching, so a callback that cancels or reschedules
    // wins over this arm rather than racing it.
    self->base_ += self->period_;
    self->timer_.Schedule(self->base_, &OnFire, self);
    self->callback_(completion);
  }

  Aio::Timer timer_;
  std::function<void(Completion)> callback_;
  aos::monotonic_clock::time_point base_;
  aos::monotonic_clock::duration period_{};
};

// Whether this kernel meets the io_uring backend's floor.  Mirrors
// RequireMinimumKernelVersion() in aio_linux.cc, which CHECKs instead.  See
// IoUringUnsupportedHere() for what the tests do with the answer.
bool KernelSupportsIoUringBackend() {
  struct utsname name;
  ABSL_PCHECK(uname(&name) == 0);
  int major = 0;
  int minor = 0;
  ABSL_CHECK_EQ(sscanf(name.release, "%d.%d", &major, &minor), 2)
      << "Could not parse the kernel release \"" << name.release << "\"";
  return major > 6 || (major == 6 && minor >= 1);
}

// Whether the io_uring cases should skip themselves: the kernel is below the
// backend's floor, on a platform that doesn't run io_uring by default.  That
// is an old kernel under a build that already defaults to epoll, and skipping
// lets the epoll cases -- the ones that platform actually runs -- get their
// turn, where the backend's own CHECK would kill the binary on the first
// io_uring case.
//
// Dies instead when the build's default is io_uring.  Then the kernel cannot
// run what the platform says it runs, and quietly testing only epoll would be
// exactly the silent fallback --aio_backend refuses to have.
bool IoUringUnsupportedHere() {
  if (KernelSupportsIoUringBackend()) {
    return false;
  }
  const std::string default_backend =
      absl::GetFlagReflectionHandle(FLAGS_aio_backend).DefaultValue();
  ABSL_CHECK_NE(default_backend, "io_uring")
      << ": this build defaults --aio_backend to io_uring (its platform "
         "declares //tools/platforms/io_uring support), but the kernel is "
         "below the backend's 6.1 floor.  Build for a platform without "
         "io_uring support, or run on a newer kernel.";
  return true;
}

// Parameterized by backend name, the same string --aio_backend takes, so a
// failing test names the backend it failed on instead of an index -- and so
// adding a backend does not mean re-teaching this a new boolean.
class AioTest : public ::testing::TestWithParam<std::string> {
 protected:
  // Whether the backend under test is io_uring.  Several tests below cover
  // behavior only it has (SINGLE_ISSUER enforcement, orphaned destruction);
  // they skip elsewhere rather than assert something epoll never promised.
  static bool IsIoUring() { return GetParam() == "io_uring"; }

  void SetUp() override {
    if (IsIoUring() && IoUringUnsupportedHere()) {
      GTEST_SKIP() << "kernel is below the io_uring backend's 6.1 floor, and "
                      "this platform defaults to epoll";
    }
    // Pace ring creation across the whole suite, not just the
    // ring-churning tests -- see ThrottleOnKernelRingTeardown().
    ThrottleOnKernelRingTeardown();
    ::absl::SetFlag(&FLAGS_aio_backend, GetParam());
    ABSL_LOG(INFO) << "Testing Aio with the " << GetParam() << " backend.";
  }

  // Restores every flag SetUp() (or the test body) touched.
  absl::FlagSaver flag_saver_;
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

  // One Poll(): it resolves both requests -- the write, then the read the
  // write made ready -- and sets both `done` flags, so the loop exits after
  // it even though only one callback has run.  io_uring completes both in
  // one submit; epoll matches by harvesting until its own I/O readies
  // nothing more.  Delivering the second callback is the next Poll()'s job.
  EXPECT_EQ(count, 1);
  EXPECT_TRUE(aio.Poll(false)) << "the second callback was still queued";
  EXPECT_STREQ(read_buf, "Hello io_uring!");
}

// A deadline already in the past fires as soon as the loop is driven, and
// aos::monotonic_clock::epoch() is just the earliest such deadline -- not a
// special value.  aos::TimerHandler::Schedule() lets callers pass it to mean
// "now", and the simulated event loop runs on a timeline whose origin *is*
// the epoch, so a ShmEventLoop that treated it differently would diverge
// from sim for the same application code.
//
// The Linux backends need help here: they arm with timerfd_settime(2), and
// an itimerspec whose it_value is exactly zero means "disarm", not "expire
// immediately" -- so the epoch is the one past deadline that would silently
// never fire.  See IoUringTimerState::Schedule().
TEST_P(AioTest, ScheduleTimerAtEpochFiresTest) {
  Aio aio;
  Aio::Timer timer(&aio);

  int fires = 0;
  timer.Schedule(
      aos::monotonic_clock::epoch(),
      [](Completion completion, void *ctx) {
        if (aos::IsOk(completion.status)) {
          ++*static_cast<int *>(ctx);
        }
      },
      &fires);

  // Bounded rather than a blocking drain: a dropped deadline means nothing
  // ever wakes the loop, so Poll(true) would hang here instead of failing.
  const auto give_up =
      aos::monotonic_clock::now() + std::chrono::milliseconds(500);
  while (fires == 0 && aos::monotonic_clock::now() < give_up) {
    aio.Poll(false);
  }
  EXPECT_EQ(fires, 1)
      << "A timer scheduled at the monotonic epoch never fired.";
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
  // The not-early bounds stay exact: an absolute timer firing before its
  // deadline is a kernel-level guarantee violation, never scheduling noise.
  // The late bounds only need to discriminate a *wrong* deadline (unit
  // errors and deadline/interval mix-ups miss by at least the whole
  // deadline, i.e. 100ms/500ms or more) from ordinary dispatch latency on
  // a loaded machine -- observed up to ~175ms late under a saturated CI
  // node (20+ pegged containers), which a tight 100ms slack flaked on at
  // ~0.02% while proving nothing.  400ms sits well clear of measured noise
  // while still catching every real failure mode by a wide margin.
  constexpr std::chrono::milliseconds kLateSlack(400);
  EXPECT_GT(timer_fired1, start_time + std::chrono::milliseconds(100));
  EXPECT_LT(timer_fired1,
            start_time + std::chrono::milliseconds(100) + kLateSlack);
  EXPECT_GT(timer_fired2, start_time + std::chrono::milliseconds(500));
  EXPECT_LT(timer_fired2,
            start_time + std::chrono::milliseconds(500) + kLateSlack);
}

// Tests that ThreadSignal events trigger the registered SignalFd wakeup
// callback in the event loop.
TEST_P(AioTest, ThreadSignalTest) {
  Aio aio;

  aos::ipc_lib::ThreadSignalReceiver sfd;

  bool signal_fired = false;
  aio.RegisterThreadSignalReceiver(&sfd,
                                   [&signal_fired]() { signal_fired = true; });

  const auto pid = aos::GetProcessId();
  const auto tid = aos::GetThreadId();

  std::thread signaler([pid, tid]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    aos::ipc_lib::ThreadSignalSender signaler_signal;
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
  aio.UnregisterThreadSignalReceiver(&sfd);
}

// Tests that a registered ThreadSignalReceiver callback is successfully invoked
// multiple times when multiple signals are sent sequentially, using multishot
// poll.
TEST_P(AioTest, MultiThreadSignalTest) {
  Aio aio;
  // A lost wakeup leaves the main thread blocked in Poll(true) with the
  // signaler waiting on signal_count -- turn that hang into a clean death.
  ScopedDeathTestWatchdog watchdog;

  aos::ipc_lib::ThreadSignalReceiver sfd;

  std::atomic<size_t> signal_count{0};
  std::vector<aos::monotonic_clock::time_point> callback_times;
  callback_times.reserve(3);
  aio.RegisterThreadSignalReceiver(&sfd, [&signal_count, &callback_times]() {
    callback_times.push_back(aos::monotonic_clock::now());
    ++signal_count;
  });

  const auto pid = aos::GetProcessId();
  const auto tid = aos::GetThreadId();

  auto start = aos::monotonic_clock::now();
  // Each send waits for its callback before the next: wakeups coalesce by
  // contract (see Aio::RegisterThreadSignalReceiver()), so free-running
  // sends could merge into fewer callbacks and a wait for exactly three
  // would hang instead of failing.  Serialized sends pin what this test is
  // actually about -- the multishot registration keeps delivering, one
  // callback per wakeup, without re-arming.
  std::atomic<bool> give_up{false};
  std::thread signaler([pid, tid, &signal_count, &give_up]() {
    for (size_t i = 0; i < 3; ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      aos::ipc_lib::ThreadSignalSender signaler_signal;
      signaler_signal.Signal(pid, tid);
      while (signal_count <= i && !give_up) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      if (give_up) {
        return;
      }
    }
  });

  {
    ScopedRealtime rt;

    // Poll until we receive all 3 signals.
    while (signal_count < 3 && aio.Poll(true)) {
    }
  }

  // Join before the first assertion.  An ASSERT_* returns from the test
  // body, and destroying a still-joinable std::thread is std::terminate();
  // give_up bounds the signaler's wait so the join cannot hang if a
  // callback never arrived.
  give_up = true;
  signaler.join();

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

  aio.UnregisterThreadSignalReceiver(&sfd);
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

// Contract test for aio.h's constraint 2: a canceled request only ever has
// to live until its callback runs.  The cancel's kernel acknowledgment
// deliberately does not name the request (it carries the loop-owned
// cancel_ack_sentinel_ identity -- see IoUringImpl::Cancel()), so freeing
// the request immediately after the Canceled callback and continuing to
// poll must be clean.  A regression here is a read of freed memory in
// DrainCompletions()'s ack handling, which needs ASAN to fail reliably.
TEST_P(AioTest, FreeCanceledRequestAfterCallback) {
  Aio aio;
  Pipe pipe;

  char buf[8];
  bool invoked = false;
  auto request = std::make_unique<AsyncRequest>();
  request->callback = [](Completion completion, void *ctx) {
    EXPECT_FALSE(aos::IsOk(completion.status));
    *static_cast<bool *>(ctx) = true;
  };
  request->context = &invoked;

  aio.AsyncRead(pipe.read_fd(), buf, request.get());
  aio.Cancel(request.get());
  while (!invoked && aio.Poll(true)) {
  }
  ASSERT_TRUE(invoked);

  // Free immediately after the callback -- the documented earliest legal
  // point -- then keep polling.  Any late traffic for the cancel must not
  // touch the freed request.
  request.reset();
  for (int i = 0; i < 10; ++i) {
    aio.Poll(false);
  }
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

// Tests that duplicate registrations for legacy fds and thread-signal receivers
// die.
TEST_P(AioTest, DuplicateRegistrationDeathTest) {
  Aio aio;
  Pipe pipe;

  aio.OnReadable(pipe.read_fd(), []() {});
  EXPECT_DEATH(aio.OnReadable(pipe.read_fd(), []() {}),
               "Duplicate in functions for");
  // A null callback is no exception -- the epoll backend used to accept it,
  // silently clearing the handler while leaving the events subscribed (which
  // the dispatch then dies on).  Unconditional everywhere, as EPoll had it.
  EXPECT_DEATH(aio.OnReadable(pipe.read_fd(), nullptr),
               "Duplicate in functions for");

  aio.OnWritable(pipe.write_fd(), []() {});
  EXPECT_DEATH(aio.OnWritable(pipe.write_fd(), []() {}),
               "Duplicate out functions for");

  aio.OnError(pipe.write_fd(), []() {});
  EXPECT_DEATH(aio.OnError(pipe.write_fd(), []() {}),
               "Duplicate error functions for");

  aos::ipc_lib::ThreadSignalReceiver sfd;
  aio.RegisterThreadSignalReceiver(&sfd, []() {});
  EXPECT_DEATH(aio.RegisterThreadSignalReceiver(&sfd, []() {}), "Duplicate.*");

  // Clean up registered resources before loop destruction.
  aio.DeleteFd(pipe.read_fd());
  aio.DeleteFd(pipe.write_fd());
  aio.UnregisterThreadSignalReceiver(&sfd);
}

// An enabled readiness event with no handler registered dies loudly at
// dispatch, as EPoll always has -- here EnableWritable() on an fd registered
// only OnReadable.  Silently skipping it instead would spin on the
// unconsumable level-triggered event.
TEST_P(AioTest, EnabledEventWithNoHandlerDeathTest) {
  Aio aio;
  Pipe pipe;

  aio.OnReadable(pipe.write_fd(), []() {});
  aio.EnableWritable(pipe.write_fd());

  // The write end of an empty pipe is immediately writable, so the first
  // few Poll()s deliver EPOLLOUT.  Bound the loop so a regression (silent
  // skip) fails the death test after 2s instead of hanging it.
  EXPECT_DEATH(
      {
        const auto stop = aos::monotonic_clock::now() + std::chrono::seconds(2);
        while (aos::monotonic_clock::now() < stop) {
          aio.Poll(false);
        }
      },
      "No handler registered for output events");

  aio.DeleteFd(pipe.write_fd());
}

// Tests that unregistering untracked legacy fds or thread-signal receivers
// dies.
TEST_P(AioTest, UntrackedUnregistrationDeathTest) {
  Aio aio;

  EXPECT_DEATH(aio.DeleteFd(999), "fd 999 not found");
  aos::ipc_lib::ThreadSignalReceiver sfd2;
  EXPECT_DEATH(aio.UnregisterThreadSignalReceiver(&sfd2),
               "(ThreadSignalReceiver not found|fd .* not found)");
}

// Tests that calling Poll() from inside a callback dies (constraint 3 in
// aio.h).
TEST_P(AioTest, NestedPollDeathTest) {
  ScopedDeathTestWatchdog watchdog;
  Aio aio;

  Aio::Timer timer(&aio);
  timer.Schedule(
      aos::monotonic_clock::now(),
      [](Completion, void *ctx) { static_cast<Aio *>(ctx)->Poll(false); },
      &aio);
  EXPECT_DEATH(
      {
        while (aio.Poll(true)) {
        }
      },
      "reentered");
}

// The other way into a nested Poll(), and equally fatal: not a callback, but
// a destructor the loop runs on its way out of one.  A callback that deletes
// its own fd parks the registration rather than freeing it mid-dispatch, and
// the parked state is freed at the end of that same Poll() -- which destroys
// the callback, which destroys its captures.
TEST_P(AioTest, NestedPollFromCaptureDestructorDeathTest) {
  ScopedDeathTestWatchdog watchdog;
  Aio aio;
  Pipe pipe;

  // Armed only once the callback has run, so the copies std::function makes
  // while the registration is being built don't poll on their way out.
  struct PollOnDestruction {
    ~PollOnDestruction() {
      if (*armed) {
        aio->Poll(false);
      }
    }
    Aio *aio;
    bool *armed;
  };

  // All of it inside the child.  The fork shares this process's epoll
  // instance, so a registration made out here and deleted in there leaves
  // nothing behind for the cleanup to remove.
  EXPECT_DEATH(
      {
        bool armed = false;
        aio.OnReadable(
            pipe.read_fd(),
            [&aio, &pipe, &armed, guard = PollOnDestruction{&aio, &armed}]() {
              armed = true;
              aio.DeleteFd(pipe.read_fd());
            });
        pipe.Write("a");
        MarkFatalStatement();
        while (aio.Poll(true)) {
        }
      },
      DiesAfterMarker("reentered"));
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

  // Mixing AsyncRead/AsyncWrite and OnReadable/OnWritable/OnEvents on one
  // fd fails on every backend: only epoll structurally needs the ban, but
  // enforcing it uniformly keeps io_uring code from relying on mixing.
  AsyncRequest req;
  char buf[10];

  // AsyncRead, then OnReadable fails.
  // The raw request has to be submitted inside the death test: forking with
  // one in flight is itself fatal now (ForkedChildWithPendingAsyncWriteDies),
  // and EXPECT_DEATH forks.  The child does both halves and dies on the
  // second, which is what this is checking either way.
  EXPECT_DEATH(
      {
        aio.AsyncRead(pipe.read_fd(), buf, &req);
        MarkFatalStatement();
        aio.OnReadable(pipe.read_fd(), []() {});
      },
      DiesAfterMarker("Cannot mix OnReadable and AsyncRead"));

  // OnReadable, then AsyncRead fails.
  aio.OnReadable(pipe.read_fd(), []() {});
  EXPECT_DEATH(aio.AsyncRead(pipe.read_fd(), buf, &req),
               "Cannot mix OnReadable and AsyncRead");
  aio.DeleteFd(pipe.read_fd());

  // AsyncWrite, then OnWritable fails.  Submitted inside, as above.
  EXPECT_DEATH(
      {
        aio.AsyncWrite(pipe.write_fd(), buf, &req);
        MarkFatalStatement();
        aio.OnWritable(pipe.write_fd(), []() {});
      },
      DiesAfterMarker("Cannot mix OnWritable and AsyncWrite"));

  // OnWritable, then AsyncWrite fails.
  aio.OnWritable(pipe.write_fd(), []() {});
  EXPECT_DEATH(aio.AsyncWrite(pipe.write_fd(), buf, &req),
               "Cannot mix OnWritable and AsyncWrite");
  aio.DeleteFd(pipe.write_fd());

  // AsyncRead, then OnEvents fails.  Submitted inside, as above.
  EXPECT_DEATH(
      {
        aio.AsyncRead(pipe.read_fd(), buf, &req);
        MarkFatalStatement();
        aio.OnEvents(pipe.read_fd(), [](uint32_t) {});
      },
      DiesAfterMarker("Cannot mix OnEvents and AsyncRead/AsyncWrite"));

  // OnEvents, then AsyncRead fails.
  aio.OnEvents(pipe.read_fd(), [](uint32_t) {});
  EXPECT_DEATH(aio.AsyncRead(pipe.read_fd(), buf, &req),
               "Cannot mix OnEvents and AsyncRead/AsyncWrite");
  aio.DeleteFd(pipe.read_fd());
}

// A stale fd -- one another component already closed -- must come back as an
// error completion carrying EBADF, not take the process down.  aio.h states
// operational failure as "status is an error, result is the positive errno",
// and io_uring delivers that natively because the fd is only validated when
// the SQE runs.  epoll validates at epoll_ctl(ADD) time, where the obvious
// spelling is a PCHECK -- and epoll is the compiled-in default on roborio and
// linux_arm64, so that spelling turns someone else's stale descriptor into a
// dead robot.
//
// FailedIoErrorTest above covers fd = -1, which takes a separate graceful
// path and never reaches epoll_ctl.
TEST_P(AioTest, ClosedFdAsyncReadReportsErrorTest) {
  Aio aio;

  // Built through Pipe so the closed-descriptor value is spelled the same on
  // every platform -- pipe(2) and close(2) are POSIX-only, and a
  // FileDescriptor is an opaque HANDLE off POSIX.
  FileDescriptor stale;
  {
    Pipe pipe;
    stale = pipe.read_fd();
    pipe.close_read_fd();
    pipe.close_write_fd();
  }

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
  aio.AsyncRead(stale, buf, &read_req);
  while (!read_req.done && aio.Poll(true)) {
  }

  ASSERT_TRUE(fired_status.has_value());
  EXPECT_FALSE(aos::IsOk(*fired_status));
  EXPECT_EQ(result_code, EBADF);
}

// Legacy handlers and raw requests are exclusive on an fd in both
// directions.  The same-direction pairings were always rejected; the
// cross-direction one (a legacy read handler alongside a pending AsyncWrite)
// used to be legal, and was the shape that let a cancelled sibling request
// strand a half-dispatched batch.
TEST_P(AioTest, LegacyAndAsyncAreExclusiveDeathTest) {
  ScopedDeathTestWatchdog watchdog;
  EXPECT_DEATH(
      {
        Aio aio;
        Pipe pipe;
        AsyncRequest write_req;
        write_req.callback = [](Completion, void *) {};
        char out[1] = {'x'};
        aio.OnReadable(pipe.read_fd(), []() {});
        MarkFatalStatement();
        aio.AsyncWrite(pipe.read_fd(), std::span<const char>(out, 1),
                       &write_req);
      },
      DiesAfterMarker("Cannot mix"));
}

// The pool has to hold as many concurrent raw-request fds as it says it
// does.  It used not to: the loop's own wakeup was a persistent AsyncRead
// that re-armed itself from its own completion, which runs inside dispatch,
// where the registration it had just retired is parked on retired_ and
// cannot be scrubbed.  The re-arm therefore needed a *fresh* slot, so a pool
// of N supported N-1 caller fds, and the Nth turned the next wakeup -- Quit()
// from ShmEventLoop's signal handler, say -- into "Async registration pool
// exhausted" during shutdown.
//
// Filling the pool exactly and then waking the loop is the shape that used
// to abort.
TEST_P(AioTest, WakeupDoesNotConsumePoolSlotTest) {
  // Fills the registration pool exactly, which is the shape that used to
  // abort.  Set rather than assumed, so it stays exact if the default moves.
  constexpr int kFds = 16;
  ::absl::SetFlag(&FLAGS_aio_pool_size, kFds);
  Aio aio;

  // Registrations the loop makes that are not raw requests -- a timer, a
  // legacy On*() handler -- must not come out of the pool either.  On epoll
  // they did: each one used a slot for good, so the 16th raw read below
  // fell through to the heap.
  Aio::Timer timer(&aio);
  Pipe legacy_pipe;
  aio.OnReadable(legacy_pipe.read_fd(), []() {});

  std::array<Pipe, kFds> pipes;
  std::array<AsyncRequest, kFds> reqs;
  std::array<char[8], kFds> bufs;
  int completed = 0;
  for (int i = 0; i < kFds; ++i) {
    reqs[i].callback = [](Completion completion, void *ctx) {
      EXPECT_TRUE(aos::IsOk(completion.status));
      ++*static_cast<int *>(ctx);
    };
    reqs[i].context = &completed;
  }
  {
    // Under realtime, so a request that has to allocate a registration dies
    // on the malloc hook instead of quietly falling back to the heap.
    ScopedRealtime rt;
    for (int i = 0; i < kFds; ++i) {
      aio.AsyncRead(pipes[i].read_fd(), bufs[i], &reqs[i]);
    }
  }

  // Every pool slot is now spoken for.  A wakeup here used to need one more.
  aio.Quit();
  aio.Run();

  for (int i = 0; i < kFds; ++i) {
    pipes[i].Write("x");
  }
  {
    ScopedRealtime rt;
    while (completed < kFds && aio.Poll(true)) {
    }
  }
  EXPECT_EQ(completed, kFds);

  aio.DeleteFd(legacy_pipe.read_fd());
}

// The ordinary read loop -- a completion callback that re-arms its own fd --
// has to fit in the pool too.  On epoll it did not: a completed read's
// registration was parked until the end of the Poll(), like a legacy one
// whose std::function might be executing, so the re-arm needed a second
// slot.  With every slot in flight, the first re-arm allocated.  A raw
// registration holds no std::function, so it goes straight back to the pool.
TEST_P(AioTest, RearmFromCallbackReusesPoolSlotTest) {
  // Fills the registration pool exactly, as above.
  constexpr int kFds = 16;
  constexpr int kRounds = 3;
  ::absl::SetFlag(&FLAGS_aio_pool_size, kFds);
  Aio aio;

  struct Reader {
    Aio *aio;
    Pipe pipe;
    AsyncRequest req;
    char buf[8];
    int reads = 0;
  };
  std::array<Reader, kFds> readers;
  for (Reader &reader : readers) {
    reader.aio = &aio;
    reader.req.context = &reader;
    reader.req.callback = [](Completion completion, void *ctx) {
      EXPECT_TRUE(aos::IsOk(completion.status));
      Reader *reader = static_cast<Reader *>(ctx);
      ++reader->reads;
      if (reader->reads < kRounds) {
        reader->aio->AsyncRead(reader->pipe.read_fd(), reader->buf,
                               &reader->req);
      }
    };
  }

  const auto total_reads = [&readers]() {
    int total = 0;
    for (const Reader &reader : readers) {
      total += reader.reads;
    }
    return total;
  };

  {
    // Under realtime throughout, so a re-arm that has to allocate a
    // registration dies on the malloc hook instead of falling back to the
    // heap.
    ScopedRealtime rt;
    for (Reader &reader : readers) {
      aio.AsyncRead(reader.pipe.read_fd(), reader.buf, &reader.req);
    }
    for (int round = 1; round <= kRounds; ++round) {
      // Every fd readable at once, so one harvest completes them all and
      // every callback re-arms with the whole pool otherwise in flight.
      for (Reader &reader : readers) {
        reader.pipe.Write("x");
      }
      while (total_reads() < round * kFds && aio.Poll(true)) {
      }
      ASSERT_EQ(total_reads(), round * kFds);
    }
  }
}

// A request with no callback is finished once `done` is set: there is
// nothing left to deliver, so the caller may free it right away.  epoll
// used to leave it queued for delivery anyway, and the next Poll() popped
// the freed request.  Covers each way a request resolves: I/O, a submit
// error, and a Cancel().
//
// A request with a callback goes first, so that the Poll() which resolves
// the callbackless ones spends its one delivery on it and returns with them
// still queued behind it.
TEST_P(AioTest, CallbacklessRequestMayBeFreedOnceDoneTest) {
  Aio aio;
  Pipe ready_pipe;
  Pipe idle_pipe;
  ready_pipe.Write("x");

  int delivered = 0;
  AsyncRequest with_callback;
  with_callback.callback = [](Completion, void *ctx) {
    ++*static_cast<int *>(ctx);
  };
  with_callback.context = &delivered;
  char with_callback_buf[8];
  aio.AsyncRead(-1, with_callback_buf, &with_callback);

  auto read = std::make_unique<AsyncRequest>();
  auto failed = std::make_unique<AsyncRequest>();
  auto canceled = std::make_unique<AsyncRequest>();
  char buf[8];
  char failed_buf[8];
  char canceled_buf[8];
  aio.AsyncRead(ready_pipe.read_fd(), buf, read.get());
  aio.AsyncRead(-1, failed_buf, failed.get());
  aio.AsyncRead(idle_pipe.read_fd(), canceled_buf, canceled.get());
  aio.Cancel(canceled.get());

  while (!(read->done && failed->done && canceled->done) && aio.Poll(true)) {
  }
  ASSERT_TRUE(read->done);
  ASSERT_TRUE(failed->done);
  ASSERT_TRUE(canceled->done);
  read.reset();
  failed.reset();
  canceled.reset();

  // Nothing may still point at them.
  while (aio.Poll(false)) {
  }
  EXPECT_EQ(delivered, 1);
}

// DeleteFd() undoes an On*() registration.  An fd carrying only a raw
// AsyncRead has none, so deleting it is a caller error and has to say so.
//
// It used to differ by backend for the same sequence: io_uring looks the fd
// up in its separate legacy table, misses, and dies with "fd not found",
// while the readiness backends found the async-only registration in their
// one unified table and released it -- Detach()ing the request without ever
// completing it.  req.done stayed false forever, and aio.h's constraint 2
// then forbids the caller from freeing or reusing it.  A silent, permanent
// leak against a loud crash.
//
// Cancel() is how a caller retires a raw request; ForgetClosedFd() is the
// same story and rejects it the same way.
TEST_P(AioTest, DeleteFdWithPendingAsyncReadDeathTest) {
  ScopedDeathTestWatchdog watchdog;
  EXPECT_DEATH(
      {
        Aio aio;
        Pipe pipe;
        AsyncRequest read_req;
        read_req.callback = [](Completion, void *) {};
        char buf[8];
        aio.AsyncRead(pipe.read_fd(), buf, &read_req);
        MarkFatalStatement();
        aio.DeleteFd(pipe.read_fd());
      },
      DiesAfterMarker("not found"));
}

// aio.h's constraint 2 permits destroying an Aio with raw requests still
// pending, so a request can outlive the instance that armed it.  Handing it
// to a second Aio then has to work.
//
// It did not on io_uring: the leftover raw_io made AsyncRead() skip
// ClaimRawFd() while still arming the SQE, so the terminal completion's
// UnlinkRawRequest() ran against a list the request was never on -- a CHECK
// abort when raw_prev happened to be null, and writes through dangling
// pointers into the dead instance's requests when it was not.
//
// ~IoUringImpl cannot scrub this on the way out: the requests are
// caller-owned and may already be gone, which is why it deliberately does
// not walk its lists.  The staleness is resolved at the next arm instead,
// the same way link.queued is.
TEST_P(AioTest, RawRequestReusedOnASecondAioTest) {
  AsyncRequest read_req;
  int completions = 0;
  read_req.callback = [](Completion, void *ctx) { ++*static_cast<int *>(ctx); };
  read_req.context = &completions;
  char buf[8];

  // A pipe each: the AsyncRequest is what gets reused here, and the
  // descriptor is incidental to that.  Reusing one would also make this
  // untestable on Windows, where a socket's completion-port association is
  // permanent -- the second Aio's CreateIoCompletionPort would fail, its
  // completions would keep going to the first (closed) port, and Poll()
  // would block forever.  Both pipes outlive both instances, so the first
  // request is still armed on a live fd when its Aio goes away.
  Pipe first_pipe;
  Pipe second_pipe;
  {
    // Armed and never drained: no write, so the read stays pending, and the
    // Aio goes away underneath it.
    Aio first;
    first.AsyncRead(first_pipe.read_fd(), buf, &read_req);
    first.Poll(false);
  }

  Aio second;
  second.AsyncRead(second_pipe.read_fd(), buf, &read_req);
  second_pipe.Write("y");
  const auto deadline = aos::monotonic_clock::now() + std::chrono::seconds(2);
  while (completions == 0 && aos::monotonic_clock::now() < deadline) {
    second.Poll(false);
  }
  EXPECT_EQ(completions, 1);
}

// The other half of the same rule: a request may only be armed once at a
// time.  RawRequestReusedOnASecondAioTest above covers the *legal* case --
// state left over from an Aio that was destroyed underneath the request --
// and this covers the illegal one, which used to be silent and corrupt
// differently on each backend.  io_uring skipped ClaimRawFd() while still
// arming the SQE, so the terminal completion unlinked from a list the request
// was never on.  epoll and IOCP left two registrations pointing at one
// request: whichever completed first ran the callback, and the other kept a
// pointer to a request the caller was by then free to reuse.
//
// Both cases carry done == false, so telling them apart means asking the loop
// what it currently has armed rather than trusting the request.
//
// This lives here rather than with the check itself because it needs a forked
// child to have its own loop: EXPECT_DEATH forks, and before this change an
// epoll instance -- an open file description -- was shared with the child, so
// the child's epoll_ctl(ADD) landed in the interest list the parent was still
// using.
TEST_P(AioTest, DoubleSubmitDeathTest) {
  Aio aio;
  Pipe first;
  Pipe second;
  AsyncRequest read_req;
  char buf[8];

  // Both submits happen inside the child: a raw request in flight across a
  // fork is separately fatal (CheckNoRawRequestsInFlightOnFork()), so arming
  // in the parent would trip that check instead of the one under test.
  //
  // A different fd, which the per-fd duplicate check cannot see.
  EXPECT_DEATH(
      {
        aio.AsyncRead(first.read_fd(), buf, &read_req);
        MarkFatalStatement();
        aio.AsyncRead(second.read_fd(), buf, &read_req);
      },
      DiesAfterMarker("still in flight"));
  // The other direction, on a different fd.
  EXPECT_DEATH(
      {
        aio.AsyncRead(first.read_fd(), buf, &read_req);
        MarkFatalStatement();
        aio.AsyncWrite(second.write_fd(), buf, &read_req);
      },
      DiesAfterMarker("still in flight"));
  // And the same fd, which reaches this check before the per-fd one.
  EXPECT_DEATH(
      {
        aio.AsyncRead(first.read_fd(), buf, &read_req);
        MarkFatalStatement();
        aio.AsyncRead(first.read_fd(), buf, &read_req);
      },
      DiesAfterMarker("still in flight"));
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
  // Each backend names itself in its operational-failure message.
  EXPECT_EQ(fired_status->error().message(), GetParam() + " error");
  EXPECT_EQ(result_code, EBADF);
}

// Completions queued inside the loop are delivered in the order they were
// resolved, on every backend.  io_uring gets this from the CQ, which is a
// queue; the readiness backends used a stack for their pending lists and
// delivered backwards, which a caller driving two requests can see.
//
// Two reads on a closed descriptor fail without waiting (see
// FailedIoErrorTest), so both are queued before either is dispatched, which
// is what makes the order observable at all.
TEST_P(AioTest, QueuedCompletionsDeliverInSubmissionOrder) {
  Aio aio;

  std::vector<int> order;
  AsyncRequest first;
  AsyncRequest second;
  first.user_data = reinterpret_cast<void *>(1);
  second.user_data = reinterpret_cast<void *>(2);
  const auto record = [](Completion completion, void *context) {
    static_cast<std::vector<int> *>(context)->push_back(
        static_cast<int>(reinterpret_cast<intptr_t>(completion.user_data)));
  };
  first.callback = record;
  second.callback = record;
  first.context = &order;
  second.context = &order;

  char buf[8];
  aio.AsyncRead(-1, buf, &first);
  aio.AsyncRead(-1, buf, &second);

  // Driven off the callbacks, not `done`: io_uring marks a request done when
  // it drains the CQE and dispatches at most one callback per Poll(), so both
  // flags flip on the first pass while only one callback has run.
  while (order.size() < 2 && aio.Poll(true)) {
  }

  EXPECT_EQ(order, (std::vector<int>{1, 2}))
      << "queued completions came back out of order";
}

// A hangup with nothing else to report -- an empty pipe whose writer closed
// -- reaches the read handler, which sees EOF.  EPoll dropped it, and the
// kernel keeps reporting it while the fd is registered, so Poll() spun with
// nothing ever told and a Run() draining after Quit() never returned.
TEST_P(AioTest, BareHangupReachesReadHandlerTest) {
  Aio aio;
  Pipe pipe;

  int reads = 0;
  ssize_t last_read = -1;
  aio.OnReadable(pipe.read_fd(), [&]() {
    ++reads;
    char buf[8];
    last_read = read(pipe.read_fd(), buf, sizeof(buf));
    aio.DeleteFd(pipe.read_fd());
  });
  pipe.close_write_fd();

  // Bounded: the bug is a Poll() that keeps reporting progress without ever
  // calling anything.
  for (int i = 0; i < 100 && reads == 0; ++i) {
    aio.Poll(false);
  }
  EXPECT_EQ(reads, 1);
  EXPECT_EQ(last_read, 0) << "the handler should have seen EOF";
  if (reads == 0) {
    aio.DeleteFd(pipe.read_fd());
  }
}

// ...and to the error handler instead, when there is one.
TEST_P(AioTest, BareHangupPrefersErrorHandlerTest) {
  Aio aio;
  Pipe pipe;

  int errors = 0;
  aio.OnReadable(pipe.read_fd(), []() { FAIL() << "read handler ran"; });
  aio.OnError(pipe.read_fd(), [&]() {
    ++errors;
    aio.DeleteFd(pipe.read_fd());
  });
  pipe.close_write_fd();

  for (int i = 0; i < 100 && errors == 0; ++i) {
    aio.Poll(false);
  }
  EXPECT_EQ(errors, 1);
  if (errors == 0) {
    aio.DeleteFd(pipe.read_fd());
  }
}

// POSIX-only: SIGPIPE, and write(2) on a pipe's descriptor.
#ifndef _WIN32
// An error with no error handler reaches the subscribed handler, as a bare
// hangup does.  A full pipe whose reader closed reports EPOLLERR alone on its
// write end, so an OnWritable()-only registration used to die on the
// error-handler CHECK instead of running the write that sees EPIPE.
TEST_P(AioTest, LoneErrorReachesWriteHandlerTest) {
  // The write below is meant to fail with EPIPE, not kill the test.
  struct sigaction ignore = {};
  ignore.sa_handler = SIG_IGN;
  struct sigaction old_action;
  ABSL_PCHECK(sigaction(SIGPIPE, &ignore, &old_action) == 0);

  Aio aio;
  Pipe pipe;
  std::vector<char> filler(1 << 16, 'x');
  while (write(pipe.write_fd(), filler.data(), filler.size()) > 0) {
  }

  int writes = 0;
  int write_errno = 0;
  aio.OnWritable(pipe.write_fd(), [&]() {
    ++writes;
    if (write(pipe.write_fd(), "x", 1) < 0) {
      write_errno = errno;
    }
    aio.DeleteFd(pipe.write_fd());
  });
  pipe.close_read_fd();

  for (int i = 0; i < 100 && writes == 0; ++i) {
    aio.Poll(false);
  }
  EXPECT_EQ(writes, 1);
  EXPECT_EQ(write_errno, EPIPE);
  if (writes == 0) {
    aio.DeleteFd(pipe.write_fd());
  }
  ABSL_PCHECK(sigaction(SIGPIPE, &old_action, nullptr) == 0);
}
#endif

// The ways a request can resolve without waiting for its fd: a submit-time
// error, a Cancel() of a request that was waiting, a zero-length read, a read
// with data already there, and a write with room.  They resolve in the order
// aio.h gives for one Poll()'s batch: everything but the cancels in
// submission order, then the cancels in theirs.  That is the order io_uring's
// kernel posts the batch's completions in on a SINGLE_ISSUER ring: the
// immediate ones inline while it submits, a canceled request's from task work
// once the submit is done.  IOCP has not been run against this.
//
// The readiness backends broke this ordering twice.  They kept cancels and
// submit-time errors on two lists and delivered every cancel first, and then
// delivered every submit-time error ahead of any I/O they had done.  Every
// ordered pair is covered, a kind followed by itself included, so a backend
// that sorts by kind the wrong way, or reverses one kind, fails some pair.
enum class ImmediateKind {
  kSubmitError,
  kCancel,
  kZeroLengthRead,
  kReadyRead,
  kWriteWithRoom,
};

const char *ImmediateKindName(ImmediateKind kind) {
  switch (kind) {
    case ImmediateKind::kSubmitError:
      return "submit error";
    case ImmediateKind::kCancel:
      return "cancel";
    case ImmediateKind::kZeroLengthRead:
      return "zero-length read";
    case ImmediateKind::kReadyRead:
      return "read with data ready";
    case ImmediateKind::kWriteWithRoom:
      return "write with room";
  }
  return "?";
}

// One request of the given kind, on a pipe of its own so that no two share
// an fd: the readiness backends hold one request per direction per fd.
struct ImmediateRequest {
  ImmediateRequest(ImmediateKind kind, int id, std::vector<int> *order)
      : kind(kind) {
    req.user_data = reinterpret_cast<void *>(static_cast<intptr_t>(id));
    req.context = order;
    req.callback = [](Completion completion, void *context) {
      static_cast<std::vector<int> *>(context)->push_back(
          static_cast<int>(reinterpret_cast<intptr_t>(completion.user_data)));
    };
    if (kind == ImmediateKind::kReadyRead) {
      pipe.Write("x");
    }
  }

  void Submit(Aio *aio) {
    switch (kind) {
      case ImmediateKind::kSubmitError:
        aio->AsyncRead(-1, buf, &req);
        break;
      case ImmediateKind::kCancel:
        aio->AsyncRead(pipe.read_fd(), buf, &req);
        aio->Cancel(&req);
        break;
      case ImmediateKind::kZeroLengthRead:
        aio->AsyncRead(pipe.read_fd(), std::span<char>(buf, 0), &req);
        break;
      case ImmediateKind::kReadyRead:
        aio->AsyncRead(pipe.read_fd(), buf, &req);
        break;
      case ImmediateKind::kWriteWithRoom:
        aio->AsyncWrite(pipe.write_fd(), std::span<const char>("x", 1), &req);
        break;
    }
  }

  ImmediateKind kind;
  Pipe pipe;
  AsyncRequest req;
  char buf[8];
};

TEST_P(AioTest, ImmediateCompletionsDeliverInSubmissionOrder) {
  constexpr std::array<ImmediateKind, 5> kKinds = {
      ImmediateKind::kSubmitError, ImmediateKind::kCancel,
      ImmediateKind::kZeroLengthRead, ImmediateKind::kReadyRead,
      ImmediateKind::kWriteWithRoom};
  for (const ImmediateKind first_kind : kKinds) {
    for (const ImmediateKind second_kind : kKinds) {
      SCOPED_TRACE(std::string(ImmediateKindName(first_kind)) + " then " +
                   ImmediateKindName(second_kind));
      Aio aio;
      std::vector<int> order;
      ImmediateRequest first(first_kind, 1, &order);
      ImmediateRequest second(second_kind, 2, &order);
      first.Submit(&aio);
      second.Submit(&aio);
      while (order.size() < 2 && aio.Poll(true)) {
      }
      // Submission order, except that a cancel goes after a request that is
      // not one.
      const bool first_goes_last = first_kind == ImmediateKind::kCancel &&
                                   second_kind != ImmediateKind::kCancel;
      const std::vector<int> expected =
          first_goes_last ? std::vector<int>{2, 1} : std::vector<int>{1, 2};
      EXPECT_EQ(order, expected);
    }
  }
}

// A zero-length read or write has nothing to wait for and completes at once,
// with a result of 0, even on an fd that is not ready -- an empty pipe's read
// end.  The readiness backends used to wait for readiness that might never
// come, so a computed empty write hung on them alone.
TEST_P(AioTest, ZeroLengthCompletesImmediately) {
  for (const bool is_read : {true, false}) {
    SCOPED_TRACE(is_read ? "read" : "write");
    Aio aio;
    Pipe pipe;
    if (!is_read) {
      // Fill the pipe, so the write end is not writable either.
      const std::string chunk(4096, 'x');
      while (write(pipe.write_fd(), chunk.data(), chunk.size()) > 0) {
      }
      ABSL_CHECK_EQ(errno, EAGAIN);
    }

    std::optional<Completion> completion;
    AsyncRequest req;
    req.callback = [](Completion c, void *ctx) {
      static_cast<std::optional<Completion> *>(ctx)->emplace(std::move(c));
    };
    req.context = &completion;
    char buf[1];
    if (is_read) {
      aio.AsyncRead(pipe.read_fd(), std::span<char>(buf, 0), &req);
    } else {
      aio.AsyncWrite(pipe.write_fd(), std::span<const char>(buf, 0), &req);
    }
    // Never blocking: a zero-length request that waits for readiness is
    // the bug, and would hang a Poll(true) here.
    for (int i = 0; i < 10 && !completion.has_value(); ++i) {
      aio.Poll(false);
    }
    ASSERT_TRUE(completion.has_value());
    EXPECT_TRUE(aos::IsOk(completion->status));
    EXPECT_EQ(completion->result, 0);
  }
}

// Arming a request is not processing anything.  A first attempt that finds
// nothing to read leaves the request waiting on the kernel, and Poll(false)
// reports that as no progress, as io_uring does.  The Poll(true) that follows
// then waits for the data and delivers it in that same call, so a loop that
// re-arms from its callback takes one Poll() per read.
TEST_P(AioTest, ArmingAloneIsNotProgress) {
  Aio aio;
  Pipe pipe;

  bool fired = false;
  AsyncRequest req;
  req.callback = [](Completion completion, void *ctx) {
    EXPECT_TRUE(aos::IsOk(completion.status));
    *static_cast<bool *>(ctx) = true;
  };
  req.context = &fired;
  char buf[8];
  aio.AsyncRead(pipe.read_fd(), buf, &req);
  EXPECT_FALSE(aio.Poll(false));
  EXPECT_FALSE(req.done);

  pipe.Write("x");
  EXPECT_TRUE(aio.Poll(true));
  EXPECT_TRUE(fired);
}

// POSIX-only: fills and drains the pipe with write(2)/read(2) on its
// descriptors.
#ifndef _WIN32
// A raw write to a full pipe waits for room: its first attempt gets EAGAIN,
// and only the fd becoming writable completes it.  Everything else that
// writes resolves at its first attempt, so this is the test that the wait
// for writability is actually armed.
TEST_P(AioTest, RawWriteWaitsForWritable) {
  Aio aio;
  Pipe pipe;
  const std::string chunk(4096, 'x');
  size_t filled = 0;
  while (write(pipe.write_fd(), chunk.data(), chunk.size()) > 0) {
    filled += chunk.size();
  }
  ABSL_PCHECK(errno == EAGAIN);

  std::optional<Completion> completion;
  AsyncRequest req;
  req.callback = [](Completion c, void *ctx) {
    static_cast<std::optional<Completion> *>(ctx)->emplace(std::move(c));
  };
  req.context = &completion;
  const char data[1] = {'y'};
  aio.AsyncWrite(pipe.write_fd(), data, &req);
  for (int i = 0; i < 5; ++i) {
    aio.Poll(false);
  }
  ASSERT_FALSE(completion.has_value()) << "the write did not wait for room";

  // Make room.  Never Poll(true) below: a write that is not waiting on
  // writability would hang it, rather than fail.
  ASSERT_EQ(pipe.Read(filled).size(), filled);
  for (int i = 0; i < 2000 && !completion.has_value(); ++i) {
    aio.Poll(false);
    if (!completion.has_value()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  ASSERT_TRUE(completion.has_value()) << "room never completed the write";
  EXPECT_TRUE(aos::IsOk(completion->status));
  EXPECT_EQ(completion->result, 1);
}
#endif

// POSIX-only: socketpair(2) and a datagram socket.
#ifndef _WIN32
// A zero-length write reaches the file, as io_uring's does: on a datagram
// socket it sends an empty datagram.  preadv2()/pwritev2() return 0 for an
// empty buffer without calling into the file at all, so the readiness
// backends used to send nothing.
TEST_P(AioTest, ZeroLengthWriteSendsEmptyDatagram) {
  int fds[2];
  ABSL_PCHECK(socketpair(AF_UNIX, SOCK_DGRAM, 0, fds) == 0);
  Aio aio;

  std::optional<Completion> completion;
  AsyncRequest req;
  req.callback = [](Completion c, void *ctx) {
    static_cast<std::optional<Completion> *>(ctx)->emplace(std::move(c));
  };
  req.context = &completion;
  char buf[1];
  aio.AsyncWrite(fds[0], std::span<const char>(buf, 0), &req);
  while (!completion.has_value() && aio.Poll(true)) {
  }
  ASSERT_TRUE(completion.has_value());
  EXPECT_TRUE(aos::IsOk(completion->status));
  EXPECT_EQ(completion->result, 0);

  // An empty datagram is there to receive: 0, not EAGAIN.
  EXPECT_EQ(recv(fds[1], buf, sizeof(buf), MSG_DONTWAIT), 0)
      << "no datagram was sent";

  ABSL_PCHECK(close(fds[0]) == 0);
  ABSL_PCHECK(close(fds[1]) == 0);
}
#endif

// Linux-only: inotify and eventfd are Linux interfaces.
#if defined(__linux__)
// A zero-length read never puts the loop thread to sleep, even on an fd whose
// own read(2) would.  A blocking inotify fd sleeps in read(2) until an event
// arrives, whatever the length, and a readiness backend that handed it the
// empty read hung Poll().  The readiness backends now complete it with 0 at
// once.  io_uring hands it to an io-wq worker, which is the one that waits
// for an event, so there it may still be pending; that is fine, because the
// loop thread is not the one waiting.
TEST_P(AioTest, ZeroLengthReadOnInotifyDoesNotBlock) {
  const int fd = inotify_init1(IN_CLOEXEC);
  ABSL_PCHECK(fd >= 0);
  Aio aio;

  std::optional<Completion> completion;
  AsyncRequest req;
  req.callback = [](Completion c, void *ctx) {
    static_cast<std::optional<Completion> *>(ctx)->emplace(std::move(c));
  };
  req.context = &completion;
  char buf[1];
  aio.AsyncRead(fd, std::span<char>(buf, 0), &req);
  // Non-blocking polls against a deadline: io_uring may finish it on an
  // io-wq worker, and a backend that blocks in the read hangs the first one.
  const auto deadline =
      aos::monotonic_clock::now() + std::chrono::milliseconds(200);
  while (!completion.has_value() && aos::monotonic_clock::now() < deadline) {
    aio.Poll(false);
  }
  if (completion.has_value()) {
    EXPECT_TRUE(aos::IsOk(completion->status));
    EXPECT_EQ(completion->result, 0);
  } else {
    ASSERT_TRUE(IsIoUring())
        << "a readiness backend left an empty read pending";
    aio.Cancel(&req);
    while (!completion.has_value() && aio.Poll(true)) {
    }
  }

  ABSL_PCHECK(close(fd) == 0);
}

// A zero-length write completes with 0 without the file's own rules applying.
// An eventfd refuses a write(2) shorter than 8 bytes with EINVAL; io_uring
// never hands it the empty write and completes it Ok.
TEST_P(AioTest, ZeroLengthWriteToEventfdCompletes) {
  const int fd = eventfd(0, EFD_CLOEXEC);
  ABSL_PCHECK(fd >= 0);
  Aio aio;

  std::optional<Completion> completion;
  AsyncRequest req;
  req.callback = [](Completion c, void *ctx) {
    static_cast<std::optional<Completion> *>(ctx)->emplace(std::move(c));
  };
  req.context = &completion;
  char buf[1];
  aio.AsyncWrite(fd, std::span<const char>(buf, 0), &req);
  // Non-blocking polls against a deadline: io_uring may finish it on an
  // io-wq worker, and a backend that blocks in the read hangs the first one.
  const auto deadline = aos::monotonic_clock::now() + std::chrono::seconds(2);
  while (!completion.has_value() && aos::monotonic_clock::now() < deadline) {
    aio.Poll(false);
  }
  ASSERT_TRUE(completion.has_value());
  EXPECT_TRUE(aos::IsOk(completion->status)) << completion->result;
  EXPECT_EQ(completion->result, 0);

  ABSL_PCHECK(close(fd) == 0);
}

// A zero-length read of an eventfd is where the backends cannot agree, so this
// pins what each one does rather than a shared answer.  io_uring hands the
// empty read to eventfd's read_iter, which refuses anything shorter than 8
// bytes with EINVAL.  The readiness backends cannot tell which files take an
// empty read (inotify does not, and would block), so they never hand one over
// and complete it with 0.  See aio.h's list of the readiness backends' gaps.
TEST_P(AioTest, ZeroLengthReadOnEventfd) {
  const int fd = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
  ABSL_PCHECK(fd >= 0);
  Aio aio;

  std::optional<Completion> completion;
  AsyncRequest req;
  req.callback = [](Completion c, void *ctx) {
    static_cast<std::optional<Completion> *>(ctx)->emplace(std::move(c));
  };
  req.context = &completion;
  char buf[1];
  aio.AsyncRead(fd, std::span<char>(buf, 0), &req);
  const auto deadline = aos::monotonic_clock::now() + std::chrono::seconds(2);
  while (!completion.has_value() && aos::monotonic_clock::now() < deadline) {
    aio.Poll(false);
  }
  ASSERT_TRUE(completion.has_value());
  if (IsIoUring()) {
    EXPECT_FALSE(aos::IsOk(completion->status));
    EXPECT_EQ(completion->result, EINVAL);
  } else {
    EXPECT_TRUE(aos::IsOk(completion->status)) << completion->result;
    EXPECT_EQ(completion->result, 0);
  }

  ABSL_PCHECK(close(fd) == 0);
}
#endif  // defined(__linux__)

// `done` can be set before the callback runs (see AsyncRequest::done), and a
// `while (!req.done && Poll(true))` loop then exits with the callback still
// queued.  Re-arming the request there is fatal on every backend.  io_uring
// used to take it, relink the queued node, run the callback with the new
// result, and die on the next Poll() with "removing a node that is not on
// this list".
TEST_P(AioTest, RearmBeforeCallbackDeathTest) {
  ScopedDeathTestWatchdog watchdog;
  EXPECT_DEATH(
      {
        Aio aio;
        Pipe pipe;
        pipe.Write("ab");
        AsyncRequest first;
        AsyncRequest second;
        int delivered = 0;
        const auto count = [](Completion, void *ctx) {
          ++*static_cast<int *>(ctx);
        };
        first.callback = count;
        first.context = &delivered;
        second.callback = count;
        second.context = &delivered;
        char buf1[1];
        char buf2[1];
        aio.AsyncRead(pipe.read_fd(), buf1, &first);
        // A second fd, so the readiness backends take both at once too.
        Pipe other;
        other.Write("c");
        aio.AsyncRead(other.read_fd(), buf2, &second);
        while (!second.done && aio.Poll(true)) {
        }
        // Both resolved in one Poll(), which delivered only the first.
        ABSL_CHECK_EQ(delivered, 1);
        MarkFatalStatement();
        aio.AsyncRead(other.read_fd(), buf2, &second);
      },
      DiesAfterMarker("still in flight"));
}

// Poll() delivers one callback, but internal work must not be what spends
// it: a pending wakeup and a ready completion come out of one Poll() as the
// completion.  epoll used to take one event per Poll() and could hand the
// whole call to its own wakeup eventfd.
TEST_P(AioTest, WakeupDoesNotSpendPollBudget) {
  Aio aio;
  Pipe pipe;

  bool fired = false;
  AsyncRequest req;
  req.callback = [](Completion completion, void *ctx) {
    EXPECT_TRUE(aos::IsOk(completion.status));
    *static_cast<bool *>(ctx) = true;
  };
  req.context = &fired;
  char buf[8];
  aio.AsyncRead(pipe.read_fd(), buf, &req);
  // Makes the first attempt on the empty pipe, so the read is waiting on the
  // kernel when the data arrives rather than finding it at submit.
  aio.Poll(false);
  ASSERT_FALSE(fired);
  // Queues the internal wakeup -- first, so epoll reports it ahead of the
  // data, which is the order that used to spend the Poll().
  aio.Quit();
  pipe.Write("x");

  EXPECT_TRUE(aio.Poll(false));
  EXPECT_TRUE(fired);
}

// A callback that keeps re-arming a read on a regular file is always ready
// again, so a loop that serviced its queued completions before looking at
// the kernel starved everything else -- timers included -- until EOF.
// Everything one harvest resolved is delivered before the next harvest, so
// the timer gets a turn after a bounded number of reads.  (This covers one
// re-arming reader.  Whether several can still keep a timer waiting, on any
// backend, is a separate question.)
TEST_P(AioTest, SelfRearmingRegularFileReadDoesNotStarveTimers) {
  Aio aio;

  std::string path = aos::testing::TestTmpDir() + "/aio_starve_XXXXXX";
  const int fd = mkstemp(path.data());
  ASSERT_GE(fd, 0);
  ASSERT_EQ(unlink(path.c_str()), 0);
  const std::string contents(4096, 'x');
  ASSERT_EQ(write(fd, contents.data(), contents.size()),
            static_cast<ssize_t>(contents.size()));
  ASSERT_EQ(lseek(fd, 0, SEEK_SET), 0);

  struct State {
    Aio *aio;
    int fd;
    char byte;
    AsyncRequest req;
    int reads = 0;
    int reads_at_timer = -1;
  } state{&aio, fd, 0, {}, 0, -1};
  state.req.context = &state;
  state.req.callback = [](Completion completion, void *ctx) {
    auto *self = static_cast<State *>(ctx);
    ASSERT_TRUE(aos::IsOk(completion.status));
    if (completion.result == 0) return;  // EOF
    ++self->reads;
    // Once the timer has had its turn the test is over.  Stop re-arming, so
    // nothing is left in flight that would have to be canceled.
    if (self->reads_at_timer >= 0) return;
    self->aio->AsyncRead(self->fd, std::span<char>(&self->byte, 1), &self->req);
  };

  Aio::Timer timer(&aio);
  timer.Schedule(
      aos::monotonic_clock::now(),
      [](Completion, void *ctx) {
        auto *self = static_cast<State *>(ctx);
        self->reads_at_timer = self->reads;
      },
      &state);
  // Make sure the timerfd has expired before the reads start.
  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  aio.AsyncRead(fd, std::span<char>(&state.byte, 1), &state.req);
  while (state.reads_at_timer < 0 && state.reads < 4096 && aio.Poll(true)) {
  }

  ASSERT_GE(state.reads_at_timer, 0) << "the timer never fired";
  EXPECT_LT(state.reads_at_timer, 8)
      << "the timer waited behind the re-arming reads";
  // At most one read is still in flight, and its callback will not re-arm.
  // Drained rather than canceled: whether a cancel beats a regular-file read
  // depends on whether the kernel ran it inline or handed it to io-wq, and a
  // Canceled completion would fail the callback's status check.
  while (!state.req.done) {
    aio.Poll(true);
  }
  while (aio.Poll(false)) {
  }
  close(fd);
}

// A request is in flight until its callback has run, and that includes a
// Cancel()ed one whose Canceled completion has not been delivered yet.
// Re-arming it then used to give two callbacks for one request on the
// readiness backends -- Canceled, then the new read -- and a caller that
// freed the request in the first (which the contract allows) had the second
// read write into freed memory.
TEST_P(AioTest, RearmBeforeCanceledDeliveryDeathTest) {
  EXPECT_DEATH(
      {
        Aio aio;
        Pipe pipe;
        AsyncRequest req;
        char buf[8];
        aio.AsyncRead(pipe.read_fd(), buf, &req);
        aio.Cancel(&req);
        MarkFatalStatement();
        aio.AsyncRead(pipe.read_fd(), buf, &req);
      },
      DiesAfterMarker("still in flight"));
}

// A Canceled completion releases the fd claim of the request it belongs to,
// and nothing else.  The readiness backends once released the claim when
// the Canceled completion was delivered, looked up by fd number, so a stale
// Canceled could release the claim of a newer request on the same fd and let
// a legacy handler in beside it.
//
// So the newer request has to be claimed and still unresolved when the
// older one's Canceled is delivered.  A submit error queued ahead of the
// older cancel makes room: its callback runs after the older cancel has
// resolved but before it is delivered, and arms and cancels the newer
// request there.
TEST_P(AioTest, CanceledCompletionReleasesOnlyItsOwnClaimDeathTest) {
  ScopedDeathTestWatchdog watchdog;
  // Everything inside the child: a raw request in flight is what this needs,
  // and it cannot be set up in the parent (see DoubleSubmitDeathTest).
  EXPECT_DEATH(
      {
        Aio aio;
        Pipe pipe;

        struct State {
          Aio *aio;
          FileDescriptor fd;
          AsyncRequest second;
          char buf2[8];
          bool first_delivered = false;
        } state;
        state.aio = &aio;
        state.fd = pipe.read_fd();

        AsyncRequest blocker;
        blocker.callback = [](Completion, void *ctx) {
          State *state = static_cast<State *>(ctx);
          state->aio->AsyncRead(state->fd, state->buf2, &state->second);
          state->aio->Cancel(&state->second);
        };
        blocker.context = &state;
        AsyncRequest first;
        first.callback = [](Completion, void *ctx) {
          static_cast<State *>(ctx)->first_delivered = true;
        };
        first.context = &state;

        char buf[8];
        aio.AsyncRead(-1, buf, &blocker);
        aio.AsyncRead(pipe.read_fd(), buf, &first);
        aio.Cancel(&first);
        while (!state.first_delivered && aio.Poll(false)) {
        }
        ABSL_CHECK(state.first_delivered);

        if (state.second.done) {
          // Resolved already, so its claim is legitimately gone and there is
          // nothing left to test.  Only io_uring may get here, when one Poll()
          // happens to reap the newer cancel too.  The readiness backends
          // cannot: first was queued for delivery, so no Poll() since the
          // newer cancel has gone back to harvest it.
          ABSL_CHECK(IsIoUring()) << "the newer cancel resolved early";
          MarkFatalStatement();
          ABSL_LOG(FATAL) << "Cannot mix OnReadable and AsyncRead";
        }
        MarkFatalStatement();
        aio.OnReadable(pipe.read_fd(), []() {});
      },
      DiesAfterMarker("Cannot mix OnReadable and AsyncRead"));
}

// Closing an fd with a request armed on it is a contract violation (see
// Aio::AsyncRead()), and epoll dies where it can see one: the DEL the cancel
// issues finds the fd gone.  Tolerating that instead would hand the
// registration back to the pool while a kernel entry for a dup()ed file could
// still name it.
//
// io_uring cannot tell -- it holds its own reference to the file and just
// cancels the read -- so it only has the contract.
TEST_P(AioTest, CancelAfterFdClosedDeathTest) {
  if (IsIoUring()) {
    GTEST_SKIP() << "io_uring holds the file; closing the fd is invisible";
  }
  EXPECT_DEATH(
      {
        Aio aio;
        Pipe pipe;
        AsyncRequest req;
        char buf[8];
        aio.AsyncRead(pipe.read_fd(), buf, &req);
        // Armed on epoll: the first attempt found nothing to read.
        aio.Poll(false);
        pipe.close_read_fd();
        MarkFatalStatement();
        aio.Cancel(&req);
      },
      DiesAfterMarker("epoll_ctl DEL failed"));
}

// A handler that deletes its own fd leaves its std::function -- and whatever
// it captured -- parked until nothing is executing out of it.  That has to be
// the end of the same Poll(), as io_uring does it.  epoll used to wait for the
// start of the next Poll(), which could be under ScopedRealtime, and freeing
// the capture there tripped the realtime free check.
TEST_P(AioTest, SelfDeletingHandlerCaptureFreedOutsideLaterRealtimePoll) {
  Aio aio;
  Pipe pipe;

  bool ran = false;
  // Heap-allocated, so destroying the capture frees.
  auto capture = std::make_shared<std::string>(256, 'x');
  aio.OnReadable(pipe.read_fd(), [&aio, &pipe, &ran, capture]() {
    ran = true;
    aio.DeleteFd(pipe.read_fd());
  });
  capture.reset();
  pipe.Write("x");
  while (!ran && aio.Poll(true)) {
  }
  ASSERT_TRUE(ran);

  {
    ScopedRealtime rt;
    aio.Poll(false);
  }
}

// Same parking, seen from the destructor: a handler that deletes its own fd
// and owns a Timer must not make the Aio's destructor report a leaked Timer.
TEST_P(AioTest, SelfDeletingHandlerOwningTimerThenDestroy) {
  Pipe pipe;
  bool ran = false;
  {
    Aio aio;
    auto timer = std::make_shared<Aio::Timer>(&aio);
    aio.OnReadable(pipe.read_fd(), [&aio, &pipe, &ran, timer]() {
      ran = true;
      aio.DeleteFd(pipe.read_fd());
    });
    timer.reset();
    pipe.Write("x");
    while (!ran && aio.Poll(true)) {
    }
    ASSERT_TRUE(ran);
  }
}

// Whether this kernel does RWF_NOWAIT on a pipe (6.4 and later).  Where it
// does not, the readiness backends have to make a blocking pipe O_NONBLOCK;
// see Aio::AsyncRead().
static bool KernelDoesNowaitOnPipes() {
  int fds[2];
  ABSL_PCHECK(pipe(fds) == 0);
  char byte = 'x';
  struct iovec iov = {.iov_base = &byte, .iov_len = 1};
  // The syscall rather than pwritev2(), which older glibc (the roboRIO's)
  // does not wrap.  0x8 is RWF_NOWAIT, which it does not define either.
  const bool supported =
      syscall(__NR_pwritev2, fds[1], &iov, 1, -1L, -1L, 0x8) == 1;
  close(fds[0]);
  close(fds[1]);
  return supported;
}

// Either kind of fd is accepted.  A blocking pipe with more to write than it
// holds must not put the loop thread to sleep in write(2), and must still be
// blocking afterward: the file description is shared with whoever else holds
// it (a parent shell's stdout, say).  The readiness backends write with
// RWF_NOWAIT and complete with a short write; io_uring does the write off the
// loop.  A hang here is the bug.
TEST_P(AioTest, BlockingFdRawWriteLeavesFdBlockingTest) {
  int fds[2];
  ABSL_PCHECK(pipe(fds) == 0);
  // The test's own draining reads must not block either.
  ABSL_PCHECK(fcntl(fds[0], F_SETFL, O_NONBLOCK) == 0);
  Aio aio;

  const std::vector<char> out(1 << 20, 'x');
  int32_t written = -1;
  AsyncRequest req;
  req.callback = [](Completion completion, void *ctx) {
    EXPECT_TRUE(aos::IsOk(completion.status));
    *static_cast<int32_t *>(ctx) = completion.result;
  };
  req.context = &written;
  aio.AsyncWrite(fds[1], std::span<const char>(out), &req);

  std::vector<char> in(1 << 16);
  while (written < 0) {
    aio.Poll(false);
    ABSL_CHECK(read(fds[0], in.data(), in.size()) >= 0 || errno == EAGAIN);
  }
  EXPECT_GT(written, 0);
  if (IsIoUring() || KernelDoesNowaitOnPipes()) {
    EXPECT_EQ(fcntl(fds[1], F_GETFL) & O_NONBLOCK, 0)
        << "the backend changed the caller's file description";
  }
  close(fds[0]);
  close(fds[1]);
}

// Cancel() is asynchronous: the request completes through Poll() with a
// Canceled status, so the fd it was armed on stays claimed until that
// completion is dispatched.  Registering a legacy handler in between is the
// same mixing error as doing it while the request was in flight.
//
// The readiness backends released the claim at Cancel() and accepted the
// OnReadable(); io_uring holds its until the terminal CQE drains and refused.
// io_uring's is the contract, because it is the one that matches what Cancel()
// documents.
TEST_P(AioTest, LegacyHandlerAfterCancelBeforeDispatchDeathTest) {
  EXPECT_DEATH(
      {
        Aio aio;
        Pipe pipe;
        AsyncRequest req;
        char buf[8];
        aio.AsyncRead(pipe.read_fd(), buf, &req);
        aio.Cancel(&req);
        MarkFatalStatement();
        // No Poll() in between, so the Canceled completion is still queued.
        aio.OnReadable(pipe.read_fd(), []() {});
      },
      DiesAfterMarker("Cannot mix OnReadable and AsyncRead"));
}

// ...and once that completion is dispatched, the fd is free again.
TEST_P(AioTest, LegacyHandlerAfterCancelDispatchTest) {
  Aio aio;
  Pipe pipe;
  AsyncRequest req;
  bool canceled = false;
  req.callback = [](Completion completion, void *context) {
    EXPECT_FALSE(aos::IsOk(completion.status));
    *static_cast<bool *>(context) = true;
  };
  req.context = &canceled;

  char buf[8];
  aio.AsyncRead(pipe.read_fd(), buf, &req);
  aio.Cancel(&req);
  while (!canceled && aio.Poll(true)) {
  }
  ASSERT_TRUE(canceled) << "the Canceled completion never arrived";

  aio.OnReadable(pipe.read_fd(), []() {});
  aio.DeleteFd(pipe.read_fd());
}

// A raw AsyncRead and AsyncWrite on one fd are two user-visible completions
// and take two Poll()s, like any other two.
//
// A socketpair end is readable and writable at once, so one kernel event
// reports both directions.  The readiness backends dispatched both callbacks
// from that single event; io_uring, which learns about completions one at a
// time, always took two.  The legacy path keeps EPoll's exception -- one fd's
// readable/writable/error handlers still run together -- but raw requests get
// no such carve-out.
TEST_P(AioTest, RawReadAndWriteOnOneFdTakeTwoPolls) {
  int fds[2];
  ABSL_PCHECK(socketpair(AF_UNIX, SOCK_STREAM, 0, fds) == 0);
  ABSL_PCHECK(fcntl(fds[0], F_SETFL, O_NONBLOCK) == 0);
  ABSL_PCHECK(fcntl(fds[1], F_SETFL, O_NONBLOCK) == 0);

  Aio aio;
  AsyncRequest read_req;
  AsyncRequest write_req;
  int completions = 0;
  const auto count = [](Completion, void *context) {
    ++*static_cast<int *>(context);
  };
  read_req.callback = count;
  write_req.callback = count;
  read_req.context = &completions;
  write_req.context = &completions;

  // Neither direction ready to start with -- nothing to read, and a full
  // send buffer -- so both first attempts find nothing and the requests wait
  // on the kernel.  Resolving at submit would bypass the event entirely.
  const std::string chunk(4096, 'z');
  while (write(fds[0], chunk.data(), chunk.size()) > 0) {
  }
  ABSL_PCHECK(errno == EAGAIN);

  char in_buf[8];
  const char out_buf[1] = {'x'};
  aio.AsyncRead(fds[0], in_buf, &read_req);
  aio.AsyncWrite(fds[0], out_buf, &write_req);
  aio.Poll(false);
  ASSERT_EQ(completions, 0);

  // Now make both ready at once, so one event reports both directions:
  // drain the peer, which frees the send buffer, then give it data to send
  // back.
  char drain[4096];
  while (read(fds[1], drain, sizeof(drain)) > 0) {
  }
  ABSL_PCHECK(errno == EAGAIN);
  ABSL_PCHECK(write(fds[1], "y", 1) == 1);

  int polls = 0;
  while (completions < 2 && polls < 10) {
    aio.Poll(true);
    ++polls;
  }

  EXPECT_EQ(completions, 2);
  EXPECT_EQ(polls, 2) << "two raw completions must take two Poll() calls";

  ABSL_PCHECK(close(fds[0]) == 0);
  ABSL_PCHECK(close(fds[1]) == 0);
}

// A request with no callback is supported -- aio.h defaults the field to
// nullptr -- and resolving one still counts as work.
//
// The readiness backends resolve an invalid fd at submit time and queue it.
// Their drain only reported progress from inside `if (req->callback)`, so a
// request with none set `done`, fell through, and blocked in the wait: a
// caller running the documented `while (!req.done && Poll(true))` drain never
// got back out to re-read `done`.  Poll(false) meanwhile answered false
// having just retired a request.
//
// The expectations below are what io_uring was measured doing, and every
// backend is held to them.  It resolves each of these requests as a CQE, so
// the first Poll() retires all three and returns true, and the Poll(false)
// after it finds nothing left and returns false.
TEST_P(AioTest, NoCallbackRequestsRetireInOnePoll) {
  Aio aio;
  AsyncRequest requests[3];
  char buf[8];
  for (auto &request : requests) {
    aio.AsyncRead(-1, buf, &request);
  }

  EXPECT_TRUE(aio.Poll(true))
      << "Poll() retired a request and reported that nothing happened";
  for (const auto &request : requests) {
    EXPECT_TRUE(request.done) << "a no-callback request was left pending";
  }
  EXPECT_FALSE(aio.Poll(false)) << "nothing should be left to do";
}

// Two ways of misusing an fd that carries only raw requests, each of which
// used to report something other than what happened.
//
// A fresh Aio per case, and both calls inside the child: a raw request in
// flight is what each needs, and it cannot be set up in the parent (see
// DoubleSubmitDeathTest for both reasons).
TEST_P(AioTest, RawOnlyFdMisuseDeathTest) {
  // A second AsyncRead on one fd is a duplicate, not a mix.  This used to
  // blame OnReadable -- a handler the caller never registered -- because the
  // raw submit path parked its own lambda in in_fn.
  //
  // Only the readiness backends reject it: they hold one request slot per
  // direction per fd, while io_uring can have any number of reads in flight
  // on one fd and accepts this.  The message is what is under test here, not
  // whether the limit exists.
  if (!IsIoUring()) {
    Aio aio;
    Pipe pipe;
    AsyncRequest first;
    AsyncRequest second;
    char buf[8];
    EXPECT_DEATH(
        {
          aio.AsyncRead(pipe.read_fd(), buf, &first);
          MarkFatalStatement();
          aio.AsyncRead(pipe.read_fd(), buf, &second);
        },
        DiesAfterMarker("Duplicate AsyncRead on fd"));
  }

  // EnableWritable() is for fds registered through OnWritable().  An fd
  // carrying only raw requests has no such registration, which io_uring
  // reports as "fd not found" because it looks in a legacy-only table.
  {
    Aio aio;
    Pipe pipe;
    AsyncRequest read_req;
    char buf[8];
    EXPECT_DEATH(
        {
          aio.AsyncRead(pipe.read_fd(), buf, &read_req);
          MarkFatalStatement();
          aio.EnableWritable(pipe.read_fd());
        },
        DiesAfterMarker("not found"));
  }
}

// Tests that async registrations recycle: every request here fully completes
// before the next distinct fd is used, so its slot must return to the pool.
// Used to abort with "Async registration pool exhausted" on the 16th
// distinct fd.
TEST_P(AioTest, AsyncRegistrationPoolRecycleTest) {
  Aio aio;

  std::array<Pipe, 20> pipes;
  char buf[8];
  for (auto &pipe : pipes) {
    AsyncRequest req;
    bool fired = false;
    req.callback = [](Completion completion, void *ctx) {
      EXPECT_TRUE(aos::IsOk(completion.status));
      *static_cast<bool *>(ctx) = true;
    };
    req.context = &fired;

    {
      // Under realtime: recycling a slot has to be pointer work, and
      // --die_on_malloc is what proves it.  A slot that leaked instead of
      // recycling would allocate on the 17th distinct fd, which is the
      // regression this test exists for.
      ScopedRealtime rt;
      aio.AsyncRead(pipe.read_fd(), buf, &req);
      pipe.Write("x");
      while (!fired && aio.Poll(true)) {
      }
    }
    EXPECT_TRUE(fired);
  }
}

// Exceeding the pool degrades, it does not abort.  The pool is a realtime
// optimisation -- arming must not allocate -- not a hard ceiling on how many
// fds may be in flight, so the 17th simultaneous registration comes from the
// heap.  A CHECK there would turn a sizing miss into a dead process.
//
// Deliberately not under ScopedRealtime: the fallback allocates, which is
// what --die_on_malloc would (correctly) abort on.  What is under realtime is
// AsyncRegistrationPoolRecycleTest above, which pins the case that should
// never need the heap at all.
TEST_P(AioTest, AsyncRegistrationPoolFallsBackBeyondCapacityTest) {
  // Comfortably past every backend's default of 16.
  constexpr int kFds = 24;
  Aio aio;

  std::array<Pipe, kFds> pipes;
  std::array<AsyncRequest, kFds> reqs;
  std::array<char[8], kFds> bufs;
  int completed = 0;
  for (int i = 0; i < kFds; ++i) {
    reqs[i].callback = [](Completion completion, void *ctx) {
      EXPECT_TRUE(aos::IsOk(completion.status));
      ++*static_cast<int *>(ctx);
    };
    reqs[i].context = &completed;
    aio.AsyncRead(pipes[i].read_fd(), bufs[i], &reqs[i]);
  }
  for (int i = 0; i < kFds; ++i) {
    pipes[i].Write("x");
  }

  while (completed < kFds && aio.Poll(true)) {
  }
  EXPECT_EQ(completed, kFds) << "the pool did not fall back past its capacity";
}

namespace {

// Counts the destructions that happen while
// DeleteOwnFdFromCallbackKeepsCapturesTest's callback is running.  The state
// is static rather than reached through the capture: the callback clears
// callback_running after DeleteFd(), which is exactly when its captures would
// be gone if this regressed.
struct DestructionTracker {
  ~DestructionTracker() {
    if (callback_running) {
      ++destroyed_while_running;
    }
  }

  static inline bool callback_running = false;
  static inline int destroyed_while_running = 0;
};

}  // namespace

// Tests that a callback which deletes its own fd does not have its captures
// destroyed while it is still running.  DeleteFd() releases the registration
// that holds the currently-executing std::function, and destroying it there
// would free the captures out from under the callback.
//
// Counted by a destructor rather than detected by reading the captures after
// DeleteFd() and relying on ASAN: if this regresses, that read is undefined
// behavior, which the compiler is free to optimize around before ASAN ever
// sees it.  So nothing after DeleteFd() touches a capture.
TEST_P(AioTest, DeleteOwnFdFromCallbackKeepsCapturesTest) {
  // Every backend's instance runs in this one process.
  DestructionTracker::callback_running = false;
  DestructionTracker::destroyed_while_running = 0;

  Aio aio;
  Pipe pipe;

  bool fired = false;
  aio.OnReadable(pipe.read_fd(),
                 [&aio, &pipe, &fired, tracker = DestructionTracker{}]() {
                   DestructionTracker::callback_running = true;
                   fired = true;
                   aio.DeleteFd(pipe.read_fd());
                   DestructionTracker::callback_running = false;
                 });
  pipe.Write("x");

  while (!fired && aio.Poll(true)) {
  }
  EXPECT_TRUE(fired);
  EXPECT_EQ(DestructionTracker::destroyed_while_running, 0)
      << "the running callback's captures were destroyed under it";

  // The retired registration is reclaimed on the next Poll(), once no
  // dispatch is in flight.
  aio.Poll(false);
}

// Tests AsyncRead()/AsyncWrite() on a regular file, which epoll_ctl(ADD)
// rejects with EPERM (always ready, no wait queue) -- this used to abort the
// epoll backend.  A first attempt that finds the data cached completes
// without reaching epoll at all, so the ADD is only tried after a page-cache
// miss; ColdRegularFileReadIsNotEmpty forces one.
TEST_P(AioTest, RegularFileAsyncReadWriteTest) {
  Aio aio;

  std::string path = aos::testing::TestTmpDir() + "/aio_regular_XXXXXX";
  int fd = mkstemp(path.data());
  ASSERT_GE(fd, 0);
  ASSERT_EQ(unlink(path.c_str()), 0);

  const char data[] = "regular file data";

  AsyncRequest write_req;
  std::optional<int32_t> write_result;
  write_req.callback = [](Completion completion, void *ctx) {
    EXPECT_TRUE(aos::IsOk(completion.status));
    static_cast<std::optional<int32_t> *>(ctx)->emplace(completion.result);
  };
  write_req.context = &write_result;
  aio.AsyncWrite(fd, std::span<const char>(data, sizeof(data) - 1), &write_req);
  while (!write_req.done && aio.Poll(true)) {
  }
  ASSERT_TRUE(write_result.has_value());
  EXPECT_EQ(*write_result, static_cast<int32_t>(sizeof(data) - 1));

  ASSERT_EQ(lseek(fd, 0, SEEK_SET), 0);

  char buf[64] = {};
  AsyncRequest read_req;
  std::optional<int32_t> read_result;
  read_req.callback = [](Completion completion, void *ctx) {
    EXPECT_TRUE(aos::IsOk(completion.status));
    static_cast<std::optional<int32_t> *>(ctx)->emplace(completion.result);
  };
  read_req.context = &read_result;
  aio.AsyncRead(fd, buf, &read_req);
  while (!read_req.done && aio.Poll(true)) {
  }
  ASSERT_TRUE(read_result.has_value());
  EXPECT_EQ(*read_result, static_cast<int32_t>(sizeof(data) - 1));
  EXPECT_STREQ(buf, "regular file data");

  ABSL_PCHECK(close(fd) == 0);
}

// Linux-only: RWF_NOWAIT, and posix_fadvise(2) to drop the cache.
#if defined(__linux__)
// A regular-file read whose range is not cached returns data, never a
// spurious 0: RWF_NOWAIT answers EAGAIN there rather than blocking, and a
// readiness backend has nothing to wait on for a regular file, so it has to
// fall back to the blocking read -- not complete with 0, which a caller reads
// as end of file, and not spin.  A partly cached range may come back short
// (see AsyncRead()), but never empty.
//
// The cache is dropped with POSIX_FADV_DONTNEED.  Whether that takes depends
// on the filesystem (tmpfs keeps its pages), so the test says whether the
// range was actually cold; either way the result must not be 0.
TEST_P(AioTest, ColdRegularFileReadIsNotEmpty) {
  std::string path = aos::testing::TestTmpDir() + "/aio_cold_XXXXXX";
  const int fd = mkstemp(path.data());
  ASSERT_GE(fd, 0);
  ASSERT_EQ(unlink(path.c_str()), 0);
  constexpr size_t kSize = 1 << 20;
  const std::string data(kSize, 'c');
  ASSERT_EQ(write(fd, data.data(), data.size()),
            static_cast<ssize_t>(data.size()));
  ASSERT_EQ(fdatasync(fd), 0);
  ASSERT_EQ(posix_fadvise(fd, 0, 0, POSIX_FADV_DONTNEED), 0);
  ASSERT_EQ(lseek(fd, 0, SEEK_SET), 0);

  // How much of the range is still cached, for the log.
  size_t resident_pages = 0;
  const size_t page = static_cast<size_t>(sysconf(_SC_PAGESIZE));
  void *map = mmap(nullptr, kSize, PROT_READ, MAP_SHARED, fd, 0);
  ASSERT_NE(map, MAP_FAILED);
  std::vector<unsigned char> vec(kSize / page);
  ASSERT_EQ(mincore(map, kSize, vec.data()), 0);
  for (unsigned char v : vec) {
    resident_pages += v & 1;
  }
  ASSERT_EQ(munmap(map, kSize), 0);
  ABSL_LOG(INFO) << resident_pages << " of " << vec.size()
                 << " pages still cached before the read";

  Aio aio;
  std::string buf(kSize, '\0');
  std::optional<Completion> completion;
  AsyncRequest req;
  req.callback = [](Completion c, void *ctx) {
    static_cast<std::optional<Completion> *>(ctx)->emplace(std::move(c));
  };
  req.context = &completion;
  aio.AsyncRead(fd, std::span<char>(buf.data(), buf.size()), &req);
  while (!completion.has_value() && aio.Poll(true)) {
  }
  ASSERT_TRUE(completion.has_value());
  ASSERT_TRUE(aos::IsOk(completion->status));
  EXPECT_GT(completion->result, 0) << "a spurious end of file";
  ASSERT_LE(completion->result, static_cast<int32_t>(kSize));
  EXPECT_EQ(buf.substr(0, completion->result),
            data.substr(0, completion->result));

  ABSL_PCHECK(close(fd) == 0);
}
#endif

// Tests that a pending AsyncRead() completes with EOF when the write end of
// an empty pipe closes.  The hangup surfaces as EPOLLHUP with no EPOLLIN,
// which the epoll backend used to spin on forever without completing the
// request.
TEST_P(AioTest, AsyncReadEofOnHangupTest) {
  Aio aio;
  Pipe pipe;

  AsyncRequest req;
  std::optional<aos::Status> fired_status;
  int32_t result = -1;
  req.callback = [](Completion completion, void *ctx) {
    auto *self_ctx =
        static_cast<std::pair<std::optional<aos::Status> *, int32_t *> *>(ctx);
    self_ctx->first->emplace(std::move(completion.status));
    *self_ctx->second = completion.result;
  };
  std::pair<std::optional<aos::Status> *, int32_t *> ctx(&fired_status,
                                                         &result);
  req.context = &ctx;

  char buf[8];
  aio.AsyncRead(pipe.read_fd(), buf, &req);
  // Make sure the read is armed and pending before hanging up.
  aio.Poll(false);
  pipe.close_write_fd();

  // Bounded non-blocking polling: on regression this hangs (or spins), and a
  // deadline turns that into a loud failure instead of a test timeout.
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!req.done && std::chrono::steady_clock::now() < deadline) {
    aio.Poll(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_TRUE(req.done) << "AsyncRead never completed after writer hangup";
  ASSERT_TRUE(fired_status.has_value());
  EXPECT_TRUE(aos::IsOk(*fired_status));
  EXPECT_EQ(result, 0);
}

// Tests that a timer destroyed by its own callback stops delivery cleanly:
// backends dispatch the user callback as the last thing they do with the
// state, and this holds them to it.
TEST_P(AioTest, TimerDestroyedByOwnCallbackTest) {
  Aio aio;

  struct Ctx {
    std::unique_ptr<Aio::Timer> timer;
    size_t count = 0;
  } ctx;
  ctx.timer = std::make_unique<Aio::Timer>(&aio);

  const auto start = aos::monotonic_clock::now();
  ctx.timer->Schedule(
      start + std::chrono::milliseconds(10),
      [](Completion completion, void *raw) {
        EXPECT_TRUE(aos::IsOk(completion.status));
        auto *ctx = static_cast<Ctx *>(raw);
        ++ctx->count;
        ctx->timer.reset();
      },
      &ctx);

  // Let the deadline pass well before servicing it, so the callback runs
  // out of a completion the loop had already been sitting on.
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  aio.Poll(true);
  while (aio.Poll(false)) {
  }
  EXPECT_EQ(ctx.count, 1u);
}

// Two timers with the same deadline both fire, so both completions are
// extracted into one dispatch batch.  The first callback then acts on the
// second timer -- which has already fired, kernel-side and as far as the
// backend's queue is concerned, but has NOT been dispatched yet.
//
// This is a genuine hole in any batch-completion design: between "the
// completion exists" and "the callback ran" there is a window where the
// owner can legitimately change its mind, and the backend has to honor the
// newer intent rather than deliver the stale firing.  The three things an
// owner can do in that window are covered separately below, because they
// have different right answers.
//
// Both timers are scheduled for a deadline already in the past and then
// serviced with a single Poll(), which is what guarantees one batch rather
// than two.
namespace {
struct TwoTimerBatch {
  Aio aio;
  std::unique_ptr<Aio::Timer> first;
  std::unique_ptr<Aio::Timer> second;
  int second_fires = 0;
  void *first_context = nullptr;

  TwoTimerBatch()
      : first(std::make_unique<Aio::Timer>(&aio)),
        second(std::make_unique<Aio::Timer>(&aio)) {}

  // Arms both for the same already-past deadline, with `on_first` running
  // as the first timer's callback.
  void Arm(CompletionCallback on_first) {
    const auto deadline =
        aos::monotonic_clock::now() + std::chrono::milliseconds(5);
    first->Schedule(deadline, on_first, this);
    second->Schedule(
        deadline,
        [](Completion completion, void *ctx) {
          if (aos::IsOk(completion.status)) {
            ++static_cast<TwoTimerBatch *>(ctx)->second_fires;
          }
        },
        this);
    // Both deadlines are well past by the time anything polls.
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
  }
};
}  // namespace

// Control for the three tests below, pinning down that the window they
// describe actually exists.  Without this they could pass for the wrong
// reason -- the second timer's completion simply not existing yet when the
// first callback ran.
//
// Both timers are due, so both are ready as far as the backend is
// concerned, but Poll() delivers exactly one completion (see Aio::Poll()).
// That is what creates the window the three tests below exercise: the first
// callback runs while the second timer has already fired and has not been
// delivered yet, and it is free to cancel, reschedule, or destroy it.
//
// Identical on every backend by construction, which is the point -- a
// consumer cannot tell io_uring from epoll by counting callbacks.  io_uring
// reaches this by draining its whole completion queue and dispatching one
// entry off it; epoll by asking the kernel for one event at a time.  Same
// contract, and the same window, out of different mechanisms.
TEST_P(AioTest, OneCompletionPerPollTest) {
  TwoTimerBatch batch;
  int first_fires = 0;
  batch.first_context = &first_fires;
  batch.Arm([](Completion, void *ctx) {
    ++*static_cast<int *>(static_cast<TwoTimerBatch *>(ctx)->first_context);
  });

  // Exactly one Poll(), no drain loop.
  batch.aio.Poll(true);
  EXPECT_EQ(first_fires, 1);
  EXPECT_EQ(batch.second_fires, 0)
      << "Poll() delivered two completions; the same-batch tests below rely "
         "on the second timer still being undelivered when the first "
         "callback runs, and prove nothing without that.";

  // ...and the second one arrives on a subsequent Poll().
  while (batch.second_fires == 0 && batch.aio.Poll(true)) {
  }
  EXPECT_EQ(batch.second_fires, 1);
}

// Canceling the second timer from the first's callback must suppress its
// pending firing entirely -- Timer::Cancel() is documented as silent, and
// "already fired but not yet delivered" is still pending.
TEST_P(AioTest, CancelATimerFiringInTheSameBatchTest) {
  TwoTimerBatch batch;
  batch.Arm([](Completion, void *ctx) {
    static_cast<TwoTimerBatch *>(ctx)->second->Cancel();
  });

  batch.aio.Poll(true);
  while (batch.aio.Poll(false)) {
  }
  EXPECT_EQ(batch.second_fires, 0)
      << "A timer canceled before its already-queued firing was dispatched "
         "delivered it anyway.";
}

// Rescheduling the second timer from the first's callback must move its
// firing to the new deadline, not deliver the old one immediately.  Getting
// this wrong is how a consumer that filters early wakeups (ShmEventLoop
// does) ends up with a timer that is armed everywhere in userspace and
// absent from the kernel.
TEST_P(AioTest, RescheduleATimerFiringInTheSameBatchTest) {
  TwoTimerBatch batch;
  batch.Arm([](Completion, void *ctx) {
    auto *self = static_cast<TwoTimerBatch *>(ctx);
    self->second->Schedule(
        aos::monotonic_clock::now() + std::chrono::milliseconds(50),
        [](Completion completion, void *inner) {
          if (aos::IsOk(completion.status)) {
            ++static_cast<TwoTimerBatch *>(inner)->second_fires;
          }
        },
        self);
  });

  batch.aio.Poll(true);
  while (batch.aio.Poll(false)) {
  }
  EXPECT_EQ(batch.second_fires, 0)
      << "The superseded firing was delivered instead of being replaced by "
         "the new schedule.";

  // ...and the new schedule must still be live.
  while (batch.second_fires == 0 && batch.aio.Poll(true)) {
  }
  EXPECT_EQ(batch.second_fires, 1) << "The rescheduled timer never fired.";
}

// Destroying the second timer from the first's callback must not leave the
// dispatch loop walking into freed memory when it reaches that timer's
// already-queued completion.
TEST_P(AioTest, DestroyATimerFiringInTheSameBatchTest) {
  TwoTimerBatch batch;
  batch.Arm([](Completion, void *ctx) {
    static_cast<TwoTimerBatch *>(ctx)->second.reset();
  });

  batch.aio.Poll(true);
  while (batch.aio.Poll(false)) {
  }
  EXPECT_EQ(batch.second_fires, 0);
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

  size_t count = 0;

  {
    ScopedRealtime rt;
    // Verify that since the pipe is empty, no callback fires.
    aio.Poll(false);
  }
  EXPECT_EQ(callback_count, 0);

  // Write some data to make the pipe readable.
  pipe.Write("a");

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
  EXPECT_EQ(pipe.Read(1), "a");

  {
    ScopedRealtime rt;
    // Verify that polling now does not trigger additional callbacks.
    aio.Poll(false);
  }
  EXPECT_EQ(callback_count, 1);

  // Write again.  Since it is persistently re-submitted, it should fire again.
  pipe.Write("a");
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
  EXPECT_EQ(pipe.Read(1), "a");

  // Unregister the descriptor.
  aio.DeleteFd(pipe.read_fd());

  // Write more data.
  pipe.Write("a");

  {
    ScopedRealtime rt;
    // Poll.  The callback should NOT fire anymore.
    aio.Poll(false);
  }
  EXPECT_EQ(callback_count, 2);

  // Clean up.
  EXPECT_EQ(pipe.Read(1), "a");
}

// One poll backs every legacy fd, armed on the epoll instance rather than on
// the fd that fired, and re-armed before the drain dispatches.  So the
// readiness that re-arm sees belongs to no particular registration:
//
// 1. A and B are both registered readable.
// 2. A becomes readable, and the loop reports A.
// 3. B becomes readable from inside A's callback, after the re-arm was
//    queued.
// 4. A's callback fully drains A.
// 5. The instance was readable the whole way through, but by the end it is
//    readable for B rather than for the fd that was just drained.
//
// B has to be delivered exactly once.  The second half of the test takes
// away B, so the surviving readiness is one the callback itself consumed:
// that has to cost a bounded number of polls rather than spin.
TEST_P(AioTest, EpollReadinessSpansRegistrations) {
  Aio aio;
  Pipe a;
  Pipe b;

  size_t a_count = 0;
  size_t b_count = 0;

  bool arm_b = true;
  aio.OnReadable(a.read_fd(), [&]() {
    ++a_count;
    // Step 3: B goes ready while A's event is mid-dispatch.
    if (arm_b) {
      b.Write("b");
    }
    // Step 4: and A is fully drained, so nothing about A is still ready.
    EXPECT_EQ(a.Read(1), "a");
  });
  aio.OnReadable(b.read_fd(), [&]() {
    ++b_count;
    EXPECT_EQ(b.Read(1), "b");
  });

  a.Write("a");

  size_t polls = 0;
  while (b_count == 0 && polls < 100) {
    aio.Poll(true);
    ++polls;
  }
  ABSL_LOG(INFO) << "Polls to deliver A and then B: " << polls;
  EXPECT_EQ(a_count, 1);
  EXPECT_EQ(b_count, 1);

  // Both drained.  A bounded number of non-blocking polls has to run out:
  // an unbounded one is the busy loop an unconsumable level-triggered
  // readiness would produce.
  size_t spins = 0;
  while (spins < 50 && aio.Poll(false)) {
    ++spins;
  }
  ABSL_LOG(INFO) << "Non-blocking polls before the loop went quiet: " << spins;
  EXPECT_LT(spins, 50);
  EXPECT_EQ(a_count, 1);
  EXPECT_EQ(b_count, 1);

  // The same thing with nothing behind it: A goes ready alone and its
  // callback drains it, so the only readiness the re-arm was queued against
  // is one that callback has since consumed.  Bounded polls again, not an
  // endless supply of them.
  arm_b = false;
  a.Write("a");

  polls = 0;
  while (a_count == 1 && polls < 100) {
    aio.Poll(true);
    ++polls;
  }
  ABSL_LOG(INFO) << "Polls to deliver A alone: " << polls;
  EXPECT_EQ(a_count, 2);
  EXPECT_EQ(b_count, 1);

  spins = 0;
  while (spins < 50 && aio.Poll(false)) {
    ++spins;
  }
  ABSL_LOG(INFO) << "Non-blocking polls after draining A alone: " << spins;
  EXPECT_LT(spins, 50);
  EXPECT_EQ(a_count, 2);
  EXPECT_EQ(b_count, 1);

  aio.DeleteFd(a.read_fd());
  aio.DeleteFd(b.read_fd());
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
  // Set only while timer1's callback is inside Schedule().  Timer 3's callback
  // running while this is set is what "nested" means; timer 3 running before
  // timer 1 at all is just an ordering the API never promised either way.
  bool in_timer1_schedule = false;
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
        s->in_timer1_schedule = true;
        s->timer2->Schedule(
            aos::monotonic_clock::now() + std::chrono::seconds(20),
            [](Completion, void *) {}, nullptr);
        s->in_timer1_schedule = false;
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
        // Getting here from inside timer1's Schedule() is the failure; getting
        // here from Poll() is fine no matter which timer got there first.
        if (s->in_timer1_schedule) {
          s->nested_callback_detected = true;
        }
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
  pipe.Write("a");

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
  pipe.Write("x");

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
  pipe.Write("x");

  {
    ScopedRealtime rt;
    aio.Poll(false);
  }
  EXPECT_EQ(readable_count, 0);

  // Register the write end and verify the registration is live -- an empty
  // pipe's write end is immediately writable.
  size_t writable_count = 0;
  FileDescriptor w_fd = pipe.write_fd();
  aio.OnWritable(w_fd, [&writable_count]() { ++writable_count; });
  {
    ScopedRealtime rt;
    while (writable_count == 0 && aio.Poll(true)) {
    }
  }
  EXPECT_GT(writable_count, 0u);

  // Close it and forget it: the callback must never fire again, and the
  // loop must keep polling cleanly with the registration gone.
  const size_t writable_count_before = writable_count;
  pipe.close_write_fd();
  aio.ForgetClosedFd(w_fd);
  {
    ScopedRealtime rt;
    aio.Poll(false);
    aio.Poll(false);
  }
  EXPECT_EQ(writable_count, writable_count_before);
}

// The one test covering the caller-driven repeating pattern end to end --
// which is to say, the pattern ShmTimerHandler uses in production, since
// Aio::Timer is one-shot.  RepeatingTimer (above) is the same three lines
// ShmTimerHandler writes: re-arm from the callback against an absolute
// grid the caller owns.
//
// The property: firing i is due at base + i*period for every i, forever.
// The deadlines are a fixed grid that neither scheduling delay nor a slow
// callback may shift.  Everything else about repeating timers is now
// caller code, so this is the piece Aio is still responsible for.
//
// This is also the property that motivated removing repeating timers from
// Aio altogether.  The io_uring backend used to implement them with
// IORING_TIMEOUT_MULTISHOT, which cannot express an absolute deadline (the
// kernel rejects MULTISHOT|ABS) and so re-armed relatively, from whenever
// task work ran -- losing the wakeup latency every single period, measured
// at 4-10us each and accumulating without bound.  Measured through this
// test against that backend: 1.5ms of accumulated error across 500 firings
// of a 2ms timer.
//
// Note what this test does and does not cover now.  It cannot fail against
// the old backend anymore, because the drifting construct is gone from the
// API: with no interval parameter, the only thing left to schedule is an
// absolute deadline, and absolute deadlines never drifted.  What it guards
// going forward is that Schedule() really honors the absolute deadline it
// is given, and that re-arming from inside the callback -- the pattern
// every periodic consumer now uses -- accumulates nothing.  That is the
// property the whole design now rests on, so it is worth pinning even
// though no current bug can violate it.
//
// The discriminator is the *minimum* error across the final window, not an
// average: scheduling noise can only ever make a firing late, so a
// phase-locked timer is guaranteed some near-exact firing in any window,
// while a drifting one is uniformly late by however much it has
// accumulated.  That makes the check immune to hiccups without needing a
// loose bound.
TEST_P(AioTest, RepeatingTimerHoldsPhaseTest) {
  constexpr auto kPeriod = std::chrono::milliseconds(2);
  constexpr size_t kFirings = 500;
  constexpr size_t kWindow = 50;

  Aio aio;

  size_t count = 0;
  std::vector<aos::monotonic_clock::duration> errors;
  errors.reserve(kFirings);
  const auto base = aos::monotonic_clock::now() + std::chrono::milliseconds(20);

  RepeatingTimer timer(&aio, [&](Completion completion) {
    if (!aos::IsOk(completion.status)) {
      return;
    }
    // Firing i (0-based) is due at base + i*period.  Every elapsed period
    // is delivered in turn, so the index and the grid stay in step even if
    // the loop falls behind.
    errors.push_back(aos::monotonic_clock::now() -
                     (base + kPeriod * static_cast<int64_t>(count)));
    ++count;
  });
  timer.Start(base, kPeriod);

  while (count < kFirings && aio.Poll(true)) {
  }
  timer.Cancel();
  ASSERT_GE(errors.size(), kFirings);

  const auto best_early =
      *std::min_element(errors.begin(), errors.begin() + kWindow);
  const auto best_late =
      *std::min_element(errors.end() - kWindow, errors.end());

  // Every firing is at-or-after its deadline; a timer that fired *early*
  // would mean the grid moved backwards.
  EXPECT_GE(best_early, aos::monotonic_clock::duration::zero());
  EXPECT_GE(best_late, aos::monotonic_clock::duration::zero());

  // The grid must not have moved between the two windows.  Phase-locked
  // this is flat, bounded only by wakeup jitter; anything that reintroduced
  // a relative re-arm would show up here as a gap that grows with the run
  // length.
  EXPECT_LT(best_late - best_early, std::chrono::milliseconds(1))
      << "Best-case lateness grew from " << best_early << " over the first "
      << kWindow << " firings to " << best_late << " over the last " << kWindow
      << " of " << kFirings
      << ".  A repeating timer must not accumulate "
         "phase error.";
}

// Regression tests for shared lifetime handling: destroying a timer from a
// sibling's callback, unregistering a receiver mid-dispatch, a reschedule
// landing on an unobserved firing.  Every backend has those shapes, so they
// run on every backend.
//
// A test needing a genuinely io_uring-only precondition (a CQ overflow, a
// staged-SQE limit) says so itself rather than the whole group being skipped:
// elsewhere the setup is a no-op and the test degrades to a liveness check,
// which is still worth running.  PreRunArmingExceedsQueueDepthDeathTest is
// the only one asserting something no other backend promises, and it skips
// itself.
//
// These churn rings aggressively on io_uring, so they call
// ThrottleOnKernelRingTeardown() periodically themselves.

// Cancel, reschedule, and destruction against a timer whose firing the loop
// has not observed yet.
//
// Historically this was two separate tests and the sharpest pair in the
// file: with the timer living in an IORING_OP_TIMEOUT, the kernel's cancel
// lookup went blind to the target for a window around every firing, so both
// a cancel and a "shape-changing" reschedule landing in that window were
// silently discarded -- leaving a timer that either would not stop or was
// wedged with nothing armed at all.  Both are one timerfd_settime() now and
// cannot miss, which collapses the two into one scenario.
//
// Kept, merged, because the failures it caught were silent, and because a
// single settime() replacing both paths is exactly the kind of claim worth
// holding down.  It still exercises the orphan path: destruction with a
// completion in flight.  A raw sleep with no Poll() in between is what
// leaves the firing unobserved.
TEST_P(AioTest, CancelRescheduleAndDestroyPastUnobservedFirings) {
  for (int iter = 0; iter < 50; ++iter) {
    if (iter % 25 == 0) {
      ThrottleOnKernelRingTeardown();
    }
    Aio aio;
    int fire_count = 0;
    RepeatingTimer timer(&aio, [&fire_count](Completion completion) {
      if (aos::IsOk(completion.status)) {
        ++fire_count;
      }
    });
    timer.Start(aos::monotonic_clock::now() + std::chrono::milliseconds(5),
                std::chrono::milliseconds(5));
    while (fire_count < 2 && aio.Poll(true)) {
    }
    ASSERT_GE(fire_count, 2) << "iter " << iter;

    // Let it fire unobserved, then land a reschedule into that churn.  It
    // must take effect rather than being lost or leaving nothing armed.
    std::this_thread::sleep_for(std::chrono::milliseconds(7));
    int single_count = 0;
    Aio::Timer single(&aio);
    single.Schedule(
        aos::monotonic_clock::now() + std::chrono::milliseconds(5),
        [](Completion completion, void *ctx) {
          if (aos::IsOk(completion.status)) {
            ++*static_cast<int *>(ctx);
          }
        },
        &single_count);
    while (single_count < 1 && aio.Poll(true)) {
    }
    ASSERT_EQ(single_count, 1) << "iter " << iter;

    // Again unobserved, then cancel.  Both timers' destructors run at the
    // end of this iteration's scope -- the orphan path, with completions
    // possibly still in flight.
    std::this_thread::sleep_for(std::chrono::milliseconds(7));
    timer.Cancel();
  }
}

// Regression test for DestroyTimerState()'s "nothing in flight, free it
// right here" fast path, which used to skip unlinking the request from
// pending_dispatch_.
//
// Three things have to be true at once to reach it.  A timer's multishot
// poll must have been terminated by the kernel (CQ overflow -- see
// TimerPollsSurviveCqOverflow below), which is what sets request.done and
// leaves no cancel ack outstanding, so the fast path applies.  That timer's
// completion must still be sitting on pending_dispatch_, queued by
// DrainCompletions() but not yet dispatched.  And something must destroy it
// in that window -- which an earlier callback in the same dispatch batch
// can legitimately do.  Freeing it there leaves the dispatch loop holding a
// pointer into freed memory, which it walks into as soon as it pops that
// node.
//
// Constructed here by overflowing a tiny CQ with many simultaneous timers
// (producing terminated polls and queued completions in bulk) and giving
// every timer a callback that destroys all the others.  Without the
// unconditional UnlinkPendingDispatch() in DestroyTimerState() this fails
// as a use-after-free under ASAN, or as the intrusive list's own "removing
// a node that is not on this list" CHECK on a normal build.
//
// Note the shape predates the timerfd redesign: the old backend had the
// identical fast path, and reached it far more easily, because there a
// plain fired single-shot timer was `done` while queued.  CancelRequest()
// has always unlinked unconditionally for exactly this reason; the fast
// path just never did.
TEST_P(AioTest, DestroyTimerWithTerminatedPollAndQueuedCompletion) {
  constexpr int kTimers = 32;
  const uint32_t saved_depth = ::absl::GetFlag(FLAGS_aio_queue_depth);
  ::absl::SetFlag(&FLAGS_aio_queue_depth, 4);  // CQ = 8 slots.
  ScopedDeathTestWatchdog watchdog;
  {
    Aio aio;
    // Enable the ring before constructing timers; each construction arms a
    // poll, and 32 of them would otherwise outrun the 4-entry SQ.
    aio.Poll(false);

    std::vector<std::unique_ptr<Aio::Timer>> timers(kTimers);
    int fired = 0;
    // Destroying is held off until the second round of dispatch.  The first
    // Poll() drains the per-firing (F_MORE) completions, which leaves the
    // terminated polls' *terminal* completions still in the kernel's
    // overflow list; only the round after that carries them, and a
    // terminal is what sets request.done and arms the fast path.
    // Destroying during round one would just orphan everything, which is
    // the safe path and proves nothing.
    bool destroy_enabled = false;
    struct Ctx {
      std::vector<std::unique_ptr<Aio::Timer>> *timers = nullptr;
      int *fired = nullptr;
      const bool *destroy_enabled = nullptr;
      int self = 0;
    };
    std::vector<Ctx> ctxs(kTimers);

    const auto deadline =
        aos::monotonic_clock::now() + std::chrono::milliseconds(5);
    for (int i = 0; i < kTimers; ++i) {
      timers[i] = std::make_unique<Aio::Timer>(&aio);
      ctxs[i] = Ctx{&timers, &fired, &destroy_enabled, i};
      timers[i]->Schedule(
          deadline,
          [](Completion, void *raw) {
            auto *ctx = static_cast<Ctx *>(raw);
            ++*ctx->fired;
            if (!*ctx->destroy_enabled) return;
            // Destroy every other timer -- including any whose completion
            // is queued behind this one in the very same dispatch batch.
            for (int j = 0; j < static_cast<int>(ctx->timers->size()); ++j) {
              if (j == ctx->self) continue;
              (*ctx->timers)[j].reset();
            }
          },
          &ctxs[i]);
    }

    // All 32 expire with nothing draining, so the 8-slot CQ overflows and
    // the kernel terminates polls in bulk.
    std::this_thread::sleep_for(std::chrono::milliseconds(60));

    // Round one: exactly one Poll(), which drains the CQ's worth of
    // per-firing completions and no more.  Draining to empty here would
    // also consume the terminals, and dispatching a terminal re-arms the
    // poll (clearing request.done) -- which is precisely the state the fast
    // path needs, so it must still be pending when destroying starts.
    aio.Poll(true);
    // Round two onward: the terminated polls' terminal completions arrive,
    // several to a batch, and the first callback frees the rest
    // mid-dispatch-loop.
    destroy_enabled = true;
    const auto deadline_stop =
        aos::monotonic_clock::now() + std::chrono::seconds(2);
    while (aos::monotonic_clock::now() < deadline_stop) {
      if (!aio.Poll(false)) break;
    }
    EXPECT_GE(fired, 1);
  }
  ::absl::SetFlag(&FLAGS_aio_queue_depth, saved_depth);
}

// Regression test: the kernel terminates any multishot op whose auxiliary
// CQE cannot be posted -- a full CQ ends the op with a terminal completion
// (io_req_post_cqe() returning false; aux CQEs get no overflow-list
// fallback, unlike ordinary completions).  "Persistent" ops are therefore
// revocable at the kernel's convenience, so every multishot consumer must
// re-arm on an unexpected terminal completion.  Each timer's poll on its
// timerfd is one such consumer: a poll killed by an overflow and never
// re-armed leaves that timer silently dead, armed kernel-side with nobody
// watching.
//
// Forcing the overflow takes many timers rather than one fast one.  A
// single repeating timer cannot overflow anything now: an undrained
// timerfd stays readable without generating a new wait-queue edge, so a
// hundred banked expirations still produce exactly one CQE (and one read()
// reporting all of them).  Independent timers do produce independent CQEs,
// so 24 of them against an 8-slot CQ, left undrained, overflows it.  Every
// timer must still be firing afterward.
TEST_P(AioTest, TimerPollsSurviveCqOverflow) {
  constexpr int kTimers = 24;
  const uint32_t saved_depth = ::absl::GetFlag(FLAGS_aio_queue_depth);
  ::absl::SetFlag(&FLAGS_aio_queue_depth, 4);
  ScopedDeathTestWatchdog watchdog;
  {
    Aio aio;
    // Enable the ring before building the timers.  Each one arms a poll at
    // construction, and MaybeSubmit() cannot flush those until the ring is
    // enabled -- so without this, 24 constructions would queue 24 SQEs
    // against a 4-entry submission queue and CHECK-fail before the test got
    // anywhere near the CQ.
    aio.Poll(false);
    std::vector<int> fire_counts(kTimers, 0);
    std::vector<std::unique_ptr<RepeatingTimer>> timers;
    for (int i = 0; i < kTimers; ++i) {
      timers.push_back(std::make_unique<RepeatingTimer>(
          &aio, [&fire_counts, i](Completion completion) {
            if (aos::IsOk(completion.status)) {
              ++fire_counts[i];
            }
          }));
      // Staggered so their CQEs arrive spread out rather than as one batch
      // the ring might absorb between polls.
      timers.back()->Start(
          aos::monotonic_clock::now() + std::chrono::milliseconds(5 + i % 5),
          std::chrono::milliseconds(5));
    }

    // Overflow the 8-slot CQ: 24 timers firing every 5ms for 100ms, with
    // nothing draining.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Drain the backlog, then require every timer to fire well past
    // anything the pre-termination backlog alone could deliver.  A timer
    // whose poll died in the overflow stalls the loop here instead (caught
    // by the watchdog).
    while (aio.Poll(false)) {
    }
    std::vector<int> target(kTimers);
    for (int i = 0; i < kTimers; ++i) {
      target[i] = fire_counts[i] + 5;
    }
    const auto reached_targets = [&]() {
      for (int i = 0; i < kTimers; ++i) {
        if (fire_counts[i] < target[i]) return false;
      }
      return true;
    };
    while (!reached_targets() && aio.Poll(true)) {
    }
    for (int i = 0; i < kTimers; ++i) {
      EXPECT_GE(fire_counts[i], target[i])
          << "timer " << i << " stopped firing after the CQ overflow";
    }

    for (auto &timer : timers) {
      timer->Cancel();
    }
  }
  ::absl::SetFlag(&FLAGS_aio_queue_depth, saved_depth);
}

// Queue exhaustion must be predictable and front-loaded, not a bare "Out
// of SQEs" somewhere downstream: nothing can flush the submission queue
// before the first Run()/Poll() enables the ring, so every op armed before
// then holds a staged SQE, and exceeding --aio_queue_depth there is
// deterministic at the arming call.  ArmSqe()'s message must name the flag
// and the cause.  (The wakeup read and the legacy-epoll poll hold two of
// the four slots; the third timer's poll is the one that cannot stage.)
TEST_P(AioTest, PreRunArmingExceedsQueueDepthDeathTest) {
  if (!IsIoUring()) {
    GTEST_SKIP() << "Only io_uring stages SQEs against --aio_queue_depth "
                    "before the ring is enabled; no other backend has a "
                    "submission queue to exhaust.";
  }
  ::absl::SetFlag(&FLAGS_aio_queue_depth, 4);
  ScopedDeathTestWatchdog watchdog;
  EXPECT_DEATH(
      {
        Aio aio;
        Aio::Timer t1(&aio);
        Aio::Timer t2(&aio);
        MarkFatalStatement();
        Aio::Timer t3(&aio);
      },
      DiesAfterMarker("aio_queue_depth"));
}

// Regression test for UnregisterThreadSignalReceiver()'s "nothing in
// flight, free it right here" fast path, which used to skip unlinking the
// request from pending_dispatch_ -- the receiver twin of
// DestroyTimerWithTerminatedPollAndQueuedCompletion above, with the same
// three-way setup: the receiver's multishot poll terminated by CQ overflow
// (request.done set, no cancel acks outstanding, so the fast path applies),
// its terminal completion queued by DrainCompletions() but not yet
// dispatched, and an earlier callback in the same dispatch batch
// unregistering the receiver.  Freeing there leaves the dispatch loop
// holding a pointer into freed memory.  A regression fails as a
// use-after-free under ASAN, or as the intrusive list's own CHECK on a
// normal build.
TEST_P(AioTest, UnregisterReceiverWithTerminatedPollAndQueuedCompletion) {
  // 28 timers against an 8-slot CQ: 8 fit as auxiliary CQEs, the other 20
  // terminate their polls, and those 20 terminals plus the receiver's --
  // fired last, so ordered last -- land on the kernel's overflow list, 21
  // entries total.  The overflow refills the freed CQ 8 at a time, so the
  // final refill batch is [4 timer terminals, receiver terminal]: exactly
  // the shape where DrainCompletions() queues the receiver's terminal
  // (setting request.done) *behind* user-visible work in one batch.
  constexpr int kTimers = 28;
  const uint32_t saved_depth = ::absl::GetFlag(FLAGS_aio_queue_depth);
  ::absl::SetFlag(&FLAGS_aio_queue_depth, 4);  // CQ = 8 slots.
  ScopedDeathTestWatchdog watchdog;
  {
    Aio aio;
    // Enable the ring before arming everything -- see the sibling tests.
    aio.Poll(false);

    aos::ipc_lib::ThreadSignalReceiver sfd;
    aio.RegisterThreadSignalReceiver(&sfd, []() {});

    int fired = 0;
    std::vector<std::unique_ptr<Aio::Timer>> timers(kTimers);
    const auto deadline =
        aos::monotonic_clock::now() + std::chrono::milliseconds(5);
    for (int i = 0; i < kTimers; ++i) {
      timers[i] = std::make_unique<Aio::Timer>(&aio);
      timers[i]->Schedule(
          deadline, [](Completion, void *raw) { ++*static_cast<int *>(raw); },
          &fired);
    }

    // All 28 timers expire with nothing draining.  Only then fire the
    // receiver's poll, so its completion is ordered after every timer's.
    // (Under DEFER_TASKRUN nothing posts until the next Poll() enters the
    // kernel; the CQ overflows there, in wake order.)
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    pthread_kill(pthread_self(), aos::ipc_lib::kWakeupSignal);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Poll() dispatches one user-visible completion (one timer callback)
    // per call.  The 25th is the first entry of that final refill batch --
    // when it has been dispatched, the receiver's terminal has been drained
    // (request.done set, no cancel acks outstanding: the fast path's exact
    // condition) but is still sitting queued on pending_dispatch_.
    while (fired < 25 && aio.Poll(true)) {
    }
    // The exact count is derived from the kernel's CQ-overflow refill
    // batching, so it is only meaningful where that batching exists.  A
    // backend dispatching more than one callback per Poll() overshoots 25
    // and would fail this for a reason that says nothing about the bug.
    // Elsewhere the loop above still lands mid-dispatch and the
    // use-after-free coverage below still applies -- it just degrades to a
    // smoke test, which is why the rest of this test stays unguarded.
    //
    // EXPECT, not ASSERT: an early return here would leave the receiver
    // registered and turn a batching drift into ~IoUringImpl()'s
    // still-registered CHECK -- an abort -- instead of a clean failure.
    // The unregister below is safe in any state.
    if (IsIoUring()) {
      EXPECT_EQ(fired, 25);
    }

    // Unregister in that window.  The fast path frees the state here; if it
    // fails to unlink the queued terminal first, the dispatch loop below
    // walks into the freed node.
    aio.UnregisterThreadSignalReceiver(&sfd);

    const auto deadline_stop =
        aos::monotonic_clock::now() + std::chrono::seconds(2);
    while (aos::monotonic_clock::now() < deadline_stop) {
      if (!aio.Poll(false)) break;
    }
    EXPECT_EQ(fired, kTimers);
    // The unregistered receiver never read the signal; don't leave it
    // pending.
    sfd.ConsumeWakeup();
  }
  ::absl::SetFlag(&FLAGS_aio_queue_depth, saved_depth);
}

// Regression test: unregistering the receiver from inside its own
// dispatched callback.  The receiver trampoline keeps using its state
// *after* invoking the user callback -- the signalfd drain loop continues,
// then the revival check runs -- so UnregisterThreadSignalReceiver()'s
// nothing-in-flight fast path must not free the state mid-dispatch; it
// parks it as a drained orphan instead, recycled at the end of the
// outermost dispatch after every frame has unwound.  The fast path is only
// reachable mid-dispatch via a terminal completion (a live multishot is
// never `done`), so the CQ-overflow shape from the previous test forces
// one, with a signal pending so the trampoline actually runs the user
// callback.  A regression is a read of freed memory in the trampoline's
// drain loop, which needs ASAN to fail reliably.
TEST_P(AioTest, UnregisterReceiverFromOwnCallbackDuringTerminalDispatch) {
  constexpr int kTimers = 28;
  const uint32_t saved_depth = ::absl::GetFlag(FLAGS_aio_queue_depth);
  ::absl::SetFlag(&FLAGS_aio_queue_depth, 4);  // CQ = 8 slots.
  ScopedDeathTestWatchdog watchdog;
  {
    Aio aio;
    // Enable the ring before arming everything -- see the sibling tests.
    aio.Poll(false);

    aos::ipc_lib::ThreadSignalReceiver sfd;
    bool unregistered = false;
    aio.RegisterThreadSignalReceiver(&sfd, [&aio, &sfd, &unregistered]() {
      if (unregistered) return;
      unregistered = true;
      aio.UnregisterThreadSignalReceiver(&sfd);
    });

    std::vector<std::unique_ptr<Aio::Timer>> timers(kTimers);
    const auto deadline =
        aos::monotonic_clock::now() + std::chrono::milliseconds(5);
    for (int i = 0; i < kTimers; ++i) {
      timers[i] = std::make_unique<Aio::Timer>(&aio);
      timers[i]->Schedule(deadline, [](Completion, void *) {}, nullptr);
    }

    // Overflow the CQ with the timer firings, then wake the receiver's
    // poll against the full CQ so the kernel terminates the multishot --
    // its terminal completion is what makes the in-callback unregister
    // take the fast path.
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    pthread_kill(pthread_self(), aos::ipc_lib::kWakeupSignal);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    const auto deadline_stop =
        aos::monotonic_clock::now() + std::chrono::seconds(2);
    while (aos::monotonic_clock::now() < deadline_stop) {
      if (!aio.Poll(false)) break;
    }
    EXPECT_TRUE(unregistered);
    if (!unregistered) {
      // Kernel batching drift kept the terminal dispatch (and so the
      // in-callback unregister) from happening: clean up so the failure
      // stays a clean EXPECT instead of tripping ~IoUringImpl()'s
      // still-registered CHECK.
      aio.UnregisterThreadSignalReceiver(&sfd);
    }
    sfd.ConsumeWakeup();
  }
  ::absl::SetFlag(&FLAGS_aio_queue_depth, saved_depth);
}

// aio.h's constraint 2 lets a caller destroy the Aio with requests still
// pending -- including one whose completion was already drained and queued
// but never dispatched.  That AsyncRequest outlives the Aio and is legal
// to reuse with another one; stale internal dispatch-queue state would
// make the second Aio silently drop its callback forever.  Two readable
// AsyncReads and a single Poll() construct the queued-but-undispatched
// state deterministically on io_uring: the drain resolves both
// completions, the one-user-visible budget delivers only one.  (On epoll
// the second request is simply still pending at destruction -- the same
// contract, a different internal state.)
TEST_P(AioTest, ReuseRequestPendingAtDestruction) {
  // The regression mode is a silently-dropped callback: the poll loop below
  // then blocks forever, and the watchdog turns that into a clean death.
  ScopedDeathTestWatchdog watchdog;
  Pipe pipe_a;
  Pipe pipe_b;
  pipe_a.Write("a");
  pipe_b.Write("b");

  bool fired_a = false;
  bool fired_b = false;
  AsyncRequest request_a;
  AsyncRequest request_b;
  request_a.callback = [](Completion, void *ctx) {
    *static_cast<bool *>(ctx) = true;
  };
  request_a.context = &fired_a;
  request_b.callback = request_a.callback;
  request_b.context = &fired_b;

  char buf_a[8];
  char buf_b[8];
  {
    Aio aio;
    aio.AsyncRead(pipe_a.read_fd(), buf_a, &request_a);
    aio.AsyncRead(pipe_b.read_fd(), buf_b, &request_b);
    aio.Poll(true);
  }
  // At most one callback ran; at least one request is left over.
  ASSERT_FALSE(fired_a && fired_b);

  // Reuse a leftover request with a fresh Aio: its callback must fire.
  AsyncRequest *reuse = fired_a ? &request_b : &request_a;
  Pipe *reuse_pipe = fired_a ? &pipe_b : &pipe_a;
  bool *reuse_fired = fired_a ? &fired_b : &fired_a;
  // The first Aio may have consumed the byte before being destroyed; make
  // the fd readable again either way.
  reuse_pipe->Write("x");

  Aio aio2;
  char buf2[8];
  aio2.AsyncRead(reuse_pipe->read_fd(), buf2, reuse);
  while (!*reuse_fired && aio2.Poll(true)) {
  }
  EXPECT_TRUE(*reuse_fired);
}

// Cancel() of a request left over from a destroyed Aio -- here, one that Aio
// had queued but never attempted -- asks the loop, not the request, whether
// it is in flight.  Its list fields still name the dead instance's queue, so
// following them would write into memory the new Aio does not own.  It is not
// in flight here, and the cancel is a no-op, as io_uring's cancel of an op
// its ring does not hold is.
TEST_P(AioTest, CancelOfRequestLeftOverFromDestroyedAioIsNoOp) {
  Pipe pipe;
  Pipe ahead_pipe;
  bool fired = false;
  AsyncRequest request;
  request.callback = [](Completion, void *ctx) {
    *static_cast<bool *>(ctx) = true;
  };
  request.context = &fired;
  // Queued ahead of it, so its links point at another live node of the
  // destroyed queue rather than at nothing.
  AsyncRequest ahead;
  char buf[8];
  char ahead_buf[8];
  {
    Aio aio;
    aio.AsyncRead(ahead_pipe.read_fd(), ahead_buf, &ahead);
    aio.AsyncRead(pipe.read_fd(), buf, &request);
  }

  Aio aio2;
  aio2.Cancel(&request);
  // io_uring does have work here -- its cancel's own acknowledgement -- so
  // only the callback is checked.
  aio2.Poll(false);
  EXPECT_FALSE(fired);
}

// Under io_uring, the construct-here/run-there downgrade rebuilds the ring
// and re-arms every persistent registration -- including each live timer's
// multishot timerfd poll, whose only coverage is here.  The caller-visible
// shape (schedule timers on the constructing thread, drive Poll() from
// another, every timer still fires) is contract on every backend, so the
// test runs everywhere.
TEST_P(AioTest, DowngradeReArmsActiveTimers) {
  ScopedDeathTestWatchdog watchdog;

  auto aio = std::make_unique<Aio>();
  constexpr int kTimers = 3;
  std::vector<std::unique_ptr<Aio::Timer>> timers;
  int fired = 0;
  const auto deadline =
      aos::monotonic_clock::now() + std::chrono::milliseconds(30);
  for (int i = 0; i < kTimers; ++i) {
    timers.push_back(std::make_unique<Aio::Timer>(aio.get()));
    timers.back()->Schedule(
        deadline, [](Completion, void *ctx) { ++*static_cast<int *>(ctx); },
        &fired);
  }

  // First drive from a different thread: EnsureBound() sees the
  // construction-thread mismatch, downgrades, and must re-arm all three
  // timer polls on the rebuilt ring for them to ever fire.
  std::thread driver([&aio, &fired]() {
    while (fired < kTimers && aio->Poll(true)) {
    }
  });
  driver.join();
  EXPECT_EQ(fired, kTimers);
}

#if defined(__linux__)
// A Timer must be destroyed before its Aio: ~Timer dereferences the Aio's
// impl, so a Timer outliving its Aio is a use-after-free.  The destructor
// CHECKs active timers so the bug dies loudly at the Aio instead.
TEST_P(AioTest, TimerOutlivesAioDeathTest) {
  ScopedDeathTestWatchdog watchdog;
  EXPECT_DEATH(
      {
        auto aio = std::make_unique<Aio>();
        Aio::Timer timer(aio.get());
        MarkFatalStatement();
        aio.reset();
      },
      DiesAfterMarker("destroyed before its Aio"));
}

// aio.h documents "All Fds must be cleaned up before this class is
// destroyed"; EPoll::~EPoll() has always CHECKed it.  So does Aio now.
TEST_P(AioTest, DestroyWithFdRegisteredDeathTest) {
  ScopedDeathTestWatchdog watchdog;
  EXPECT_DEATH(
      {
        Pipe pipe;
        Aio aio;
        aio.OnReadable(pipe.read_fd(), []() {});
        MarkFatalStatement();
      },
      DiesAfterMarker("before destroying the Aio"));
}

TEST_P(AioTest, DestroyWithReceiverRegisteredDeathTest) {
  ScopedDeathTestWatchdog watchdog;
  EXPECT_DEATH(
      {
        aos::ipc_lib::ThreadSignalReceiver sfd;
        Aio aio;
        aio.RegisterThreadSignalReceiver(&sfd, []() {});
        MarkFatalStatement();
      },
      DiesAfterMarker("unregistered before destroying"));
}

// Completion::user_data is documented as "the opaque pointer supplied by
// the caller", and Timer::Schedule() takes no user_data -- so a timer
// completion must carry nullptr, not some internal state pointer.
TEST_P(AioTest, TimerCompletionUserDataIsNull) {
  Aio aio;
  Aio::Timer timer(&aio);

  struct Result {
    bool fired = false;
    void *user_data;
  } result;
  result.user_data = &result;  // Anything non-null.
  timer.Schedule(
      aos::monotonic_clock::now(),
      [](Completion completion, void *ctx) {
        auto *r = static_cast<Result *>(ctx);
        r->fired = true;
        r->user_data = completion.user_data;
      },
      &result);
  while (!result.fired && aio.Poll(true)) {
  }
  EXPECT_TRUE(result.fired);
  EXPECT_EQ(result.user_data, nullptr);
}

// A SetEvents() mask made only of bits the epoll translation drops (like
// EPOLLHUP, which the kernel always reports and never accepts in a mask)
// must keep the fd registered, exactly as EPoll::DoEpollCtl() keyed its
// add-vs-remove decision on the caller's untranslated mask.  Keying it on
// the translated mask unregistered the fd entirely, so the hangup below
// would never be delivered.
TEST_P(AioTest, SetEventsUntranslatedMaskKeepsRegistration) {
  Aio aio;
  Pipe pipe;

  int events_seen = 0;
  aio.OnEvents(pipe.read_fd(), [&aio, &pipe, &events_seen](uint32_t events) {
    EXPECT_TRUE(events & EPOLLERR);
    ++events_seen;
    // Stop the level-triggered hangup from re-firing.
    aio.DeleteFd(pipe.read_fd());
  });
  aio.SetEvents(pipe.read_fd(), EPOLLHUP);

  pipe.close_write_fd();
  // Non-blocking with a deadline: the regression mode is "the fd got
  // unregistered, no event will ever arrive", which must fail the EXPECT
  // below rather than park forever in a blocking Poll().
  const auto deadline_stop =
      aos::monotonic_clock::now() + std::chrono::seconds(2);
  while (events_seen == 0 && aos::monotonic_clock::now() < deadline_stop) {
    aio.Poll(false);
  }
  EXPECT_EQ(events_seen, 1);
}

// Pins the coalescing contract from Aio::RegisterThreadSignalReceiver()'s
// docs: every pending wakeup is consumed first, then the callback runs
// exactly once.  kWakeupSignal is a realtime signal, so the three sends
// below genuinely queue three pending siginfos; a per-signal dispatch
// would invoke the callback three times.
TEST_P(AioTest, PendingWakeupsCoalesceIntoOneCallback) {
  Aio aio;
  aos::ipc_lib::ThreadSignalReceiver sfd;

  // Bind and enable the loop before queueing the signals, so delivery
  // happens through the armed receiver rather than at registration time.
  aio.Poll(false);
  int count = 0;
  aio.RegisterThreadSignalReceiver(&sfd, [&count]() { ++count; });

  for (int i = 0; i < 3; ++i) {
    pthread_kill(pthread_self(), aos::ipc_lib::kWakeupSignal);
  }

  while (count == 0 && aio.Poll(true)) {
  }
  // Drain any further deliveries the backend queued for the same burst.
  const auto deadline_stop =
      aos::monotonic_clock::now() + std::chrono::milliseconds(200);
  while (aos::monotonic_clock::now() < deadline_stop) {
    if (!aio.Poll(false)) break;
  }
  EXPECT_EQ(count, 1);
  aio.UnregisterThreadSignalReceiver(&sfd);
}

// Pins the contract from Aio::RegisterThreadSignalReceiver()'s docs: once
// UnregisterThreadSignalReceiver() returns, the signalfd is the caller's
// again -- a wakeup that was still kernel-side at unregister time is
// discarded without reading the fd, so a successor receiver registered on
// the same fd owns every pending signal.
TEST_P(AioTest, SuccessorReceiverOwnsPendingWakeups) {
  Aio aio;
  aos::ipc_lib::ThreadSignalReceiver sfd;

  // Enable the loop, then arm a wakeup while nothing polls: the first
  // receiver's kernel-side poll fires, but its completion is never
  // dispatched before the unregister.
  aio.Poll(false);
  aio.RegisterThreadSignalReceiver(&sfd, []() {});
  pthread_kill(pthread_self(), aos::ipc_lib::kWakeupSignal);
  aio.UnregisterThreadSignalReceiver(&sfd);

  int count = 0;
  aio.RegisterThreadSignalReceiver(&sfd, [&count]() { ++count; });
  const auto deadline_stop =
      aos::monotonic_clock::now() + std::chrono::seconds(2);
  while (count == 0 && aos::monotonic_clock::now() < deadline_stop) {
    aio.Poll(false);
  }
  EXPECT_EQ(count, 1) << "the unregistered receiver consumed the wakeup";
  aio.UnregisterThreadSignalReceiver(&sfd);
}

// A receiver callback that unregisters its own receiver keeps executing
// after UnregisterThreadSignalReceiver() returns -- so the std::function
// (and its captures) must not be destroyed out from under it.  The old
// implementation assigned nullptr to the executing function; with a
// heap-allocated capture this is a use-after-free ASAN catches.
TEST_P(AioTest, ReceiverCallbackCapturesSurviveSelfUnregister) {
  Aio aio;
  aos::ipc_lib::ThreadSignalReceiver sfd;

  const std::string canary(64, 'x');  // Big enough to defeat SSO.
  bool checked = false;
  aio.RegisterThreadSignalReceiver(&sfd, [&aio, &sfd, canary, &checked]() {
    if (checked) return;
    checked = true;
    aio.UnregisterThreadSignalReceiver(&sfd);
    // The capture must still be alive after the unregister.
    EXPECT_EQ(canary, std::string(64, 'x'));
  });

  pthread_kill(pthread_self(), aos::ipc_lib::kWakeupSignal);
  while (!checked && aio.Poll(true)) {
  }
  EXPECT_TRUE(checked);
}

// A callback that deletes its own fd, closes it, and registers a fresh fd
// that reuses the same number -- all within one firing -- must not have
// the old event's remaining bits dispatched into the new registration.
// The dispatch holds the original registration and stops on its fd = -1
// tombstone; it never consults the fd number again, so reuse cannot
// mislead it.
TEST_P(AioTest, StaleEventBitsDoNotReachReusedFdNumber) {
  Aio aio;

  int fds[2];
  ABSL_PCHECK(pipe(fds) == 0);
  ABSL_PCHECK(fcntl(fds[0], F_SETFL, O_NONBLOCK) == 0);
  int new_fds[2] = {-1, -1};
  bool fired = false;
  int spurious_new_errors = 0;

  aio.OnReadable(fds[0], [&]() {
    if (fired) return;
    fired = true;
    char buf[8];
    ABSL_PCHECK(read(fds[0], buf, sizeof(buf)) >= 0);
    aio.DeleteFd(fds[0]);
    ABSL_PCHECK(close(fds[0]) == 0);
    // POSIX hands out the lowest free descriptor: the new pipe's read end
    // reuses the number we just closed.
    ABSL_PCHECK(pipe(new_fds) == 0);
    ABSL_CHECK_EQ(new_fds[0], fds[0]);
    aio.OnError(new_fds[0],
                [&spurious_new_errors]() { ++spurious_new_errors; });
  });

  // Data plus a closed write end: the firing carries readable and error
  // bits together, and only the readable one belongs to the callback
  // above -- the error bit must die with the old registration.
  ABSL_PCHECK(write(fds[1], "x", 1) == 1);
  ABSL_PCHECK(close(fds[1]) == 0);

  for (int i = 0; i < 100 && !fired; ++i) {
    aio.Poll(true);
  }
  EXPECT_TRUE(fired);
  aio.Poll(false);
  EXPECT_EQ(spurious_new_errors, 0)
      << "stale event bits reached the reused fd number's new registration";

  aio.DeleteFd(new_fds[0]);
  ABSL_PCHECK(close(new_fds[0]) == 0);
  ABSL_PCHECK(close(new_fds[1]) == 0);
}
#endif  // defined(__linux__)

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

// Helper function to fill up a pipe using OnWritable callbacks.
// It uses select() to query writability and runs the event loop until
// the pipe buffer is full and select() returns 0.
void FillPipe(Aio &aio, int fd) {
  while (true) {
    fd_set write_fds;
    FD_ZERO(&write_fds);
    FD_SET(fd, &write_fds);
    struct timeval timeout = {0, 0};
    int ret = select(fd + 1, nullptr, &write_fds, nullptr, &timeout);
    if (ret <= 0) {
      break;
    }
    {
      ScopedRealtime rt;
      aio.Poll(true);
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
  FillPipe(aio, pipe.write_fd());
  EXPECT_GT(number_writes, 0);

  // Now, if we try again, we shouldn't do anything because buffer is full.
  const int bytes_in_pipe = number_writes;
  number_writes = 0;
  FillPipe(aio, pipe.write_fd());
  EXPECT_EQ(number_writes, 0);

  // Empty the pipe, then fill it up again.
  for (int i = 0; i < bytes_in_pipe; ++i) {
    ASSERT_EQ(" ", pipe.Read(1));
  }
  number_writes = 0;
  FillPipe(aio, pipe.write_fd());
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

// Tests that a callback deleting its own fd doesn't leave the dispatch code
// executing out of a freed registration.
//
// DeleteFd() from inside one of the fd's own callbacks means one of that
// LegacyState's std::functions is the code currently executing -- freeing it
// inline would free the lambda's captures out from under the running
// callback -- the state must stay allocated, not merely tombstoned.  So
// DeleteFd() parks the state on retired_legacy_states_ instead, and it is
// freed at the end of that same outermost dispatch, after every callback
// has returned.  A regression is a read of freed memory, which needs ASAN
// to fail reliably; without it this test can pass while still being wrong.
//
// Deleting an fd from inside its own callback is an ordinary path -- a timer
// callback that destroys its timer lands in ~TimerState -> DeleteFd().
TEST_P(AioTest, DeleteFdFromOwnCallback) {
  Aio aio;

  Pipe pipe;
  bool fired = false;
  aio.OnReadable(pipe.read_fd(), [&aio, &pipe, &fired]() {
    fired = true;
    aio.DeleteFd(pipe.read_fd());
  });
  pipe.Write("x");

  aio.Poll(true);
  EXPECT_TRUE(fired);

  // Nothing left registered; a further Poll() finds no events.
  aio.Poll(false);
}

// EPOLLHUP is not an error event here, matching EPoll.  A hangup that
// arrives with data still buffered rides along with the readable bit, so a
// legacy reader gets that data first, from one firing.  (A bare hangup,
// once the data is gone, goes to the read handler -- see
// BareHangupReachesReadHandlerTest.)
TEST_P(AioTest, LegacyReadableIgnoresHangup) {
  Aio aio;
  Pipe pipe;

  int bytes_read = 0;
  aio.OnReadable(pipe.read_fd(), [&pipe, &bytes_read]() {
    char buf[16];
    const ssize_t n = read(pipe.read_fd(), buf, sizeof(buf));
    ABSL_PCHECK(n >= 0) << "read failed";
    bytes_read += n;
  });

  pipe.Write("x");
  pipe.close_write_fd();

  // One firing: EPOLLIN|EPOLLHUP, of which only the EPOLLIN half translates.
  aio.Poll(true);
  EXPECT_EQ(bytes_read, 1);

  aio.DeleteFd(pipe.read_fd());
}

// The same registration, polled past the hangup instead of stopping at it.
//
// There is no err_fn here, so a backend that reports a read-side hangup as an
// error rather than as readability dies on the err_fn CHECK the first time
// the loop comes back around.  The IOCP backend did exactly that: its
// readiness watch is one zero-byte WSARecv standing in for both of kqueue's
// filters, and it translated the peer's orderly shutdown to kErr regardless
// of which side had gone away.  An ordinary OnReadable()-only registration
// therefore killed the process the moment its peer closed.
//
// Every backend hands the bare hangup that follows the buffered byte to the
// read handler (epoll via RouteHangupOrError(), kqueue as EVFILT_READ|EV_EOF),
// but all this asserts is what matters here: it is not an error and does
// not require an error handler.
TEST_P(AioTest, LegacyReadableSurvivesPollingPastHangup) {
  Aio aio;
  Pipe pipe;

  int bytes_read = 0;
  aio.OnReadable(pipe.read_fd(), [&pipe, &bytes_read]() {
    char buf[16];
    const ssize_t n = read(pipe.read_fd(), buf, sizeof(buf));
    if (n > 0) {
      bytes_read += n;
    }
  });

  pipe.Write("x");
  pipe.close_write_fd();

  aio.Poll(true);
  // Past the buffered byte now: every one of these sees only the hangup.
  for (int i = 0; i < 5; ++i) {
    aio.Poll(false);
  }
  EXPECT_EQ(bytes_read, 1);

  aio.DeleteFd(pipe.read_fd());
}

// Registering a before-wait function from inside one is disallowed: the
// push_back could reallocate the vector out from under the executing
// std::function.  Pinned as a CHECK rather than left as silent UB (which
// is what EPoll's range-for did).
TEST_P(AioTest, BeforeWaitFromBeforeWaitDeathTest) {
  ScopedDeathTestWatchdog watchdog;
  EXPECT_DEATH(
      {
        Aio aio;
        aio.BeforeWait([&aio]() {
          aio.BeforeWait([]() {});
          aio.Quit();
        });
        MarkFatalStatement();
        aio.Run();
      },
      DiesAfterMarker("may not be called from a before-wait function"));
}

// Tests that calling Quit from a BeforeWait callback successfully stops the
// loop.
TEST_P(AioTest, QuitInBeforeWait) {
  Aio aio;
  aio.BeforeWait([&aio]() { aio.Quit(); });
  aio.Run();
}

// Tests that a Quit() concurrent with Run() startup always stops the loop, and
// that the loop is left stopped afterwards.
//
// Quit() is documented async-safe, so it may be called from another thread (or
// a signal handler) at any point relative to Run().  This covers the two
// interleavings that are reachable in practice: Quit() landing entirely before
// Run() starts, and Quit() landing once Run() is already blocked in Poll().
//
// It does NOT reliably reach the narrowest interleaving, where Quit() lands
// between Run() reading quit_requested_ and Run() storing to run_ -- that
// window is a couple of instructions wide and did not reproduce here even with
// a barrier and 2000 attempts.  Run()'s loop consults quit_requested_ as well
// as run_ precisely so that interleaving stays harmless: the store would
// clobber Quit()'s `run_ = false`, leaving quit_requested_ as the only record
// that a shutdown was asked for.  A regression there would hang rather than
// fail an assertion.
TEST_P(AioTest, QuitRacingWithRunStartup) {
  // 200 iterations, deliberately not more: every fresh Aio is a kernel ring
  // whose teardown costs at least one RCU grace period on a workqueue capped
  // at 64 concurrent teardowns (io_ring_exit_work + percpu_ref_kill's
  // call_rcu -- see the ADR's teardown-throughput section), regardless of
  // SINGLE_ISSUER.  This loop is the suite's dominant ring creator; at an
  // earlier 2000 iterations, massively parallel --runs_per_test invocations
  // outran kernel teardown fleet-wide and accumulated tens of GB of
  // unreclaimable slab per node.  Interleaving coverage lives in the
  // barrier below, not the count -- and at stress-run scale the count
  // multiplies out anyway (200 x 10000 runs = 2M interleavings).
  for (int i = 0; i < 200; ++i) {
    if (i % 50 == 0) {
      ThrottleOnKernelRingTeardown();
    }
    Aio aio;
    // Spin both threads up against a barrier so Quit() and Run() start
    // together; just spawning the thread lets Run() reach Poll() first every
    // time, which is the easy interleaving and not the interesting one.
    std::atomic<bool> go{false};
    std::thread quitter([&aio, &go]() {
      while (!go.load(std::memory_order_acquire)) {
      }
      aio.Quit();
    });
    go.store(true, std::memory_order_release);
    aio.Run();
    quitter.join();
  }
}

// Tests that unregistering a ThreadSignalReceiver correctly cancels the
// underlying multishot poll request and prevents any use-after-free or extra
// callbacks.
TEST_P(AioTest, UnregisterThreadSignalReceiverTest) {
  Aio aio;
  aos::ipc_lib::ThreadSignalReceiver sfd;

  int count = 0;
  aio.RegisterThreadSignalReceiver(&sfd, [&count]() { ++count; });

  // Send kWakeupSignal to our thread.
  pthread_kill(pthread_self(), aos::ipc_lib::kWakeupSignal);

  // Poll until the signal is handled.
  while (count == 0 && aio.Poll(true)) {
  }
  EXPECT_EQ(count, 1);

  // Unregister the ThreadSignalReceiver.
  aio.UnregisterThreadSignalReceiver(&sfd);

  // Send the signal again.
  pthread_kill(pthread_self(), aos::ipc_lib::kWakeupSignal);

  // Clean up the pending wakeup so it doesn't stay pending.
  sfd.ConsumeWakeup();

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
  // An Aio built in the parent stays fully functional in a forked child.
  // Registers both an fd event and a SignalFd, to exercise every
  // re-registration loop in HandleFork().
  //
  // On Windows there is no fork: the death-test child re-execs the binary and
  // rebuilds this state from scratch.  That exercises less, but it is exactly
  // what every death test in the tree relies on, so run it there too.
  Aio aio;
  Pipe pipe;
  aos::ipc_lib::ThreadSignalReceiver sfd;

  int signal_count = 0;
  int fd_count = 0;

  aio.RegisterThreadSignalReceiver(&sfd, [&]() { ++signal_count; });

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

  aio.UnregisterThreadSignalReceiver(&sfd);
  aio.DeleteFd(pipe.write_fd());
}

// Regression test for detecting a fork too late.  Fork detection used to run
// only inside Poll(): an operation issued by a forked child *before* its first
// Poll() went into the stale inherited ring/epoll and was silently discarded
// when Poll() rebuilt the backend, so it never completed and Poll() blocked
// forever.  Here the child issues an AsyncRead and makes data available, both
// before its first Poll(), and the read must complete.  The alarm() turns the
// hang into a loud ExitedWithCode(42) failure instead of a wedged suite.
//
// POSIX-only: this is about inherited state, and the Windows death-test child
// starts from a fresh process with nothing stale to detect.
#ifndef _WIN32
TEST_P(AioTest, ForkOperationBeforePollDeathTest) {
  Aio aio;
  Pipe pipe;

  EXPECT_EXIT(
      {
        alarm(30);

        AsyncRequest read_req;
        read_req.callback = [](Completion completion, void *) {
          EXPECT_TRUE(aos::IsOk(completion.status));
        };

        char read_buf[8] = {0};
        aio.AsyncRead(pipe.read_fd(), read_buf, &read_req);

        const char msg = 'x';
        ABSL_PCHECK(write(pipe.write_fd(), &msg, 1) == 1);

        while (!read_req.done && aio.Poll(true)) {
        }
        exit(read_req.done ? 42 : 1);
      },
      ::testing::ExitedWithCode(42), "");
}
#endif  // !_WIN32

// Bare fork() with waitpid(), so Windows is excluded the way the other
// bare-fork tests in this file are (see ForkOperationBeforePollDeathTest).
#ifndef _WIN32
// A timer armed in the parent must still fire in the *parent* after a forked
// child has run the same timer to completion.
//
// timerfds are ordinary descriptors, so a fork leaves both processes naming
// one kernel timer.  The child's read() on firing consumed the parent's
// expiration and the parent's timer then never fired at all -- and the child
// did not have to be malicious about it: Schedule(), Cancel() or just
// ~Timer on the way out re-armed or disarmed the parent's timer through the
// shared fd.  HandleFork() gives the child its own timerfds instead.
//
// TimerForkTest above covers the child half and passes either way, because
// nothing there ever asks the parent whether its own timer survived.
TEST_P(AioTest, TimerSurvivesAChildConsumingItTest) {
  Aio aio;
  Aio::Timer timer(&aio);

  int timer_count = 0;
  timer.Schedule(
      aos::monotonic_clock::now(),
      [](Completion, void *context) { ++*static_cast<int *>(context); },
      &timer_count);

  const pid_t child = fork();
  ABSL_PCHECK(child >= 0);
  if (child == 0) {
    // Drive the inherited timer to completion, which is what used to eat the
    // parent's expiration.
    while (timer_count == 0 && aio.Poll(true)) {
    }
    _exit(timer_count == 1 ? 42 : 1);
  }
  int status = 0;
  ABSL_PCHECK(waitpid(child, &status, 0) == child);
  ASSERT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 42) << "the child never saw its own timer";

  // The parent's timer is a different kernel object and is still armed.
  const auto deadline = aos::monotonic_clock::now() + std::chrono::seconds(2);
  while (timer_count == 0 && aos::monotonic_clock::now() < deadline) {
    aio.Poll(false);
  }
  EXPECT_EQ(timer_count, 1) << "the child consumed the parent's expiration";
}

// The loop's wakeup is an eventfd, and a forked child inherits it like any
// other descriptor.  If both processes keep polling the same one, either
// side's pending read consumes writes meant for the other -- so a Quit()
// aimed at the parent gets eaten by the child and the parent stays blocked.
//
// The child is made to poll only after the parent has already written its
// wakeup, which is the ordering that loses it; a pipe sequences the two
// rather than a sleep.
TEST_P(AioTest, WakeupSurvivesAChildConsumingItTest) {
  Aio aio;

  // Sequencing only, in both directions -- not part of what is under test.
  int to_child[2], to_parent[2];
  ABSL_PCHECK(pipe(to_child) == 0);
  ABSL_PCHECK(pipe(to_parent) == 0);

  const pid_t child = fork();
  ABSL_PCHECK(child >= 0);
  if (child == 0) {
    close(to_child[1]);
    close(to_parent[0]);
    // Wait until the parent's wakeup has been written.
    char byte = 0;
    const bool got_go = read(to_child[0], &byte, 1) == 1;
    // Drive the inherited loop.  Sharing the parent's eventfd, this is what
    // consumes the wakeup the parent just wrote for itself.
    for (int i = 0; i < 10; ++i) {
      aio.Poll(false);
    }
    const bool told_parent = write(to_parent[1], &byte, 1) == 1;
    _exit(got_go && told_parent ? 42 : 1);
  }
  close(to_child[0]);
  close(to_parent[1]);

  // Write the wakeup, then let the child run.
  aio.Quit();
  const char go = 'g';
  ABSL_PCHECK(write(to_child[1], &go, 1) == 1);
  char done = 0;
  ABSL_PCHECK(read(to_parent[0], &done, 1) == 1);

  int status = 0;
  ABSL_PCHECK(waitpid(child, &status, 0) == child);
  ASSERT_TRUE(WIFEXITED(status));
  ASSERT_EQ(WEXITSTATUS(status), 42) << "the child never got its go-ahead";

  // The parent's wakeup is its own, so it is still there to be seen.
  bool woke = false;
  const auto deadline = aos::monotonic_clock::now() + std::chrono::seconds(2);
  while (!woke && aos::monotonic_clock::now() < deadline) {
    woke = aio.Poll(false);
  }
  EXPECT_TRUE(woke) << "the child consumed the parent's wakeup";

  close(to_child[1]);
  close(to_parent[0]);
}
#endif  // !_WIN32

TEST_P(AioTest, TimerForkTest) {
  // A timer armed in the parent still fires in a forked child, i.e. the
  // backend re-registers pending timeouts when it recreates the loop.
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

// Regression test: in some real call paths (EventSchedulerScheduler::RunFor()
// under a gtest death test) constructing a Timer is the first thing a forked
// child does, and construction arms a poll -- so Initialize() has to run the
// same fork check every other ring-touching entry point does.  Without it the
// child stages an SQE into the stale inherited ring and dies with -EEXIST out
// of io_uring_submit().  Found via logger_test's
// LoggerDeathTest.CrashOnFallBehind, which blamed MaybeSubmit() under
// Aio::Timer::Timer() rather than anything timer-shaped.
TEST_P(AioTest, ConstructTimerInForkedChildTest) {
  Aio aio;
  // Drive the loop once in the parent, so the ring is bound and enabled
  // before the fork -- an unenabled ring would decline the submission on
  // its own and hide the bug.
  aio.Poll(false);

  EXPECT_EXIT(
      {
        // The child's very first Aio interaction is building a timer.
        Aio::Timer timer(&aio);
        int fired = 0;
        timer.Schedule(
            aos::monotonic_clock::now(),
            [](Completion, void *ctx) { ++*static_cast<int *>(ctx); }, &fired);
        while (fired == 0 && aio.Poll(true)) {
        }
        exit(fired == 1 ? 42 : 1);
      },
      ::testing::ExitedWithCode(42), "");
}

// Regression test: a repeating timer armed before a fork must keep
// repeating correctly in the child afterward, not silently revert to firing
// once and stopping.
TEST_P(AioTest, ForkDuringRepeatingTimerDeathTest) {
  Aio aio;

  int fire_count = 0;
  RepeatingTimer timer(&aio, [&fire_count](Completion completion) {
    if (aos::IsOk(completion.status)) {
      ++fire_count;
    }
  });
  timer.Start(aos::monotonic_clock::now() + std::chrono::milliseconds(10),
              std::chrono::milliseconds(10));

  // Let it fire a few times before forking.
  while (fire_count < 3 && aio.Poll(true)) {
  }
  ASSERT_GE(fire_count, 3);
  const int fires_before_fork = fire_count;

  EXPECT_EXIT(
      {
        ScopedDeathTestWatchdog watchdog;
        // If HandleFork() silently demoted this timer back to single-shot
        // (or dropped it entirely), fire_count would stall right where the
        // parent left it instead of continuing to climb.
        while (fire_count < fires_before_fork + 3 && aio.Poll(true)) {
        }
        exit(fire_count >= fires_before_fork + 3 ? 42 : 1);
      },
      ::testing::ExitedWithCode(42), "");
}

// Regression test for a kernel quirk in IORING_SETUP_DEFER_TASKRUN rings (see
// global_parent_fork_count's comment in aio_linux.cc): a repeating timer
// outstanding in the parent becomes uncancelable after any fork() the parent
// was party to -- the kernel's cancel/remove lookup returns -ENOENT even
// though the op is demonstrably still alive and firing.  A plain fork()+exec()
// where the child never touches this Aio (starterd's normal pattern) is
// enough.  CheckForParentFork() fixes it with one lazy resync at the next
// entry point after a fork.
TEST_P(AioTest, ForkChildNeverTouchesAioTest) {
  // Bounds worst-case runtime if this ever regresses: the reap loop's own
  // kMaxReapAttempts bound would otherwise take on the order of a minute to
  // trip (10000 iterations at this timer's 10ms period) before crashing
  // with an actionable message -- this just gets there faster.
  ScopedDeathTestWatchdog watchdog;

  Aio aio;

  int fire_count = 0;
  RepeatingTimer timer(&aio, [&fire_count](Completion completion) {
    if (aos::IsOk(completion.status)) {
      ++fire_count;
    }
  });
  timer.Start(aos::monotonic_clock::now() + std::chrono::milliseconds(10),
              std::chrono::milliseconds(10));

  while (fire_count < 3 && aio.Poll(true)) {
  }
  ASSERT_GE(fire_count, 3);

  pid_t pid = fork();
  ASSERT_GE(pid, 0);
  if (pid == 0) {
    // Deliberately never touches `aio` (or anything io_uring-related) --
    // just enough elapsed time for a few more periods to have passed, to
    // match the shape that reproduced the bug.
    usleep(40000);
    _exit(0);
  }
  int status = 0;
  ASSERT_EQ(waitpid(pid, &status, 0), pid);

  // Canceling this (via ~Timer() below) must not hang.
}

// Regression test for IoUringImpl::CheckSubmitterThread(): destroying an Aio
// from a different thread than the one that first called Run()/Poll() on it
// must die loudly.  io_uring-specific -- IORING_SETUP_SINGLE_ISSUER is what
// makes this a real constraint; the epoll backend has no such thing.
TEST_P(AioTest, DestroyFromWrongThreadDeathTest) {
  if (!IsIoUring()) {
    GTEST_SKIP() << "Same-thread destructor enforcement is io_uring-specific.";
  }
  EXPECT_DEATH(
      {
        // Construct and first-Poll() on the same thread (this one) -- no
        // auto-downgrade (see the next test) should be triggered, so
        // SINGLE_ISSUER's binding, and thus the enforcement, stays live.
        auto aio = std::make_unique<Aio>();
        aio->Poll(false);
        MarkFatalStatement();
        std::thread t([&aio]() { aio.reset(); });
        t.join();
      },
      DiesAfterMarker("different thread"));
}

// Regression test for the SINGLE_ISSUER-vs-PI-futex conflict: see
// documentation/adr/0001-aio-io-uring-single-issuer.md.  In short,
// IORING_SETUP_SINGLE_ISSUER wants destruction on Run()'s thread, but AOS's
// shared-memory queues (aos/ipc_lib/lockless_queue.cc's
// RobustOwnershipTracker) require every sender/watcher/pinner to be
// destroyed on its construction thread instead.  When an Aio's construction
// thread differs from the thread that first calls Run()/Poll() on it, the
// io_uring backend must downgrade away from SINGLE_ISSUER for that instance
// rather than enforce same-thread destruction.  Exercised here by
// constructing on this thread, Run()-ing on a worker thread, and destroying
// back on this thread again -- unlike the previous test, this must not
// crash.
TEST_P(AioTest, ConstructOnOneThreadRunOnAnotherTest) {
  auto aio = std::make_unique<Aio>();
  std::thread t([&aio]() { aio->Poll(false); });
  t.join();
  aio.reset();
}

// A raw AsyncRead/AsyncWrite submitted before the loop is first driven
// cannot survive the construct-here/run-there downgrade's ring rebuild:
// unlike the persistent registrations, there is no registry to re-arm it
// from.  The downgrade must refuse loudly rather than drop the request
// silently (it would otherwise just never complete).
TEST_P(AioTest, DowngradeWithRawRequestInFlightDeathTest) {
  if (!IsIoUring()) {
    GTEST_SKIP() << "The SINGLE_ISSUER downgrade is io_uring-specific.";
  }
  ScopedDeathTestWatchdog watchdog;
  EXPECT_DEATH(
      {
        Aio aio;
        Pipe pipe;
        AsyncRequest request;
        char buf[8];
        aio.AsyncRead(pipe.read_fd(), buf, &request);
        MarkFatalStatement();
        // First drive from a different thread triggers the downgrade.
        std::thread t([&aio]() { aio.Poll(false); });
        t.join();
      },
      DiesAfterMarker("AsyncRead/AsyncWrite requests in flight"));
}

// Regression test: legacy fd registration (OnReadable/OnWritable/OnError/
// OnEvents/EnableWritable/DisableWritable/SetEvents/DeleteFd) is backed by
// one shared, embedded epoll instance, not a per-fd io_uring request -- see
// documentation/adr/0001-aio-io-uring-single-issuer.md.  Mask changes and
// removal are plain epoll_ctl() calls, which never touch the ring at all,
// so none of them can be "the first call into an Aio" in the sense that
// triggers EnsureBound()/DowngradeFromSingleIssuer().  Exercised here the
// same way as the previous test (construct on this thread, touch the Aio
// from a different one) specifically via DeleteFd(), to pin down that this
// path in particular has no thread-binding dependency left at all.
TEST_P(AioTest, DeleteFdFromDifferentThreadTest) {
  Pipe pipe;
  auto aio = std::make_unique<Aio>();
  aio->OnReadable(pipe.read_fd(), []() {});

  std::thread t([&aio, &pipe]() { aio->DeleteFd(pipe.read_fd()); });
  t.join();
}

// Regression test: a receiver unregistered before the loop's first Poll(),
// with that first Poll() coming from a thread other than the one that built
// the loop, must leave nothing behind that stops a later registration of it
// from working.
//
// Every backend holds to that, so every backend runs it; io_uring's
// SINGLE_ISSUER ring is where it has teeth.  There the unregister orphans the
// receiver's state with a cancel SQE staged locally -- the ring is still
// disabled, so nothing has reached the kernel -- and the first drive from
// another thread makes DowngradeFromSingleIssuer() replace the ring.  The
// staged cancel and any completions the orphan was waiting for die with it,
// and ReArmPersistentRegistrations()'s scrub must recycle such orphans rather
// than leave them parked forever waiting for CQEs that can never arrive.  The
// re-registration afterward reuses the recycled state (see
// RegisterThreadSignalReceiver()'s freelist pop) and must still deliver
// wakeups.  The scoped watchdog turns a wedge into a clean failure if this
// regresses.
TEST_P(AioTest, UnregisterThreadSignalReceiverTriggersDowngradeTest) {
  ScopedDeathTestWatchdog watchdog;

  auto aio = std::make_unique<Aio>();
  aos::ipc_lib::ThreadSignalReceiver sfd;
  aio->RegisterThreadSignalReceiver(&sfd, []() {});

  std::thread t([&aio, &sfd]() {
    aio->UnregisterThreadSignalReceiver(&sfd);
    // The first drive of the loop, from a non-construction thread.  On a
    // SINGLE_ISSUER ring this is what actually reaches EnsureBound() ->
    // DowngradeFromSingleIssuer() (Unregister itself never drives the ring),
    // with the orphan parked.
    aio->Poll(false);
  });
  t.join();

  // The loop is not bound to that thread (a downgraded ring has no thread
  // binding), so this thread may drive it now.  Re-register -- reusing the
  // recycled orphan on io_uring -- and verify wakeups still flow end to end.
  int count = 0;
  aio->RegisterThreadSignalReceiver(&sfd, [&count]() { ++count; });
  pthread_kill(pthread_self(), aos::ipc_lib::kWakeupSignal);
  while (count == 0 && aio->Poll(true)) {
  }
  EXPECT_EQ(count, 1);
  aio->UnregisterThreadSignalReceiver(&sfd);
}

// A timer cancelled before a fork stays cancelled in the child, and the child
// can still use it.
//
// Timer::Cancel() is one timerfd_settime(2) on every backend, so nothing is in
// flight at the fork.  What the child has to get right is its rebuild:
// HandleFork() gives every timer a fresh timerfd and sets it again only if it
// is still armed.  The deadline here has already passed when it is cancelled,
// so a child that re-armed it anyway would fire it at once.  Then a
// re-Schedule() in the child has to fire normally.
//
// On Windows the death-test child re-execs from scratch rather than inheriting
// state, so there is nothing stale to recover; the code should still work, so
// we run it there too.  alarm() (the watchdog that turns a hang into a loud
// failure instead of wedging the suite) is the only POSIX-only piece.
TEST_P(AioTest, CancelTimerBeforeForkDeathTest) {
  Aio aio;
  Aio::Timer timer(&aio);

  int fired = 0;
  timer.Schedule(
      aos::monotonic_clock::now(),
      [](Completion, void *ctx) { ++*static_cast<int *>(ctx); }, &fired);
  // Cancelled before any Poll(), so the expired firing is never delivered.
  timer.Cancel();

  EXPECT_EXIT(
      {
        ScopedDeathTestWatchdog watchdog;
        // The first call here runs HandleFork().  The cancelled timer must
        // not come back.
        const auto stop =
            aos::monotonic_clock::now() + std::chrono::milliseconds(50);
        while (aos::monotonic_clock::now() < stop) {
          aio.Poll(false);
        }
        if (fired != 0) {
          exit(2);
        }
        timer.Schedule(
            aos::monotonic_clock::now(),
            [](Completion, void *ctx) { ++*static_cast<int *>(ctx); }, &fired);
        while (fired == 0 && aio.Poll(true)) {
        }
        exit(fired == 1 ? 42 : 1);
      },
      ::testing::ExitedWithCode(42), "");
}

// Regression test: HandleFork() re-arms every active timer in a single pass.
// With a shallow ring (--aio_queue_depth below the number of live timers) that
// pass queues more submission entries than the ring is deep; without draining
// the queue mid-pass, io_uring_get_sqe() returns null and the child aborts with
// "Out of SQEs".  Schedule more timers than the depth, fork, and require the
// child to rebuild and fire them all.
//
TEST_P(AioTest, ForkWithManyTimersDeathTest) {
  absl::FlagSaver flag_saver;

  // kNumTimers plus the wakeup read exceeds the kQueueDepth-entry submission
  // queue, so HandleFork()'s single reconstruction pass has to drain as it
  // fills.
  constexpr int kQueueDepth = 4;
  constexpr int kNumTimers = 6;
  static_assert(kNumTimers + 1 > kQueueDepth, "must overflow the submit queue");

  absl::SetFlag(&FLAGS_aio_queue_depth, kQueueDepth);

  Aio aio;
  std::vector<std::unique_ptr<Aio::Timer>> timers;
  for (int i = 0; i < kNumTimers; ++i) {
    timers.push_back(std::make_unique<Aio::Timer>(&aio));
    // Far-future so every timer is still pending at the fork, giving
    // HandleFork() all kNumTimers to re-arm.  Poll() flushes each submission so
    // the parent's own scheduling never overflows the queue -- only the
    // single-pass reconstruction does.
    timers.back()->Schedule(
        aos::monotonic_clock::now() + std::chrono::hours(1),
        [](Completion, void *) {}, nullptr);
    aio.Poll(false);
  }

  EXPECT_EXIT(
      {
        ScopedDeathTestWatchdog watchdog;
        // This Poll() rebuilds the ring and re-arms every pending request in a
        // single pass -- more submission entries than the ring is deep.
        // Surviving that pass, rather than aborting with "Out of SQEs", is the
        // assertion.
        aio.Poll(false);
        exit(42);
      },
      ::testing::ExitedWithCode(42), "");

  // Tear the timers down one at a time, draining between each.  Cancelling
  // costs two completions (the cancel itself, plus the timeout's -ECANCELED)
  // and the reap path deliberately doesn't advance the completion queue, so
  // destroying all kNumTimers back-to-back would overflow a queue this shallow.
  // That teardown limit is a separate concern from the reconstruction under
  // test, so keep it out of the way rather than tuning kNumTimers around it.
  for (auto &timer : timers) {
    timer.reset();
    aio.Poll(false);
  }
}

// Regression test for rescheduling an already-armed timer from an RT
// thread: it must not block, must not allocate, and the superseded
// schedule's callback must never reach the user -- only the new one.
//
// This is ordinary production usage rather than an edge case.
// ShmTimerHandler re-arms from inside its own callback on every firing, and
// external code reschedules live timers routinely; the historical failures
// here (TooBigConnect/StarterChainTest/TimerChangeParameters all hitting
// aos::CheckNotRealtime()) came from a reschedule path that could block.
// It is now a single timerfd_settime(2) on every backend, which is the
// strongest form of this guarantee available -- so the test is really
// pinning down that no future change reintroduces an asynchronous
// reschedule underneath it.
TEST_P(AioTest, RescheduleArmedTimerWhileRealtimeTest) {
  Aio aio;
  Aio::Timer timer(&aio);

  bool old_fired = false;
  int new_fire_count = 0;

  {
    ScopedRealtime rt;
    // Arm far enough out that it's still armed when superseded below.
    timer.Schedule(
        aos::monotonic_clock::now() + std::chrono::seconds(10),
        [](Completion, void *ctx) { *static_cast<bool *>(ctx) = true; },
        &old_fired);

    // Retarget the still-armed op to a much nearer deadline, with a
    // different callback.
    timer.Schedule(
        aos::monotonic_clock::now() + std::chrono::milliseconds(20),
        [](Completion completion, void *ctx) {
          if (aos::IsOk(completion.status)) {
            ++(*static_cast<int *>(ctx));
          }
        },
        &new_fire_count);

    while (new_fire_count < 1 && aio.Poll(true)) {
    }
  }

  EXPECT_FALSE(old_fired) << "Superseded schedule's callback ran anyway.";
  EXPECT_EQ(new_fire_count, 1);

  {
    ScopedRealtime rt;
    timer.Cancel();
  }
}

// Destroying a timer from an RT thread dies deterministically on every
// backend: DestroyTimerState() can free, so it CheckNotRealtime()s up
// front rather than crashing data-dependently under the malloc hook.
TEST_P(AioTest, DeleteTimerWhileRealtimeDeathTest) {
  Aio aio;

  EXPECT_DEATH(
      {
        std::optional<Aio::Timer> timer;
        timer.emplace(&aio);
        timer->Schedule(
            aos::monotonic_clock::now() + std::chrono::seconds(10),
            [](Completion, void *) {}, nullptr);
        MarkFatalStatement();
        ScopedRealtime rt;
        timer.reset();  // Destructor runs here, while marked realtime.
      },
      // Pin the death to CheckNotRealtime()'s CHECK, not just any abort --
      // an empty matcher would pass on unrelated crashes.
      DiesAfterMarker("GetIsRealtime"));
}

// Destroying an armed timer (off RT) orphans its state -- the poll on its
// timerfd cancelled, user callback stripped -- and continued polling drains
// and recycles it; a new timer then reuses the freelist, including its
// already-created timerfd.  ASAN checks the lifetime story end to end.
TEST_P(AioTest, DeleteArmedTimerOrphansAndRecycles) {
  if (!IsIoUring()) {
    GTEST_SKIP() << "Orphaned destruction is io_uring-specific.";
  }
  Aio aio;

  {
    Aio::Timer timer(&aio);
    timer.Schedule(
        aos::monotonic_clock::now() + std::chrono::milliseconds(5),
        [](Completion, void *) {}, nullptr);
    // Drive the loop so the poll is really armed kernel-side before the
    // destructor has to cancel it.
    aio.Poll(false);
  }  // Orphaned here.

  // Drain the orphan's terminal completion; the sweep recycles it.
  const auto deadline = aos::monotonic_clock::now() + std::chrono::seconds(2);
  while (aos::monotonic_clock::now() < deadline) {
    aio.Poll(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  // A fresh timer picks the state back up off the freelist and must work.
  Aio::Timer reused(&aio);
  int fired = 0;
  reused.Schedule(
      aos::monotonic_clock::now() + std::chrono::milliseconds(5),
      [](Completion, void *ctx) { ++*static_cast<int *>(ctx); }, &fired);
  while (fired == 0 && aio.Poll(true)) {
  }
  EXPECT_EQ(fired, 1);
}

// Confirms Schedule()/Cancel() (async) are unaffected by the enforcement
// above: both must keep working, unblocked, from an RT thread.
TEST_P(AioTest, AsyncCancelWhileRealtimeDoesNotDie) {
  Aio aio;
  Aio::Timer timer(&aio);
  timer.Schedule(
      aos::monotonic_clock::now() + std::chrono::seconds(10),
      [](Completion, void *) {}, nullptr);

  {
    ScopedRealtime rt;
    timer.Cancel();
  }
  // Reaching here without dying is the assertion.
}

// An unrecognized --aio_backend is fatal rather than a silent fallback.
// Linux-only: it is the only platform with more than one backend.
#ifdef __linux__
TEST(AioBackendFlagTest, UnknownBackendDies) {
  absl::FlagSaver flag_saver;
  ::absl::SetFlag(&FLAGS_aio_backend, "nonsense");
  EXPECT_DEATH({ Aio aio; }, "Unknown --aio_backend");
}
#endif

// A forked child that touches an Aio with a caller-submitted AsyncWrite in
// flight at the fork dies, rather than continuing into a shape with no correct
// answer -- see Aio::Impl::CheckNoRawRequestsInFlightOnFork().
//
// Checked where the child rebuilds, not in a pthread_atfork handler: fork()
// and fork()+exec() must stay fine, and only a child that goes on to use the
// loop has a problem.  EXPECT_DEATH's own fork() is the fork under test, and
// the write is aimed at a pipe nobody drains, which keeps it outstanding.
#ifndef _WIN32
TEST_P(AioTest, ForkedChildWithPendingAsyncWriteDies) {
  Aio aio;
  Pipe pipe;

  std::vector<char> filler(1 << 16, 'x');
  while (write(pipe.write_fd(), filler.data(), filler.size()) > 0) {
  }

  AsyncRequest request;
  request.callback = [](Completion, void *) {};
  std::vector<char> payload(64, 'y');
  aio.AsyncWrite(pipe.write_fd(), std::span<const char>(payload), &request);
  aio.Poll(false);
  ASSERT_FALSE(request.done) << "The write completed; nothing is pending.";

  EXPECT_DEATH({ aio.Poll(false); }, "in flight at the fork");

  // Drain the parent's own copy, which is unaffected by the fork.  No
  // DeleteFd(): a raw AsyncWrite never creates a legacy registration on the
  // io_uring backend, so there would be nothing to delete.
  aio.Cancel(&request);
  while (!request.done && aio.Poll(false)) {
  }
}

// A raw request stays in flight until its callback has run, so a fork between
// one resolving and its callback running is refused just like one with the
// I/O still outstanding.  One Poll() resolves both reads below and delivers
// one callback; the other is queued when the fork happens.  Letting the child
// through dropped that callback on io_uring, whose rebuild discards the
// dispatch queue, and delivered it on epoll -- in the child as well as the
// parent.
TEST_P(AioTest, ForkedChildWithUndeliveredRawCallbackDies) {
  Aio aio;
  Pipe first;
  Pipe second;
  first.Write("1");
  second.Write("2");

  int delivered = 0;
  char buf[2][1];
  AsyncRequest requests[2];
  for (AsyncRequest &request : requests) {
    request.callback = [](Completion, void *ctx) {
      ++*static_cast<int *>(ctx);
    };
    request.context = &delivered;
  }
  aio.AsyncRead(first.read_fd(), std::span<char>(buf[0]), &requests[0]);
  aio.AsyncRead(second.read_fd(), std::span<char>(buf[1]), &requests[1]);
  ASSERT_TRUE(aio.Poll(true));
  ASSERT_EQ(delivered, 1);
  ASSERT_TRUE(requests[0].done && requests[1].done)
      << "one Poll() did not resolve both reads";

  EXPECT_DEATH(
      {
        MarkFatalStatement();
        aio.Poll(false);
      },
      DiesAfterMarker("in flight at the fork"));

  // The parent's own copy is unaffected by the fork.
  while (delivered < 2 && aio.Poll(false)) {
  }
  EXPECT_EQ(delivered, 2);
}

// A one-shot timer that has fired and been delivered does not fire again in a
// forked child.  The child's rebuild gives every timer a fresh timerfd and
// sets it again only if it is still armed; re-arming every timer with a
// callback set the delivered one again at its old deadline, which had passed,
// so the child got a second firing at once.
TEST_P(AioTest, FiredTimerDoesNotFireAgainInForkedChildTest) {
  Aio aio;
  Aio::Timer timer(&aio);

  int fired = 0;
  timer.Schedule(
      aos::monotonic_clock::now(),
      [](Completion, void *ctx) { ++*static_cast<int *>(ctx); }, &fired);
  while (fired == 0 && aio.Poll(true)) {
  }
  ASSERT_EQ(fired, 1);

  EXPECT_EXIT(
      {
        ScopedDeathTestWatchdog watchdog;
        const auto stop =
            aos::monotonic_clock::now() + std::chrono::milliseconds(50);
        while (aos::monotonic_clock::now() < stop) {
          aio.Poll(false);
        }
        exit(fired == 1 ? 42 : 43);
      },
      ::testing::ExitedWithCode(42), "");

  // Nor again in the parent.
  aio.Poll(false);
  EXPECT_EQ(fired, 1);
}

// ForgetClosedFd() as a forked child's first call to the loop.  The fd is
// already closed, so the child's rebuild must not try to register it again:
// on io_uring the rebuild ran first and died adding the closed fd to the new
// epoll instance.
TEST_P(AioTest, ForgetClosedFdAsForkedChildsFirstCallTest) {
  Aio aio;
  Pipe pipe;
  aio.OnReadable(pipe.read_fd(), []() {});

  EXPECT_EXIT(
      {
        ABSL_PCHECK(close(pipe.read_fd()) == 0);
        aio.ForgetClosedFd(pipe.read_fd());
        aio.Poll(false);
        exit(42);
      },
      ::testing::ExitedWithCode(42), "");

  aio.DeleteFd(pipe.read_fd());
}

// Several fds closed in a forked child, all forgotten before the child does
// anything else with the loop.  ForgetClosedFd() only drops bookkeeping, so
// it must not rebuild the ring: the first one used to, on io_uring, and the
// rebuild re-added the other fd the child had already closed and died on it.
TEST_P(AioTest, ForgetSeveralClosedFdsInForkedChildTest) {
  Aio aio;
  Pipe a;
  Pipe b;
  aio.OnReadable(a.read_fd(), []() {});
  aio.OnReadable(b.read_fd(), []() {});

  EXPECT_EXIT(
      {
        ABSL_PCHECK(close(a.read_fd()) == 0);
        ABSL_PCHECK(close(b.read_fd()) == 0);
        aio.ForgetClosedFd(a.read_fd());
        aio.ForgetClosedFd(b.read_fd());
        aio.Poll(false);
        exit(42);
      },
      ::testing::ExitedWithCode(42), "");

  aio.DeleteFd(a.read_fd());
  aio.DeleteFd(b.read_fd());
}

// A forked child that closes a registered fd must ForgetClosedFd() it before
// its next use of the loop: the rebuild re-registers every fd it still knows
// on the child's own kernel state, and a closed one is a contract violation it
// can see.  Every backend dies here, with the same message.
TEST_P(AioTest, ForkedChildWithClosedUnforgottenFdDies) {
  Aio aio;
  Pipe a;
  Pipe b;
  aio.OnReadable(a.read_fd(), []() {});
  aio.OnReadable(b.read_fd(), []() {});

  EXPECT_DEATH(
      {
        ABSL_PCHECK(close(a.read_fd()) == 0);
        ABSL_PCHECK(close(b.read_fd()) == 0);
        aio.ForgetClosedFd(a.read_fd());
        MarkFatalStatement();
        aio.Poll(false);
      },
      DiesAfterMarker("while it was still registered with this Aio"));

  aio.DeleteFd(a.read_fd());
  aio.DeleteFd(b.read_fd());
}

// The lowest free descriptor number, which is the one the next descriptor this
// thread creates gets.  How the tests below find a timer's timerfd without a
// hook into the backend: probe, then construct the timer.
int LowestFreeFd() {
  const int probe = dup(STDERR_FILENO);
  ABSL_PCHECK(probe >= 0);
  ABSL_PCHECK(close(probe) == 0);
  return probe;
}

// The rebuild's closed-fd check knows the loop's own descriptors from the
// caller's.  A timer's timerfd was never the caller's to forget, so a child
// that closes it is told so, not told to ForgetClosedFd() it.
TEST_P(AioTest, ForkedChildClosingTimerFdDies) {
  Aio aio;
  const int timer_fd = LowestFreeFd();
  Aio::Timer timer(&aio);
  ASSERT_NE(fcntl(timer_fd, F_GETFD), -1)
      << "the timer did not take fd " << timer_fd;

  EXPECT_DEATH(
      {
        ABSL_PCHECK(close(timer_fd) == 0);
        MarkFatalStatement();
        aio.Poll(false);
      },
      DiesAfterMarker("which belongs to this Aio's own timer"));
}

// ForgetClosedFd() of the loop's own descriptor is a misuse on every backend,
// fork or no fork.
TEST_P(AioTest, ForgetClosedFdOfTimerFdDies) {
  Aio aio;
  const int timer_fd = LowestFreeFd();
  Aio::Timer timer(&aio);
  ASSERT_NE(fcntl(timer_fd, F_GETFD), -1)
      << "the timer did not take fd " << timer_fd;

  EXPECT_DEATH(aio.ForgetClosedFd(timer_fd),
               "belongs to this Aio's own timer, not to a registration");
}

// A forked child that closed a registered fd and then builds a timer: the
// fork check runs before the timerfd exists.  Otherwise the timerfd takes the
// closed number, the check finds it open, and the stale handler is
// re-registered on the timer's fd ("Duplicate in functions").
TEST_P(AioTest, ForkedChildMakingTimerAfterClosingRegisteredFdDies) {
  Aio aio;
  Pipe pipe;
  aio.OnReadable(pipe.read_fd(), []() {});

  EXPECT_DEATH(
      {
        ABSL_PCHECK(close(pipe.read_fd()) == 0);
        MarkFatalStatement();
        Aio::Timer timer(&aio);
      },
      DiesAfterMarker("while it was still registered with this Aio"));

  aio.DeleteFd(pipe.read_fd());
}

// Without a fork: a caller closes a registered fd without ForgetClosedFd(),
// and the loop's next timerfd takes its number.  That dies where the reuse
// happens, naming the cause, rather than as a collision later -- io_uring
// used to take it silently and report the timer's fd as the caller's.
TEST_P(AioTest, ClosedRegisteredFdReusedByTimerDies) {
  // Re-exec'd rather than forked: a forked death-test child would trip the
  // fork check first (the closed registered fd), and the point here is the
  // same sequence with no fork at all.
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  Aio aio;
  const int fd = LowestFreeFd();
  Pipe pipe;
  if (pipe.read_fd() != fd) {
    GTEST_SKIP() << "the pipe's read end did not take the lowest free fd";
  }
  aio.OnReadable(pipe.read_fd(), []() {});

  EXPECT_DEATH(
      {
        pipe.close_read_fd();
        MarkFatalStatement();
        Aio::Timer timer(&aio);
      },
      DiesAfterMarker("now this Aio's own timer, is still registered by the "
                      "caller"));

  aio.DeleteFd(pipe.read_fd());
}

// The same sequence done right: ForgetClosedFd() straight after close(), and
// the timer that then takes the number is the loop's own and nothing else.
TEST_P(AioTest, ForgetClosedFdThenTimerReusesNumberTest) {
  Aio aio;
  const int fd = LowestFreeFd();
  Pipe pipe;
  if (pipe.read_fd() != fd) {
    GTEST_SKIP() << "the pipe's read end did not take the lowest free fd";
  }
  aio.OnReadable(pipe.read_fd(), []() {});
  pipe.close_read_fd();
  aio.ForgetClosedFd(fd);

  Aio::Timer timer(&aio);
  ASSERT_NE(fcntl(fd, F_GETFD), -1) << "the timer did not take fd " << fd;
  EXPECT_DEATH(aio.ForgetClosedFd(fd),
               "belongs to this Aio's own timer, not to a registration");
}

#endif

// gtest's death-test child is a fork() of the test process on POSIX, and a
// fresh re-exec of the binary on Windows, which rebuilds the test's state
// from scratch rather than inheriting it.
constexpr bool kDeathTestChildIsForked =
#if defined(_WIN32)
    false;
#else
    true;
#endif

// Cancel() does not end a request's flight: it stays in flight until its
// Canceled callback has run.  So forking between the Cancel() and that
// delivery is the same mistake as forking with the I/O outstanding, and the
// child dies on its first use of the loop.
TEST_P(AioTest, ForkedChildWithUndeliveredCancelDies) {
  if (!kDeathTestChildIsForked)
    GTEST_SKIP() << "the death-test child is a fresh re-exec here, not a fork: "
                    "it re-arms and re-cancels the read and delivers the "
                    "cancel normally, so nothing stale is left to die on";
  Aio aio;
  Pipe pipe;

  AsyncRequest request;
  request.callback = [](Completion, void *) {};
  char buf[1];
  aio.AsyncRead(pipe.read_fd(), std::span<char>(buf), &request);
  aio.Poll(false);
  ASSERT_FALSE(request.done) << "The read completed; nothing is pending.";
  aio.Cancel(&request);

  EXPECT_DEATH(
      {
        MarkFatalStatement();
        aio.Poll(false);
      },
      DiesAfterMarker("in flight at the fork"));

  while (!request.done && aio.Poll(true)) {
  }
}

// The way to fork with a request that is no longer wanted: Cancel() it and
// Poll() until its Canceled callback has run.  Then nothing is in flight and
// the child can use the loop.
TEST_P(AioTest, ForkAfterCancelDeliveredIsFineTest) {
  Aio aio;
  Pipe pipe;

  bool delivered = false;
  AsyncRequest request;
  request.callback = [](Completion, void *ctx) {
    *static_cast<bool *>(ctx) = true;
  };
  request.context = &delivered;
  char buf[1];
  aio.AsyncRead(pipe.read_fd(), std::span<char>(buf), &request);
  aio.Poll(false);
  aio.Cancel(&request);
  while (!delivered && aio.Poll(true)) {
  }
  ASSERT_TRUE(delivered);

  EXPECT_EXIT(
      {
        aio.Poll(false);
        exit(42);
      },
      ::testing::ExitedWithCode(42), "");
}

INSTANTIATE_TEST_SUITE_P(AioBackends, AioTest,
                         ::testing::Values("io_uring", "epoll"),
                         [](const ::testing::TestParamInfo<std::string> &info) {
                           return info.param;
                         });

}  // namespace aos::testing
