#include "aos/events/shm_event_loop.h"

#ifndef _WIN32
#include <csignal>
#else
#include <windows.h>
#endif

#include <atomic>
#include <chrono>
#include <filesystem>
#include <memory>
#include <optional>
#include <string_view>
#include <thread>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "aos/events/aio.h"
#include "aos/events/event_loop_param_test.h"
#include "aos/events/test_message_generated.h"
#include "aos/events/thread_event_loop.h"
#include "aos/ipc_lib/memory_mapped_queue.h"
#include "aos/ipc_lib/process_local_queue.h"
#include "aos/ipc_lib/shm_mapping.h"
#include "aos/network/team_number.h"
#include "aos/realtime.h"

ABSL_DECLARE_FLAG(std::string, aio_backend);

namespace aos::testing {
namespace {
namespace chrono = ::std::chrono;

// Which QueueEventLoop a factory builds.  Everything in this file runs against
// both: they share an implementation, and what differs -- where the queues
// live and how long -- is exactly what these tests should not be able to
// tell apart, except where a test asks about it by name.
//
// One binary runs one kind.  These suites are expensive enough that doubling
// a binary would double its wall time, so AOS_THREAD_EVENT_LOOP selects the
// kind and the BUILD file makes a test target per kind and backend.
enum class LoopKind { kShm, kThread };

#if defined(AOS_THREAD_EVENT_LOOP)
constexpr LoopKind kLoopKind = LoopKind::kThread;
#else
constexpr LoopKind kLoopKind = LoopKind::kShm;
#endif

std::string LoopKindName(LoopKind kind) {
  switch (kind) {
    case LoopKind::kShm:
      return "shm";
    case LoopKind::kThread:
      return "thread";
  }
  return "unknown";
}

// Empties the test's queues.  Shared memory is wiped by unlinking the test's
// queues under --shm_base; process-local queues have nothing to unlink and are
// reset in place.  Those are reset for either kind, since a few tests build a
// ThreadEventLoop next to a ShmEventLoop, and a queue one leaves behind would
// meet the next test's configuration of the channel in the same process.
void EmptyQueues(LoopKind kind, const Configuration *configuration) {
  if (kind == LoopKind::kShm) {
    // Only this test's own channels, since run outside bazel the shm base is
    // the live system's.
    const std::string shm_base = absl::GetFlag(FLAGS_shm_base);
    for (const Channel *channel : *configuration->channels()) {
      ipc_lib::UnlinkShm(ipc_lib::ShmPath(shm_base, channel));
    }
  }
  ipc_lib::ProcessLocalQueue::ResetAll();
}

std::unique_ptr<QueueEventLoop> MakeLoop(LoopKind kind,
                                         const Configuration *configuration) {
  switch (kind) {
    case LoopKind::kShm:
      return std::make_unique<ShmEventLoop>(configuration);
    case LoopKind::kThread:
      return std::make_unique<ThreadEventLoop>(configuration);
  }
  return nullptr;
}

std::unique_ptr<QueueEventLoop> MakeLoop(LoopKind kind,
                                         const Configuration *configuration,
                                         Aio *aio) {
  switch (kind) {
    case LoopKind::kShm:
      return std::make_unique<ShmEventLoop>(configuration, aio);
    case LoopKind::kThread:
      return std::make_unique<ThreadEventLoop>(configuration, aio);
  }
  return nullptr;
}

class ShmEventLoopTestFactory : public EventLoopTestFactory {
 public:
  // The backend name --aio_backend takes, rather than a bool, so adding a
  // backend does not mean re-teaching this a new flag shape.
  //
  // With borrow_aio, every event loop is built on an Aio this owns, and Run()
  // drives the primary's Aio around Startup() and Shutdown() instead of
  // calling Run() on the event loop.  Anything Run() does that those two miss
  // then fails the suite.
  explicit ShmEventLoopTestFactory(std::string backend = "io_uring",
                                   LoopKind kind = LoopKind::kShm,
                                   bool borrow_aio = false)
      : backend_(std::move(backend)), kind_(kind), borrow_aio_(borrow_aio) {
    // Clean up anything left there before.
    EmptyQueues(kind_, configuration());
  }

  ~ShmEventLoopTestFactory() {
    absl::SetFlag(&FLAGS_override_hostname, "");
    EmptyQueues(kind_, configuration());
  }

  ::std::unique_ptr<EventLoop> Make(std::string_view name) override {
    return MakeQueueEventLoop(name);
  }

  ::std::unique_ptr<EventLoop> MakePrimary(std::string_view name) override {
    ::std::unique_ptr<QueueEventLoop> loop = MakeQueueEventLoop(name);
    primary_event_loop_ = loop.get();
    primary_aio_ = borrow_aio_ ? aios_.back().get() : nullptr;
    return loop;
  }

  Status Run() override {
    ABSL_CHECK(primary_event_loop_ != nullptr);
    if (primary_aio_ == nullptr) {
      return primary_event_loop_->Run();
    }
    // Deliberately not primary_event_loop_->Run(): the point is that this
    // works without it.
    primary_event_loop_->Startup();
    primary_aio_->Run();
    return primary_event_loop_->Shutdown();
  }

  std::unique_ptr<ExitHandle> MakeExitHandle() override {
    ABSL_CHECK(primary_event_loop_ != nullptr);
    return primary_event_loop_->MakeExitHandle();
  }

  void Exit() override {
    ABSL_CHECK(primary_event_loop_ != nullptr);
    primary_event_loop_->Exit();
  }

  void SleepFor(::std::chrono::nanoseconds duration) override {
    ::std::this_thread::sleep_for(duration);
  }

 private:
  ::std::unique_ptr<QueueEventLoop> MakeQueueEventLoop(std::string_view name) {
    absl::SetFlag(&FLAGS_aio_backend, backend_);
    if (configuration()->has_nodes()) {
      absl::SetFlag(&FLAGS_override_hostname,
                    std::string(my_node()->hostname()->string_view()));
    }
    ::std::unique_ptr<QueueEventLoop> loop;
    if (borrow_aio_) {
      aios_.push_back(std::make_unique<Aio>());
      loop = MakeLoop(kind_, configuration(), aios_.back().get());
    } else {
      loop = MakeLoop(kind_, configuration());
    }
    loop->set_name(name);
    return loop;
  }

  ::aos::QueueEventLoop *primary_event_loop_ = nullptr;
  // The primary's Aio when borrow_aio_, which Run() drives.
  Aio *primary_aio_ = nullptr;
  const std::string backend_;
  const LoopKind kind_;
  const bool borrow_aio_;
  // One per event loop when borrow_aio_.  The tests destroy their event loops
  // before this factory, so these outlive every loop built on them.
  std::vector<std::unique_ptr<Aio>> aios_;
};

auto CommonParameters(std::string backend) {
  return ::testing::Combine(
      ::testing::Values([backend = std::move(backend)]() {
        return new ShmEventLoopTestFactory(backend, kLoopKind);
      }),
      ::testing::Values(ReadMethod::COPY, ReadMethod::PIN),
      ::testing::Values(DoTimingReports::kYes, DoTimingReports::kNo));
}

// The same suite on borrowed Aios.  One read method and one timing setting are
// enough: what this covers is Startup() and Shutdown(), which neither changes.
// No timing reports, since their 1 Hz rescan of every watcher would hide a
// dead wakeup path behind a slow one.
auto BorrowedParameters(std::string backend) {
  return ::testing::Combine(::testing::Values([backend = std::move(backend)]() {
                              return new ShmEventLoopTestFactory(
                                  backend, kLoopKind, /*borrow_aio=*/true);
                            }),
                            ::testing::Values(ReadMethod::COPY),
                            ::testing::Values(DoTimingReports::kNo));
}

// The suite names say Shm whichever kind this binary was built for; the
// target name is what says which.
#ifdef __linux__
#ifndef AOS_EPOLL_ONLY
INSTANTIATE_TEST_SUITE_P(ShmEventLoopCommonTestIoUring, AbstractEventLoopTest,
                         CommonParameters("io_uring"));
INSTANTIATE_TEST_SUITE_P(ShmEventLoopCommonDeathTestIoUring,
                         AbstractEventLoopDeathTest,
                         CommonParameters("io_uring"));
INSTANTIATE_TEST_SUITE_P(ShmEventLoopBorrowedAioTestIoUring,
                         AbstractEventLoopTest, BorrowedParameters("io_uring"));
INSTANTIATE_TEST_SUITE_P(ShmEventLoopBorrowedAioDeathTestIoUring,
                         AbstractEventLoopDeathTest,
                         BorrowedParameters("io_uring"));
#endif  // AOS_EPOLL_ONLY

#ifndef AOS_IO_URING_ONLY
INSTANTIATE_TEST_SUITE_P(ShmEventLoopCommonTestEpoll, AbstractEventLoopTest,
                         CommonParameters("epoll"));
INSTANTIATE_TEST_SUITE_P(ShmEventLoopCommonDeathTestEpoll,
                         AbstractEventLoopDeathTest, CommonParameters("epoll"));
INSTANTIATE_TEST_SUITE_P(ShmEventLoopBorrowedAioTestEpoll,
                         AbstractEventLoopTest, BorrowedParameters("epoll"));
INSTANTIATE_TEST_SUITE_P(ShmEventLoopBorrowedAioDeathTestEpoll,
                         AbstractEventLoopDeathTest,
                         BorrowedParameters("epoll"));
#endif  // AOS_IO_URING_ONLY
#elif defined(_WIN32)
INSTANTIATE_TEST_SUITE_P(ShmEventLoopCommonTestIocp, AbstractEventLoopTest,
                         CommonParameters("iocp"));
INSTANTIATE_TEST_SUITE_P(ShmEventLoopCommonDeathTestIocp,
                         AbstractEventLoopDeathTest, CommonParameters("iocp"));
INSTANTIATE_TEST_SUITE_P(ShmEventLoopBorrowedAioTestIocp, AbstractEventLoopTest,
                         BorrowedParameters("iocp"));
INSTANTIATE_TEST_SUITE_P(ShmEventLoopBorrowedAioDeathTestIocp,
                         AbstractEventLoopDeathTest,
                         BorrowedParameters("iocp"));
#else
INSTANTIATE_TEST_SUITE_P(ShmEventLoopCommonTestKQueue, AbstractEventLoopTest,
                         CommonParameters("kqueue"));
INSTANTIATE_TEST_SUITE_P(ShmEventLoopCommonDeathTestKQueue,
                         AbstractEventLoopDeathTest,
                         CommonParameters("kqueue"));
INSTANTIATE_TEST_SUITE_P(ShmEventLoopBorrowedAioTestKQueue,
                         AbstractEventLoopTest, BorrowedParameters("kqueue"));
INSTANTIATE_TEST_SUITE_P(ShmEventLoopBorrowedAioDeathTestKQueue,
                         AbstractEventLoopDeathTest,
                         BorrowedParameters("kqueue"));
#endif

}  // namespace

bool IsRealtime() {
  int scheduler;
#if defined(__linux__)
  ABSL_PCHECK((scheduler = sched_getscheduler(0)) != -1);
#else
  scheduler = aos::GetCurrentThreadSchedulingPolicy();
#endif

  {
    // If we are RT, logging the scheduler will crash us.  Mark that we just
    // don't care.
    aos::ScopedNotRealtime nrt;
    ABSL_LOG(INFO) << "scheduler is " << scheduler;
  }

  const bool result = scheduler == SCHED_FIFO || scheduler == SCHED_RR;
  // Confirm that the scheduler matches AOS' interpretation of if we are
  // realtime or not.
  if (result) {
    aos::CheckRealtime();
  } else {
    aos::CheckNotRealtime();
  }
  return result;
}

class ShmEventLoopTest : public ::testing::TestWithParam<
                             std::tuple<ReadMethod, std::string, LoopKind>> {
 public:
  ShmEventLoopTest()
      : factory_(std::get<1>(GetParam()), std::get<2>(GetParam())) {
    if (std::get<0>(GetParam()) == ReadMethod::PIN) {
      factory_.PinReads();
    }
  }

  ShmEventLoopTestFactory *factory() { return &factory_; }

  // Helper functions for testing when a fetcher cannot fetch the next message
  // because it was overwritten
  void TestNextMessageNotAvailable(const bool skip_timing_report) {
    auto loop1 = factory()->MakePrimary("loop1");
    if (skip_timing_report) {
      loop1->SkipTimingReport();
    }
    auto fetcher = loop1->MakeFetcher<TestMessage>("/test");
    auto loop2 = factory()->Make("loop2");
    auto sender = loop2->MakeSender<TestMessage>("/test");
    bool ran = false;
    // Set just before the death test.  Where gtest cannot fork (Windows) the
    // death test runs this whole test again in a child process, so the
    // statement takes as long as the test has so far, and the phased loop
    // would make up all of those periods in one burst after it, faster than
    // the channel allows.
    bool stop_sending = false;
    // AddPhasedLoop()'s callback takes the number of cycles elapsed since
    // the last call: normally 1, but more if a scheduling delay made this
    // invocation late enough to skip one or more periods.  Sending that
    // many messages, instead of always just 1, guarantees the total sent
    // by a given wall-clock time tracks elapsed_time / period: any period
    // skipped in real time is made up in a burst next time this runs, so
    // no messages are lost to scheduling delay.  That matters below, where
    // the deadline assumes a specific number of messages have been sent by
    // then.
    loop1->AddPhasedLoop(
        [&sender, &stop_sending](int cycles) {
          if (stop_sending) {
            return;
          }
          for (int i = 0; i < cycles; ++i) {
            auto builder = sender.MakeBuilder();
            TestMessage::Builder test_builder(*builder.fbb());
            test_builder.add_value(0);
            builder.CheckOk(builder.Send(test_builder.Finish()));
          }
        },
        std::chrono::milliseconds(2));
    // "/test"'s queue depth is frequency(800) * channel_storage_duration
    // (2s default, EventLoopTestFactory doesn't override it for "/test") =
    // 1600 slots.  At the 2ms send period above (500/s), the queue is
    // guaranteed to have wrapped past after 1600/500 = 3.2s -- with the
    // catch-up sending above, that holds regardless of scheduling delay, so
    // this deadline only needs a small margin over the 3.2s minimum, not a
    // large one hedging against lost messages.
    loop1
        ->AddTimer([this, &fetcher, &ran, &stop_sending]() {
          stop_sending = true;
          EXPECT_DEATH(fetcher.FetchNext(),
                       "The next message is no longer "
                       "available.*\"/test\".*\"aos\\.TestMessage\"");
          factory()->Exit();
          ran = true;
        })
        ->Schedule(loop1->monotonic_now() + std::chrono::seconds(5));
    factory()->Run();
    EXPECT_TRUE(ran);
  }
  void TestNextMessageNotAvailableNoRun(const bool skip_timing_report) {
    auto loop1 = factory()->MakePrimary("loop1");
    if (skip_timing_report) {
      loop1->SkipTimingReport();
    }
    auto fetcher = loop1->MakeFetcher<TestMessage>("/test");
    auto loop2 = factory()->Make("loop2");
    auto sender = loop2->MakeSender<TestMessage>("/test");
    time::PhasedLoop phased_loop(std::chrono::milliseconds(2),
                                 loop2->monotonic_now());
    for (int i = 0; i < 2000; ++i) {
      auto builder = sender.MakeBuilder();
      TestMessage::Builder test_builder(*builder.fbb());
      test_builder.add_value(0);
      builder.CheckOk(builder.Send(test_builder.Finish()));
      phased_loop.SleepUntilNext();
    }
    EXPECT_DEATH(fetcher.FetchNext(),
                 "The next message is no longer "
                 "available.*\"/test\".*\"aos\\.TestMessage\"");
  }

 private:
  ShmEventLoopTestFactory factory_;
};

using ShmEventLoopDeathTest = ShmEventLoopTest;

// Tests that we don't leave the calling thread realtime when calling Send
// before Run.
TEST_P(ShmEventLoopTest, SendBeforeRun) {
  auto loop = factory()->MakePrimary("primary");
  loop->SetRuntimeRealtimePriority(1);

  auto loop2 = factory()->Make("loop2");
  loop2->SetRuntimeRealtimePriority(2);
  loop2->MakeWatcher("/test", [](const TestMessage &) {});
  // Need the other one running for its watcher to record in SHM that it wants
  // wakers to boost their priority, so leave it running in a thread for this
  // test.
  std::thread loop2_thread(
      [&loop2]() { static_cast<QueueEventLoop *>(loop2.get())->Run(); });
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto sender = loop->MakeSender<TestMessage>("/test");
  EXPECT_FALSE(IsRealtime());
  {
    aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
    TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
    builder.add_value(200);
    msg.CheckOk(msg.Send(builder.Finish()));
  }
  EXPECT_FALSE(IsRealtime());

  static_cast<QueueEventLoop *>(loop2.get())->Exit();
  loop2_thread.join();
}

#ifndef _WIN32
namespace {
void IgnoreSignalForTest(int) {}

void (*SigtermHandler())(int) {
  struct sigaction action{};
  ABSL_PCHECK(sigaction(SIGTERM, nullptr, &action) == 0);
  return action.sa_handler;
}
}  // namespace

// Tests that Startup() replaces the SIGTERM handler only when handle_signals is
// set, and that Shutdown() restores it.
TEST_P(ShmEventLoopTest, HandleSignals) {
  struct sigaction ignore{};
  ignore.sa_handler = &IgnoreSignalForTest;
  sigemptyset(&ignore.sa_mask);
  struct sigaction previous{};
  ASSERT_EQ(sigaction(SIGTERM, &ignore, &previous), 0);

  for (const bool handle_signals : {true, false}) {
    auto loop = factory()->MakePrimary("primary");
    static_cast<QueueEventLoop *>(loop.get())
        ->set_handle_signals(handle_signals);
    void (*while_running)(int) = nullptr;
    loop->OnRun([&]() {
      while_running = SigtermHandler();
      factory()->Exit();
    });
    factory()->Run();

    EXPECT_EQ(while_running == &IgnoreSignalForTest, !handle_signals);
    EXPECT_EQ(SigtermHandler(), &IgnoreSignalForTest);
  }

  ASSERT_EQ(sigaction(SIGTERM, &previous, nullptr), 0);
}
#endif

#if defined(_WIN32)
namespace {

// The event loop HoldCtrlC() stops.
std::atomic<ShmEventLoop *> ctrl_c_loop{nullptr};

// A console control handler that stops ctrl_c_loop, then holds the dispatch
// long enough for Run() to return and the event loop to unregister while it
// is still in progress, before passing the event on.
BOOL WINAPI HoldCtrlC(DWORD) {
  ctrl_c_loop.load()->Exit();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  return FALSE;
}

}  // namespace

// A real console Ctrl-C stops a running event loop, even one with nothing due
// to wake it.  Under Python the CRT's handler only sets a flag, so this is
// also what lets Ctrl-C stop a Python ShmEventLoop that is watching quiet
// channels.
//
// With hold, the last event loop unregisters while a dispatch is still in
// progress, which deadlocked when that removed the handler: Windows holds its
// handler-list lock for the whole dispatch.
TEST_P(ShmEventLoopTest, ConsoleCtrlCExitsRun) {
  // In a death-test child, which on Windows is a fresh process running only
  // this test: a console of its own, so the event reaches it and nothing
  // else, and no event loop registered before the console changed, which
  // would have lost the handler with the old console.
  const auto child = [this](bool hold) {
    FreeConsole();
    ABSL_CHECK(AllocConsole()) << GetLastError();
    ABSL_CHECK(SetConsoleCtrlHandler(nullptr, FALSE)) << GetLastError();

    std::unique_ptr<EventLoop> loop = factory()->MakePrimary("primary");
    loop->MakeWatcher("/test", [](const TestMessage &) {});
    std::atomic<bool> gave_up{false};
    TimerHandler *const give_up = loop->AddTimer([&]() {
      gave_up = true;
      factory()->Exit();
    });
    loop->OnRun([&]() {
      give_up->Schedule(loop->monotonic_now() + std::chrono::seconds(10));
      if (hold) {
        // Added after the event loop's own handler, so it runs first.
        ctrl_c_loop = static_cast<ShmEventLoop *>(loop.get());
        ABSL_CHECK(SetConsoleCtrlHandler(&HoldCtrlC, TRUE)) << GetLastError();
      }
      ABSL_CHECK(GenerateConsoleCtrlEvent(CTRL_C_EVENT, 0)) << GetLastError();
    });
    (void)factory()->Run();
    std::exit(gave_up ? 1 : 0);
  };
  EXPECT_EXIT(child(false), ::testing::ExitedWithCode(0), "")
      << "Ctrl-C did not stop the event loop";
  EXPECT_EXIT(child(true), ::testing::ExitedWithCode(0), "")
      << "Ctrl-C deadlocked against the event loop unregistering";
}

// Only an absolute --shm_base means one directory to every process.
TEST_P(ShmEventLoopDeathTest, ShmBaseThatIsNotAbsoluteDies) {
  for (const char *shm_base : {"/dev/shm/aos", "D:aos"}) {
    EXPECT_DEATH(
        {
          absl::SetFlag(&FLAGS_shm_base, shm_base);
          ShmEventLoop loop(factory()->configuration());
        },
        "is not absolute on Windows");
  }
}
#endif  // _WIN32

// Tests that every handler type is realtime and runs.  There are threads
// involved and it's easy to miss one.
TEST_P(ShmEventLoopTest, AllHandlersAreRealtime) {
  auto loop = factory()->MakePrimary("primary");
  auto loop2 = factory()->Make("loop2");

  loop->SetRuntimeRealtimePriority(1);

  auto sender = loop2->MakeSender<TestMessage>("/test");

  bool did_onrun = false;
  bool did_timer = false;
  bool did_watcher = false;

  auto timer = loop->AddTimer([this, &did_timer]() {
    EXPECT_TRUE(IsRealtime());
    did_timer = true;
    factory()->Exit();
  });

  loop->MakeWatcher("/test", [&did_watcher](const TestMessage &) {
    EXPECT_TRUE(IsRealtime());
    did_watcher = true;
  });

  loop->OnRun([&loop, &did_onrun, &sender, timer]() {
    EXPECT_TRUE(IsRealtime());
    did_onrun = true;
    timer->Schedule(loop->monotonic_now() + chrono::milliseconds(100));

    aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
    TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
    builder.add_value(200);
    msg.CheckOk(msg.Send(builder.Finish()));
  });

  factory()->Run();

  EXPECT_TRUE(did_onrun);
  EXPECT_TRUE(did_timer);
  EXPECT_TRUE(did_watcher);
}

// Tests that missing a deadline inside the function still results in PhasedLoop
// running at the right offset.
TEST_P(ShmEventLoopTest, DelayedPhasedLoop) {
  auto loop1 = factory()->MakePrimary("primary");

  ::std::vector<::aos::monotonic_clock::time_point> times;

  constexpr chrono::milliseconds kOffset = chrono::milliseconds(400);

  loop1->AddPhasedLoop(
      [this, &times, &loop1, &kOffset](int count) {
        const ::aos::monotonic_clock::time_point monotonic_now =
            loop1->monotonic_now();

        // Compute our offset.
        const ::aos::monotonic_clock::duration remainder =
            monotonic_now.time_since_epoch() -
            chrono::duration_cast<chrono::seconds>(
                monotonic_now.time_since_epoch());

        // Make sure we we are called near where we should be even when we
        // delay.
        constexpr chrono::milliseconds kEpsilon(200);
        EXPECT_LT(remainder, kOffset + kEpsilon);
        EXPECT_GT(remainder, kOffset - kEpsilon);

        // Confirm that we see the missed count when we sleep.
        if (times.size() == 0) {
          ABSL_CHECK_EQ(count, 1);
        } else {
          ABSL_CHECK_EQ(count, 3);
        }

        times.push_back(loop1->monotonic_now());
        if (times.size() == 2) {
          factory()->Exit();
        }

        // Now, add a large delay.  This should push us up to 3 cycles.
        ::std::this_thread::sleep_for(chrono::milliseconds(2500));
      },
      chrono::seconds(1), kOffset);

  factory()->Run();

  EXPECT_EQ(times.size(), 2u);
}

// Tests that the ShmEventLoop::Exit() method causes the ShmEventLoop to return
// with a successful status.
TEST_P(ShmEventLoopTest, SuccessfulExitTest) {
  auto loop1 = factory()->MakePrimary("primary");
  auto exit_handle = factory()->MakeExitHandle();

  loop1->OnRun([this, &exit_handle]() {
    factory()->Exit();
    // The second Exit() call should get ignored.
    exit_handle->Exit(MakeError("Hello, World!"));
  });

  EXPECT_TRUE(factory()->Run().has_value());
}

// Test GetWatcherSharedMemory in a few basic scenarios.
TEST_P(ShmEventLoopDeathTest, GetWatcherSharedMemory) {
  auto generic_loop1 = factory()->MakePrimary("primary");
  QueueEventLoop *const loop1 =
      static_cast<QueueEventLoop *>(generic_loop1.get());
  const auto channel = configuration::GetChannel(
      loop1->configuration(), "/test", TestMessage::GetFullyQualifiedName(),
      loop1->name(), loop1->node());

  // First verify it handles an invalid channel reasonably.
  EXPECT_DEATH(loop1->GetWatcherSharedMemory(channel),
               "No watcher found for channel");

  // Then, actually create a watcher, and verify it returns something sane.
  absl::Span<const char> shared_memory;
  bool ran = false;
  loop1->MakeWatcher("/test", [this, &shared_memory,
                               &ran](const TestMessage &message) {
    EXPECT_FALSE(ran);
    ran = true;
    // If we're using pinning, then we can verify that the message is actually
    // in the specified region.
    if (std::get<0>(GetParam()) == ReadMethod::PIN) {
      EXPECT_GE(reinterpret_cast<const char *>(&message),
                shared_memory.begin());
      EXPECT_LT(reinterpret_cast<const char *>(&message), shared_memory.end());
    }
    factory()->Exit();
  });
  loop1->SetWatcherUseWritableMemory(channel, true);
  shared_memory = loop1->GetWatcherSharedMemory(channel);
  EXPECT_FALSE(shared_memory.empty());

  auto loop2 = factory()->Make("sender");
  auto sender = loop2->MakeSender<TestMessage>("/test");
  generic_loop1->OnRun([&sender]() {
    auto builder = sender.MakeBuilder();
    TestMessage::Builder test_builder(*builder.fbb());
    test_builder.add_value(1);
    builder.CheckOk(builder.Send(test_builder.Finish()));
  });
  factory()->Run();
  EXPECT_TRUE(ran);
}

TEST_P(ShmEventLoopTest, GetSenderSharedMemory) {
  auto generic_loop1 = factory()->MakePrimary("primary");
  QueueEventLoop *const loop1 =
      static_cast<QueueEventLoop *>(generic_loop1.get());

  // Check that GetSenderSharedMemory returns non-null/non-empty memory span.
  auto sender = loop1->MakeSender<TestMessage>("/test");
  const absl::Span<char> shared_memory = loop1->GetSenderSharedMemory(&sender);
  EXPECT_FALSE(shared_memory.empty());

  auto builder = sender.MakeBuilder();
  uint8_t *buffer;
  builder.fbb()->CreateUninitializedVector(5, &buffer);
  EXPECT_GE(reinterpret_cast<char *>(buffer), shared_memory.begin());
  EXPECT_LT(reinterpret_cast<char *>(buffer), shared_memory.end());
}

TEST_P(ShmEventLoopTest, GetFetcherPrivateMemory) {
  auto generic_loop1 = factory()->MakePrimary("primary");
  QueueEventLoop *const loop1 =
      static_cast<QueueEventLoop *>(generic_loop1.get());

  // Check that GetFetcherPrivateMemory returns non-null/non-empty memory span.
  auto fetcher = loop1->MakeFetcher<TestMessage>("/test");
  const auto private_memory = loop1->GetFetcherPrivateMemory(&fetcher);
  EXPECT_FALSE(private_memory.empty());

  auto loop2 = factory()->Make("sender");
  auto sender = loop2->MakeSender<TestMessage>("/test");
  {
    auto builder = sender.MakeBuilder();
    const std::vector<uint8_t> data{1, 2, 3};
    flatbuffers::Offset<flatbuffers::Vector<uint8_t>> data_offset =
        builder.fbb()->CreateVector(data);

    TestMessage::Builder test_builder(*builder.fbb());
    test_builder.add_value(1);
    test_builder.add_data(data_offset);

    builder.CheckOk(builder.Send(test_builder.Finish()));
  }

  ASSERT_TRUE(fetcher.Fetch());
  EXPECT_GE(fetcher.context().data, private_memory.begin());
  EXPECT_LT(fetcher.context().data, private_memory.end());

  if (std::get<0>(GetParam()) == ReadMethod::PIN) {
    // For pinned messages only, we can get access to the full underlying
    // memory. For copied messages, we only get a sub-portion of the memory so
    // this test won't work. Validate that we can also access the underlying
    // memory in a writable manner.
    ASSERT_TRUE(fetcher->has_data());
    EXPECT_THAT(*fetcher->data(), ::testing::ElementsAre(1, 2, 3));

    // Get read-write access to the fetcher's underlying memory. It should not
    // be overlapping with the private memory.
    const auto shared_memory = loop1->GetFetcherSharedMemory(&fetcher);
    EXPECT_TRUE(shared_memory.end() <= private_memory.data() ||
                shared_memory.data() >= private_memory.end());
    const ptrdiff_t offset =
        fetcher->data()->data() -
        reinterpret_cast<const uint8_t *>(private_memory.data());
    // Do some sanity checks that we got the right data.
    EXPECT_EQ(shared_memory[offset + 0], 1);
    EXPECT_EQ(shared_memory[offset + 1], 2);
    EXPECT_EQ(shared_memory[offset + 2], 3);

    // Change the fetcher's result by poking directly at the shared memory.
    shared_memory[offset] = 5;
    EXPECT_THAT(*fetcher->data(), ::testing::ElementsAre(5, 2, 3));
  }
}

// Validates that we can make fetchers point at writable memory.
TEST_P(ShmEventLoopTest, SetFetcherUseWritableMemory) {
  auto generic_loop1 = factory()->MakePrimary("primary");
  QueueEventLoop *const loop1 =
      static_cast<QueueEventLoop *>(generic_loop1.get());

  // Check that GetFetcherSharedMemory returns non-null/non-empty memory span.
  auto fetcher = loop1->MakeFetcher<TestMessage>("/test");
  const auto shared_memory = loop1->GetFetcherSharedMemory(&fetcher);
  EXPECT_FALSE(shared_memory.empty());

  loop1->SetFetcherUseWritableMemory(&fetcher, true);

  // Send out a simple message with some bytes stuffed in.
  auto loop2 = factory()->Make("sender");
  auto sender = loop2->MakeSender<TestMessage>("/test");
  {
    auto builder = sender.MakeBuilder();
    const std::vector<uint8_t> data{1, 2, 3};
    flatbuffers::Offset<flatbuffers::Vector<uint8_t>> data_offset =
        builder.fbb()->CreateVector(data);
    TestMessage::Builder test_builder(*builder.fbb());
    test_builder.add_data(data_offset);
    builder.CheckOk(builder.Send(test_builder.Finish()));
  }

  ASSERT_TRUE(fetcher.Fetch());

  if (std::get<0>(GetParam()) == ReadMethod::PIN) {
    // For pinned messages only, we can get access to the full underlying
    // memory. For copied messages, we only get a sub-portion of the memory so
    // this test won't work. Validate that we can also access the underlying
    // memory in a writable manner.
    ASSERT_TRUE(fetcher->has_data());
    EXPECT_THAT(*fetcher->data(), ::testing::ElementsAre(1, 2, 3));

    // Get read-write access to the fetcher's underlying memory. Since we called
    // SetFetcherUseWritableMemory(true), we can modify the memory directly.
    uint8_t *data = const_cast<uint8_t *>(fetcher->data()->data());

    // Validate that the message is in the writable section.
    EXPECT_GE(data, reinterpret_cast<uint8_t *>(shared_memory.begin()));
    EXPECT_LT(data, reinterpret_cast<uint8_t *>(shared_memory.end()));

    // Change the fetcher's result by poking directly at the shared memory.
    *data = 5;
    EXPECT_THAT(*fetcher->data(), ::testing::ElementsAre(5, 2, 3));
  }
}

// Tests that corrupting the bytes around the data buffer results in a crash.
TEST_P(ShmEventLoopDeathTest, OutOfBoundsWrite) {
  auto loop1 = factory()->Make("loop1");
  std::unique_ptr<aos::RawSender> sender =
      loop1->MakeRawSender(configuration::GetChannel(
          loop1->configuration(), "/test", "aos.TestMessage", "", nullptr));
  for (size_t i = 0; i < kChannelDataRedzone; ++i) {
    SCOPED_TRACE(std::to_string(i));
    EXPECT_DEATH(
        {
          // Can't use `data()[-1 -i]` here because that's undefined behaviour.
          // We do manual pointer arithmetic to work around that.
          ++(*(static_cast<char *>(sender->data()) - 1 - i));
          sender->CheckOk(sender->Send(0));
        },
        "Somebody wrote outside the buffer of their message");
    EXPECT_DEATH(
        {
          ++static_cast<char *>(sender->data())[sender->size() + i];
          sender->CheckOk(sender->Send(0));
        },
        "Somebody wrote outside the buffer of their message");
  }
}

// Tests that the next message not being available prints a helpful error in the
// normal case.
TEST_P(ShmEventLoopDeathTest, NextMessageNotAvailable) {
  TestNextMessageNotAvailable(false);
}

// Tests that the next message not being available prints a helpful error with
// timing reports disabled.
TEST_P(ShmEventLoopDeathTest, NextMessageNotAvailableNoTimingReports) {
  TestNextMessageNotAvailable(true);
}

// Tests that the next message not being available prints a helpful error even
// when Run is never called.
TEST_P(ShmEventLoopDeathTest, NextMessageNotAvailableNoRun) {
  TestNextMessageNotAvailableNoRun(false);
}

// Tests that the next message not being available prints a helpful error even
// when Run is never called without timing reports.
TEST_P(ShmEventLoopDeathTest, NextMessageNotAvailableNoRunNoTimingReports) {
  TestNextMessageNotAvailableNoRun(true);
}

// Test that an ExitHandle outliving its EventLoop is caught.
TEST_P(ShmEventLoopDeathTest, ExitHandleOutlivesEventLoop) {
  auto loop1 = factory()->MakePrimary("loop1");
  auto exit_handle =
      static_cast<QueueEventLoop *>(loop1.get())->MakeExitHandle();
  EXPECT_DEATH(loop1.reset(),
               "All ExitHandles must be destroyed before the event loop");
}

// Once the loop runs, timers and phased loops belong to its thread, on every
// backend.
TEST_P(ShmEventLoopDeathTest, TimerFromAnotherThread) {
  auto loop = factory()->MakePrimary("primary");
  TimerHandler *timer = loop->AddTimer([]() {});
  loop->OnRun([&]() {
    std::thread([&]() { timer->Schedule(loop->monotonic_now()); }).join();
  });
  EXPECT_DEATH(factory()->Run(), "thread that runs the event loop");
}

TEST_P(ShmEventLoopDeathTest, DisableTimerFromAnotherThread) {
  auto loop = factory()->MakePrimary("primary");
  TimerHandler *timer = loop->AddTimer([]() {});
  loop->OnRun([&]() { std::thread([&]() { timer->Disable(); }).join(); });
  EXPECT_DEATH(factory()->Run(), "thread that runs the event loop");
}

TEST_P(ShmEventLoopDeathTest, PhasedLoopFromAnotherThread) {
  auto loop = factory()->MakePrimary("primary");
  PhasedLoopHandler *phased_loop =
      loop->AddPhasedLoop([](int) {}, std::chrono::seconds(1));
  loop->OnRun([&]() {
    std::thread([&]() {
      phased_loop->Reschedule(loop->monotonic_now());
    }).join();
  });
  EXPECT_DEATH(factory()->Run(), "thread that runs the event loop");
}

// Before the loop runs, any thread may set its timers up.
TEST_P(ShmEventLoopTest, TimerFromAnotherThreadBeforeRun) {
  auto loop = factory()->MakePrimary("primary");
  int fires = 0;
  TimerHandler *timer = loop->AddTimer([&]() {
    ++fires;
    factory()->Exit();
  });
  std::thread([&]() { timer->Schedule(loop->monotonic_now()); }).join();
  factory()->Run();
  EXPECT_EQ(fires, 1);
}

// After Run() returns, any thread may use the timers again, so a loop built
// on one thread and run on another can be torn down where it was built.
TEST_P(ShmEventLoopTest, TimerFromAnotherThreadAfterRun) {
  auto loop = factory()->MakePrimary("primary");
  TimerHandler *exit_timer = loop->AddTimer([&]() { factory()->Exit(); });
  TimerHandler *timer = loop->AddTimer([]() {});
  PhasedLoopHandler *phased_loop =
      loop->AddPhasedLoop([](int) {}, std::chrono::seconds(1));
  loop->OnRun([&]() {
    exit_timer->Schedule(loop->monotonic_now());
    timer->Schedule(loop->monotonic_now() + std::chrono::seconds(10));
  });
  std::thread([&]() { factory()->Run(); }).join();
  timer->Disable();
  timer->Schedule(loop->monotonic_now() + std::chrono::seconds(10));
  phased_loop->Reschedule(loop->monotonic_now());
  timer->Disable();
}

// A started event loop on a borrowed Aio has to be shut down before it goes,
// as Run() returning shuts down an owned one.
TEST_P(ShmEventLoopDeathTest, DestroyingAStartedLoopOnABorrowedAioDies) {
  absl::SetFlag(&FLAGS_aio_backend, std::get<1>(GetParam()));
  EXPECT_DEATH(
      {
        Aio aio;
        std::unique_ptr<QueueEventLoop> event_loop =
            MakeLoop(std::get<2>(GetParam()), factory()->configuration(), &aio);
        event_loop->SkipTimingReport();
        event_loop->SkipAosLog();
        event_loop->Startup();
        event_loop.reset();
      },
      "On a borrowed Aio");
}

// Run() drives the Aio, which a borrowed one is not this event loop's to do.
TEST_P(ShmEventLoopDeathTest, RunIsRefusedOnABorrowedAio) {
  absl::SetFlag(&FLAGS_aio_backend, std::get<1>(GetParam()));
  Aio aio;
  std::unique_ptr<QueueEventLoop> event_loop =
      MakeLoop(std::get<2>(GetParam()), factory()->configuration(), &aio);
  EXPECT_DEATH((void)event_loop->Run(), "borrowed Aio");
}

// Polls a borrowed Aio, without blocking, until duration has passed.
void PollFor(Aio &aio, monotonic_clock::duration duration) {
  const monotonic_clock::time_point end = monotonic_clock::now() + duration;
  while (monotonic_clock::now() < end) {
    aio.Poll(false);
  }
}

// The Aio outlives the event loop it carried, and is polled on without it.
// Under ASAN this catches a before-wait hook left behind on the Aio.  The
// Sender is torn down the way every application's is: after Shutdown().
TEST_P(ShmEventLoopTest, BorrowedAioOutlivesItsEventLoop) {
  absl::SetFlag(&FLAGS_aio_backend, std::get<1>(GetParam()));
  Aio aio;
  for (int i = 0; i < 2; ++i) {
    int ticks = 0;
    {
      std::unique_ptr<QueueEventLoop> event_loop_ptr =
          MakeLoop(std::get<2>(GetParam()), factory()->configuration(), &aio);
      QueueEventLoop &event_loop = *event_loop_ptr;
      event_loop.SkipTimingReport();
      event_loop.SkipAosLog();
      std::optional<aos::Sender<TestMessage>> sender =
          event_loop.MakeSender<TestMessage>("/test");
      TimerHandler *const timer = event_loop.AddTimer([&]() { ++ticks; });
      event_loop.OnRun([&]() {
        timer->Schedule(event_loop.monotonic_now(), chrono::milliseconds(1));
      });
      event_loop.Startup();
      PollFor(aio, chrono::milliseconds(20));
      ASSERT_TRUE(IsOk(event_loop.Shutdown()));
      sender.reset();
    }
    EXPECT_GT(ticks, 0);
    PollFor(aio, chrono::milliseconds(20));
  }
}

// A shut-down event loop on a borrowed Aio is what one on an owned Aio is
// between runs: what it scheduled stays scheduled, more may be scheduled, and
// the next Startup() runs it all.
TEST_P(ShmEventLoopTest, ShutDownEventLoopKeepsItsScheduleForTheNextStartup) {
  absl::SetFlag(&FLAGS_aio_backend, std::get<1>(GetParam()));
  Aio aio;
  std::unique_ptr<QueueEventLoop> event_loop_ptr =
      MakeLoop(std::get<2>(GetParam()), factory()->configuration(), &aio);
  QueueEventLoop &event_loop = *event_loop_ptr;
  event_loop.SkipAosLog();
  event_loop.MakeWatcher("/test", [](const TestMessage &) {});
  int ticks = 0;
  int phases = 0;
  TimerHandler *const timer = event_loop.AddTimer([&]() { ++ticks; });
  event_loop.AddPhasedLoop([&](int) { ++phases; }, chrono::milliseconds(1));
  event_loop.OnRun([&]() {
    timer->Schedule(event_loop.monotonic_now(), chrono::milliseconds(1));
  });

  event_loop.Startup();
  PollFor(aio, chrono::milliseconds(20));
  EXPECT_TRUE(IsOk(event_loop.Shutdown()));
  const int ticks_at_shutdown = ticks;
  const int phases_at_shutdown = phases;
  EXPECT_GT(ticks_at_shutdown, 0);
  EXPECT_GT(phases_at_shutdown, 0);

  int late = 0;
  TimerHandler *const late_timer = event_loop.AddTimer([&]() { ++late; });
  late_timer->Schedule(event_loop.monotonic_now());

  event_loop.Startup();
  PollFor(aio, chrono::milliseconds(20));
  EXPECT_GT(ticks, ticks_at_shutdown);
  EXPECT_GT(phases, phases_at_shutdown);
  EXPECT_EQ(late, 1);
  EXPECT_TRUE(IsOk(event_loop.Shutdown()));
}

// A ShmEventLoop and a ThreadEventLoop on the same channel, in one process,
// do not exchange messages.
TEST_P(ShmEventLoopTest, ThreadAndShmEventLoopsDoNotShareQueues) {
  absl::SetFlag(&FLAGS_aio_backend, std::get<1>(GetParam()));
  ShmEventLoop shm_loop(factory()->configuration());
  ThreadEventLoop thread_loop(factory()->configuration());
  for (const bool from_shm : {true, false}) {
    QueueEventLoop &sending =
        from_shm ? static_cast<QueueEventLoop &>(shm_loop) : thread_loop;
    QueueEventLoop &fetching =
        from_shm ? static_cast<QueueEventLoop &>(thread_loop) : shm_loop;
    const int value = from_shm ? 971 : 972;
    aos::Sender<TestMessage> sender = sending.MakeSender<TestMessage>("/test");
    aos::Fetcher<TestMessage> fetcher =
        fetching.MakeFetcher<TestMessage>("/test");
    auto builder = sender.MakeBuilder();
    TestMessage::Builder test_builder(*builder.fbb());
    test_builder.add_value(value);
    builder.CheckOk(builder.Send(test_builder.Finish()));
    EXPECT_FALSE(fetcher.Fetch() && fetcher->value() == value);
  }
}

// A ThreadEventLoop leaves nothing under --shm_base.
TEST_P(ShmEventLoopTest, ThreadEventLoopLeavesNoFiles) {
  absl::SetFlag(&FLAGS_aio_backend, std::get<1>(GetParam()));
  const std::string queue_file = ipc_lib::ShmPath(
      absl::GetFlag(FLAGS_shm_base),
      configuration::GetChannel(factory()->configuration(), "/test",
                                "aos.TestMessage", "", nullptr));
  std::error_code ec;
  std::filesystem::remove(queue_file, ec);
  {
    ThreadEventLoop thread_loop(factory()->configuration());
    aos::Sender<TestMessage> sender =
        thread_loop.MakeSender<TestMessage>("/test");
    auto builder = sender.MakeBuilder();
    TestMessage::Builder test_builder(*builder.fbb());
    test_builder.add_value(1);
    builder.CheckOk(builder.Send(test_builder.Finish()));
  }
  EXPECT_FALSE(std::filesystem::exists(queue_file));
}

// Run() shuts an owned event loop down before it returns, so destroying one
// from its own callback is a misuse.
TEST_P(ShmEventLoopDeathTest, DestroyingAnOwnedEventLoopFromItsCallbackDies) {
  EXPECT_DEATH(
      {
        std::unique_ptr<EventLoop> event_loop = factory()->MakePrimary("loop");
        event_loop->SkipTimingReport();
        event_loop->SkipAosLog();
        TimerHandler *const timer =
            event_loop->AddTimer([&]() { event_loop.reset(); });
        event_loop->OnRun(
            [&]() { timer->Schedule(event_loop->monotonic_now()); });
        factory()->Run();
      },
      "destroyed while running");
}

// Shutdown() is what Run() returning is on an owned Aio, which never happens
// inside one of the event loop's own callbacks.
TEST_P(ShmEventLoopDeathTest, ShutdownFromACallbackDies) {
  absl::SetFlag(&FLAGS_aio_backend, std::get<1>(GetParam()));
  EXPECT_DEATH(
      {
        Aio aio;
        std::unique_ptr<QueueEventLoop> event_loop_ptr =
            MakeLoop(std::get<2>(GetParam()), factory()->configuration(), &aio);
        QueueEventLoop &event_loop = *event_loop_ptr;
        event_loop.SkipTimingReport();
        event_loop.SkipAosLog();
        TimerHandler *const timer =
            event_loop.AddTimer([&]() { (void)event_loop.Shutdown(); });
        event_loop.OnRun(
            [&]() { timer->Schedule(event_loop.monotonic_now()); });
        event_loop.Startup();
        PollFor(aio, chrono::milliseconds(20));
      },
      "Shutdown\\(\\) from inside one of this event loop's own callbacks");
}

// The watchers Shutdown() unregisters belong to the thread that ran Startup().
TEST_P(ShmEventLoopDeathTest, ShutdownOffTheLoopThreadDies) {
  absl::SetFlag(&FLAGS_aio_backend, std::get<1>(GetParam()));
  EXPECT_DEATH(
      {
        Aio aio;
        std::unique_ptr<QueueEventLoop> event_loop_ptr =
            MakeLoop(std::get<2>(GetParam()), factory()->configuration(), &aio);
        QueueEventLoop &event_loop = *event_loop_ptr;
        event_loop.SkipTimingReport();
        event_loop.SkipAosLog();
        event_loop.MakeWatcher("/test", [](const TestMessage &) {});
        std::thread([&]() {
          event_loop.Startup();
          PollFor(aio, chrono::milliseconds(5));
        }).join();
        (void)event_loop.Shutdown();
      },
      "thread that ran Startup");
}

// A borrowed Aio may only be polled while the event loop on it is started, as
// an owned one is only polled inside Run(): not before Startup(), and not
// after Shutdown().
TEST_P(ShmEventLoopDeathTest, PollingTheAioWhileNotStartedDies) {
  absl::SetFlag(&FLAGS_aio_backend, std::get<1>(GetParam()));
  for (const bool started_once : {false, true}) {
    EXPECT_DEATH(
        {
          Aio aio;
          std::unique_ptr<QueueEventLoop> event_loop_ptr = MakeLoop(
              std::get<2>(GetParam()), factory()->configuration(), &aio);
          QueueEventLoop &event_loop = *event_loop_ptr;
          event_loop.SkipTimingReport();
          event_loop.SkipAosLog();
          if (started_once) {
            event_loop.Startup();
            ABSL_CHECK(IsOk(event_loop.Shutdown()));
          }
          aio.Poll(false);
        },
        "Polled the Aio while an event loop on it was not started");
  }
}

// An event loop's wakeups, realtime priority and logging are per thread, so
// two started on one thread, even on two Aios, would take each other's.
TEST_P(ShmEventLoopDeathTest, TwoStartedEventLoopsOnOneThreadDie) {
  absl::SetFlag(&FLAGS_aio_backend, std::get<1>(GetParam()));
  EXPECT_DEATH(
      {
        Aio aio_a;
        Aio aio_b;
        ShmEventLoop a(factory()->configuration(), &aio_a);
        ShmEventLoop b(factory()->configuration(), &aio_b);
        a.SkipTimingReport();
        a.SkipAosLog();
        b.SkipTimingReport();
        b.SkipAosLog();
        a.Startup();
        b.Startup();
      },
      "Another event loop is already started on this thread");
}

// The Aio's wakeups and timers belong to the thread that ran Startup().  This
// starts on one thread and polls from another made after it exits, which can
// reuse its std::thread::id.
TEST_P(ShmEventLoopDeathTest, PollingTheAioOffTheStartupThreadDies) {
  absl::SetFlag(&FLAGS_aio_backend, std::get<1>(GetParam()));
  EXPECT_DEATH(
      {
        Aio aio;
        ShmEventLoop event_loop(factory()->configuration(), &aio);
        event_loop.SkipTimingReport();
        event_loop.SkipAosLog();
        std::thread([&]() { event_loop.Startup(); }).join();
        std::thread([&]() { aio.Poll(false); }).join();
      },
      "Polled the Aio on a different thread");
}

// An Aio carries one event loop's wakeups, so a second event loop on it would
// never hear about its messages.
TEST_P(ShmEventLoopDeathTest, TwoEventLoopsOnOneAioDie) {
  absl::SetFlag(&FLAGS_aio_backend, std::get<1>(GetParam()));
  EXPECT_DEATH(
      {
        Aio aio;
        std::unique_ptr<QueueEventLoop> a =
            MakeLoop(std::get<2>(GetParam()), factory()->configuration(), &aio);
        std::unique_ptr<QueueEventLoop> b =
            MakeLoop(std::get<2>(GetParam()), factory()->configuration(), &aio);
      },
      "Only one event loop may be on an Aio");
}

// TODO(austin): Test that missing a deadline with a timer recovers as expected.

// The trailing parameter is the name of the backend to run.  On Linux,
// AOS_EPOLL_ONLY / AOS_IO_URING_ONLY split io_uring and epoll into separate
// test targets that run in parallel; the plain Linux build runs both.  Every
// other platform has a single backend, so it instantiates just that one.  The
// AOS_*_ONLY splits only mean something on Linux; elsewhere the epoll target
// runs the platform's own backend, as CommonParameters() above does.
#if defined(__linux__) && defined(AOS_EPOLL_ONLY)
#define SHM_EVENT_LOOP_BACKENDS ::testing::Values("epoll")
#elif defined(__linux__) && defined(AOS_IO_URING_ONLY)
#define SHM_EVENT_LOOP_BACKENDS ::testing::Values("io_uring")
#elif defined(__linux__)
#define SHM_EVENT_LOOP_BACKENDS ::testing::Values("io_uring", "epoll")
#elif defined(_WIN32)
// Windows has only iocp, and --aio_backend is fatal on any other name.
#define SHM_EVENT_LOOP_BACKENDS ::testing::Values("iocp")
#else
// macOS has only kqueue, and --aio_backend is fatal on any other name.
#define SHM_EVENT_LOOP_BACKENDS ::testing::Values("kqueue")
#endif

#define SHM_EVENT_LOOP_KINDS ::testing::Values(kLoopKind)

// The ReadMethod half is already in each suite's name, so naming these by
// backend and loop kind is unambiguous -- and beats the tuple index gtest
// would otherwise print.
auto BackendName(const ::testing::TestParamInfo<
                 std::tuple<ReadMethod, std::string, LoopKind>> &info) {
  return std::get<1>(info.param) + "_" + LoopKindName(std::get<2>(info.param));
}

INSTANTIATE_TEST_SUITE_P(ShmEventLoopCopyTest, ShmEventLoopTest,
                         ::testing::Combine(::testing::Values(ReadMethod::COPY),
                                            SHM_EVENT_LOOP_BACKENDS,
                                            SHM_EVENT_LOOP_KINDS),
                         BackendName);
INSTANTIATE_TEST_SUITE_P(ShmEventLoopPinTest, ShmEventLoopTest,
                         ::testing::Combine(::testing::Values(ReadMethod::PIN),
                                            SHM_EVENT_LOOP_BACKENDS,
                                            SHM_EVENT_LOOP_KINDS),
                         BackendName);
INSTANTIATE_TEST_SUITE_P(ShmEventLoopCopyDeathTest, ShmEventLoopDeathTest,
                         ::testing::Combine(::testing::Values(ReadMethod::COPY),
                                            SHM_EVENT_LOOP_BACKENDS,
                                            SHM_EVENT_LOOP_KINDS),
                         BackendName);
INSTANTIATE_TEST_SUITE_P(ShmEventLoopPinDeathTest, ShmEventLoopDeathTest,
                         ::testing::Combine(::testing::Values(ReadMethod::PIN),
                                            SHM_EVENT_LOOP_BACKENDS,
                                            SHM_EVENT_LOOP_KINDS),
                         BackendName);

}  // namespace aos::testing
