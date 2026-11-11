#include "aos/events/shm_event_loop.h"

#ifndef _WIN32
#include <csignal>
#else
#include <windows.h>
#endif
#ifdef __linux__
#include <fcntl.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "absl/flags/declare.h"
#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "aos/events/aio.h"
#include "aos/events/event_loop_param_test.h"
#include "aos/events/test_message_generated.h"
#include "aos/events/thread_event_loop.h"
#include "aos/ipc_lib/memory_mapped_queue.h"
#include "aos/ipc_lib/process_local_queue.h"
#include "aos/ipc_lib/shm_mapping.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/network/team_number.h"
#include "aos/realtime.h"
#include "aos/testing/tmpdir.h"
#include "aos/util/file.h"

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

// A process-directed SIGTERM can arrive on any thread, including while
// another thread is registering or unregistering an event loop's handler.
// The handler used to CHECK that it could take the lock those hold, which
// only the registering thread's own mask guaranteed, and aborted.  Each round
// sends one SIGTERM, since a second one is fatal while a loop is registered.
TEST_P(ShmEventLoopTest, SignalWhileOtherThreadsRegister) {
  struct sigaction ignore{};
  ignore.sa_handler = &IgnoreSignalForTest;
  sigemptyset(&ignore.sa_mask);
  struct sigaction previous{};
  ASSERT_EQ(sigaction(SIGTERM, &ignore, &previous), 0);

  for (int round = 0; round < 50; ++round) {
    std::atomic<bool> stop{false};
    std::vector<std::thread> threads;
    for (int i = 0; i < 4; ++i) {
      threads.emplace_back([this, &stop]() {
        Aio aio;
        std::unique_ptr<QueueEventLoop> loop =
            MakeLoop(std::get<2>(GetParam()), factory()->configuration(), &aio);
        loop->SkipTimingReport();
        loop->SkipAosLog();
        while (!stop) {
          loop->Startup();
          (void)loop->Shutdown();
        }
      });
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    ASSERT_EQ(kill(getpid(), SIGTERM), 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    stop = true;
    for (std::thread &thread : threads) {
      thread.join();
    }
    // Every loop is gone, so the handler from before is back.
    EXPECT_EQ(SigtermHandler(), &IgnoreSignalForTest);
  }

  ASSERT_EQ(sigaction(SIGTERM, &previous, nullptr), 0);
}

// The first signal exits the running loops, and a second one, sent because
// they were too slow to go, kills the process with it.  A loop started after
// the first has been handled counts afresh.
TEST_P(ShmEventLoopTest, SecondSignalKills) {
  const auto child = [this]() {
    for (int run = 0; run < 2; ++run) {
      auto loop = factory()->MakePrimary("primary");
      static_cast<QueueEventLoop *>(loop.get())->set_handle_signals(true);
      loop->OnRun([]() { ABSL_CHECK_EQ(raise(SIGINT), 0); });
      factory()->Run();
      std::fprintf(stderr, "run %d exited\n", run);
    }
    auto loop = factory()->MakePrimary("primary");
    static_cast<QueueEventLoop *>(loop.get())->set_handle_signals(true);
    loop->OnRun([]() {
      ABSL_CHECK_EQ(raise(SIGINT), 0);
      ABSL_CHECK_EQ(raise(SIGINT), 0);
    });
    factory()->Run();
    std::exit(0);
  };
  EXPECT_EXIT(child(), ::testing::KilledBySignal(SIGINT), "run 1 exited");
}

// A loop started after a signal gets its own first signal, even with an earlier
// loop still registered and not going anywhere.  So here the second signal only
// exits the new loop, and it takes a third to kill the process.
TEST_P(ShmEventLoopTest, LoopStartedAfterASignalGetsItsOwnFirstSignal) {
  const auto child = [this]() {
    std::atomic<bool> started{false};
    std::atomic<bool> release{false};
    std::thread stuck([this, &started, &release]() {
      Aio aio;
      std::unique_ptr<QueueEventLoop> loop =
          MakeLoop(std::get<2>(GetParam()), factory()->configuration(), &aio);
      loop->SkipTimingReport();
      loop->SkipAosLog();
      loop->set_handle_signals(true);
      loop->Startup();
      started = true;
      while (!release) {
        std::this_thread::yield();
      }
      (void)loop->Shutdown();
    });
    while (!started) {
      std::this_thread::yield();
    }
    // raise() returns after the handler has run on this thread.
    ABSL_CHECK_EQ(raise(SIGTERM), 0);

    auto loop = factory()->MakePrimary("primary");
    static_cast<QueueEventLoop *>(loop.get())->set_handle_signals(true);
    loop->OnRun([]() { ABSL_CHECK_EQ(raise(SIGTERM), 0); });
    factory()->Run();
    std::fprintf(stderr, "second signal only exited the new loop\n");

    ABSL_CHECK_EQ(raise(SIGTERM), 0);
    release = true;
    stuck.join();
    std::exit(0);
  };
  EXPECT_EXIT(child(), ::testing::KilledBySignal(SIGTERM),
              "second signal only exited the new loop");
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

// Verifies that GetOrCreateMemoryMappedQueue returns the same instance for the
// same channel, confirming the process-global cache deduplicates mmaps.
TEST_P(ShmEventLoopTest, SharedMmapCacheSameChannel) {
  if (kLoopKind != LoopKind::kShm) {
    GTEST_SKIP() << "Only ShmEventLoop shares queues through the cache";
  }
  auto loop = factory()->MakePrimary("primary");
  const Channel *channel = configuration::GetChannel(
      loop->configuration(), "/test", "aos.TestMessage", "", nullptr);
  ASSERT_NE(channel, nullptr);

  auto q1 = ipc_lib::GetOrCreateMemoryMappedQueue(
      absl::GetFlag(FLAGS_shm_base), static_cast<std::filesystem::perms>(0770),
      loop->configuration(), channel);
  auto q2 = ipc_lib::GetOrCreateMemoryMappedQueue(
      absl::GetFlag(FLAGS_shm_base), static_cast<std::filesystem::perms>(0770),
      loop->configuration(), channel);

  // Both should be the exact same object.
  EXPECT_EQ(q1.get(), q2.get());
  EXPECT_EQ(q1->GetMutableSharedMemory().data(),
            q2->GetMutableSharedMemory().data());
}

// Verifies that the shared mmap cache garbage-collects entries once all holders
// drop their references.  After dropping all shared_ptrs, a new call should
// return a freshly constructed MemoryMappedQueue object.
TEST_P(ShmEventLoopTest, SharedMmapCacheGarbageCollects) {
  if (kLoopKind != LoopKind::kShm) {
    GTEST_SKIP() << "Only ShmEventLoop shares queues through the cache";
  }
  auto loop = factory()->MakePrimary("primary");
  const Channel *channel = configuration::GetChannel(
      loop->configuration(), "/test", "aos.TestMessage", "", nullptr);
  ASSERT_NE(channel, nullptr);

  ipc_lib::MemoryMappedQueue *first_ptr = nullptr;
  {
    auto q = ipc_lib::GetOrCreateMemoryMappedQueue(
        absl::GetFlag(FLAGS_shm_base),
        static_cast<std::filesystem::perms>(0770), loop->configuration(),
        channel);
    first_ptr = q.get();
    // q destroyed here, dropping the last shared_ptr ref and destroying the
    // MemoryMappedQueue (munmap).
  }

  // The old weak_ptr should have expired, so this constructs a new object.
  auto q2 = ipc_lib::GetOrCreateMemoryMappedQueue(
      absl::GetFlag(FLAGS_shm_base), static_cast<std::filesystem::perms>(0770),
      loop->configuration(), channel);
  // The new MemoryMappedQueue must be a distinct object (the old one was
  // destroyed).  Note: the OS *may* reuse the same mmap VA, but the C++
  // object itself must be different.
  EXPECT_NE(first_ptr, q2.get());
}

// Verifies that a sender and fetcher on the same channel end up sharing the
// same MemoryMappedQueue from the cache.  We verify this indirectly: get the
// cache entry, then create sender/fetcher and confirm the cache entry's
// use_count increased (meaning more holders share it).
TEST_P(ShmEventLoopTest, SenderAndFetcherShareCacheEntry) {
  if (kLoopKind != LoopKind::kShm) {
    GTEST_SKIP() << "Only ShmEventLoop shares queues through the cache";
  }
  auto loop = factory()->MakePrimary("primary");
  const Channel *channel = configuration::GetChannel(
      loop->configuration(), "/test", "aos.TestMessage", "", nullptr);
  ASSERT_NE(channel, nullptr);

  // Get an initial reference via the cache.
  auto q = ipc_lib::GetOrCreateMemoryMappedQueue(
      absl::GetFlag(FLAGS_shm_base), static_cast<std::filesystem::perms>(0770),
      loop->configuration(), channel);
  long initial_count = q.use_count();

  // Creating a sender on the same channel should share the same queue.
  auto sender = loop->MakeSender<TestMessage>("/test");
  EXPECT_GT(q.use_count(), initial_count);

  long after_sender_count = q.use_count();

  // Creating a fetcher on the same channel should also share the same queue.
  auto fetcher = loop->MakeFetcher<TestMessage>("/test");
  EXPECT_GT(q.use_count(), after_sender_count);
}

// Two ShmEventLoops running on different threads of one process share the
// cached MemoryMappedQueue for every channel they both touch (/test and /test2
// here).  Verifies that messages still flow in both directions through the
// shared mappings: loop1 sends on /test, loop2 echoes each message back on
// /test2, and loop1 exits once it has seen enough echoes.
TEST_P(ShmEventLoopTest, TwoThreadedEventLoopsShareCachedQueues) {
  if (kLoopKind != LoopKind::kShm) {
    GTEST_SKIP() << "Only ShmEventLoop shares queues through the cache";
  }
  auto loop1 = factory()->MakePrimary("primary");
  auto loop2 = factory()->Make("loop2");

  aos::Sender<TestMessage> sender1 = loop1->MakeSender<TestMessage>("/test");
  aos::Sender<TestMessage> sender2 = loop2->MakeSender<TestMessage>("/test2");

  // loop2 echoes every message it sees on /test back out on /test2.
  loop2->MakeWatcher("/test", [&sender2](const TestMessage &message) {
    aos::Sender<TestMessage>::Builder builder = sender2.MakeBuilder();
    TestMessage::Builder test_builder = builder.MakeBuilder<TestMessage>();
    test_builder.add_value(message.value());
    builder.CheckOk(builder.Send(test_builder.Finish()));
  });

  constexpr int kEchoesToWaitFor = 10;
  int echoes = 0;
  int sent = 0;
  loop1->MakeWatcher("/test2", [&](const TestMessage &message) {
    EXPECT_GT(message.value(), 0);
    EXPECT_LE(message.value(), sent);
    if (++echoes == kEchoesToWaitFor) {
      factory()->Exit();
    }
  });

  // Keep sending until loop1 has seen enough echoes, so the test does not
  // depend on loop2's watcher being registered before the first send.
  aos::TimerHandler *send_timer = loop1->AddTimer([&]() {
    aos::Sender<TestMessage>::Builder builder = sender1.MakeBuilder();
    TestMessage::Builder test_builder = builder.MakeBuilder<TestMessage>();
    test_builder.add_value(++sent);
    builder.CheckOk(builder.Send(test_builder.Finish()));
  });
  // Fail rather than hang if messages stop flowing.
  aos::TimerHandler *watchdog = loop1->AddTimer([&]() {
    ADD_FAILURE() << "Only saw " << echoes << " echoes before the deadline";
    factory()->Exit();
  });
  loop1->OnRun([&]() {
    send_timer->Schedule(loop1->monotonic_now(), chrono::milliseconds(20));
    watchdog->Schedule(loop1->monotonic_now() + chrono::seconds(10));
  });

  std::thread loop2_thread(
      [&loop2]() { static_cast<ShmEventLoop *>(loop2.get())->Run(); });

  factory()->Run();

  static_cast<ShmEventLoop *>(loop2.get())->Exit();
  loop2_thread.join();

  EXPECT_EQ(echoes, kEchoesToWaitFor);
  EXPECT_GE(sent, kEchoesToWaitFor);
}

#ifdef __linux__
// hugetlbfs is Linux-only.
// Two channels whose queue memory sizes straddle the configured
// shm_hugepage_threshold, both sourced on (and so readable on) node1.  A tmpfs
// dir stands in for the hugetlbfs mount, which AllowHugePagesOffHugetlbfs-
// ForTesting() permits; that is what makes this testable without one.
class MemoryMappedQueueTest : public ::testing::Test {
 protected:
  MemoryMappedQueueTest()
      : shm_dir_(absl::StrCat(TestTmpDir(), "/mmq_shm")),
        huge_dir_(absl::StrCat(TestTmpDir(), "/mmq_huge")),
        big_memory_size_(QueueMemorySize(MakeConfig("", 0), "/big")),
        small_memory_size_(QueueMemorySize(MakeConfig("", 0), "/small")),
        // Above the small queue, at or below the big one.
        threshold_((small_memory_size_ + big_memory_size_) / 2),
        fbs_config_(MakeConfig(huge_dir_, threshold_)),
        config_(&fbs_config_.message()),
        node1_(configuration::GetNode(config_, "node1")),
        big_channel_(configuration::GetChannel(config_, "/big", ".aos.test", "",
                                               node1_)),
        small_channel_(configuration::GetChannel(config_, "/small", ".aos.test",
                                                 "", node1_)) {
    mkdir(shm_dir_.c_str(), 0777);
    mkdir(huge_dir_.c_str(), 0777);
    ipc_lib::AllowHugePagesOffHugetlbfsForTesting(true);
  }

  ~MemoryMappedQueueTest() override {
    ipc_lib::AllowHugePagesOffHugetlbfsForTesting(false);
    aos::util::UnlinkRecursive(shm_dir_);
    aos::util::UnlinkRecursive(huge_dir_);
  }

  // Returns the configuration, with shm_hugepage_base and
  // shm_hugepage_threshold set unless huge_dir is empty.
  static FlatbufferDetachedBuffer<Configuration> MakeConfig(
      std::string_view huge_dir, size_t threshold) {
    std::string hugepage_fields;
    if (!huge_dir.empty()) {
      hugepage_fields = absl::StrFormat(
          R"json(  "shm_hugepage_base": "%s",
  "shm_hugepage_threshold": %d,
)json",
          huge_dir, threshold);
    }
    return aos::JsonToFlatbuffer<Configuration>(absl::StrFormat(
        R"json({
%s  "channel_storage_duration": 1000000000,
  "channels": [
    {
      "name": "/big",
      "type": ".aos.test",
      "source_node": "node1",
      "max_size": %d,
      "frequency": %d
    },
    {
      "name": "/small",
      "type": ".aos.test",
      "source_node": "node1",
      "max_size": %d,
      "frequency": %d
    }
  ],
  "nodes": [
    { "name": "node1" }
  ]
})json",
        hugepage_fields, kBigSize, kFrequency, kSmallSize, kFrequency));
  }

  static size_t QueueMemorySize(
      const FlatbufferDetachedBuffer<Configuration> &config,
      std::string_view name) {
    const Channel *channel = configuration::GetChannel(
        &config.message(), name, ".aos.test", "",
        configuration::GetNode(&config.message(), "node1"));
    return ipc_lib::LocklessQueueMemorySize(
        ipc_lib::MakeQueueConfiguration(&config.message(), channel));
  }

  static long MinorFaults() {
    struct rusage usage;
    getrusage(RUSAGE_SELF, &usage);
    return usage.ru_minflt;
  }

  static constexpr std::filesystem::perms kPermissions =
      static_cast<std::filesystem::perms>(0770);
  static constexpr size_t kBigSize = 1200;
  static constexpr size_t kSmallSize = 100;
  static constexpr size_t kFrequency = 100;

  const std::string shm_dir_;
  const std::string huge_dir_;
  const size_t big_memory_size_;
  const size_t small_memory_size_;
  const size_t threshold_;
  FlatbufferDetachedBuffer<Configuration> fbs_config_;
  const Configuration *config_;
  const aos::Node *node1_;
  const aos::Channel *big_channel_;
  const aos::Channel *small_channel_;
};

using MemoryMappedQueueDeathTest = MemoryMappedQueueTest;

// A queue above the threshold is created under shm_hugepage_base and the
// shm_base path becomes a symlink to it.
TEST_F(MemoryMappedQueueTest, LargeQueueCreatedUnderHugepageBase) {
  ipc_lib::MemoryMappedQueue queue(shm_dir_, kPermissions, config_,
                                   big_channel_);

  const std::string path = ipc_lib::ShmPath(shm_dir_, big_channel_);
  const std::string hugepages_path = ipc_lib::ShmPath(huge_dir_, big_channel_);

  struct stat st;
  ASSERT_EQ(lstat(path.c_str(), &st), 0);
  ASSERT_TRUE(S_ISLNK(st.st_mode)) << path << " is not a symlink";
  char link_target[4096];
  const ssize_t len =
      readlink(path.c_str(), link_target, sizeof(link_target) - 1);
  ASSERT_GT(len, 0);
  link_target[len] = '\0';
  EXPECT_EQ(hugepages_path, link_target);

  ASSERT_EQ(stat(hugepages_path.c_str(), &st), 0);
  // Not a hugetlbfs dir, so no rounding: size is the logical queue size.
  EXPECT_EQ(static_cast<size_t>(st.st_size), big_memory_size_);
}

// A second MemoryMappedQueue opens the same channel through the symlink and
// shares the memory.
TEST_F(MemoryMappedQueueTest, SecondQueueFollowsSymlink) {
  ipc_lib::MemoryMappedQueue creator(shm_dir_, kPermissions, config_,
                                     big_channel_);

  ipc_lib::MemoryMappedQueue reader(shm_dir_, kPermissions, config_,
                                    big_channel_);

  // Offset past the lockless queue headers, inside the message area.
  const size_t offset = big_memory_size_ - 8;
  creator.GetMutableSharedMemory()[offset] = 42;
  EXPECT_EQ(reader.GetConstSharedMemory()[offset], 42);
}

// A queue below the threshold stays a regular file under shm_base.
TEST_F(MemoryMappedQueueTest, SmallQueueStaysOnShmBase) {
  ipc_lib::MemoryMappedQueue queue(shm_dir_, kPermissions, config_,
                                   small_channel_);

  struct stat st;
  ASSERT_EQ(lstat(ipc_lib::ShmPath(shm_dir_, small_channel_).c_str(), &st), 0);
  EXPECT_TRUE(S_ISREG(st.st_mode));
  EXPECT_EQ(stat(ipc_lib::ShmPath(huge_dir_, small_channel_).c_str(), &st), -1);
}

// A regular file already at the shm path of a queue that should be backed by
// huge pages is refused, rather than run on, and no orphan is left on the
// hugetlbfs mount.
TEST_F(MemoryMappedQueueDeathTest, ExistingRegularFileDies) {
  const std::string path = ipc_lib::ShmPath(shm_dir_, big_channel_);
  util::MkdirP(path, kPermissions);
  int fd = open(path.c_str(), O_RDWR | O_CREAT | O_EXCL,
                static_cast<mode_t>(kPermissions));
  ASSERT_NE(fd, -1);
  ASSERT_EQ(ftruncate(fd, big_memory_size_), 0);
  ASSERT_EQ(close(fd), 0);

  EXPECT_DEATH(
      {
        ipc_lib::MemoryMappedQueue queue(shm_dir_, kPermissions, config_,
                                         big_channel_);
      },
      "should be a symlink to");

  struct stat st;
  ASSERT_EQ(lstat(path.c_str(), &st), 0);
  EXPECT_TRUE(S_ISREG(st.st_mode)) << path << " was replaced by a symlink";
  const std::string hugepages_path = ipc_lib::ShmPath(huge_dir_, big_channel_);
  EXPECT_EQ(lstat(hugepages_path.c_str(), &st), -1)
      << "orphan left at " << hugepages_path;
}

// A shm_hugepage_base that isn't on hugetlbfs is refused.
TEST_F(MemoryMappedQueueDeathTest, NotOnHugetlbfsDies) {
  ipc_lib::AllowHugePagesOffHugetlbfsForTesting(false);
  EXPECT_DEATH(
      {
        ipc_lib::MemoryMappedQueue queue(shm_dir_, kPermissions, config_,
                                         big_channel_);
      },
      "is not on hugetlbfs");
}

// HugePagesNeededForNode counts only queues at or above the threshold.
TEST_F(MemoryMappedQueueTest, HugePagesNeededForNodeCountsLargeQueues) {
  constexpr size_t kHugePageSize = 2 * 1024 * 1024;
  const size_t expected =
      (big_memory_size_ + kHugePageSize - 1) / kHugePageSize;
  EXPECT_EQ(expected,
            ipc_lib::HugePagesNeededForNode(
                config_, node1_, (small_memory_size_ + big_memory_size_) / 2,
                kHugePageSize));
  // Nothing above the threshold -> no pages.
  EXPECT_EQ(0u, ipc_lib::HugePagesNeededForNode(
                    config_, node1_, big_memory_size_ + 1, kHugePageSize));
  // Threshold of 0 counts every queue readable on the node.
  EXPECT_EQ(expected + (small_memory_size_ + kHugePageSize - 1) / kHugePageSize,
            ipc_lib::HugePagesNeededForNode(config_, node1_, 0, kHugePageSize));
}

// Many MemoryMappedQueues, constructed directly (not through the cache) so
// each does its own open/ftruncate/symlink the way separate processes would,
// race to create the same large queue under shm_hugepage_base.  Exactly one
// hugepage file and one symlink must result and every queue must share the
// same memory.
TEST_F(MemoryMappedQueueTest, ParallelCreationOfLargeQueueSharesMemory) {
  constexpr int kThreads = 16;
  constexpr int kIterations = 20;
  const std::string path = ipc_lib::ShmPath(shm_dir_, big_channel_);
  const std::string hugepages_path = ipc_lib::ShmPath(huge_dir_, big_channel_);
  const std::string huge_folder = ipc_lib::ShmFolder(huge_dir_, big_channel_);

  for (int iteration = 0; iteration < kIterations; ++iteration) {
    std::vector<std::unique_ptr<ipc_lib::MemoryMappedQueue>> queues(kThreads);
    std::atomic<bool> start{false};
    std::vector<std::thread> threads;
    threads.reserve(kThreads);
    for (int i = 0; i < kThreads; ++i) {
      threads.emplace_back([this, &queues, &start, i]() {
        while (!start.load(std::memory_order_relaxed)) {
        }
        queues[i] = std::make_unique<ipc_lib::MemoryMappedQueue>(
            shm_dir_, kPermissions, config_, big_channel_);
      });
    }
    start.store(true);
    for (std::thread &thread : threads) {
      thread.join();
    }

    struct stat st;
    ASSERT_EQ(lstat(path.c_str(), &st), 0);
    ASSERT_TRUE(S_ISLNK(st.st_mode)) << path << " is not a symlink";
    char link_target[4096];
    const ssize_t len =
        readlink(path.c_str(), link_target, sizeof(link_target) - 1);
    ASSERT_GT(len, 0);
    link_target[len] = '\0';
    EXPECT_EQ(hugepages_path, link_target);

    ASSERT_EQ(stat(hugepages_path.c_str(), &st), 0);
    EXPECT_EQ(static_cast<size_t>(st.st_size), big_memory_size_);

    size_t entries = 0;
    for (const auto &entry : std::filesystem::directory_iterator(huge_folder)) {
      (void)entry;
      ++entries;
    }
    EXPECT_EQ(entries, 1u);

    // Queue i writes (char)(i + 1) at offset big_memory_size_ - 1 - i; every
    // queue must then read all of them back through the shared mapping.
    for (int i = 0; i < kThreads; ++i) {
      queues[i]->GetMutableSharedMemory()[big_memory_size_ - 1 - i] =
          static_cast<char>(i + 1);
    }
    for (int j = 0; j < kThreads; ++j) {
      for (int i = 0; i < kThreads; ++i) {
        EXPECT_EQ(queues[j]->GetConstSharedMemory()[big_memory_size_ - 1 - i],
                  static_cast<char>(i + 1))
            << "queue " << j << " reading the byte queue " << i << " wrote";
      }
    }

    queues.clear();
    aos::util::UnlinkRecursive(shm_dir_);
    aos::util::UnlinkRecursive(huge_dir_);
    ASSERT_EQ(mkdir(shm_dir_.c_str(), 0777), 0);
    ASSERT_EQ(mkdir(huge_dir_.c_str(), 0777), 0);
  }
}

// The write pre-fault run by a second MemoryMappedQueue must store to every
// page without changing the data.
TEST_F(MemoryMappedQueueTest, PageFaultDataWriteKeepsDataInline) {
  ipc_lib::MemoryMappedQueue creator(shm_dir_, kPermissions, config_,
                                     small_channel_);
  const std::vector<char> before(creator.GetConstSharedMemory().begin(),
                                 creator.GetConstSharedMemory().end());

  ipc_lib::MemoryMappedQueue second(shm_dir_, kPermissions, config_,
                                    small_channel_);

  EXPECT_TRUE(std::equal(before.begin(), before.end(),
                         second.GetConstSharedMemory().begin()));
}

// After a second MemoryMappedQueue's constructor write pre-fault, writing to
// every page of its read-write mapping takes no further minor faults.
// Trivially true on x86 (hardware sets the dirty bit on store); the real
// check is aarch64 without CONFIG_ARM64_HW_AFDBM, where a store to a clean
// PTE traps.
TEST_F(MemoryMappedQueueTest, SecondMappingWritesDoNotFault) {
  ipc_lib::MemoryMappedQueue creator(shm_dir_, kPermissions, config_,
                                     big_channel_);

  ipc_lib::MemoryMappedQueue reader(shm_dir_, kPermissions, config_,
                                    big_channel_);

  const long faults_before = MinorFaults();
  absl::Span<char> mem = reader.GetMutableSharedMemory();
  const long page_size = sysconf(_SC_PAGESIZE);
  for (size_t i = 0; i < mem.size(); i += page_size) {
    __atomic_fetch_or(reinterpret_cast<volatile char *>(&mem[i]), 0,
                      __ATOMIC_RELAXED);
  }
  EXPECT_EQ(0, MinorFaults() - faults_before);
}

#endif  // __linux__

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
