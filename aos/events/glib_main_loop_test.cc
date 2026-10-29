#include "aos/events/glib_main_loop.h"

#include <glib.h>

#include <thread>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "gtest/gtest.h"

#include "aos/configuration.h"
#include "aos/events/pipe.h"
#include "aos/events/shm_event_loop.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/testing/path.h"

namespace aos::testing {
using aos::testing::ArtifactPath;

const FlatbufferDetachedBuffer<Configuration> &Config() {
  static const FlatbufferDetachedBuffer<Configuration> result =
      configuration::ReadConfig(ArtifactPath("aos/events/aos_config.json"));
  return result;
}

enum class EventLoopType { kShm, kSimulated };

class GlibMainLoopTest : public ::testing::TestWithParam<EventLoopType> {
 public:
  GlibMainLoopTest() {
    if (GetParam() == EventLoopType::kShm) {
      // Instantiate an ShmEventLoop directly.
      shm_event_loop_ = std::make_unique<ShmEventLoop>(Config());
      glib_main_loop_ = std::make_unique<GlibMainLoop>(shm_event_loop_.get());
    } else {
      // Set up an event loop factory and then the necessary pieces to run the
      // GlibMainLoop.
      simulated_event_loop_factory_ =
          std::make_unique<SimulatedEventLoopFactory>(&Config().message());
      simulated_event_loop_factory_->SetRealtimeReplayRate(1.0);
      event_loop_ = simulated_event_loop_factory_->MakeEventLoop("test");
      event_loop_->SkipTimingReport();
      glib_main_loop_ = std::make_unique<GlibMainLoop>(
          event_loop_.get(), simulated_event_loop_factory_->scheduler_aio(),
          [this]() { simulated_event_loop_factory_->Exit(); });

      // Schedule a periodic event so that we actually start "realtime playback"
      // at time zero. Otherwise, playback immediately skips to the first event
      // which may be an arbitrary amount of time after time zero. We also want
      // to specify a periodic event so that there are always events being
      // waited on. Otherwise, the event loop will quit before the fd events
      // have had a chance to execute.
      event_loop_->AddTimer([]() { ABSL_VLOG(1) << "null event"; })
          ->Schedule(event_loop_->monotonic_now(), std::chrono::seconds(1));
    }
  }

  // Retrieves the event loop being tested.
  EventLoop *event_loop() {
    if (shm_event_loop_) {
      return shm_event_loop_.get();
    }
    return event_loop_.get();
  }

  // Retrieves the GlibMainLoop being tested.
  GlibMainLoop *glib_main_loop() { return glib_main_loop_.get(); }

  // Destroys the GlibMainLoop, leaving the event loop it was on.
  void DestroyGlibMainLoop() { glib_main_loop_.reset(); }

  // Runs the event loop until something calls Exit().
  void Run() {
    if (shm_event_loop_) {
      shm_event_loop_->Run();
    } else {
      simulated_event_loop_factory_->Run();
    }
  }

  // Exits the event loop execution started by Run().
  void Exit() {
    if (shm_event_loop_) {
      shm_event_loop_->Exit();
    } else {
      simulated_event_loop_factory_->Exit();
    }
  }

  // Watches pipe's read end for condition through glib, and runs until the
  // watch is dispatched or 2 seconds have passed.  Returns what the watch was
  // dispatched with, or 0 if it never was.  The watch removes itself when it
  // is dispatched, and with close_in_callback closes the read end too, before
  // GlibMainLoop learns that glib has stopped asking about it.
  GIOCondition WatchPipe(Pipe &pipe, GIOCondition condition,
                         bool close_in_callback = false) {
    struct State {
      GlibMainLoopTest *test;
      Pipe *pipe;
      bool close_in_callback;
      GIOCondition dispatched;
    } state{this, &pipe, close_in_callback, static_cast<GIOCondition>(0)};
    GIOChannel *const channel = g_io_channel_unix_new(pipe.read_fd());
    const guint watch = g_io_add_watch(
        channel, condition,
        [](GIOChannel *, GIOCondition dispatched, gpointer data) -> gboolean {
          State *const state = static_cast<State *>(data);
          state->dispatched = dispatched;
          if (state->close_in_callback) {
            state->pipe->close_read_fd();
          }
          state->test->Exit();
          return G_SOURCE_REMOVE;
        },
        &state);

    TimerHandler *const give_up = event_loop()->AddTimer([this]() { Exit(); });
    event_loop()->OnRun([this, give_up]() {
      give_up->Schedule(event_loop()->monotonic_now() +
                        std::chrono::seconds(2));
    });
    Run();

    if (state.dispatched == 0) {
      g_source_remove(watch);
    }
    g_io_channel_unref(channel);
    return state.dispatched;
  }

 private:
  std::unique_ptr<ShmEventLoop> shm_event_loop_;
  std::unique_ptr<SimulatedEventLoopFactory> simulated_event_loop_factory_;
  std::unique_ptr<EventLoop> event_loop_;
  std::unique_ptr<GlibMainLoop> glib_main_loop_;
};

// Tests just creating and destroying without running.
TEST_P(GlibMainLoopTest, CreateDestroy) {
  // Setup handled in constructor.
}

// Tests just creating, running, and then destroying, without adding any
// events from the glib side.
TEST_P(GlibMainLoopTest, CreateRunDestroy) {
  bool ran = false;
  event_loop()
      ->AddTimer([this, &ran]() {
        Exit();
        ran = true;
      })
      ->Schedule(event_loop()->monotonic_now() +
                 std::chrono::milliseconds(100));
  Run();
  EXPECT_TRUE(ran);
}

// Tests that the event loop keeps running after the GlibMainLoop on it is gone.
TEST_P(GlibMainLoopTest, DestroyWhileRunning) {
  bool ran = false;
  event_loop()
      ->AddTimer([this]() { DestroyGlibMainLoop(); })
      ->Schedule(event_loop()->monotonic_now() + std::chrono::milliseconds(10));
  event_loop()
      ->AddTimer([this, &ran]() {
        Exit();
        ran = true;
      })
      ->Schedule(event_loop()->monotonic_now() +
                 std::chrono::milliseconds(100));
  Run();
  EXPECT_TRUE(ran);
}

// Tests just a single idle source.
TEST_P(GlibMainLoopTest, IdleSource) {
  int runs = 0;
  const auto callback = glib_main_loop()->AddIdle([this, &runs]() -> gboolean {
    if (runs++ >= 100) {
      Exit();
    }
    return true;
  });
  Run();
  EXPECT_GT(runs, 100);
  // It can run a few extra times, but not too many.
  EXPECT_LT(runs, 110);
}

// Tests just a single timeout which calls exit on the EventLoop side.
TEST_P(GlibMainLoopTest, TimeoutExitAos) {
  int runs = 0;
  const auto callback = glib_main_loop()->AddTimeout(
      [this, &runs]() -> gboolean {
        if (runs++ >= 3) {
          Exit();
        }
        return true;
      },
      50);
  const auto before = event_loop()->monotonic_now();
  Run();
  const auto after = event_loop()->monotonic_now();
  EXPECT_EQ(runs, 4);
  // Verify it took at least this long, but don't bother putting an upper bound
  // because it can take arbitrarily long due to scheduling delays.
  EXPECT_GE(after - before, std::chrono::milliseconds(200));
}

// Tests just a single timeout which calls exit on the glib side.
TEST_P(GlibMainLoopTest, TimeoutExitGlib) {
  int runs = 0;
  const auto callback = glib_main_loop()->AddTimeout(
      [this, &runs]() -> gboolean {
        if (runs++ >= 3) {
          g_main_loop_quit(glib_main_loop()->g_main_loop());
        }
        return true;
      },
      50);
  const auto before = event_loop()->monotonic_now();
  Run();
  const auto after = event_loop()->monotonic_now();
  EXPECT_EQ(runs, 4);
  // Verify it took at least this long, but don't bother putting an upper bound
  // because it can take arbitrarily long due to scheduling delays.
  EXPECT_GE(after - before, std::chrono::milliseconds(200));
}

// Tests a single timeout which removes itself, and a EventLoop timer to end
// the test.
TEST_P(GlibMainLoopTest, TimeoutRemoveSelf) {
  int runs = 0;
  const auto callback = glib_main_loop()->AddTimeout(
      [&runs]() -> gboolean {
        ++runs;
        return false;
      },
      50);
  bool ran = false;
  TimerHandler *timer = event_loop()->AddTimer([this, &ran]() {
    ABSL_LOG(INFO) << "Triggering exit at " << event_loop()->monotonic_now();
    Exit();
    ran = true;
  });
  event_loop()->OnRun([this, timer] {
    timer->Schedule(event_loop()->monotonic_now() +
                    std::chrono::milliseconds(100));
  });
  ABSL_LOG(INFO) << "Triggering run at " << event_loop()->monotonic_now();
  Run();
  EXPECT_TRUE(ran);
  EXPECT_EQ(runs, 1);
}

// A watch that removes itself from its own callback is taken off the Aio
// the next time GlibMainLoop asks glib what to watch.
TEST_P(GlibMainLoopTest, WatchRemovesItself) {
  Pipe pipe;
  pipe.Write("x");
  EXPECT_NE(WatchPipe(pipe, G_IO_IN), 0) << "the watch was never dispatched";
}

// A watch whose callback removes it and closes its fd leaves GlibMainLoop
// to forget a closed fd, rather than to remove an open one.
TEST_P(GlibMainLoopTest, WatchClosesItsFdAndRemovesItself) {
  Pipe pipe;
  pipe.Write("x");
  EXPECT_NE(WatchPipe(pipe, G_IO_IN, /*close_in_callback=*/true), 0)
      << "the watch was never dispatched";
}

// Run the above tests against both ShmEventLoop and a simulated event loop.
INSTANTIATE_TEST_SUITE_P(
    GlibMainLoopTest, GlibMainLoopTest,
    ::testing::Values(EventLoopType::kShm, EventLoopType::kSimulated),
    [](const ::testing::TestParamInfo<EventLoopType> &info) {
      switch (info.param) {
        case EventLoopType::kShm:
          return "shm";
        case EventLoopType::kSimulated:
          return "simulated";
      }
    });

}  // namespace aos::testing
