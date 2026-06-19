#include "aos/events/glib_main_loop.h"

#include <glib.h>

#include <thread>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "gtest/gtest.h"

#include "aos/configuration.h"
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
          event_loop_.get(), simulated_event_loop_factory_->scheduler_epoll(),
          [this]() { simulated_event_loop_factory_->Exit(); });

      // Schedule a periodic event so that we actually start "realtime playback"
      // at time zero. Otherwise, playback immediately skips to the first event
      // which may be an arbitrary amount of time after time zero. We also want
      // to specify a periodic event so that there are always events being
      // waited on. Otherwise, the event loop will quit before the epoll events
      // have had a chance to execute.
      event_loop_->AddTimer([]() { VLOG(1) << "null event"; })
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
