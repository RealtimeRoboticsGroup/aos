#include "frc/wpilib/loop_output_handler.h"

#include <chrono>

#include "gtest/gtest.h"

#include "aos/events/simulated_event_loop.h"
#include "aos/realtime.h"
#include "aos/testing/test_logging.h"
#include "aos/time/time.h"
#include "frc/wpilib/loop_output_handler_test_generated.h"

namespace frc::wpilib::testing {
namespace {
namespace chrono = ::std::chrono;
using ::aos::monotonic_clock;
}  // namespace

class LoopOutputHandlerTest : public ::testing::Test {
 public:
  LoopOutputHandlerTest()
      : configuration_(aos::configuration::ReadConfig(
            "frc/wpilib/loop_output_handler_test_config.json")),
        event_loop_factory_(&configuration_.message()),
        loop_output_hander_event_loop_(
            event_loop_factory_.MakeEventLoop("output")),
        test_event_loop_(event_loop_factory_.MakeEventLoop("test")) {
    ::aos::testing::EnableTestLogging();
  }

  aos::FlatbufferDetachedBuffer<aos::Configuration> configuration_;
  ::aos::SimulatedEventLoopFactory event_loop_factory_;
  ::std::unique_ptr<::aos::EventLoop> loop_output_hander_event_loop_;
  ::std::unique_ptr<::aos::EventLoop> test_event_loop_;
};

// Test loop output handler which logs and counts.
class TestLoopOutputHandler
    : public LoopOutputHandler<LoopOutputHandlerTestOutput> {
 public:
  TestLoopOutputHandler(::aos::EventLoop *event_loop, const ::std::string &name)
      : LoopOutputHandler(event_loop, name) {}

  int count() const { return count_; }

  ::aos::monotonic_clock::time_point last_time() const { return last_time_; }
  ::aos::monotonic_clock::time_point stop_time() const { return stop_time_; }

 protected:
  void Write(const LoopOutputHandlerTestOutput &output) override {
    aos::CheckRealtime();
    // We don't care if this is RT if we are testing.
    aos::ScopedNotRealtime nrt;
    LOG(INFO) << "output " << aos::FlatbufferToJson(&output);
    ++count_;
    last_time_ = event_loop()->monotonic_now();
  }

  void Stop() override {
    aos::CheckRealtime();
    // We don't care if this is RT if we are testing.
    aos::ScopedNotRealtime nrt;
    stop_time_ = event_loop()->monotonic_now();
    LOG(INFO) << "Stopping";
  }

 private:
  int count_ = 0;

  ::aos::monotonic_clock::time_point last_time_ =
      ::aos::monotonic_clock::min_time;
  ::aos::monotonic_clock::time_point stop_time_ =
      ::aos::monotonic_clock::min_time;
};

// Test that the watchdog calls Stop at the right time.
TEST_F(LoopOutputHandlerTest, WatchdogTest) {
  TestLoopOutputHandler loop_output(loop_output_hander_event_loop_.get(),
                                    "/test");

  ::aos::Sender<LoopOutputHandlerTestOutput> output_sender =
      test_event_loop_->MakeSender<LoopOutputHandlerTestOutput>("/test");

  const monotonic_clock::time_point start_time =
      test_event_loop_->monotonic_now();

  int count = 0;
  // Send outputs for 1 second.
  ::aos::TimerHandler *timer_handle = test_event_loop_->AddTimer(
      [this, &start_time, &output_sender, &loop_output, &count]() {
        EXPECT_EQ(count, loop_output.count());
        if (test_event_loop_->monotonic_now() <
            start_time + chrono::seconds(1)) {
          auto builder = output_sender.MakeBuilder();
          LoopOutputHandlerTestOutput::Builder output_builder =
              builder.MakeBuilder<LoopOutputHandlerTestOutput>();
          output_builder.add_voltage(5.0);
          EXPECT_EQ(builder.Send(output_builder.Finish()),
                    aos::RawSender::Error::kOk);

          ++count;
        }
        LOG(INFO) << "Ping";
      });

  // Kick off the ping timer handler.
  test_event_loop_->OnRun([this, &timer_handle]() {
    timer_handle->Schedule(test_event_loop_->monotonic_now(),
                           chrono::milliseconds(5));
  });

  event_loop_factory_.RunFor(chrono::seconds(2));

  // Confirm the watchdog
  EXPECT_EQ(loop_output.stop_time(),
            loop_output.last_time() + chrono::milliseconds(100));
}

}  // namespace frc::wpilib::testing
