#include "frc/wpilib/can_sensor_reader.h"

using frc::wpilib::CANSensorReader;
using frc::wpilib::kCANUpdateFreqHz;

CANSensorReader::CANSensorReader(
    aos::EventLoop *event_loop,
    std::vector<ctre::phoenix6::BaseStatusSignal *> signals_registry,
    std::vector<std::shared_ptr<TalonFX>> talonfxs,
    std::function<void(ctre::phoenix::StatusCode status)> flatbuffer_callback,
    SignalSync sync)
    : event_loop_(event_loop),
      sync_(sync),
      signals_(signals_registry.begin(), signals_registry.end()),
      talonfxs_(talonfxs),
      flatbuffer_callback_(flatbuffer_callback) {
  event_loop->SetRuntimeRealtimePriority(40);

  // TODO(max): Decide if we want to keep this on this core.
  event_loop->SetRuntimeAffinity(aos::MakeCpusetFromCpus({1}));

  CHECK(flatbuffer_callback_);
  timer_handler_ = event_loop->AddTimer([this]() { Loop(); });
  timer_handler_->set_name("CANSensorReader Loop");

  event_loop->OnRun([this]() {
    timer_handler_->Schedule(event_loop_->monotonic_now(),
                             1 / kCANUpdateFreqHz);
  });
}

void CANSensorReader::Loop() {
  ctre::phoenix::StatusCode status;
  if (sync_ == SignalSync::kDoSync) {
    status = ctre::phoenix6::BaseStatusSignal::WaitForAll(20_ms, signals_);
  } else {
    status = ctre::phoenix6::BaseStatusSignal::RefreshAll(signals_);
  }

  if (!status.IsOK()) {
    AOS_LOG(ERROR, "Failed to read signals from talonfx motors: %s: %s",
            status.GetName(), status.GetDescription());
  }

  flatbuffer_callback_(status);
}
