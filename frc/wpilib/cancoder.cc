#include "frc/wpilib/cancoder.h"

#include <limits>

#include "absl/log/absl_check.h"

#include "aos/logging/logging.h"
#include "frc/wpilib/can_status.h"
#include "frc/wpilib/talonfx.h"

using frc::wpilib::CANcoder;
using frc::wpilib::kCANUpdateFreqHz;

CANcoder::CANcoder(int device_id, bool inverted, std::string canbus,
                   std::vector<ctre::phoenix6::BaseStatusSignal *> *signals)
    : cancoder_(device_id, canbus),
      device_id_(device_id),
      sensor_direction_(inverted
                            ? ctre::phoenix6::signals::SensorDirectionValue::
                                  Clockwise_Positive
                            : ctre::phoenix6::signals::SensorDirectionValue::
                                  CounterClockwise_Positive),
      position_(cancoder_.GetPosition()),
      absolute_position_(cancoder_.GetAbsolutePosition()) {
  ABSL_CHECK(signals != nullptr);

  CheckSetupStatus(position_.SetUpdateFrequency(kCANUpdateFreqHz),
                   "set the position rate");
  signals->push_back(&position_);

  CheckSetupStatus(absolute_position_.SetUpdateFrequency(kCANUpdateFreqHz),
                   "set the absolute position rate");
  signals->push_back(&absolute_position_);
}

void CANcoder::CheckSetupStatus(ctre::phoenix::StatusCode status,
                                const char *action) {
  if (!CheckCANSetupStatus(status, "cancoder", device_id_, action)) {
    setup_failed_ = true;
  }
}

void CANcoder::PrintConfigs() {
  ctre::phoenix6::configs::CANcoderConfiguration configuration;
  ctre::phoenix::StatusCode status =
      cancoder_.GetConfigurator().Refresh(configuration);
  CheckSetupStatus(status, "get the configuration");
  AOS_LOG(INFO, "configuration: %s", configuration.ToString().c_str());
}

void CANcoder::WriteConfigs() {
  ctre::phoenix6::configs::MagnetSensorConfigs magnet_sensor;
  magnet_sensor.SensorDirection = sensor_direction_;

  ctre::phoenix6::configs::CANcoderConfiguration configuration;
  configuration.MagnetSensor = magnet_sensor;

  ctre::phoenix::StatusCode status =
      cancoder_.GetConfigurator().Apply(configuration);
  CheckSetupStatus(status, "set the configuration");

  PrintConfigs();
}

void CANcoder::SerializePosition(control_loops::CANcoderStatic *can_cancoder,
                                 double gear_ratio) {
  if (setup_failed_) {
    errors_.Set(control_loops::CANDeviceError::SETUP_FAILED);
  }
  if (ctre::phoenix6::BaseStatusSignal::IsAllGood(position_,
                                                  absolute_position_)) {
    failed_reads_ = 0;
  } else {
    ++failed_reads_;
    errors_.Set(control_loops::CANDeviceError::READ_FAILED);
  }

  // The zeroing estimators treat a NaN absolute encoder as disconnected, and
  // fault the joint for good on the first one they see.  Wait for several
  // failures in a row so one missed read doesn't do that.
  const bool disconnected = failed_reads_ >= kDisconnectedFailedReads;
  constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

  can_cancoder->set_id(device_id_);
  can_cancoder->set_position(disconnected ? kNaN : position() * gear_ratio);
  can_cancoder->set_absolute_position(
      disconnected ? kNaN : absolute_position() * gear_ratio);
  can_cancoder->set_timestamp(GetTimestamp());
  errors_.ToStaticFlatbuffer(can_cancoder->add_errors());
}
