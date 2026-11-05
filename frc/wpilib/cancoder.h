#ifndef FRC_WPILIB_CANCODER_H_
#define FRC_WPILIB_CANCODER_H_

#include <atomic>
#include <chrono>
#include <cinttypes>
#include <string>
#include <vector>

#include "ctre/phoenix6/CANcoder.hpp"

#include "aos/containers/error_list.h"
#include "frc/control_loops/can_cancoder_static.h"

namespace frc::wpilib {

// Class which represents an absolute encoder read by a CANcoder over CAN.
class CANcoder {
 public:
  // inverted false means that counterclockwise is positive when looking at the
  // LED side of the CANcoder.
  //
  // Signals which should be read together are added to signals.
  CANcoder(int device_id, bool inverted, std::string canbus,
           std::vector<ctre::phoenix6::BaseStatusSignal *> *signals);

  void PrintConfigs();

  // This replaces all the configs on the device, so it clears any magnet offset
  // set from Tuner X.  Calibrate the offset in software instead.
  void WriteConfigs();

  ctre::phoenix6::hardware::CANcoder *cancoder() { return &cancoder_; }

  // The positions of the CANcoder are multiplied by gear_ratio.  They are NaN
  // once the CANcoder looks disconnected, until it is read successfully again.
  void SerializePosition(control_loops::CANcoderStatic *can_cancoder,
                         double gear_ratio);

  int device_id() const { return device_id_; }
  float position() const {
    return static_cast<units::angle::radian_t>(position_.GetValue()).value();
  }
  float absolute_position() const {
    return static_cast<units::angle::radian_t>(absolute_position_.GetValue())
        .value();
  }

  // returns the monotonic timestamp of the latest timesynced reading in the
  // timebase of the the syncronized CAN bus clock.
  int64_t GetTimestamp() {
    std::chrono::nanoseconds latest_timestamp =
        position_.GetTimestamp().GetTime();

    return latest_timestamp.count();
  }

 private:
  // Latches SETUP_FAILED if status says the CANcoder is disconnected, and dies
  // for any other failure.  See CheckCANSetupStatus.
  void CheckSetupStatus(ctre::phoenix::StatusCode status, const char *action);

  ctre::phoenix6::hardware::CANcoder cancoder_;
  int device_id_;

  ctre::phoenix6::signals::SensorDirectionValue sensor_direction_;

  ctre::phoenix6::StatusSignal<units::angle::turn_t> position_,
      absolute_position_;

  // WriteConfigs can run on a different thread than SerializePosition.
  std::atomic<bool> setup_failed_ = false;
  // Only touched by SerializePosition.
  ::aos::ErrorList<control_loops::CANDeviceError> errors_;

  // The number of failed reads in a row before we call the CANcoder
  // disconnected.
  static constexpr int kDisconnectedFailedReads = 5;
  // Only touched by SerializePosition.
  int failed_reads_ = 0;
};

}  // namespace frc::wpilib

#endif  // FRC_WPILIB_CANCODER_H_
