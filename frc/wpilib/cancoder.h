#ifndef FRC_WPILIB_CANCODER_H_
#define FRC_WPILIB_CANCODER_H_

#include <chrono>
#include <cinttypes>
#include <vector>

#include "ctre/phoenix6/CANcoder.hpp"
#include "frc/control_loops/can_coder_static.h"

namespace frc::wpilib {

// Represents a CTRE CANcoder.
class CanCoder {
 public:
  CanCoder(int device_id, std::string canbus,
           std::vector<ctre::phoenix6::BaseStatusSignal *> *signals);

  ctre::phoenix6::hardware::CANcoder *cancoder() { return &cancoder_; }

  float position() const {
    return static_cast<units::angle::radian_t>(position_.GetValue()).value();
  }
  float absolute_position() const {
    return static_cast<units::angle::radian_t>(absolute_position_.GetValue()).value();
  }

  // The position of the CANCoder encoder shaft is multiplied by gear_ratio
  void SerializePosition(control_loops::CanCoderReadingStatic *can_coder,
                         double gear_ratio);

  // returns the monotonic timestamp of the latest timesynced reading in the
  // timebase of the the syncronized CAN bus clock.
  int64_t GetTimestamp() {
    std::chrono::nanoseconds latest_timestamp =
        position_.GetTimestamp().GetTime();

    return latest_timestamp.count();
  }

 private:
  ctre::phoenix6::hardware::CANcoder cancoder_;
  int device_id_;

  ctre::phoenix6::StatusSignal<units::angle::turn_t> position_;
  ctre::phoenix6::StatusSignal<units::angle::turn_t> absolute_position_;
};
}  // namespace frc::wpilib
#endif  // FRC_WPILIB_CANCODER_H_
