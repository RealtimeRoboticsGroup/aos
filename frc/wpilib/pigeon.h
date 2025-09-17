#ifndef FRC_WPILIB_PIGEON_H_
#define FRC_WPILIB_PIGEON_H_

#include <chrono>
#include <cinttypes>
#include <vector>

#include "ctre/phoenix6/Pigeon2.hpp"
#include "frc/wpilib/pigeon_static.h"

namespace frc::wpilib {

// Represents a CTRE CANcoder.
class Pigeon2 {
 public:
  Pigeon2(int device_id, std::string canbus,
           std::vector<ctre::phoenix6::BaseStatusSignal *> *signals);

  ctre::phoenix6::hardware::Pigeon2 *pigeon() { return &pigeon_; }

  float gyro_x() const {
    return static_cast<units::angular_velocity::radians_per_second_t>(gyro_x_.GetValue())
        .value();
  }

  void SerializePosition(fbs::Pigeon2Static *pigeon);

  int64_t GetTimestamp() {
    std::chrono::nanoseconds latest_timestamp =
        gyro_x_.GetTimestamp().GetTime();

    return latest_timestamp.count();
  }

 private:
  ctre::phoenix6::hardware::Pigeon2 pigeon_;
  int device_id_;

  ctre::phoenix6::StatusSignal<units::angular_velocity::degrees_per_second_t>
      gyro_x_;
};
}  // namespace frc::wpilib
#endif  // FRC_WPILIB_PIGEON_H_
