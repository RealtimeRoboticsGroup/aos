#include "frc/wpilib/pigeon.h"

namespace frc::wpilib {

inline constexpr units::frequency::hertz_t kCANUpdateFreqHz = 200_Hz;

Pigeon2::Pigeon2(int device_id, std::string canbus,
                 std::vector<ctre::phoenix6::BaseStatusSignal *> *signals)
    : pigeon_(device_id, canbus),
      device_id_(device_id),
      gyro_x_(pigeon_.GetAngularVelocityXDevice()) {
  gyro_x_.SetUpdateFrequency(kCANUpdateFreqHz);
  if (signals != nullptr) {
    signals->push_back(&gyro_x_);
  }
}

void Pigeon2::SerializePosition(fbs::Pigeon2Static *pigeon) {
  pigeon->set_id(device_id_);
  pigeon->set_gyro_x(gyro_x());
  pigeon->set_timestamp(GetTimestamp());
}
}  // namespace frc::wpilib
