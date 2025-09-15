#include "frc/wpilib/cancoder.h"

using frc::wpilib::CanCoder;

inline constexpr units::frequency::hertz_t kCANUpdateFreqHz = 200_Hz;

CanCoder::CanCoder(int device_id, std::string canbus,
                   std::vector<ctre::phoenix6::BaseStatusSignal *> *signals)
    : cancoder_(device_id, canbus),
      device_id_(device_id),
      position_(cancoder_.GetPosition()),
      absolute_position_(cancoder_.GetAbsolutePosition()) {
  position_.SetUpdateFrequency(kCANUpdateFreqHz);
  if (signals != nullptr) {
    signals->push_back(&position_);
  }

  absolute_position_.SetUpdateFrequency(kCANUpdateFreqHz);
  if (signals != nullptr) {
    signals->push_back(&absolute_position_);
  }
}

void CanCoder::SerializePosition(
    control_loops::CanCoderReadingStatic *can_coder, double gear_ratio) {
  can_coder->set_id(device_id_);
  can_coder->set_encoder(position() * gear_ratio);
  can_coder->set_absolute_encoder(absolute_position() * gear_ratio);
  can_coder->set_timestamp(GetTimestamp());
}
