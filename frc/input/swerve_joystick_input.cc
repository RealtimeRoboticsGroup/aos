#include "frc/input/swerve_joystick_input.h"

#include "frc/input/driver_station_data.h"
#include "frc/input/redundant_joystick_data.h"

using frc::input::driver_station::ControlBit;

namespace frc::input {

void SwerveJoystickInput::RunIteration(
    const ::frc::input::driver_station::Data &unsorted_data) {
  if (input_config_.use_redundant_joysticks) {
    driver_station::RedundantData redundant_data_storage(unsorted_data);
    DoRunIteration(redundant_data_storage);
  } else {
    DoRunIteration(unsorted_data);
  }
}

void SwerveJoystickInput::DoRunIteration(
    const ::frc::input::driver_station::Data &data) {
  drivetrain_input_reader_->HandleDrivetrain(data);
  HandleTeleop(data);
  action_queue_.Tick();
  was_running_ = action_queue_.Running();
}

}  // namespace frc::input
