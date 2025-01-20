#include "frc/wpilib/buffered_solenoid.h"

#include "frc/wpilib/buffered_pcm.h"

namespace frc::wpilib {

void BufferedSolenoid::Set(bool value) { pcm_->DoSet(number_, value); }

}  // namespace frc::wpilib
