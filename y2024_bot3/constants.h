#ifndef Y2024_BOT3_CONSTANTS_H_
#define Y2024_BOT3_CONSTANTS_H_

#include <array>
#include <cmath>
#include <cstdint>

#include "frc/constants.h"
#include "frc/control_loops/pose.h"
#include "frc/control_loops/static_zeroing_single_dof_profiled_subsystem.h"
#include "frc/zeroing/absolute_encoder.h"
#include "frc/zeroing/pot_and_absolute_encoder.h"
#include "y2024_bot3/constants/constants_generated.h"

namespace y2024_bot3::constants {

constexpr uint16_t kThirdRobotTeamNumber = 9971;

struct Values {
  static const int kSuperstructureCANWriterPriority = 35;
  static const int kDrivetrainWriterPriority = 35;
  static const int kDrivetrainTxPriority = 36;
  static const int kDrivetrainRxPriority = 36;

  struct PotAndAbsEncoderConstants {
    ::frc::control_loops::StaticZeroingSingleDOFProfiledSubsystemParams<
        ::frc::zeroing::PotAndAbsoluteEncoderZeroingEstimator>
        subsystem_params;
    double potentiometer_offset;
  };

  struct AbsoluteEncoderConstants {
    ::frc::control_loops::StaticZeroingSingleDOFProfiledSubsystemParams<
        ::frc::zeroing::AbsoluteEncoderZeroingEstimator>
        subsystem_params;
  };

  struct PotConstants {
    ::frc::control_loops::StaticZeroingSingleDOFProfiledSubsystemParams<
        ::frc::zeroing::RelativeEncoderZeroingEstimator>
        subsystem_params;
    double potentiometer_offset;
  };

  // todo: get the correct values for all these constants
  static constexpr double kIntakeRollerOutputRatio = (16.0 / 34.0);
  static constexpr double kArmOutputRatio = 1.0 / 32.0;

  static constexpr double kArmPotRatio() { return (12.0 / 48.0); }

  static constexpr double kArmEncoderCountsPerRevolution() { return 4096.0; }

  static constexpr double kArmEncoderRatio() { return (1.0 / 4.0); }

  static constexpr double kArmPotRadiansPerVolt() {
    return kArmPotRatio() * (10.0 /*turns*/ / 5.0 /*volts*/) *
           (2 * M_PI /*radians*/);
  }

  static constexpr double kRotationModuleRatio() { return (1.0 / 12.1); }

  static constexpr double kTranslationModuleRatio() {
    return (12.0 / 54.0 * 38.0 / 16.0 * 15.0 / 45.0) * 1.8 * 0.0254;
  }

  static constexpr double kMaxDrivetrainEncoderPulsesPerSecond() {
    return 1200000;
  }
};

// Creates and returns a Values instance for the constants.
// Should be called before realtime because this allocates memory.
// Only the first call to either of these will be used.
constants::Values MakeValues(uint16_t team);

// Calls MakeValues with aos::network::GetTeamNumber()
constants::Values MakeValues();

}  // namespace y2024_bot3::constants

#endif  // Y2024_BOT3_CONSTANTS_H_
