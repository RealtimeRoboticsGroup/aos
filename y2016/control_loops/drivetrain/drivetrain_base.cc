#include "y2016/control_loops/drivetrain/drivetrain_base.h"

#include <chrono>

#include "frc/control_loops/drivetrain/drivetrain_config.h"
#include "frc/control_loops/state_feedback_loop.h"
#include "y2016/constants.h"
#include "y2016/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2016/control_loops/drivetrain/hybrid_velocity_drivetrain.h"
#include "y2016/control_loops/drivetrain/kalman_drivetrain_motor_plant.h"
#include "y2016/control_loops/drivetrain/polydrivetrain_dog_motor_plant.h"

using ::frc::control_loops::drivetrain::DrivetrainConfig;

namespace chrono = ::std::chrono;

namespace y2016::control_loops::drivetrain {

using ::frc::constants::ShifterHallEffect;

const ShifterHallEffect kThreeStateDriveShifter{0.0, 0.0, 0.25, 0.75};

const DrivetrainConfig<double> &GetDrivetrainConfig() {
  static DrivetrainConfig<double> kDrivetrainConfig{
      ::frc::control_loops::drivetrain::ShifterType::kHallEffectShifter,
      ::frc::control_loops::drivetrain::LoopType::kClosedLoop,
      ::frc::control_loops::drivetrain::GyroType::kSpartanGyro,
      ::frc::control_loops::drivetrain::ImuType::kImuX,

      drivetrain::MakeDrivetrainLoop,
      drivetrain::MakeVelocityDrivetrainLoop,
      drivetrain::MakeKFDrivetrainLoop,
      drivetrain::MakeHybridVelocityDrivetrainLoop,

      chrono::duration_cast<chrono::nanoseconds>(
          chrono::duration<double>(drivetrain::kDt)),
      drivetrain::kRobotRadius,
      drivetrain::kWheelRadius,
      drivetrain::kV,

      drivetrain::kHighGearRatio,
      drivetrain::kLowGearRatio,
      drivetrain::kJ,
      drivetrain::kMass,
      kThreeStateDriveShifter,
      kThreeStateDriveShifter,
      true /* default_high_gear */,
      constants::GetValues().down_error,
      0.25 /* wheel_non_linearity */,
      1.0 /* quickturn_wheel_multiplier */,
      1.0 /* wheel_multiplier */,
  };

  return kDrivetrainConfig;
};

}  // namespace y2016::control_loops::drivetrain
