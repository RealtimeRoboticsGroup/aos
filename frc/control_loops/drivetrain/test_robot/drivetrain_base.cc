#include "frc/control_loops/drivetrain/test_robot/drivetrain_base.h"

#include <chrono>

#include "frc/control_loops/drivetrain/drivetrain_config.h"
#include "frc/control_loops/drivetrain/test_robot/drivetrain_dog_motor_plant.h"
#include "frc/control_loops/drivetrain/test_robot/hybrid_velocity_drivetrain.h"
#include "frc/control_loops/drivetrain/test_robot/kalman_drivetrain_motor_plant.h"
#include "frc/control_loops/drivetrain/test_robot/polydrivetrain_dog_motor_plant.h"
#include "frc/control_loops/state_feedback_loop.h"

using ::frc::control_loops::drivetrain::DrivetrainConfig;

namespace chrono = ::std::chrono;

namespace frc::control_loops::drivetrain::test_robot {

using ::frc::constants::ShifterHallEffect;

const ShifterHallEffect kThreeStateDriveShifter{0.0, 0.0, 0.25, 0.75};

const DrivetrainConfig<double> &GetDrivetrainConfig() {
  static DrivetrainConfig<double> kDrivetrainConfig{
      ::frc::control_loops::drivetrain::ShifterType::kHallEffectShifter,
      ::frc::control_loops::drivetrain::LoopType::kClosedLoop,
      ::frc::control_loops::drivetrain::GyroType::kSpartanGyro,
      ::frc::control_loops::drivetrain::ImuType::kImuX,

      MakeDrivetrainLoop,
      MakeVelocityDrivetrainLoop,
      MakeKFDrivetrainLoop,
      MakeHybridVelocityDrivetrainLoop,

      chrono::duration_cast<chrono::nanoseconds>(chrono::duration<double>(kDt)),
      kRobotRadius,
      kWheelRadius,
      kV,

      kHighGearRatio,
      kLowGearRatio,
      kJ,
      kMass,
      kThreeStateDriveShifter,
      kThreeStateDriveShifter,
      true /* default_high_gear */,
      0.0 /* down_offset */,
      0.25 /* wheel_non_linearity */,
      1.0 /* quickturn_wheel_multiplier */,
      1.0 /* wheel_multiplier */,
  };

  return kDrivetrainConfig;
};

}  // namespace frc::control_loops::drivetrain::test_robot
