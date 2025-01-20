#include "y2022/control_loops/drivetrain/drivetrain_base.h"

#include <chrono>

#include "frc/control_loops/drivetrain/drivetrain_config.h"
#include "frc/control_loops/state_feedback_loop.h"
#include "y2022/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2022/control_loops/drivetrain/hybrid_velocity_drivetrain.h"
#include "y2022/control_loops/drivetrain/kalman_drivetrain_motor_plant.h"
#include "y2022/control_loops/drivetrain/polydrivetrain_dog_motor_plant.h"

using ::frc::control_loops::drivetrain::DownEstimatorConfigT;
using ::frc::control_loops::drivetrain::DrivetrainConfig;

namespace chrono = ::std::chrono;

namespace y2022::control_loops::drivetrain {

using ::frc::constants::ShifterHallEffect;

const ShifterHallEffect kThreeStateDriveShifter{0.0, 0.0, 0.25, 0.75};

const DrivetrainConfig<double> &GetDrivetrainConfig() {
  // Yaw of the IMU relative to the robot frame.
  static constexpr double kImuYaw = 0.0;
  static DrivetrainConfig<double> kDrivetrainConfig{
      ::frc::control_loops::drivetrain::ShifterType::kSimpleShifter,
      ::frc::control_loops::drivetrain::LoopType::kClosedLoop,
      ::frc::control_loops::drivetrain::GyroType::kSpartanGyro,
      ::frc::control_loops::drivetrain::ImuType::kImuFlippedX,

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
      0 /* down_offset if using constants use
     constants::GetValues().down_error */
      ,
      0.7 /* wheel_non_linearity */,
      1.2 /* quickturn_wheel_multiplier */,
      1.2 /* wheel_multiplier */,
      true /*pistol_grip_shift_enables_line_follow*/,
      (Eigen::Matrix<double, 3, 3>() << std::cos(kImuYaw), -std::sin(kImuYaw),
       0.0, std::sin(kImuYaw), std::cos(kImuYaw), 0.0, 0.0, 0.0, 1.0)
          .finished(),
      false /*is_simulated*/,
      DownEstimatorConfigT{{}, 0.015, 1000}};

  return kDrivetrainConfig;
};

}  // namespace y2022::control_loops::drivetrain
