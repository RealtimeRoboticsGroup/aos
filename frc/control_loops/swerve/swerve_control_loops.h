#ifndef FRC971_CONTROL_LOOPS_SWERVE_SWERVE_CONTROL_LOOPS_H_
#define FRC971_CONTROL_LOOPS_SWERVE_SWERVE_CONTROL_LOOPS_H_

#include "aos/time/time.h"
#include "frc/control_loops/control_loop.h"
#include "frc/control_loops/profiled_subsystem_generated.h"
#include "frc/control_loops/profiled_subsystem_static.h"
#include "frc/control_loops/static_zeroing_single_dof_profiled_subsystem.h"
#include "frc/control_loops/swerve/inverse_kinematics.h"
#include "frc/control_loops/swerve/linear_velocity_controller.h"
#include "frc/control_loops/swerve/naive_estimator.h"
#include "frc/control_loops/swerve/swerve_drivetrain_can_position_generated.h"
#include "frc/control_loops/swerve/swerve_drivetrain_goal_generated.h"
#include "frc/control_loops/swerve/swerve_drivetrain_output_static.h"
#include "frc/control_loops/swerve/swerve_drivetrain_position_generated.h"
#include "frc/control_loops/swerve/swerve_drivetrain_status_static.h"
#include "frc/control_loops/swerve/swerve_zeroing_static.h"
#include "frc/control_loops/swerve/velocity_ekf.h"
#include "frc/queues/gyro_generated.h"
#include "frc/wpilib/imu_batch_generated.h"
#include "frc/wpilib/pigeon_generated.h"
#include "frc/zeroing/continuous_absolute_encoder.h"
#include "frc/zeroing/imu_zeroer.h"

namespace frc::control_loops::swerve {

inline void PopulateSwerveModuleRotation(
    SwerveModuleStatusStatic *swerve_module_table,
    const frc::control_loops::AbsoluteEncoderProfiledJointStatus
        *rotation_status) {
  auto rotation = swerve_module_table->add_rotation();
  CHECK(rotation->FromFlatbuffer(rotation_status));
}

// Handles the translation and rotation current for each swerve module
class SwerveControlLoops
    : public ::frc::controls::ControlLoop<Goal, Position, StatusStatic,
                                             OutputStatic> {
 public:
  typedef LinearVelocityController::Scalar Scalar;
  using AbsoluteEncoderSubsystem =
      ::frc::control_loops::StaticZeroingSingleDOFProfiledSubsystem<
          ::frc::zeroing::ContinuousAbsoluteEncoderZeroingEstimator,
          ::frc::control_loops::AbsoluteEncoderProfiledJointStatus>;
  using States = LinearVelocityController::States;

  explicit SwerveControlLoops(
      ::aos::EventLoop *event_loop,
      const frc::control_loops::
          StaticZeroingSingleDOFProfiledSubsystemCommonParams *rotation_params,
      const SwerveZeroing *zeroing_params,
      const NaiveEstimator::Parameters &params,
      const LinearVelocityController::ControllerWeights &lvc_weights,
      const ::std::string &name = "/swerve");

 protected:
  void RunIteration(
      const Goal *goal, const Position *position,
      aos::Sender<OutputStatic>::StaticBuilder *output_builder,
      aos::Sender<StatusStatic>::StaticBuilder *status_builder) override;
  int iteration_counter_ = 0;
  aos::Fetcher<CanPosition> can_position_fetcher_;
  aos::Fetcher<frc::wpilib::fbs::Pigeon2> pigeon_fetcher_;
  aos::Fetcher<frc::sensors::GyroReading> gyro_fetcher_;
  aos::Fetcher<frc::IMUValuesBatch> imu_fetcher_;
  std::optional<double> yaw_gyro_zero_;
  zeroing::Averager<double, 200> yaw_gyro_zeroer_;
  frc::zeroing::ImuZeroer imu_zeroer_;
  NaiveEstimator naive_estimator_;
  LinearVelocityController velocity_controller_;
  struct JoystickHeadingGoal {
    Scalar heading;
    aos::monotonic_clock::time_point last_time;
  };
  std::optional<JoystickHeadingGoal> desired_heading_;
  InverseKinematics<Scalar> inverse_kinematics_;
  VelocityEkf<Scalar> velocity_ekf_;
  LinearVelocityController::Input U_ = LinearVelocityController::Input::Zero();
  bool ekf_initialized_ = false;
};

}  // namespace frc::control_loops::swerve

#endif  // FRC971_CONTROL_LOOPS_SWERVE_SWERVE_CONTROL_LOOPS_H_
