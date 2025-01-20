#ifndef FRC_WPILIB_SWERVE_DRIVETRAIN_WRITER_H_
#define FRC_WPILIB_SWERVE_DRIVETRAIN_WRITER_H_

#include "ctre/phoenix6/TalonFX.hpp"

#include "frc/can_configuration_generated.h"
#include "frc/control_loops/swerve/swerve_drivetrain_output_generated.h"
#include "frc/wpilib/loop_output_handler.h"
#include "frc/wpilib/swerve/swerve_module.h"
#include "frc/wpilib/talonfx.h"

namespace frc::wpilib::swerve {

// Reads from the swerve output flatbuffer and uses wpilib to set the current
// for each motor.
class DrivetrainWriter : public ::frc::wpilib::LoopOutputHandler<
                             ::frc::control_loops::swerve::Output> {
 public:
  DrivetrainWriter(::aos::EventLoop *event_loop, int drivetrain_writer_priority,
                   double max_voltage);

  void set_talonfxs(SwerveModules modules);

  void HandleCANConfiguration(const CANConfiguration &configuration);

 private:
  void WriteConfigs();

  void Write(const ::frc::control_loops::swerve::Output &output) override;

  void Stop() override;

  double SafeSpeed(double voltage);

  SwerveModules modules_;

  double max_voltage_;
};

}  // namespace frc::wpilib::swerve

#endif  // FRC_WPILIB_SWERVE_DRIVETRAIN_WRITER_H_
