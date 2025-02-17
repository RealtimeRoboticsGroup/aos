#ifndef FRC_CONTROL_LOOPS_SWERVE_NAIVE_ESTIMATOR_H_
#define FRC_CONTROL_LOOPS_SWERVE_NAIVE_ESTIMATOR_H_
#include "aos/time/time.h"
#include "frc/control_loops/swerve/simplified_dynamics.h"
#include "frc/control_loops/swerve/swerve_drivetrain_can_position_generated.h"
#include "frc/control_loops/swerve/swerve_drivetrain_position_generated.h"
#include "frc/control_loops/swerve/swerve_drivetrain_status_static.h"
#include "frc/control_loops/swerve/swerve_zeroing_static.h"
#include "frc/zeroing/continuous_absolute_encoder.h"

namespace frc::control_loops::swerve {

// This class provides an extremely simplified estimator for determining the
// current state of a swerve drivebase. This largely exists as a consequence of
// some ad-hoc testing that had to be done, and is kept around to serve as a
// sanity-check against more sophisticed estimators.
//
// The basic principle of operation for this is to very directly take the
// current sensor measurements and apply them to know the exact current state of
// the system. For things which require differentiation or integration, simple
// methods are used.
class NaiveEstimator {
 public:
  typedef float Scalar;
  using Dynamics = SimplifiedDynamics<Scalar>;
  using Parameters = Dynamics::Parameters;
  using ModuleParams = Dynamics::ModuleParams;
  using State = Dynamics::VelocityState<Scalar>;
  using States = Dynamics::States;
  NaiveEstimator(const SwerveZeroing *zeroing_params, const Parameters &params);

  // Provides an estimate of the current state of the system.
  State Update(aos::monotonic_clock::time_point now, const Position *position,
               const CanPosition *can_position, Scalar yaw_rate);

  void PopulateStatus(NaiveEstimatorStatusStatic *fbs) const;

 private:
  // Tracks the drive velocity, in m/s, of each module.
  std::array<Scalar, 4> velocities_;
  // Most recent drive encoder readings from the Talons.
  std::array<Scalar, 4> last_drive_positions_;
  // The time (from the clock on the Talon) of each drive encoder reading.
  std::array<aos::monotonic_clock::time_point, 4> last_drive_update_;
  // Zeroing estimators for each steer module.
  std::array<frc::zeroing::ContinuousAbsoluteEncoderZeroingEstimator, 4>
      zeroing_;
  State state_;
  Parameters params_;
  // When the Update() method was last called.
  std::optional<aos::monotonic_clock::time_point> last_update_;
};
}  // namespace frc::control_loops::swerve
#endif  // FRC_CONTROL_LOOPS_SWERVE_NAIVE_ESTIMATOR_H_
