#ifndef FRC971_CONTROL_LOOPS_SWERVE_GOAL_SOLVER_H_
#define FRC971_CONTROL_LOOPS_SWERVE_GOAL_SOLVER_H_
#include "aos/events/event_loop.h"
#include "frc971/control_loops/swerve/inverse_dynamics.h"
#include "frc971/control_loops/swerve/inverse_kinematics.h"
#include "frc971/control_loops/swerve/linear_velocity_controller.h"
#include "frc971/control_loops/swerve/swerve_drivetrain_goal_static.h"
#include "frc971/control_loops/swerve/swerve_drivetrain_joystick_goal_static.h"
namespace frc971::control_loops::swerve {
class GoalSolver {
 public:
  typedef float Scalar;
  using InverseDynamics = LinearVelocityController::InverseDynamics;
  GoalSolver(aos::EventLoop *event_loop,
             const LinearVelocityController::DynamicsParameters &params);

 private:
  void HandleJoystickGoal(const JoystickGoal &msg);
  InverseDynamics inverse_dynamics_;
  aos::Sender<GoalStatic> goal_sender_;
};
}  // namespace frc971::control_loops::swerve
#endif  // FRC971_CONTROL_LOOPS_SWERVE_GOAL_SOLVER_H_
