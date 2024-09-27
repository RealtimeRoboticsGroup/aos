#include "frc971/control_loops/swerve/goal_solver.h"

#include "frc971/control_loops/swerve/motors.h"
#include "frc971/math/flatbuffers_matrix.h"

namespace frc971::control_loops::swerve {
GoalSolver::GoalSolver(
    aos::EventLoop *event_loop,
    const LinearVelocityController::DynamicsParameters &params)
    : inverse_dynamics_(LinearVelocityController::MakeInverseDynamics(params)),
      goal_sender_(event_loop->MakeSender<GoalStatic>("/drivetrain")) {
  event_loop->MakeWatcher("/drivetrain", [this](const JoystickGoal &msg) {
    HandleJoystickGoal(msg);
  });
}
void GoalSolver::HandleJoystickGoal(const JoystickGoal &msg) {
  const InverseDynamics::StateInput feedforwards =
      inverse_dynamics_.InvertDynamics(LinearVelocityController::MakeGoal(
          {msg.vx(), msg.vy(), msg.omega()}));
  auto builder = goal_sender_.MakeStaticBuilder();
  auto goal = builder->add_linear_velocity_goal();
  FromEigen(feedforwards.state.cast<double>(), goal->add_state());
  FromEigen(feedforwards.input.cast<double>(), goal->add_input());
  builder.CheckOk(builder.Send());
}
}  // namespace frc971::control_loops::swerve
