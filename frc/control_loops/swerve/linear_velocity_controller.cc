#include "frc971/control_loops/swerve/linear_velocity_controller.h"

#include "absl/flags/flag.h"

ABSL_FLAG(double, thetas_q, 1.0, "");
ABSL_FLAG(double, omegas_q, 1e-3, "");
ABSL_FLAG(double, is_r, 1e-3, "");
ABSL_FLAG(double, vel_q, 10.0, "");

namespace frc971::control_loops::swerve {

LinearVelocityController::Parameters LinearVelocityController::MakeParameters(
    const LinearVelocityController::DynamicsParameters &params) {
  StateSquare Q = StateSquare::Zero();
  // We don't really care much about the actual angles of the swerve modules,
  // but make them non-zero to help guide things.
  Q.diagonal()(States::kThetas0) = absl::GetFlag(FLAGS_thetas_q);
  Q.diagonal()(States::kThetas1) = absl::GetFlag(FLAGS_thetas_q);
  Q.diagonal()(States::kThetas2) = absl::GetFlag(FLAGS_thetas_q);
  Q.diagonal()(States::kThetas3) = absl::GetFlag(FLAGS_thetas_q);
  Q.diagonal()(States::kOmegas0) = absl::GetFlag(FLAGS_omegas_q);
  Q.diagonal()(States::kOmegas1) = absl::GetFlag(FLAGS_omegas_q);
  Q.diagonal()(States::kOmegas2) = absl::GetFlag(FLAGS_omegas_q);
  Q.diagonal()(States::kOmegas3) = absl::GetFlag(FLAGS_omegas_q);
  Q.diagonal()(States::kTheta) = 1.2;
  Q.diagonal()(States::kVx) = absl::GetFlag(FLAGS_vel_q);
  Q.diagonal()(States::kVy) = absl::GetFlag(FLAGS_vel_q);
  Q.diagonal()(States::kOmega) = 1.4;

  InputSquare R = InputSquare::Zero();
  for (size_t index = 0; index < 4; ++index) {
    R.diagonal()(2 * index) = absl::GetFlag(FLAGS_is_r);
    R.diagonal()(2 * index + 1) = 1e-3;
  }
  return Parameters{.Q = Q,
                    .R = R,
                    .dt = kDt,
                    .dynamics = std::make_unique<VirtualDynamics>(params)};
}

namespace {
LinearVelocityController::Dynamics::ModuleParams MakeModule(
    const Eigen::Matrix<LinearVelocityController::Scalar, 2, 1> &position) {
  return {.position = position,
          .slip_angle_coefficient = 200.0,
          .slip_angle_alignment_coefficient = 10.0,
          .steer_motor = KrakenFOC(),
          .drive_motor = KrakenFOC(),
          .steer_ratio = 0.1,
          .drive_ratio = 0.01,
          .extra_steer_inertia = 0.0};
}
}  // namespace

LinearVelocityController::DynamicsParameters
LinearVelocityController::MakeDynamicsParameters() {
  return {.mass = 60,
          .moment_of_inertia = 2,
          .modules = {
              MakeModule({1.0, 1.0}),
              MakeModule({-1.0, 1.0}),
              MakeModule({-1.0, -1.0}),
              MakeModule({1.0, -1.0}),
          }};
}

LinearVelocityController::LinearVelocityController(Parameters params,
                                                   const DynamicsParameters &)
    : controller_(std::move(params)) {}

LinearVelocityController::State LinearVelocityController::MakeGoal(
    const Goal &goal) {
  State state = State::Zero();
  state(States::kVx) = goal.vx;
  state(States::kVy) = goal.vy;
  state(States::kOmega) = goal.omega;
  return state;
}

LinearVelocityController::ControllerResult
LinearVelocityController::RunRawController(const State &X, const State &goal,
                                           const Input &U_ff) {
  Controller::ControllerResult result =
      controller_.RunController(X, goal, U_ff);
  return {
      .U = result.U,
      .debug = {.U_ff = result.debug.U_ff,
                .U_feedback = result.debug.U_feedback,
                .feedback_contributions = result.debug.feedback_contributions,
                .goal = goal,
                .sb02od_exit_code = result.debug.sb02od_exit_code}};
}
}  // namespace frc971::control_loops::swerve
