#ifndef FRC971_CONTROL_LOOPS_SWERVE_LINEAR_VELOCITY_CONTROLLER_H_
#define FRC971_CONTROL_LOOPS_SWERVE_LINEAR_VELOCITY_CONTROLLER_H_
#include "frc971/control_loops/swerve/linearized_controller.h"
#include "frc971/control_loops/swerve/simplified_dynamics.h"

namespace frc971::control_loops::swerve {

class LinearVelocityController {
 public:
  using Scalar = float;
  using Dynamics = SimplifiedDynamics<Scalar>;
  static constexpr size_t kNumStates = Dynamics::kNumVelocityStates;
  using Controller = LinearizedController<kNumStates, Scalar>;
  using States = Dynamics::States;
  using Inputs = Dynamics::Inputs;
  using State = Controller::State;
  using Input = Controller::Input;
  using StateSquare = Controller::StateSquare;
  using InputSquare = Controller::InputSquare;
  using BMatrix = Controller::BMatrix;
  using Parameters = Controller::Parameters;
  using DynamicsParameters = Dynamics::Parameters;

  class VelocityControllerDynamics {
   public:
    VelocityControllerDynamics(Dynamics dynamics)
        : dynamics_(std::move(dynamics)) {}
    template <typename ScalarT>
    Eigen::Matrix<ScalarT, kNumStates, 1> operator()(
        const Eigen::Matrix<ScalarT, kNumStates, 1> &X,
        const Eigen::Matrix<ScalarT, kNumInputs, 1> &U) const {
      return dynamics_.VelocityDynamics(X, U);
    }

   private:
    const Dynamics dynamics_;
  };

  using VirtualDynamics = Dynamics::VirtualVelocityDynamics;

  static constexpr std::chrono::milliseconds kDt{10};

  struct Goal {
    Scalar vx;
    Scalar vy;
    Scalar omega;
  };

  struct ControllerDebug {
    Input U_ff;
    Input U_feedback;
    Eigen::Matrix<Scalar, kNumInputs, kNumStates> feedback_contributions;
    State goal;
    int sb02od_exit_code;
  };

  struct ControllerResult {
    Input U;
    ControllerDebug debug;
  };

  static Parameters MakeParameters(
      const DynamicsParameters &params = MakeDynamicsParameters());
  static DynamicsParameters MakeDynamicsParameters();

  LinearVelocityController(
      Parameters params = MakeParameters(),
      const DynamicsParameters &dynamics_params = MakeDynamicsParameters());

  ControllerResult RunRawController(const State &X, const State &goal,
                                    const Input &U_ff);

  static State MakeGoal(const Goal &goal);

 private:
  Controller controller_;
};

}  // namespace frc971::control_loops::swerve
#endif  // FRC971_CONTROL_LOOPS_SWERVE_LINEAR_VELOCITY_CONTROLLER_H_
