#ifndef FRC971_CONTROL_LOOPS_SWERVE_INVERSE_DYNAMICS_H_
#define FRC971_CONTROL_LOOPS_SWERVE_INVERSE_DYNAMICS_H_
#include <memory>

#include "absl/log/log.h"
#include "absl/log/vlog_is_on.h"
#include "absl/strings/str_format.h"

#include "aos/time/time.h"
#include "frc/control_loops/jacobian.h"
#include "frc/control_loops/swerve/auto_diff_jacobian.h"
#include "frc/control_loops/swerve/linearization_utils.h"
#include "include/ceres/tiny_solver.h"
#include "include/ceres/tiny_solver_autodiff_function.h"

namespace frc::control_loops::swerve {

template <typename ScalarT, typename State, typename Input,
          typename CeresFunctor>
class InverseDynamicsBase {
 public:
  static constexpr int kStateSize = State::SizeAtCompileTime;
  static constexpr int kInputSize = Input::SizeAtCompileTime;
  struct StateInput {
    State state;
    Input input;
  };

  InverseDynamicsBase(const State &frozen_states)
      :  // Convert to zeros & ones.
        frozen_states_(
            frozen_states.template cast<bool>().template cast<ScalarT>()) {}
  void set_max_solver_iterations(int iterations) {
    solver_.options.max_num_iterations = iterations;
  }

  StateInput InvertDynamics(const State &partial_goal) {
    Eigen::Matrix<ScalarT, CeresFunctor::NUM_PARAMETERS, 1> parameters;
    parameters.setZero();
    parameters.template topRows<kStateSize>() = partial_goal;
    const auto &summary = solver_.Solve(GetFunctor(partial_goal), &parameters);
    VLOG(1) << "Final cost " << summary.final_cost;
    VLOG(1) << "Iterations " << summary.iterations;
    VLOG(1) << "Status " << summary.status;
    return {.state = parameters.template topRows<kStateSize>(),
            .input = parameters.template block<kInputSize, 1>(kStateSize, 0)};
  }

 protected:
  virtual CeresFunctor GetFunctor(const State &partial_goal) = 0;

  State frozen_states_;

 private:
  ceres::TinySolver<CeresFunctor> solver_;
};

namespace internal {
template <typename BaseScalar, typename Dynamics, int kNumStates,
          int kNumInputs>
class DynamicsConstraint {
 public:
  DynamicsConstraint(
      const Dynamics &dynamics,
      const Eigen::Matrix<BaseScalar, kNumStates, 1> &frozen_states,
      const Eigen::Matrix<BaseScalar, kNumStates, 1> &partial_goal)
      : dynamics_(dynamics),
        frozen_states_(frozen_states),
        partial_goal_(partial_goal) {}

  template <typename Scalar>
  Eigen::Matrix<Scalar, 2 * kNumStates, 1> operator()(
      // Can we just template the eigen type here?
      const Eigen::Map<const Eigen::Matrix<Scalar, kNumStates + kNumInputs, 1>>
          params) const {
    Eigen::Matrix<Scalar, 2 * kNumStates, 1> residuals;
    const Eigen::Matrix<Scalar, kNumStates, 1> X_params(
        params.template topRows<kNumStates>());
    const Eigen::Matrix<Scalar, kNumInputs, 1> U(
        params.template bottomRows<kNumInputs>());
    const Eigen::Matrix<Scalar, kNumStates, 1> X =
        partial_goal_ +
        (Eigen::Matrix<BaseScalar, kNumStates, 1>::Ones() - frozen_states_)
            .cwiseProduct(X_params);
    residuals.template block<kNumStates, 1>(0, 0) = dynamics_(X, U);
    residuals.template block<kNumStates, 1>(kNumStates, 0) =
        (X_params - partial_goal_).cwiseProduct(frozen_states_);
    return residuals;
  }

 private:
  const Dynamics &dynamics_;
  const Eigen::Matrix<BaseScalar, kNumStates, 1> &frozen_states_;
  const Eigen::Matrix<BaseScalar, kNumStates, 1> &partial_goal_;
};

template <typename OutputScalar, typename InputScalar, int Rows, int Cols,
          std::enable_if_t<std::is_scalar_v<InputScalar>, bool> = true>
Eigen::Matrix<OutputScalar, Rows, Cols> ConvertMatrix(
    const Eigen::Matrix<InputScalar, Rows, Cols> &in) {
  return in.template cast<OutputScalar>();
}

template <typename OutputScalar, typename InputScalar, int Rows, int Cols,
          std::enable_if_t<!std::is_scalar_v<InputScalar>, bool> = true>
Eigen::Matrix<OutputScalar, Rows, Cols> ConvertMatrix(
    const Eigen::Matrix<InputScalar, Rows, Cols> &in) {
  Eigen::Matrix<OutputScalar, Rows, Cols> out;
  for (int col = 0; col < Cols; ++col) {
    for (int row = 0; row < Rows; ++row) {
      out(col, row) = in(col, row).a;
    }
  }
  return out;
}

template <bool kAutoDiff, int kNumInputs, int kNumOutputs, typename Scalar,
          typename Function, std::enable_if_t<kAutoDiff, bool> = true>
Eigen::Matrix<Scalar, kNumOutputs, kNumInputs> Jacobian(
    const Function &function, const Eigen::Matrix<Scalar, kNumInputs, 1> &X) {
  return AutoDiffJacobian<Scalar, Function, kNumInputs, kNumOutputs>::Jacobian(
      function, X);
}

template <bool kAutoDiff, int kNumInputs, int kNumOutputs, typename Scalar,
          typename Function, std::enable_if_t<!kAutoDiff, bool> = true>
Eigen::Matrix<Scalar, kNumOutputs, kNumInputs> Jacobian(
    const Function &function, const Eigen::Matrix<Scalar, kNumInputs, 1> &X) {
  return NumericalJacobian<kNumOutputs>(
      [&function](const Eigen::Matrix<Scalar, kNumInputs, 1> &input) {
        return function(Eigen::Map<const Eigen::Matrix<Scalar, kNumInputs, 1>>(
            input.data()));
      },
      X);
}

struct ResidualTiming {
  size_t count = 0;
  std::chrono::nanoseconds constraints{0};
  std::chrono::nanoseconds constraints_partial{0};
  template <typename Sink>
  friend void AbslStringify(Sink &sink, const ResidualTiming &timing) {
    absl::Format(
        &sink,
        "(constraints: %.2e sec total, constraints partial: %.2e sec total, %d "
        "samples)",
        aos::time::DurationInSeconds(timing.constraints),
        aos::time::DurationInSeconds(timing.constraints_partial), timing.count);
  }
};

template <typename BaseScalar, typename Scalar, typename Dynamics,
          int kNumStates, int kNumInputs, bool kSupportsAutoDiff>
void DynamicsResidual(
    const Dynamics &dynamics,
    const Eigen::Matrix<BaseScalar, kNumStates, 1> &frozen_states,
    const Eigen::Matrix<BaseScalar, kNumStates, 1> &partial_goal,
    const Eigen::Matrix<BaseScalar, kNumStates + kNumInputs,
                        kNumStates + kNumInputs> &Q,
    const Eigen::Map<const Eigen::Matrix<
        Scalar, kNumStates + kNumInputs + 2 * kNumStates, 1>>
        params,
    Eigen::Map<Eigen::Matrix<Scalar, 2 * kNumStates + (kNumStates + kNumInputs),
                             1>> *residuals,
    ResidualTiming *timing = nullptr) {
  aos::monotonic_clock::time_point start_time, constraints_time;
  if (timing != nullptr) {
    start_time = aos::monotonic_clock::now();
  }
  Eigen::Matrix<Scalar, kNumStates + kNumInputs, 1> primal_params =
      params.template block<kNumStates + kNumInputs, 1>(0, 0);
  DynamicsConstraint<BaseScalar, Dynamics, kNumStates, kNumInputs> constraint(
      dynamics, frozen_states, partial_goal);
  residuals->template block<2 * kNumStates, 1>(0, 0) = constraint(
      Eigen::Map<const Eigen::Matrix<Scalar, kNumStates + kNumInputs, 1>>{
          primal_params.data()});

  if (timing != nullptr) {
    constraints_time = aos::monotonic_clock::now();
  }

  // We do an evil thing here: In order to capture the partial derivative of the
  // constraints function, we use auto-differentiation. However, this method
  // itself may be called by something trying to do auto-differentiation and we
  // can't nest the ceres jets. Instead, just calculate the partial about the
  // current values and presume that the lost precision will be negligible.
  Eigen::Matrix<BaseScalar, 2 * kNumStates, kNumStates + kNumInputs>
      constraints_partial =
          Jacobian<kSupportsAutoDiff, kNumStates + kNumInputs, 2 * kNumStates>(
              constraint, ConvertMatrix<BaseScalar>(primal_params));

  // Partial derivative of the lagrangian for our KKT conditions.
  // L(x, v) = 0.5 * x' * Q * x + v' * g(x)
  // dL(x, v) / dx = Q * x + v' * dg(x) / dx
  residuals->template block<kNumStates + kNumInputs, 1>(2 * kNumStates, 0) =
      Q * primal_params +
      constraints_partial.transpose() *
          params.template block<2 * kNumStates, 1>(kNumStates + kNumInputs, 0);

  if (timing != nullptr) {
    timing->count++;
    timing->constraints += constraints_time - start_time;
    timing->constraints_partial +=
        aos::monotonic_clock::now() - constraints_time;
  }
}

template <typename ScalarT, typename State, typename Input>
struct NumericalCeresFunctor {
  typedef ScalarT Scalar;
  static constexpr int kStateSize = State::SizeAtCompileTime;
  static constexpr int kInputSize = Input::SizeAtCompileTime;
  enum {
    // First kStateSize residuals are Xdot; the second kStateSize residuals
    // are used to drive the frozen state variables to their desired values.
    // Third kStateSize are the partials of the Lagrangian w.r.t the input
    // state.
    // Final kInputSize residuals are the partisl of the Lagrangian w.r.t the
    // input inputs.
    NUM_RESIDUALS = 2 * kStateSize + (kStateSize + kInputSize),
    // Parameters are first the state + input that we are solving for and then
    // the dual variables for the equality constraints.
    NUM_PARAMETERS = kStateSize + kInputSize + 2 * kStateSize,
  };
  NumericalCeresFunctor(
      const DynamicsInterface<Scalar, kStateSize, kInputSize> *dynamics,
      const State &frozen_states, const State &partial_goal,
      const Eigen::Matrix<Scalar, kStateSize + kInputSize,
                          kStateSize + kInputSize> &Q)
      : dynamics_(dynamics),
        frozen_states_(frozen_states),
        partial_goal_(frozen_states_.cwiseProduct(partial_goal)),
        Q_(Q) {}

  bool operator()(const Scalar *parameters, Scalar *residuals,
                  Scalar *jacobian) const {
    const Eigen::Map<const Eigen::Matrix<Scalar, NUM_PARAMETERS, 1>>
        eigen_parameters(parameters);
    Eigen::Map<Eigen::Matrix<Scalar, NUM_RESIDUALS, 1>> eigen_residuals(
        residuals);
    internal::DynamicsResidual<
        Scalar, Scalar, DynamicsInterface<Scalar, kStateSize, kInputSize>,
        kStateSize, kInputSize, false>(*dynamics_, frozen_states_,
                                       partial_goal_, Q_, eigen_parameters,
                                       &eigen_residuals);
    if (jacobian != nullptr) {
      Eigen::Map<Eigen::Matrix<Scalar, NUM_RESIDUALS, NUM_PARAMETERS>>
          eigen_jacobian(jacobian);
      eigen_jacobian = NumericalJacobian<NUM_RESIDUALS, NUM_PARAMETERS, Scalar>(
          [this](const Eigen::Matrix<Scalar, NUM_PARAMETERS, 1> &X) {
            Eigen::Matrix<Scalar, NUM_RESIDUALS, 1> residuals;
            Eigen::Map<Eigen::Matrix<Scalar, NUM_RESIDUALS, 1>> residuals_map(
                residuals.data());
            internal::DynamicsResidual<
                Scalar, Scalar,
                DynamicsInterface<Scalar, kStateSize, kInputSize>, kStateSize,
                kInputSize, false>(
                *dynamics_, frozen_states_, partial_goal_, Q_,
                Eigen::Map<const Eigen::Matrix<Scalar, NUM_PARAMETERS, 1>>{
                    X.data()},
                &residuals_map);
            return residuals;
          },
          eigen_parameters);
    }
    return true;
  }

  const DynamicsInterface<Scalar, kStateSize, kInputSize> *dynamics_;
  State frozen_states_;
  State partial_goal_;
  Eigen::Matrix<Scalar, kStateSize + kInputSize, kStateSize + kInputSize> Q_;
};

template <typename BaseScalar, typename State, typename Input,
          typename Dynamics>
class AutoDiffCeresFunctor {
 public:
  static constexpr int kStateSize = State::SizeAtCompileTime;
  static constexpr int kInputSize = Input::SizeAtCompileTime;

  enum {
    // First kStateSize residuals are Xdot; the second kStateSize residuals
    // are used to drive the frozen state variables to their desired values.
    // Third kStateSize are the partials of the Lagrangian w.r.t the input
    // state.
    // Final kInputSize residuals are the partisl of the Lagrangian w.r.t the
    // input inputs.
    NUM_RESIDUALS = 2 * kStateSize + (kStateSize + kInputSize),
    // Parameters are first the state + input that we are solving for and then
    // the dual variables for the equality constraints.
    NUM_PARAMETERS = kStateSize + kInputSize + 2 * kStateSize,
  };

  typedef ceres::TinySolverAutoDiffFunction<
      AutoDiffCeresFunctor<BaseScalar, State, Input, Dynamics>, NUM_RESIDUALS,
      NUM_PARAMETERS, BaseScalar>
      TinySolverFunctor;

  AutoDiffCeresFunctor(const Dynamics &dynamics, const State &frozen_states,
                       const State &partial_goal,
                       const Eigen::Matrix<BaseScalar, kStateSize + kInputSize,
                                           kStateSize + kInputSize> &Q)
      : dynamics_(dynamics),
        frozen_states_(frozen_states),
        partial_goal_(frozen_states_.cwiseProduct(partial_goal)),
        Q_(Q) {}

  ~AutoDiffCeresFunctor() { VLOG(1) << "Timing: " << timing_; }

  template <typename Scalar>
  bool operator()(const Scalar *const parameters,
                  Scalar *const residuals) const {
    const Eigen::Map<const Eigen::Matrix<Scalar, NUM_PARAMETERS, 1>>
        eigen_parameters(parameters);
    Eigen::Map<Eigen::Matrix<Scalar, NUM_RESIDUALS, 1>> eigen_residuals(
        residuals);
    internal::DynamicsResidual<BaseScalar, Scalar, Dynamics, kStateSize,
                               kInputSize, true>(
        dynamics_, frozen_states_, partial_goal_, Q_, eigen_parameters,
        &eigen_residuals, VLOG_IS_ON(1) ? &timing_ : nullptr);
    return true;
  }

 private:
  const Dynamics &dynamics_;
  State frozen_states_;
  State partial_goal_;
  Eigen::Matrix<BaseScalar, kStateSize + kInputSize, kStateSize + kInputSize>
      Q_;
  // ceres needs a const function; storing timing information does not play nice
  // with that.
  mutable internal::ResidualTiming timing_;
};

template <typename Scalar, int StateSize, int InputSize,
          int CombinedSize = StateSize + InputSize>
Eigen::Matrix<Scalar, CombinedSize, CombinedSize> ConstructCombinedCost(
    const Eigen::Matrix<Scalar, StateSize, StateSize> &state_cost,
    const Eigen::Matrix<Scalar, InputSize, InputSize> &input_cost) {
  Eigen::Matrix<Scalar, CombinedSize, CombinedSize> combined;
  combined.setZero();
  combined.template block<StateSize, StateSize>(0, 0) = state_cost;
  combined.template block<InputSize, InputSize>(StateSize, StateSize) =
      input_cost;
  return combined;
}
}  // namespace internal

template <typename ScalarT, typename State, typename Input>
class NumericalInverseDynamics
    : public InverseDynamicsBase<
          ScalarT, State, Input,
          internal::NumericalCeresFunctor<ScalarT, State, Input>> {
 public:
  typedef internal::NumericalCeresFunctor<ScalarT, State, Input> CeresFunctor;
  static constexpr int kStateSize = static_cast<int>(State::RowsAtCompileTime);
  static constexpr int kInputSize = static_cast<int>(Input::RowsAtCompileTime);
  typedef DynamicsInterface<ScalarT, kStateSize, kInputSize> Dynamics;
  typedef Eigen::Matrix<ScalarT, kStateSize, kStateSize> StateCost;
  typedef Eigen::Matrix<ScalarT, kInputSize, kInputSize> InputCost;
  typedef Eigen::Matrix<ScalarT, kStateSize + kInputSize,
                        kStateSize + kInputSize>
      CombinedCost;
  NumericalInverseDynamics(std::unique_ptr<Dynamics> dynamics,
                           const State &frozen_states,
                           const StateCost &state_cost,
                           const InputCost &input_cost)
      : InverseDynamicsBase<ScalarT, State, Input, CeresFunctor>(frozen_states),
        dynamics_(std::move(dynamics)),
        cost_matrix_(internal::ConstructCombinedCost(state_cost, input_cost)) {}

 private:
  CeresFunctor GetFunctor(const State &partial_goal) override {
    return CeresFunctor(dynamics_.get(), this->frozen_states_, partial_goal,
                        cost_matrix_);
  }
  std::unique_ptr<Dynamics> dynamics_;
  CombinedCost cost_matrix_;
};

template <typename BaseScalar, typename State, typename Input,
          typename Dynamics>
class AutoDiffInverseDynamics
    : public InverseDynamicsBase<
          BaseScalar, State, Input,
          typename internal::AutoDiffCeresFunctor<
              BaseScalar, State, Input, Dynamics>::TinySolverFunctor> {
 public:
  typedef internal::AutoDiffCeresFunctor<BaseScalar, State, Input, Dynamics>
      CeresFunctor;
  typedef CeresFunctor::TinySolverFunctor TinySolverFunctor;
  static constexpr int kStateSize = static_cast<int>(State::RowsAtCompileTime);
  static constexpr int kInputSize = static_cast<int>(Input::RowsAtCompileTime);
  typedef Eigen::Matrix<BaseScalar, kStateSize, kStateSize> StateCost;
  typedef Eigen::Matrix<BaseScalar, kInputSize, kInputSize> InputCost;
  typedef Eigen::Matrix<BaseScalar, kStateSize + kInputSize,
                        kStateSize + kInputSize>
      CombinedCost;
  AutoDiffInverseDynamics(Dynamics dynamics, const State &frozen_states,
                          const StateCost &state_cost,
                          const InputCost &input_cost)
      : InverseDynamicsBase<BaseScalar, State, Input, TinySolverFunctor>(
            frozen_states),
        dynamics_(std::move(dynamics)),
        cost_matrix_(internal::ConstructCombinedCost(state_cost, input_cost)) {}

 private:
  TinySolverFunctor GetFunctor(const State &partial_goal) override {
    functor_.emplace(dynamics_, this->frozen_states_, partial_goal,
                     cost_matrix_);
    return TinySolverFunctor(functor_.value());
  }
  Dynamics dynamics_;
  CombinedCost cost_matrix_;
  std::optional<CeresFunctor> functor_;
};

}  // namespace frc::control_loops::swerve
#endif  // FRC971_CONTROL_LOOPS_SWERVE_INVERSE_DYNAMICS_H_
