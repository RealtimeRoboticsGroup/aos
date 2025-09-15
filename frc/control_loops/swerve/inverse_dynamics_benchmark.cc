#include "aos/init.h"
#include "aos/realtime.h"
#include "frc/control_loops/swerve/inverse_dynamics.h"
#include "frc/control_loops/swerve/linear_velocity_controller.h"
#include "frc/control_loops/swerve/linearization_utils.h"
#include "frc/control_loops/swerve/simplified_dynamics.h"

ABSL_FLAG(int, iterations, 100, "Number of iterations to run solvers for.");
ABSL_FLAG(
    int, max_inverse_dynamics_solver_iterations, 50,
    "Maximum number of iterations to run the inverse dynamics solver for.");

namespace frc::control_loops::swerve {

namespace {
constexpr int kNumStates = SimplifiedDynamics<double>::kNumVelocityStates;
template <typename Scalar>
static SimplifiedDynamics<Scalar>::ModuleParams MakeModule(
    const Eigen::Matrix<Scalar, 2, 1> &position) {
  return typename SimplifiedDynamics<Scalar>::ModuleParams{
      .position = position,
      .slip_angle_coefficient = 200.0,
      .slip_angle_alignment_coefficient = 1.0,
      .steer_motor = KrakenFOC(),
      .drive_motor = KrakenFOC(),
      .steer_ratio = 0.1,
      .drive_ratio = 0.01};
}
template <typename Scalar>
static SimplifiedDynamics<Scalar>::Parameters MakeParams() {
  return {.mass = 60,
          .moment_of_inertia = 2,
          .modules = {
              MakeModule<Scalar>({1.0, 1.0}),
              MakeModule<Scalar>({-1.0, 1.0}),
              MakeModule<Scalar>({-1.0, -1.0}),
              MakeModule<Scalar>({1.0, -1.0}),
          }};
}
}  // namespace

class SimplifiedDynamicsAutoDiff {
 public:
  SimplifiedDynamicsAutoDiff(SimplifiedDynamics<double> dynamics)
      : dynamics_(std::move(dynamics)) {}
  template <typename Scalar>
  Eigen::Matrix<Scalar, kNumStates, 1> operator()(
      const Eigen::Matrix<Scalar, kNumStates, 1> &X,
      const Eigen::Matrix<Scalar, kNumInputs, 1> &U) const {
    return dynamics_.VelocityDynamics(X, U);
  }

 private:
  const SimplifiedDynamics<double> dynamics_;
};

template <typename Scalar,
          typename State = Eigen::Matrix<Scalar, kNumStates, 1>,
          typename Input = Eigen::Matrix<Scalar, kNumInputs, 1>>
class SimplifiedDynamicsNumerical
    : public DynamicsInterface<Scalar, kNumStates, kNumInputs> {
 public:
  SimplifiedDynamicsNumerical(
      const SimplifiedDynamics<Scalar>::Parameters &params)
      : dynamics_(params) {}
  State operator()(const State &X, const Input &U) const {
    return dynamics_.VelocityDynamics(X, U);
  }

 private:
  const SimplifiedDynamics<Scalar> dynamics_;
};

using AutoInverseDynamics =
    AutoDiffInverseDynamics<double, Eigen::Matrix<double, kNumStates, 1>,
                            Eigen::Matrix<double, kNumInputs, 1>,
                            SimplifiedDynamicsAutoDiff>;

template <typename Scalar>
using NumericalInverseSimplifiedDynamics =
    NumericalInverseDynamics<Scalar, Eigen::Matrix<Scalar, kNumStates, 1>,
                             Eigen::Matrix<Scalar, kNumInputs, 1>>;

template <typename Scalar>
void TestDynamicsLinearization() {
  const size_t kIterations = absl::GetFlag(FLAGS_iterations);
  SimplifiedDynamics<Scalar> dynamics(MakeParams<Scalar>());
  auto start_time = aos::monotonic_clock::now();
  for (size_t i = 0; i < kIterations; ++i) {
    dynamics.LinearizedDynamics(
        SimplifiedDynamics<Scalar>::template PositionState<>::Zero(),
        SimplifiedDynamics<Scalar>::template Input<>::Zero());
  }
  auto duration = aos::monotonic_clock::now() - start_time;
  LOG(INFO) << "Average time for " << kIterations << " iterations of "
            << (sizeof(Scalar) == 8 ? "double" : "single")
            << " precision dynamics linearization: "
            << aos::time::DurationInSeconds(duration / kIterations) << " sec";
}

int Main() {
  const size_t kIterations = absl::GetFlag(FLAGS_iterations);

  LOG(INFO) << "Running dynamics linearization:";

  TestDynamicsLinearization<float>();
  TestDynamicsLinearization<double>();
  {
    SimplifiedDynamics<double> dynamics(MakeParams<double>());
    auto functor =
        [&dynamics](
            const Eigen::Matrix<
                double, SimplifiedDynamics<double>::kNumPositionStates, 1> &X,
            const Eigen::Matrix<double, kNumInputs, 1> &U) {
          return dynamics.Dynamics(X, U);
        };
    auto start_time = aos::monotonic_clock::now();
    for (size_t i = 0; i < kIterations; ++i) {
      NumericalJacobianX<SimplifiedDynamics<double>::kNumPositionStates,
                         kNumInputs, double>(
          functor, SimplifiedDynamics<double>::PositionState<>::Zero(),
          SimplifiedDynamics<double>::Input<>::Zero());

      NumericalJacobianU<SimplifiedDynamics<double>::kNumPositionStates,
                         kNumInputs, double>(
          functor, SimplifiedDynamics<double>::PositionState<>::Zero(),
          SimplifiedDynamics<double>::Input<>::Zero());
    }
    auto duration = aos::monotonic_clock::now() - start_time;
    LOG(INFO) << "Average time for " << kIterations << " iterations of "
              << " numerical (double-precision) dynamics linearization: "
              << aos::time::DurationInSeconds(duration / kIterations) << " sec";
  }

  LOG(INFO) << "Running dynamics inversion:";

  const auto frozen_states = LinearVelocityController::MakeGoal(
      LinearVelocityController::Goal{1.0, 1.0, 1.0});
  const auto goal = LinearVelocityController::MakeGoal(
      LinearVelocityController::Goal{1.0, 1.0, 1.0});
  const auto Q = LinearVelocityController::MakeInverseQ();
  const auto R = LinearVelocityController::MakeInverseR();
  {
    AutoInverseDynamics auto_dynamics(
        SimplifiedDynamics<double>(MakeParams<double>()),
        frozen_states.cast<double>(), Q.cast<double>(), R.cast<double>());
    auto_dynamics.set_max_solver_iterations(
        absl::GetFlag(FLAGS_max_inverse_dynamics_solver_iterations));
    auto start_time = aos::monotonic_clock::now();
    for (size_t i = 0; i < kIterations; ++i) {
      auto_dynamics.InvertDynamics(goal.cast<double>());
    }
    auto duration = aos::monotonic_clock::now() - start_time;
    LOG(INFO) << "Average time for " << kIterations
              << " iterations of auto diff solver: "
              << aos::time::DurationInSeconds(duration / kIterations) << " sec";
  }

  {
    NumericalInverseSimplifiedDynamics<double> dynamics(
        std::make_unique<SimplifiedDynamicsNumerical<double>>(
            MakeParams<double>()),
        frozen_states.cast<double>(), Q.cast<double>(), R.cast<double>());
    auto start_time = aos::monotonic_clock::now();
    for (size_t i = 0; i < kIterations; ++i) {
      dynamics.InvertDynamics(goal.cast<double>());
    }
    auto duration = aos::monotonic_clock::now() - start_time;
    LOG(INFO) << "Average time for " << kIterations
              << " iterations of double precision numerical solver: "
              << aos::time::DurationInSeconds(duration / kIterations) << " sec";
  }
  return EXIT_SUCCESS;
}
}  // namespace frc::control_loops::swerve
int main(int argc, char *argv[]) {
  aos::InitGoogle(&argc, &argv);
  return frc::control_loops::swerve::Main();
}

// NOTES:
// clang-format off
// On personal desktop:
// I1005 17:26:52.416648 1744410 inverse_dynamics_benchmark.cc:99] Running dynamics linearization:
// I1005 17:26:52.416715 1744410 inverse_dynamics_benchmark.cc:92] Average time for 10 iterations of single precision dynamics linearization: 5.379e-06 sec
// I1005 17:26:52.416830 1744410 inverse_dynamics_benchmark.cc:92] Average time for 10 iterations of double precision dynamics linearization: 9.219e-06 sec
// I1005 17:26:52.416835 1744410 inverse_dynamics_benchmark.cc:104] Running dynamics inversion:
// I1005 17:26:52.445371 1744410 inverse_dynamics_benchmark.cc:121] Average time for 10 iterations of auto diff solver: 0.00285309 sec
// I1005 17:26:53.191648 1744410 inverse_dynamics_benchmark.cc:136] Average time for 10 iterations of double precision numerical solver: 0.0746266 sec
// On rio:
// admin@roboRIO-9971-FRC:~# ./inverse_dynamics_benchmark.stripped
// I0630 13:18:19.887538    2085 inverse_dynamics_benchmark.cc:99] Running dynamics linearization:
// I0630 13:18:19.924157    2085 inverse_dynamics_benchmark.cc:92] Average time for 100 iterations of single precision dynamics linearization: 0.000358444 sec
// I0630 13:18:19.987610    2085 inverse_dynamics_benchmark.cc:92] Average time for 100 iterations of double precision dynamics linearization: 0.000621202 sec
// I0630 13:18:19.987818    2085 inverse_dynamics_benchmark.cc:104] Running dynamics inversion:
// I0630 13:18:38.899835    2085 inverse_dynamics_benchmark.cc:121] Average time for 100 iterations of auto diff solver: 0.189111 sec
// ^C
// admin@roboRIO-9971-FRC:~# ./inverse_dynamics_benchmark.stripped --iterations 10
// I0630 13:19:20.096624    2105 inverse_dynamics_benchmark.cc:99] Running dynamics linearization:
// I0630 13:19:20.101340    2105 inverse_dynamics_benchmark.cc:92] Average time for 10 iterations of single precision dynamics linearization: 0.000450178 sec
// I0630 13:19:20.111757    2105 inverse_dynamics_benchmark.cc:92] Average time for 10 iterations of double precision dynamics linearization: 0.00098299 sec
// I0630 13:19:20.111993    2105 inverse_dynamics_benchmark.cc:104] Running dynamics inversion:
// I0630 13:19:22.012306    2105 inverse_dynamics_benchmark.cc:121] Average time for 10 iterations of auto diff solver: 0.189961 sec
// ^C
// admin@roboRIO-9971-FRC:~# ./inverse_dynamics_benchmark.stripped --iterations 1
// I0630 13:19:27.782587    2108 inverse_dynamics_benchmark.cc:99] Running dynamics linearization:
// I0630 13:19:27.783533    2108 inverse_dynamics_benchmark.cc:92] Average time for 1 iterations of single precision dynamics linearization: 0.000353148 sec
// I0630 13:19:27.785668    2108 inverse_dynamics_benchmark.cc:92] Average time for 1 iterations of double precision dynamics linearization: 0.00063115 sec
// I0630 13:19:27.785827    2108 inverse_dynamics_benchmark.cc:104] Running dynamics inversion:
// I0630 13:19:27.968108    2108 inverse_dynamics_benchmark.cc:121] Average time for 1 iterations of auto diff solver: 0.181671 sec
// I0630 13:19:30.615376    2108 inverse_dynamics_benchmark.cc:136] Average time for 1 iterations of double precision numerical solver: 2.64664 sec
// admin@roboRIO-9971-FRC:~# ./inverse_dynamics_benchmark.stripped --iterations 10
// I0630 13:19:32.573435    2111 inverse_dynamics_benchmark.cc:99] Running dynamics linearization:
// I0630 13:19:32.577350    2111 inverse_dynamics_benchmark.cc:92] Average time for 10 iterations of single precision dynamics linearization: 0.000365176 sec
// I0630 13:19:32.584215    2111 inverse_dynamics_benchmark.cc:92] Average time for 10 iterations of double precision dynamics linearization: 0.000604903 sec
// I0630 13:19:32.584446    2111 inverse_dynamics_benchmark.cc:104] Running dynamics inversion:
// I0630 13:19:34.464060    2111 inverse_dynamics_benchmark.cc:121] Average time for 10 iterations of auto diff solver: 0.187949 sec
// I0630 13:20:01.695265    2111 inverse_dynamics_benchmark.cc:136] Average time for 10 iterations of double precision numerical solver: 2.72305 sec
//
// admin@roboRIO-9971-FRC:~# ./inverse_dynamics_benchmark.stripped --iterations 100
// I0630 13:17:05.253287    1994 inverse_dynamics_benchmark.cc:101] Running dynamics linearization:
// I0630 13:17:05.291527    1994 inverse_dynamics_benchmark.cc:92] Average time for 100 iterations of single precision dynamics linearization: 0.000378978 sec
// I0630 13:17:05.349662    1994 inverse_dynamics_benchmark.cc:92] Average time for 100 iterations of double precision dynamics linearization: 0.000577637 sec
// I0630 13:17:05.523138    1994 inverse_dynamics_benchmark.cc:127] Average time for 100 iterations of  numerical (double-precision) dynamics linearization: 0.00173181 sec
//
//
// clang-format on
