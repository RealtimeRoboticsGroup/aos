#ifndef FRC_CONTROL_LOOPS_JACOBIAN_H_
#define FRC_CONTROL_LOOPS_JACOBIAN_H_

#include <Eigen/Dense>

namespace frc::control_loops {

template <int num_states, int num_inputs, typename Scalar, typename F>
::Eigen::Matrix<Scalar, num_states, num_inputs> NumericalJacobian(
    const F &fn, ::Eigen::Matrix<Scalar, num_inputs, 1> input) {
  constexpr Scalar kEpsilon = 1e-5;
  ::Eigen::Matrix<Scalar, num_states, num_inputs> result =
      ::Eigen::Matrix<Scalar, num_states, num_inputs>::Zero();

  // It's more expensive, but +- epsilon will be more accurate
  for (int i = 0; i < num_inputs; ++i) {
    ::Eigen::Matrix<Scalar, num_inputs, 1> dX_plus = input;
    dX_plus(i, 0) += kEpsilon;
    ::Eigen::Matrix<Scalar, num_inputs, 1> dX_minus = input;
    dX_minus(i, 0) -= kEpsilon;
    result.col(i) = (fn(dX_plus) - fn(dX_minus)) / (kEpsilon * 2.0);
  }
  return result;
}

// Implements a numerical jacobian with respect to X for f(X, U, ...).
template <int num_states, int num_u, typename Scalar, typename F,
          typename... Args>
::Eigen::Matrix<Scalar, num_states, num_states> NumericalJacobianX(
    const F &fn, ::Eigen::Matrix<Scalar, num_states, 1> X,
    ::Eigen::Matrix<Scalar, num_u, 1> U, Args &&...args) {
  return NumericalJacobian<num_states, num_states>(
      [&](::Eigen::Matrix<Scalar, num_states, 1> X) {
        return fn(X, U, args...);
      },
      X);
}

// Implements a numerical jacobian with respect to U for f(X, U, ...).
template <int num_states, int num_u, typename Scalar, typename F,
          typename... Args>
::Eigen::Matrix<Scalar, num_states, num_u> NumericalJacobianU(
    const F &fn, ::Eigen::Matrix<Scalar, num_states, 1> X,
    ::Eigen::Matrix<Scalar, num_u, 1> U, Args &&...args) {
  return NumericalJacobian<num_states, num_u>(
      [&](::Eigen::Matrix<Scalar, num_u, 1> U) { return fn(X, U, args...); },
      U);
}

}  // namespace frc::control_loops

#endif  // FRC_CONTROL_LOOPS_JACOBIAN_H_
