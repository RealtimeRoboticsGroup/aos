#include "frc/control_loops/double_jointed_arm/ekf.h"

#include <iostream>

#include "Eigen/Dense"
#include "absl/flags/flag.h"

#include "frc/control_loops/double_jointed_arm/dynamics.h"
#include "frc/control_loops/jacobian.h"

ABSL_FLAG(double, proximal_voltage_error_uncertainty, 8.0,
          "Proximal joint voltage error uncertainty.");
ABSL_FLAG(double, distal_voltage_error_uncertainty, 2.0,
          "Distal joint voltage error uncertainty.");

namespace frc::control_loops::arm {

namespace {
// TODO(austin): When tuning this, make sure to verify that you are waiting
// enough cycles to make sure it converges at startup. Otherwise you will have a
// bad day.
::Eigen::Matrix<double, 6, 6> Q_covariance(
    (::Eigen::DiagonalMatrix<double, 6>().diagonal() << ::std::pow(0.1, 2),
     ::std::pow(2.0, 2), ::std::pow(0.1, 2), ::std::pow(2.0, 2),
     ::std::pow(absl::GetFlag(FLAGS_proximal_voltage_error_uncertainty), 2),
     ::std::pow(absl::GetFlag(FLAGS_distal_voltage_error_uncertainty), 2))
        .finished()
        .asDiagonal());
}  // namespace

EKF::EKF(const Dynamics *dynamics) : dynamics_(dynamics) {
  X_hat_.setZero();
  Q_covariance =
      ((::Eigen::DiagonalMatrix<double, 6>().diagonal() << ::std::pow(0.1, 2),
        ::std::pow(2.0, 2), ::std::pow(0.1, 2), ::std::pow(2.0, 2),
        ::std::pow(absl::GetFlag(FLAGS_proximal_voltage_error_uncertainty), 2),
        ::std::pow(absl::GetFlag(FLAGS_distal_voltage_error_uncertainty), 2))
           .finished()
           .asDiagonal());
  P_ = Q_covariance;
  P_reset_ = P_;
  //::std::cout << "Reset P: " << P_ << ::std::endl;
  // TODO(austin): Running the EKF 2000 cycles works, but isn't super clever.
  // We could just solve the DARE.

  for (int i = 0; i < 1000; ++i) {
    Predict(::Eigen::Matrix<double, 2, 1>::Zero(), 0.00505);
    Correct(::Eigen::Matrix<double, 2, 1>::Zero(), 0.00505);
  }
  P_half_converged_ = P_;
  //::std::cout << "Stabilized P: " << P_ << ::std::endl;
  for (int i = 0; i < 1000; ++i) {
    Predict(::Eigen::Matrix<double, 2, 1>::Zero(), 0.00505);
    Correct(::Eigen::Matrix<double, 2, 1>::Zero(), 0.00505);
  }
  //::std::cout << "Really stabilized P: " << P_ << ::std::endl;
  P_converged_ = P_;

  Reset(::Eigen::Matrix<double, 4, 1>::Zero());
}

void EKF::Reset(const ::Eigen::Matrix<double, 4, 1> &X) {
  X_hat_.setZero();
  P_ = P_converged_;
  X_hat_.block<4, 1>(0, 0) = X;
}

void EKF::Predict(const ::Eigen::Matrix<double, 2, 1> &U, double dt) {
  const ::Eigen::Matrix<double, 6, 6> A =
      ::frc::control_loops::NumericalJacobianX<6, 2>(
          [this](const auto &X_hat_, const auto &U, double dt) {
            return dynamics_->UnboundedEKFDiscreteDynamics(X_hat_, U, dt);
          },
          X_hat_, U, dt);

  X_hat_ = dynamics_->UnboundedEKFDiscreteDynamics(X_hat_, U, dt);
  P_ = A * P_ * A.transpose() + Q_covariance;
}

void EKF::Correct(const ::Eigen::Matrix<double, 2, 1> &Y, double /*dt*/) {
  const ::Eigen::Matrix<double, 2, 2> R_covariance(
      (::Eigen::DiagonalMatrix<double, 2>().diagonal() << ::std::pow(0.01, 2),
       ::std::pow(0.01, 2))
          .finished()
          .asDiagonal());
  // H is the jacobian of the h(x) measurement prediction function
  const ::Eigen::Matrix<double, 2, 6> H_jacobian =
      (::Eigen::Matrix<double, 2, 6>() << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 1.0, 0.0, 0.0, 0.0)
          .finished();

  // Update step Measurement residual error of proximal and distal joint
  // angles.
  const ::Eigen::Matrix<double, 2, 1> Y_hat =
      Y - (::Eigen::Matrix<double, 2, 1>() << X_hat_(0), X_hat_(2)).finished();
  // Residual covariance
  const ::Eigen::Matrix<double, 2, 2> S =
      H_jacobian * P_ * H_jacobian.transpose() + R_covariance;

  // K is the Near-optimal Kalman gain
  const ::Eigen::Matrix<double, 6, 2> kalman_gain =
      P_ * H_jacobian.transpose() * S.inverse();
  // Updated state estimate
  X_hat_ = X_hat_ + kalman_gain * Y_hat;
  // Updated covariance estimate
  P_ = (::Eigen::Matrix<double, 6, 6>::Identity() - kalman_gain * H_jacobian) *
       P_;
}

}  // namespace frc::control_loops::arm
