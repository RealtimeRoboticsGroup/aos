#include "frc/control_loops/double_jointed_arm/dynamics.h"

#include "gtest/gtest.h"

#include "frc/control_loops/double_jointed_arm/test_constants.h"

namespace frc::control_loops::arm::testing {

// Tests that zero inputs result in no acceleration and no motion.
// This isn't all that rigerous, but it's a good start.
TEST(DynamicsTest, Acceleration) {
  Dynamics dynamics(kArmConstants);

  EXPECT_TRUE(dynamics
                  .Acceleration(::Eigen::Matrix<double, 4, 1>::Zero(),
                                ::Eigen::Matrix<double, 2, 1>::Zero())
                  .isApprox(::Eigen::Matrix<double, 4, 1>::Zero()));

  EXPECT_TRUE(
      dynamics
          .UnboundedDiscreteDynamics(::Eigen::Matrix<double, 4, 1>::Zero(),
                                     ::Eigen::Matrix<double, 2, 1>::Zero(), 0.1)
          .isApprox(::Eigen::Matrix<double, 4, 1>::Zero()));

  const ::Eigen::Matrix<double, 4, 1> X =
      (::Eigen::Matrix<double, 4, 1>() << M_PI / 2.0, 0.0, 0.0, 0.0).finished();

  ::std::cout << dynamics.FF_U(X, ::Eigen::Matrix<double, 2, 1>::Zero(),
                               ::Eigen::Matrix<double, 2, 1>::Zero())
              << ::std::endl;

  ::std::cout << dynamics.UnboundedDiscreteDynamics(
                     X,
                     dynamics.FF_U(X, ::Eigen::Matrix<double, 2, 1>::Zero(),
                                   ::Eigen::Matrix<double, 2, 1>::Zero()),
                     0.01)
              << ::std::endl;

  EXPECT_TRUE(dynamics
                  .UnboundedDiscreteDynamics(
                      X,
                      dynamics.FF_U(X, ::Eigen::Matrix<double, 2, 1>::Zero(),
                                    ::Eigen::Matrix<double, 2, 1>::Zero()),
                      0.01)
                  .isApprox(X));
}

}  // namespace frc::control_loops::arm::testing
