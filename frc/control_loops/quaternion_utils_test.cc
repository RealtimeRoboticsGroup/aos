#include "frc/control_loops/quaternion_utils.h"

#include <random>

#include "Eigen/Dense"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "gtest/gtest.h"

#include "aos/testing/random_seed.h"
#include "frc/control_loops/jacobian.h"
#include "frc/control_loops/runge_kutta.h"

namespace frc::controls::testing {

// Tests that small perturbations around a couple quaternions averaged out
// return the original quaternion.
TEST(DownEstimatorTest, QuaternionMean) {
  Eigen::Matrix<double, 4, 7> vectors;
  std::vector<Eigen::Vector4d> quaternion_list;
  vectors.col(0) << 0, 0, 0, 1;
  for (int i = 0; i < 3; ++i) {
    Eigen::Matrix<double, 4, 1> perturbation;
    perturbation.setZero();
    perturbation(i, 0) = 0.1;

    vectors.col(i * 2 + 1) = vectors.col(0) + perturbation;
    vectors.col(i * 2 + 2) = vectors.col(0) - perturbation;
  }

  for (int i = 0; i < 7; ++i) {
    vectors.col(i).normalize();
    quaternion_list.push_back(Eigen::Vector4d(vectors.col(i)));
  }

  Eigen::Matrix<double, 4, 1> mean = QuaternionMean(vectors);

  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(mean(i, 0), vectors(i, 0), 0.001) << ": Failed on index " << i;
  }

  // Test version of code that passes in a vector of quaternions
  mean = QuaternionMean(quaternion_list);

  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(mean(i, 0), quaternion_list[0](i, 0), 0.001)
        << ": Failed on index " << i << " with mean = " << mean
        << " and quat_list = " << quaternion_list[0];
  }
}

// Tests that ToRotationVectorFromQuaternion works for a 0 rotation.
TEST(DownEstimatorTest, ToRotationVectorFromQuaternionAtZero) {
  Eigen::Matrix<double, 3, 1> vector = ToRotationVectorFromQuaternion(
      Eigen::Quaternion<double>(
          Eigen::AngleAxis<double>(0.0, Eigen::Vector3d::UnitX()))
          .coeffs());

  EXPECT_NEAR(0.0, (vector - Eigen::Vector3d::Zero()).norm(), 1e-4);
}

// Tests that ToRotationVectorFromQuaternion works for a real rotation.
TEST(DownEstimatorTest, ToRotationVectorFromQuaternion) {
  Eigen::Matrix<double, 3, 1> vector = ToRotationVectorFromQuaternion(
      Eigen::Quaternion<double>(
          Eigen::AngleAxis<double>(M_PI * 0.5, Eigen::Vector3d::UnitX()))
          .coeffs());

  EXPECT_NEAR(0.0, (vector - Eigen::Vector3d::UnitX() * M_PI * 0.5).norm(),
              1e-4);
}

// Tests that ToRotationVectorFromQuaternion works for a solution with negative
// coefficients.
TEST(DownEstimatorTest, ToRotationVectorFromQuaternionNegative) {
  Eigen::Matrix<double, 3, 1> vector = ToRotationVectorFromQuaternion(
      Eigen::Quaternion<double>(
          -Eigen::Quaternion<double>(
               Eigen::AngleAxis<double>(M_PI * 0.5, Eigen::Vector3d::UnitX()))
               .coeffs())
          .coeffs());

  EXPECT_NEAR(0.0, (vector - Eigen::Vector3d::UnitX() * M_PI * 0.5).norm(),
              1e-4);
}

// Tests that ToQuaternionFromRotationVector works for a 0 rotation.
TEST(DownEstimatorTest, ToQuaternionFromRotationVectorAtZero) {
  Eigen::Matrix<double, 4, 1> quaternion =
      ToQuaternionFromRotationVector(Eigen::Vector3d::Zero());

  EXPECT_NEAR(
      0.0,
      (quaternion - Eigen::Quaternion<double>(
                        Eigen::AngleAxis<double>(0.0, Eigen::Vector3d::UnitX()))
                        .coeffs())
          .norm(),
      1e-4);
}

// Tests that ToQuaternionFromRotationVector works for a real rotation.
TEST(DownEstimatorTest, ToQuaternionFromRotationVector) {
  Eigen::Matrix<double, 4, 1> quaternion =
      ToQuaternionFromRotationVector(Eigen::Vector3d::UnitX() * M_PI * 0.5);

  EXPECT_NEAR(0.0,
              (quaternion - Eigen::Quaternion<double>(
                                Eigen::AngleAxis<double>(
                                    M_PI * 0.5, Eigen::Vector3d::UnitX()))
                                .coeffs())

                  .norm(),
              1e-4);
}

// Tests that ToQuaternionFromRotationVector correctly clips a rotation vector
// that is too large in magnitude.
TEST(DownEstimatorTest, ToQuaternionFromLargeRotationVector) {
  const double kMaxAngle = 2.0;
  const Eigen::Vector3d rotation_vector =
      Eigen::Vector3d::UnitX() * kMaxAngle * 2.0;
  const Eigen::Matrix<double, 3, 1> clipped_vector =
      ToRotationVectorFromQuaternion(
          ToQuaternionFromRotationVector(rotation_vector, kMaxAngle));

  EXPECT_NEAR(0.0, (rotation_vector / 2.0 - clipped_vector).norm(), 1e-4);
}

// Tests that ToQuaternionFromRotationVector and ToRotationVectorFromQuaternion
// works for random rotations.
TEST(DownEstimatorTest, RandomQuaternions) {
  std::mt19937 generator(aos::testing::RandomSeed());
  std::uniform_real_distribution<double> random_scalar(-1.0, 1.0);

  for (int i = 0; i < 1000; ++i) {
    Eigen::Matrix<double, 3, 1> axis;
    axis << random_scalar(generator), random_scalar(generator),
        random_scalar(generator);
    EXPECT_GE(axis.norm(), 1e-6);
    axis.normalize();

    const double angle = random_scalar(generator) * M_PI;

    Eigen::Matrix<double, 4, 1> quaternion =
        ToQuaternionFromRotationVector(axis * angle);

    Eigen::Quaternion<double> answer(Eigen::AngleAxis<double>(angle, axis));

    EXPECT_NEAR(quaternion(3, 0), std::cos(angle / 2.0), 1e-8);
    EXPECT_NEAR(answer.w(), std::cos(angle / 2.0), 1e-8);

    EXPECT_NEAR(1.0, (answer.coeffs() * quaternion.transpose()).norm(), 1e-6);

    const Eigen::Matrix<double, 3, 1> recalculated_axis =
        ToRotationVectorFromQuaternion(quaternion);

    EXPECT_NEAR(std::abs(angle), recalculated_axis.norm(), 1e-8);

    EXPECT_NEAR(0.0, (axis * angle - recalculated_axis).norm(), 1e-8);
  }
}

// Do a known transformation to see if quaternion integration is working
// correctly.
TEST(DownEstimatorTest, QuaternionIntegral) {
  Eigen::Vector3d ux = Eigen::Vector3d::UnitX();
  Eigen::Vector3d uy = Eigen::Vector3d::UnitY();
  Eigen::Vector3d uz = Eigen::Vector3d::UnitZ();

  Eigen::Quaternion<double> q(
      Eigen::AngleAxis<double>(0.5 * M_PI, Eigen::Vector3d::UnitY()));

  Eigen::Quaternion<double> q0(
      Eigen::AngleAxis<double>(0, Eigen::Vector3d::UnitY()));

  auto qux = q * ux;

  VLOG(1) << "Q is w: " << q.w() << " vec: " << q.vec();
  VLOG(1) << "ux is " << ux;
  VLOG(1) << "qux is " << qux;

  // Start by rotating around the X body vector for pi/2
  Eigen::Quaternion<double> integral1(control_loops::RungeKutta(
      std::bind(&QuaternionDerivative, ux, std::placeholders::_1), q0.coeffs(),
      0.5 * M_PI));

  VLOG(1) << "integral1 * uz => " << integral1 * uz;

  // Then rotate around the Y body vector for pi/2
  Eigen::Quaternion<double> integral2(control_loops::RungeKutta(
      std::bind(&QuaternionDerivative, uy, std::placeholders::_1),
      integral1.normalized().coeffs(), 0.5 * M_PI));

  VLOG(1) << "integral2 * uz => " << integral2 * uz;

  // Then rotate around the X body vector for -pi/2
  Eigen::Quaternion<double> integral3(control_loops::RungeKutta(
      std::bind(&QuaternionDerivative, -ux, std::placeholders::_1),
      integral2.normalized().coeffs(), 0.5 * M_PI));

  integral1.normalize();
  integral2.normalize();
  integral3.normalize();

  VLOG(1) << "Integral is w: " << integral1.w() << " vec: " << integral1.vec()
          << " norm " << integral1.norm();

  VLOG(1) << "Integral is w: " << integral3.w() << " vec: " << integral3.vec()
          << " norm " << integral3.norm();

  VLOG(1) << "ux => " << integral3 * ux;
  EXPECT_NEAR(0.0, (ux - integral1 * ux).norm(), 5e-2);
  EXPECT_NEAR(0.0, (uz - integral1 * uy).norm(), 5e-2);
  EXPECT_NEAR(0.0, (-uy - integral1 * uz).norm(), 5e-2);

  EXPECT_NEAR(0.0, (uy - integral2 * ux).norm(), 5e-2);
  EXPECT_NEAR(0.0, (uz - integral2 * uy).norm(), 5e-2);
  EXPECT_NEAR(0.0, (ux - integral2 * uz).norm(), 5e-2);

  EXPECT_NEAR(0.0, (uy - integral3 * ux).norm(), 5e-2);
  EXPECT_NEAR(0.0, (-ux - integral3 * uy).norm(), 5e-2);
  EXPECT_NEAR(0.0, (uz - integral3 * uz).norm(), 5e-2);
}

}  // namespace frc::controls::testing
