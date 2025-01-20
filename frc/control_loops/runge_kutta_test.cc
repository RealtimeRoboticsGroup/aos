#include "frc/control_loops/runge_kutta.h"

#include "gtest/gtest.h"

namespace frc::control_loops::testing {

// Tests that integrating dx/dt = e^x works.
TEST(RungeKuttaTest, Exponential) {
  ::Eigen::Matrix<double, 1, 1> y0;
  y0(0, 0) = 1.0;

  ::Eigen::Matrix<double, 1, 1> y1 = RungeKutta(
      [](::Eigen::Matrix<double, 1, 1> x) {
        ::Eigen::Matrix<double, 1, 1> y;
        y(0, 0) = ::std::exp(x(0, 0));
        return y;
      },
      y0, 0.1);
  EXPECT_NEAR(y1(0, 0), -std::log(std::exp(-1.0) - 0.1), 1e-5);
}

// Now do it with sub steps.
TEST(RungeKuttaTest, ExponentialSteps) {
  ::Eigen::Matrix<double, 1, 1> y0;
  y0(0, 0) = 1.0;

  ::Eigen::Matrix<double, 1, 1> y1 = RungeKuttaSteps(
      [](::Eigen::Matrix<double, 1, 1> x) {
        ::Eigen::Matrix<double, 1, 1> y;
        y(0, 0) = ::std::exp(x(0, 0));
        return y;
      },
      y0, 0.1, 10);
  EXPECT_NEAR(y1(0, 0), -std::log(std::exp(-1.0) - 0.1), 1e-8);
}

// Tests that integrating dx/dt = e^x works when we provide a U.
TEST(RungeKuttaTest, ExponentialWithU) {
  ::Eigen::Matrix<double, 1, 1> y0;
  y0(0, 0) = 0.0;

  ::Eigen::Matrix<double, 1, 1> y1 = RungeKuttaU(
      [](::Eigen::Matrix<double, 1, 1> x, ::Eigen::Matrix<double, 1, 1> u) {
        ::Eigen::Matrix<double, 1, 1> y;
        y(0, 0) = ::std::exp(u(0, 0) * x(0, 0));
        return y;
      },
      y0, (::Eigen::Matrix<double, 1, 1>() << 1.0).finished(), 0.1);
  EXPECT_NEAR(y1(0, 0), ::std::exp(0.1) - ::std::exp(0), 1e-3);
}

::Eigen::Matrix<double, 1, 1> RungeKuttaTimeVaryingSolution(double t) {
  return (::Eigen::Matrix<double, 1, 1>()
          << 12.0 * ::std::exp(t) / (::std::pow(::std::exp(t) + 1.0, 2.0)))
      .finished();
}

// Tests RungeKutta with a time varying solution.
// Now, lets test RK4 with a time varying solution.  From
// http://www2.hawaii.edu/~jmcfatri/math407/RungeKuttaTest.html:
//   x' = x (2 / (e^t + 1) - 1)
//
// The true (analytical) solution is:
//
// x(t) = 12 * e^t / ((e^t + 1)^2)
TEST(RungeKuttaTest, RungeKuttaTimeVarying) {
  ::Eigen::Matrix<double, 1, 1> y0 = RungeKuttaTimeVaryingSolution(5.0);

  ::Eigen::Matrix<double, 1, 1> y1 = RungeKutta(
      [](double t, ::Eigen::Matrix<double, 1, 1> x) {
        return (::Eigen::Matrix<double, 1, 1>()
                << x(0, 0) * (2.0 / (::std::exp(t) + 1.0) - 1.0))
            .finished();
      },
      y0, 5.0, 1.0);
  EXPECT_NEAR(y1(0, 0), RungeKuttaTimeVaryingSolution(6.0)(0, 0), 1e-3);
}

// Now do it with a ton of sub steps.
TEST(RungeKuttaTest, RungeKuttaTimeVaryingSteps) {
  ::Eigen::Matrix<double, 1, 1> y0 = RungeKuttaTimeVaryingSolution(5.0);

  ::Eigen::Matrix<double, 1, 1> y1 = RungeKuttaSteps(
      [](double t, ::Eigen::Matrix<double, 1, 1> x) {
        return (::Eigen::Matrix<double, 1, 1>()
                << x(0, 0) * (2.0 / (::std::exp(t) + 1.0) - 1.0))
            .finished();
      },
      y0, 5.0, 1.0, 10);
  EXPECT_NEAR(y1(0, 0), RungeKuttaTimeVaryingSolution(6.0)(0, 0), 1e-7);
}

// Tests that integrating dx/dt = e^x works with RK45
TEST(RungeKuttaTest, RungeKuttaTimeVaryingAdaptive) {
  ::Eigen::Matrix<double, 1, 1> y0 = RungeKuttaTimeVaryingSolution(5.0);

  size_t count = 0;

  ::Eigen::Matrix<double, 1, 1> y1 = AdaptiveRungeKutta(
      [&](double t, ::Eigen::Matrix<double, 1, 1> x) {
        ++count;
        return (::Eigen::Matrix<double, 1, 1>()
                << x(0, 0) * (2.0 / (::std::exp(t) + 1.0) - 1.0))
            .finished();
      },
      y0, 5.0, 1.0, 1e-6, 1e-9);
  EXPECT_NEAR(y1(0, 0), RungeKuttaTimeVaryingSolution(6.0)(0, 0), 1e-7);

  // Using sympy as a benchmark, we should expect to see the function called 38
  // times.
  EXPECT_EQ(count, 38);

  LOG(INFO) << "Got " << y1(0, 0) << " vs expected "
            << RungeKuttaTimeVaryingSolution(6.0)(0, 0);
}

}  // namespace frc::control_loops::testing
