#include "frc971/control_loops/swerve/linear_velocity_controller.h"

#include "gtest/gtest.h"

#include "aos/realtime.h"

namespace frc971::control_loops::swerve::test {
class LinearVelocityControllerTest : public ::testing::Test {
 protected:
  typedef LinearVelocityController::States States;
  typedef LinearVelocityController::Inputs Inputs;
  typedef LinearVelocityController::State State;
  typedef LinearVelocityController::Input Input;
  LinearVelocityControllerTest() {}
  LinearVelocityController controller_;
};

// Checks that we output zero currents when our goal velocities are zero and we
// are at zero.
TEST_F(LinearVelocityControllerTest, ZeroAtZero) {
  auto result =
      controller_.RunController(State::Zero(), {.vx = 0, .vy = 0, .omega = 0});
  EXPECT_EQ(State::Zero(), result.debug.goal);
}

// Check that driving in a straight line results in reasonable dynamics.
TEST_F(LinearVelocityControllerTest, StraightLine) {
  auto result = controller_.RunController(State::Zero(),
                                          {.vx = 1.0, .vy = 0, .omega = 0});
  EXPECT_EQ(1.0, result.debug.goal(States::kVx));
  result.debug.goal(States::kVx) = 0.0;
  EXPECT_EQ(State::Zero(), result.debug.goal);
  EXPECT_EQ(Input::Zero(), result.U);
  EXPECT_EQ(Input::Zero(), result.debug.U_ff);
  // Feedback should definitely be positive in practice?
  EXPECT_EQ(Input::Zero(), result.debug.U_feedback);

  {
    aos::ScopedRealtime realtime;
    result = controller_.RunController(result.debug.goal,
                                       {.vx = 1.0, .vy = 0, .omega = 0});
  }
  EXPECT_EQ(1.0, result.debug.goal(States::kVx));
  result.debug.goal(States::kVx) = 0.0;
  EXPECT_EQ(State::Zero(), result.debug.goal);
  EXPECT_EQ(Input::Zero(), result.U);
  EXPECT_EQ(Input::Zero(), result.debug.U_ff);
  EXPECT_EQ(Input::Zero(), result.debug.U_feedback);
}

// Check that attempting to spin in place results in reasonable dynamics.
// TODO(james): We need to make the dynamics inversion also minimize the control
// inputs.
TEST_F(LinearVelocityControllerTest, SpinInPlace) {
  auto result = controller_.RunController(State::Zero(),
                                          {.vx = 0.0, .vy = 0, .omega = 1});
  State expected = State::Zero();
  // Technically it is permissible to end up with every single wheel at either
  // the specified value or +/- PI from the specified value. In the future we
  // may want to have something that biases us towards the current values of X.
  expected(States::kThetas0) = -M_PI_4;
  expected(States::kThetas1) = M_PI_4;
  expected(States::kThetas2) = -M_PI_4;
  expected(States::kThetas3) = M_PI_4;
  expected(States::kOmega) = 1.0;
  EXPECT_LT((expected - result.debug.goal).norm(), 5e-3)
      << "Expected:\n"
      << expected.transpose() << "\ngot:\n"
      << result.debug.goal.transpose();
  EXPECT_EQ(result.U, result.debug.U_ff + result.debug.U_feedback);
  EXPECT_LT(result.debug.U_ff.norm(), 1e-4) << result.debug.U_ff.transpose();

  const double nominal_drive_current = std::abs(result.U(Inputs::kId0));
  // All drive motors should have ~the same current; however, some of them are
  // driving backwards and so need signs flipped.
  for (std::pair<int, double> module : std::vector<std::pair<int, double>>{
           {0, -1.0}, {1, -1.0}, {2, 1.0}, {3, 1.0}}) {
    EXPECT_NEAR(0.0, result.U(Inputs::kIs0 + 2 * module.first), 1e-2);
    EXPECT_NEAR(nominal_drive_current * module.second,
                result.U(Inputs::kId0 + 2 * module.first), 1e-6);
  }

  {
    aos::ScopedRealtime realtime;
    result = controller_.RunController(result.debug.goal,
                                       {.vx = 0.0, .vy = 0, .omega = 1});
  }
  EXPECT_LT((expected - result.debug.goal).norm(), 5e-3)
      << "Expected:\n"
      << expected.transpose() << "\ngot:\n"
      << result.debug.goal.transpose();
  EXPECT_LT(result.U.norm(), 1e-4) << result.U.transpose();
  EXPECT_LT(result.debug.U_ff.norm(), 1e-4) << result.debug.U_ff.transpose();
  EXPECT_EQ(Input::Zero(), result.debug.U_feedback);
}

}  // namespace frc971::control_loops::swerve::test
