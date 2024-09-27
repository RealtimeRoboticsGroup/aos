#include "frc971/control_loops/swerve/inverse_dynamics.h"

#include "gtest/gtest.h"

#include "aos/realtime.h"

namespace frc971::control_loops::swerve::test {

using SimpleInverseDynamics =
    NumericalInverseDynamics<double, Eigen::Vector2d,
                             Eigen::Matrix<double, 1, 1>>;
class SimpleDynamics : public SimpleInverseDynamics::Dynamics {
 public:
  virtual ~SimpleDynamics() {}
  Eigen::Vector2d operator()(
      const Eigen::Vector2d &X,
      const Eigen::Matrix<double, 1, 1> &U) const override {
    return Eigen::Matrix2d{{0.0, 1.0}, {0.0, -0.1}} * X +
           Eigen::Vector2d{{0.0}, {1.0}} * U;
  }
};

class SimpleDynamicsTemplated {
 public:
  template <typename Scalar>
  Eigen::Matrix<Scalar, 2, 1> operator()(
      const Eigen::Matrix<Scalar, 2, 1> &X,
      const Eigen::Matrix<Scalar, 1, 1> &U) const {
    return Eigen::Matrix2d{{0.0, 1.0}, {0.0, -0.1}} * X +
           Eigen::Vector2d{{0.0}, {1.0}} * U;
  }
};

// Set up some dynamics whose behavior is underconstrained so that we can test
// the effects of the cost functions on the inverse dynamics.
class UnderconstrainedDynamics : public SimpleInverseDynamics::Dynamics {
 public:
  virtual ~UnderconstrainedDynamics() {}
  Eigen::Vector2d operator()(
      const Eigen::Vector2d &X,
      const Eigen::Matrix<double, 1, 1> &U) const override {
    return Eigen::Matrix2d{{0.0, 0.0}, {1.0, 1.0}} * X +
           Eigen::Vector2d{{0.0}, {1.0}} * U;
  }
};

class UnderconstrainedDynamicsTemplated {
 public:
  template <typename Scalar>
  Eigen::Matrix<Scalar, 2, 1> operator()(
      const Eigen::Matrix<Scalar, 2, 1> &X,
      const Eigen::Matrix<Scalar, 1, 1> &U) const {
    return Eigen::Matrix2d{{0.0, 0.0}, {1.0, 1.0}} * X +
           Eigen::Vector2d{{0.0}, {1.0}} * U;
  }
};
template <typename Dynamics>
using SimpleAutoDiffInverseDynamics =
    AutoDiffInverseDynamics<double, Eigen::Vector2d,
                            Eigen::Matrix<double, 1, 1>, Dynamics>;

class InverseDynamicsTest : public ::testing::Test {
 protected:
  template <typename NumericalDynamics = SimpleDynamics>
  SimpleInverseDynamics MakeNumerical(
      const Eigen::Vector2d &frozen_states,
      const Eigen::Matrix2d &Q = Eigen::Matrix2d::Zero(),
      const Eigen::Matrix<double, 1, 1> &R =
          Eigen::Matrix<double, 1, 1>::Zero()) {
    return {std::make_unique<NumericalDynamics>(), frozen_states, Q, R};
  }
  template <typename AutoDiffDynamics = SimpleDynamicsTemplated>
  SimpleAutoDiffInverseDynamics<AutoDiffDynamics> MakeAutoDiff(
      const Eigen::Vector2d &frozen_states,
      const Eigen::Matrix2d &Q = Eigen::Matrix2d::Zero(),
      const Eigen::Matrix<double, 1, 1> &R =
          Eigen::Matrix<double, 1, 1>::Zero()) {
    return {AutoDiffDynamics(), frozen_states, Q, R};
  }

  template <typename NumericalDynamics = SimpleDynamics,
            typename AutoDiffDynamics = SimpleDynamicsTemplated>
  SimpleInverseDynamics::StateInput CheckThereAndBack(
      const Eigen::Vector2d &frozen_states, const Eigen::Vector2d &goal_state,
      const Eigen::Matrix2d &Q = Eigen::Matrix2d::Zero(),
      const Eigen::Matrix<double, 1, 1> &R =
          Eigen::Matrix<double, 1, 1>::Zero()) {
    SimpleInverseDynamics inverse =
        MakeNumerical<NumericalDynamics>(frozen_states, Q, R);
    SimpleAutoDiffInverseDynamics<AutoDiffDynamics> auto_inverse =
        MakeAutoDiff<AutoDiffDynamics>(frozen_states, Q, R);
    SimpleInverseDynamics::StateInput result;
    typename SimpleAutoDiffInverseDynamics<AutoDiffDynamics>::StateInput
        auto_result;
    {
      // Sanity-check that we don't malloc during the actual dynamics
      // inversion that we would need to call at runtime.
      aos::ScopedRealtime realtime;
      result = inverse.InvertDynamics(goal_state);
      auto_result = auto_inverse.InvertDynamics(goal_state);
    }
    for (int index = 0; index < frozen_states.size(); ++index) {
      SCOPED_TRACE(index);
      if (frozen_states(index)) {
        EXPECT_NEAR(goal_state(index), result.state(index), 1e-6);
        EXPECT_NEAR(goal_state(index), auto_result.state(index), 1e-6);
      }
      EXPECT_NEAR(result.state(index), auto_result.state(index), 1e-8);
    }
    // Check that the solution for the input is comparable for each solver
    // method.
    EXPECT_NEAR(result.input(0), auto_result.input(0), 1e-8);

    NumericalDynamics dynamics;

    Eigen::Vector2d Xdot = dynamics(result.state, result.input);
    EXPECT_TRUE((Xdot.cwiseAbs().array() < 1e-6).all()) << Xdot;
    return result;
  }
};

// Confirm that we can invert the dynamics when all of the states are frozen
// (this situation implies that we are attempting to choose a control input such
// that we will stay exactly at the provided state).
TEST_F(InverseDynamicsTest, AllFrozenStates) {
  // A state of zero should be perfectly solvable.
  CheckThereAndBack(Eigen::Vector2d::Ones(), Eigen::Vector2d::Zero());
  SimpleInverseDynamics inverse = MakeNumerical(Eigen::Vector2d::Ones());
  SimpleInverseDynamics::StateInput result1 =
      inverse.InvertDynamics(Eigen::Vector2d::Ones());
  // Because the states are frozen, they should result in exactly the provided
  // inputs.
  EXPECT_EQ(Eigen::Vector2d::Ones(), result1.state);
  // In order to hold a non-zero velocity, we need a non-zero U to make the
  // derivative of state[1] be zero; however, the derivative of state[0]
  // cannot be zero in this state.
  EXPECT_NEAR(0.1, result1.input(0), 1e-6);
}

// Check that if no states are frozen (and thus the problem is
// under-constrained) that we get out reasonable results.
TEST_F(InverseDynamicsTest, NoFrozenStates) {
  SimpleInverseDynamics::StateInput result =
      CheckThereAndBack(Eigen::Vector2d::Zero(), Eigen::Vector2d::Ones());
  EXPECT_NEAR(0.0, result.input(0), 1e-6);
  // This state is under-constrained, and so the solver should leave it
  // untouched from where we started.
  EXPECT_EQ(1, result.state(0));
}

// Tests that we can constrain the position state of our toy problem. The
// velocity state should be forced to zero and the input will be driven to zero.
TEST_F(InverseDynamicsTest, ConstrainPosition) {
  CheckThereAndBack(Eigen::Vector2d{{1.0}, {0.0}}, Eigen::Vector2d::Ones());
}

// Tests that we can constrain the velocity state of our toy problem. The
// position state will be unconstrained and the input should be driven to match
// the fact that there is a feedback force on the velocity state.
TEST_F(InverseDynamicsTest, ConstrainVelocity) {
  SimpleInverseDynamics inverse = MakeNumerical(Eigen::Vector2d{{0.0}, {1.0}});
  SimpleInverseDynamics::StateInput result1 =
      inverse.InvertDynamics(Eigen::Vector2d::Ones());
  // Because the states are frozen, they should result in exactly the provided
  // inputs.
  EXPECT_EQ(Eigen::Vector2d::Ones(), result1.state);
  // In order to hold a non-zero velocity, we need a non-zero U to make the
  // derivative of state[1] be zero; however, the derivative of state[0]
  // cannot be zero in this state.
  EXPECT_NEAR(0.1, result1.input(0), 1e-6);
}

// Tests that we can constrain the position state of our toy problem and that if
// we also supply non-zero cost matrices that that does *not* cause us to
// violate the constraints.
TEST_F(InverseDynamicsTest, ConstrainPositionWithCost) {
  CheckThereAndBack(Eigen::Vector2d{{1.0}, {0.0}}, Eigen::Vector2d::Ones(),
                    Eigen::Matrix2d::Identity(),
                    Eigen::Matrix<double, 1, 1>{{1.0}});
}

// Tests that if we have a non-zero cost but are entirely unconstrained that we
// can use the costs to drive the state/inputs to zero.
TEST_F(InverseDynamicsTest, UnconstrainedWithCost) {
  SimpleInverseDynamics::StateInput result = CheckThereAndBack(
      Eigen::Vector2d{{0.0}, {0.0}}, Eigen::Vector2d::Ones(),
      Eigen::Matrix2d::Identity(), Eigen::Matrix<double, 1, 1>{{1.0}});
  EXPECT_LT(result.state.norm(), 1e-6);
  EXPECT_NEAR(0.0, result.input(0), 1e-6);
}

// Tests that if we have underconstrained dynamics that we can get varying
TEST_F(InverseDynamicsTest, UnderconstrainedWithCost) {
  // The "underconstrained" dynamics are such that
  // xdot(1) = x(0) + x(1) + u(0)
  // When we demand that x(1) be equal to 1.0, then x(0) + u(0) should be
  // zero. How those get balanced depends on the cost matrices.

  // For equal cost matrices, we should equally split the load between x(0) and
  // u(0).
  SimpleInverseDynamics::StateInput result =
      CheckThereAndBack<UnderconstrainedDynamics,
                        UnderconstrainedDynamicsTemplated>(
          Eigen::Vector2d{{0.0}, {1.0}}, Eigen::Vector2d{{0.0}, {1.0}},
          Eigen::Matrix2d::Identity(), Eigen::Matrix<double, 1, 1>{{1.0}});
  EXPECT_NEAR(-0.5, result.state(0), 1e-6);
  EXPECT_NEAR(-0.5, result.input(0), 1e-6);

  // If the input cost is zero, we should end up with x(0) at zero.
  result = CheckThereAndBack<UnderconstrainedDynamics,
                             UnderconstrainedDynamicsTemplated>(
      Eigen::Vector2d{{0.0}, {1.0}}, Eigen::Vector2d{{0.0}, {1.0}},
      Eigen::Matrix2d::Identity(), Eigen::Matrix<double, 1, 1>{{0.0}});
  EXPECT_NEAR(0.0, result.state(0), 1e-6);
  EXPECT_NEAR(-1.0, result.input(0), 1e-6);

  // If the state cost is zero, we should end up with the input at zero.
  result = CheckThereAndBack<UnderconstrainedDynamics,
                             UnderconstrainedDynamicsTemplated>(
      Eigen::Vector2d{{0.0}, {1.0}}, Eigen::Vector2d{{0.0}, {1.0}},
      Eigen::Matrix2d::Zero(), Eigen::Matrix<double, 1, 1>{{1.0}});
  EXPECT_NEAR(-1.0, result.state(0), 1e-6);
  EXPECT_NEAR(0.0, result.input(0), 1e-6);

  // Sanity check that a less trivial distribution of costs also behaves
  // as expected.
  result = CheckThereAndBack<UnderconstrainedDynamics,
                             UnderconstrainedDynamicsTemplated>(
      Eigen::Vector2d{{0.0}, {1.0}}, Eigen::Vector2d{{0.0}, {1.0}},
      Eigen::Matrix2d::Identity(), Eigen::Matrix<double, 1, 1>{{0.5}});
  EXPECT_NEAR(-1.0 / 3.0, result.state(0), 1e-6);
  EXPECT_NEAR(-2.0 / 3.0, result.input(0), 1e-6);
}

}  // namespace frc971::control_loops::swerve::test
