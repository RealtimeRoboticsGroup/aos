#include "frc/control_loops/drivetrain/trajectory.h"

#include <chrono>
#include <vector>

#include "absl/flags/declare.h"
#include "absl/flags/flag.h"
#include "gtest/gtest.h"

#include "aos/testing/test_shm.h"
#include "frc/control_loops/drivetrain/drivetrain_test_lib.h"
#if defined(SUPPORT_PLOT)
#include "third_party/matplotlib-cpp/matplotlibcpp.h"
#endif

ABSL_DECLARE_FLAG(bool, plot);

ABSL_FLAG(std::string, output_file, "",
          "If set, logs all channels to the provided logfile.");

namespace frc::control_loops::drivetrain::testing {

namespace chrono = ::std::chrono;

double a(double /*v*/, double /*x*/) { return 2.0; }

// Tests that the derivitives of xy integrate back up to the position.
TEST(IntegrateAccelForDistanceTest, IntegrateAccelForDistance) {
  double v = 0.0;
  const size_t count = 10;
  const double dx = 4.0 / static_cast<double>(count);
  for (size_t i = 0; i < count; ++i) {
    v = IntegrateAccelForDistance(a, v, 0.0, dx);
  }
  EXPECT_NEAR(4.0, v, 1e-8);
}

struct SplineTestParams {
  ::Eigen::Matrix<double, 2, 6> control_points;
  double lateral_acceleration;
  double longitudinal_acceleration;
  double velocity_limit;
  double voltage_limit;
  ::std::function<void(Trajectory *)> trajectory_modification_fn;
  // Number of iterations to attempt to use the path-relative state on--because
  // of the number of numerical approximations involved, we generally aren't
  // actually able to use a large number of iterations before the errors
  // accumulate too much (or I am doing something wrong).
  size_t valid_path_relative_iterations;
};

void NullTrajectoryModificationFunction(Trajectory *) {}

class ParameterizedSplineTest
    : public ::testing::TestWithParam<SplineTestParams> {
 public:
  ::aos::testing::TestSharedMemory shm_;

  const DistanceSpline *distance_spline_;
  ::std::unique_ptr<Trajectory> trajectory_;
  std::unique_ptr<aos::FlatbufferDetachedBuffer<fb::Trajectory>>
      trajectory_buffer_;
  ::std::unique_ptr<FinishedTrajectory> finished_trajectory_;
  ::std::vector<::Eigen::Matrix<double, 3, 1>> length_plan_xva_;

  ParameterizedSplineTest() : dt_config_(GetTestDrivetrainConfig()) {}

  void SetUp() {
    const int spline_index = 12345;
    // Run lots of steps to make the feedforwards terms more accurate.
    trajectory_ = ::std::unique_ptr<Trajectory>(new Trajectory(
        DistanceSpline(GetParam().control_points), &dt_config_,
        /*constraints=*/nullptr, spline_index, GetParam().velocity_limit));
    distance_spline_ = &trajectory_->spline();
    trajectory_->set_lateral_acceleration(GetParam().lateral_acceleration);
    trajectory_->set_longitudinal_acceleration(
        GetParam().longitudinal_acceleration);
    trajectory_->set_voltage_limit(GetParam().voltage_limit);

    GetParam().trajectory_modification_fn(trajectory_.get());

    initial_plan_ = trajectory_->plan();
    trajectory_->VoltageFeasibilityPass(Trajectory::VoltageLimit::kAggressive);
    aggressive_voltage_plan_ = trajectory_->plan();
    trajectory_->VoltageFeasibilityPass(
        Trajectory::VoltageLimit::kConservative);
    voltage_plan_ = trajectory_->plan();
    trajectory_->LateralAccelPass();
    curvature_plan_ = trajectory_->plan();
    trajectory_->ForwardPass();
    forward_plan_ = trajectory_->plan();
    trajectory_->BackwardPass();
    backward_plan_ = trajectory_->plan();

    trajectory_->CalculatePathGains();

    length_plan_xva_ = trajectory_->PlanXVA(dt_config_.dt);

    flatbuffers::FlatBufferBuilder fbb;

    fbb.Finish(trajectory_->Serialize(&fbb));

    trajectory_buffer_ = std::make_unique<aos::FlatbufferDetachedBuffer<
        frc::control_loops::drivetrain::fb::Trajectory>>(fbb.Release());

    EXPECT_EQ(spline_index, trajectory_buffer_->message().handle());

    finished_trajectory_ = std::make_unique<FinishedTrajectory>(
        &dt_config_, &trajectory_buffer_->message());
  }

  void TearDown() {
    printf("  Spline takes %f seconds to follow\n",
           length_plan_xva_.size() *
               ::aos::time::DurationInSeconds(dt_config_.dt));
#if defined(SUPPORT_PLOT)
    if (absl::GetFlag(FLAGS_plot)) {
      ::std::vector<double> distances = trajectory_->Distances();

      for (size_t i = 0; i < length_plan_xva_.size(); ++i) {
        length_plan_t_.push_back(i *
                                 ::aos::time::DurationInSeconds(dt_config_.dt));
        length_plan_x_.push_back(length_plan_xva_[i](0));
        length_plan_v_.push_back(length_plan_xva_[i](1));
        length_plan_a_.push_back(length_plan_xva_[i](2));
      }

      ::std::vector<double> plan_segment_center_distance;
      ::std::vector<double> plan_type;
      for (fb::SegmentConstraint segment_type :
           trajectory_->plan_segment_type()) {
        plan_type.push_back(static_cast<int>(segment_type));
      }
      for (size_t i = 0; i < distances.size() - 1; ++i) {
        plan_segment_center_distance.push_back(
            (distances[i] + distances[i + 1]) / 2.0);
      }

      matplotlibcpp::figure();
      matplotlibcpp::plot(plan_segment_center_distance, plan_type,
                          {{"label", "plan_type"}});
      matplotlibcpp::plot(distances, backward_plan_, {{"label", "backward"}});
      matplotlibcpp::plot(distances, forward_plan_, {{"label", "forward"}});
      matplotlibcpp::plot(distances, curvature_plan_, {{"label", "lateral"}});
      matplotlibcpp::plot(distances, aggressive_voltage_plan_,
                          {{"label", "aggressive voltage"}});
      matplotlibcpp::plot(distances, voltage_plan_, {{"label", "voltage"}});
      matplotlibcpp::plot(distances, initial_plan_, {{"label", "initial"}});
      matplotlibcpp::legend();

      matplotlibcpp::figure();
      matplotlibcpp::plot(length_plan_t_, length_plan_x_, {{"label", "x"}});
      matplotlibcpp::plot(length_plan_t_, length_plan_v_, {{"label", "v"}});
      matplotlibcpp::plot(length_plan_t_, length_plan_a_, {{"label", "a"}});
      matplotlibcpp::plot(length_plan_t_, length_plan_vl_, {{"label", "Vl"}});
      matplotlibcpp::plot(length_plan_t_, length_plan_vr_, {{"label", "Vr"}});
      matplotlibcpp::legend();

      matplotlibcpp::show();
    }
#endif
  }

  const DrivetrainConfig<double> dt_config_;
  static constexpr double kXPos = 0.05;
  static constexpr double kYPos = 0.05;
  static constexpr double kThetaPos = 0.2;
  static constexpr double kVel = 0.5;
  const ::Eigen::DiagonalMatrix<double, 5> Q =
      (::Eigen::DiagonalMatrix<double, 5>().diagonal()
           << 1.0 / ::std::pow(kXPos, 2),
       1.0 / ::std::pow(kYPos, 2), 1.0 / ::std::pow(kThetaPos, 2),
       1.0 / ::std::pow(kVel, 2), 1.0 / ::std::pow(kVel, 2))
          .finished()
          .asDiagonal();

  const ::Eigen::DiagonalMatrix<double, 2> R =
      (::Eigen::DiagonalMatrix<double, 2>().diagonal()
           << 1.0 / ::std::pow(12.0, 2),
       1.0 / ::std::pow(12.0, 2))
          .finished()
          .asDiagonal();

  ::std::vector<double> initial_plan_;
  ::std::vector<double> curvature_plan_;
  ::std::vector<double> forward_plan_;
  ::std::vector<double> backward_plan_;
  ::std::vector<double> voltage_plan_;
  ::std::vector<double> aggressive_voltage_plan_;

  ::std::vector<double> length_plan_t_;
  ::std::vector<double> length_plan_x_;
  ::std::vector<double> length_plan_v_;
  ::std::vector<double> length_plan_a_;
  ::std::vector<double> length_plan_vl_;
  ::std::vector<double> length_plan_vr_;
};

// Tests that the VoltageVelocityLimit function produces correct results by
// calculating the limit at a variety of points and then ensuring that we can
// indeed drive +/- 12V at the limit and that we can't go faster.
TEST_P(ParameterizedSplineTest, VoltageFeasibilityCheck) {
  for (size_t i = 0; i < length_plan_xva_.size(); ++i) {
    const double distance = length_plan_xva_[i](0);
    ::Eigen::Matrix<double, 2, 1> U;
    const ::Eigen::Matrix<double, 2, 1> K2 =
        trajectory_->K2(distance_spline_->DTheta(distance));
    const ::Eigen::Matrix<double, 2, 1> K1 =
        trajectory_->K1(distance_spline_->DDTheta(distance));
    const ::Eigen::Matrix<double, 2, 2> A =
        trajectory_->velocity_drivetrain().plant().coefficients().A_continuous;
    const ::Eigen::Matrix<double, 2, 2> B =
        trajectory_->velocity_drivetrain().plant().coefficients().B_continuous;
    const double conservative_limit = trajectory_->VoltageVelocityLimit(
        distance, Trajectory::VoltageLimit::kConservative, &U);
    ASSERT_LT(0.0, conservative_limit)
        << "Voltage limit should be strictly positive.";
    const bool on_straight_line = distance_spline_->DTheta(distance) == 0 &&
                                  distance_spline_->DDTheta(distance) == 0;
    if (on_straight_line) {
      EXPECT_EQ(::std::numeric_limits<double>::infinity(), conservative_limit);
    } else {
      const ::Eigen::Matrix<double, 2, 1> wheel_accel =
          A * K2 * conservative_limit + B * U;
      // TODO(james): Technically, K2 can contain zeros.
      const ::Eigen::Matrix<double, 2, 1> implied_accels =
          K2.cwiseInverse().cwiseProduct(
              wheel_accel - K1 * ::std::pow(conservative_limit, 2));
      EXPECT_NEAR(implied_accels(0), implied_accels(1), 1e-9);
      EXPECT_TRUE((GetParam().voltage_limit == U.array().abs()).all()) << U;
      const double accel = implied_accels(0);
      // Check that we really are at a limit by confirming that even slightly
      // disturbing the acceleration in either direction would result in invalid
      // accelerations.
      for (const double perturbed_accel : {accel - 1e-5, accel + 1e-5}) {
        const ::Eigen::Matrix<double, 2, 1> perturbed_wheel_accels =
            K2 * perturbed_accel + K1 * conservative_limit * conservative_limit;
        const ::Eigen::Matrix<double, 2, 1> perturbed_voltages =
            B.inverse() *
            (perturbed_wheel_accels - A * K2 * conservative_limit);
        EXPECT_GT(perturbed_voltages.lpNorm<::Eigen::Infinity>(),
                  GetParam().voltage_limit)
            << "We were able to perturb the voltage!!.";
      }
    }
    length_plan_vl_.push_back(U(0));
    length_plan_vr_.push_back(U(1));

    // And check the same for the "aggressive" configuration:
    {
      const double aggressive_limit = trajectory_->VoltageVelocityLimit(
          distance, Trajectory::VoltageLimit::kAggressive, &U);
      if (on_straight_line) {
        EXPECT_EQ(::std::numeric_limits<double>::infinity(), aggressive_limit);
        continue;
      }
      EXPECT_TRUE((GetParam().voltage_limit == U.array().abs()).all()) << U;
      const ::Eigen::Matrix<double, 2, 1> wheel_accel =
          A * K2 * aggressive_limit + B * U;
      const ::Eigen::Matrix<double, 2, 1> implied_accels =
          K2.cwiseInverse().cwiseProduct(wheel_accel -
                                         K1 * ::std::pow(aggressive_limit, 2));
      EXPECT_NEAR(implied_accels(0), implied_accels(1), 1e-9);
      EXPECT_LE(conservative_limit, aggressive_limit)
          << "The aggressive velocity limit should not be less than the "
             "conservative.";
      const double accel = implied_accels(0);
      for (const double perturbed_accel : {accel - 1e-5, accel + 1e-5}) {
        const ::Eigen::Matrix<double, 2, 1> perturbed_wheel_accels =
            K2 * perturbed_accel + K1 * aggressive_limit * aggressive_limit;
        const ::Eigen::Matrix<double, 2, 1> perturbed_voltages =
            B.inverse() * (perturbed_wheel_accels - A * K2 * aggressive_limit);
        EXPECT_GT(perturbed_voltages.lpNorm<::Eigen::Infinity>(),
                  GetParam().voltage_limit)
            << "We were able to perturb the voltage!!.";
      }
    }
  }
}

// Tests that the friction-based velocity limits are correct.
TEST_P(ParameterizedSplineTest, FrictionLimitCheck) {
  // To do this check, retrieve the lateral acceleration velocity limit and
  // confirm that we can indeed travel at that velocity without violating
  // friction constraints and that we cannot go any faster.
  for (size_t i = 0; i < length_plan_xva_.size(); ++i) {
    const double distance = length_plan_xva_[i](0);
    const ::Eigen::Matrix<double, 2, 1> K2 =
        trajectory_->K2(distance_spline_->DTheta(distance));
    const ::Eigen::Matrix<double, 2, 1> K1 =
        trajectory_->K1(distance_spline_->DDTheta(distance));
    const ::Eigen::Matrix<double, 2, 2> A =
        trajectory_->velocity_drivetrain().plant().coefficients().A_continuous;
    const ::Eigen::Matrix<double, 2, 2> B =
        trajectory_->velocity_drivetrain().plant().coefficients().B_continuous;
    const bool on_straight_line = distance_spline_->DTheta(distance) == 0 &&
                                  distance_spline_->DDTheta(distance) == 0;
    const double velocity_limit =
        trajectory_->LateralVelocityCurvature(distance);
    ASSERT_LT(0.0, velocity_limit)
        << "Acceleration limit should be strictly positive.";
    if (on_straight_line) {
      EXPECT_EQ(::std::numeric_limits<double>::infinity(), velocity_limit);
      continue;
    }
    const double lat_accel =
        velocity_limit * velocity_limit * distance_spline_->DTheta(distance);
    const double allowed_lng_accel =
        GetParam().longitudinal_acceleration *
        ::std::sqrt(1.0 -
                    ::std::pow(lat_accel / GetParam().lateral_acceleration, 2));
    ::Eigen::Matrix<double, 2, 1> wheel_accels;
    wheel_accels << 1.0, ((K2(0) * K2(1) > 0.0) ? -1.0 : 1.0);
    wheel_accels *= allowed_lng_accel;
    const ::Eigen::Matrix<double, 2, 1> implied_accels1 =
        K2.cwiseInverse().cwiseProduct(wheel_accels -
                                       K1 * ::std::pow(velocity_limit, 2));
    const ::Eigen::Matrix<double, 2, 1> implied_accels2 =
        K2.cwiseInverse().cwiseProduct(-wheel_accels -
                                       K1 * ::std::pow(velocity_limit, 2));
    const double accels1_err =
        ::std::abs(implied_accels1(0) - implied_accels1(1));
    const double accels2_err =
        ::std::abs(implied_accels2(0) - implied_accels2(1));
    EXPECT_LT(::std::min(accels1_err, accels2_err), 1e-10);
    const double implied_accel =
        (accels1_err < accels2_err) ? implied_accels1(0) : implied_accels2(0);
    // Confirm that we are indeed on the edge of feasibility by testing that
    // we can't accelerate any faste/slower at this velocity without violating
    // acceleration constraints.
    for (const double perturbed_accel :
         {implied_accel - 1e-5, implied_accel + 1e-5}) {
      const ::Eigen::Matrix<double, 2, 1> perturbed_wheel_accels =
          K2 * perturbed_accel + K1 * velocity_limit * velocity_limit;
      EXPECT_GT(perturbed_wheel_accels.lpNorm<::Eigen::Infinity>(),
                allowed_lng_accel)
          << "We were able to perturb the acceleration!!.";
    }
    {
      const ::Eigen::Matrix<double, 2, 1> U =
          B.inverse() * (wheel_accels - A * K2 * velocity_limit);
      length_plan_vl_.push_back(U(0));
      length_plan_vr_.push_back(U(1));
    }

    {
      // Also test the utility function for determining the acceleration
      // limits:
      double min_accel, max_accel;
      trajectory_->FrictionLngAccelLimits(distance, velocity_limit, &min_accel,
                                          &max_accel);
      // If on the limit, the min/max acceleration limits should be identical.
      EXPECT_NEAR(min_accel, max_accel, 1e-10);
      EXPECT_NEAR(implied_accel, min_accel, 1e-10);
    }
  }
}

// Tests that following a spline with feed forwards only gets pretty darn close
// to the right point.
TEST_P(ParameterizedSplineTest, FFSpline) {
  ::Eigen::Matrix<double, 5, 1> state = ::Eigen::Matrix<double, 5, 1>::Zero();
  state = finished_trajectory_->GoalState(0.0, 0.0);

  for (size_t i = 0; i < length_plan_xva_.size(); ++i) {
    const double distance = length_plan_xva_[i](0);

    const ::Eigen::Matrix<double, 2, 1> U_ff =
        finished_trajectory_->FFVoltage(distance);
    const ::Eigen::Matrix<double, 2, 1> U = U_ff;

    length_plan_vl_.push_back(U(0));
    length_plan_vr_.push_back(U(1));
    state = RungeKuttaU(
        [this](const ::Eigen::Matrix<double, 5, 1> &X,
               const ::Eigen::Matrix<double, 2, 1> &U) {
          return ContinuousDynamics(trajectory_->velocity_drivetrain().plant(),
                                    dt_config_.Tlr_to_la(), X, U);
        },
        state, U, ::aos::time::DurationInSeconds(dt_config_.dt));
  }

  EXPECT_LT((state - finished_trajectory_->GoalState(
                         finished_trajectory_->length(), 0.0))
                .norm(),
            4e-2);
}

// Tests that Iteratively computing the XVA plan is the same as precomputing it.
TEST_P(ParameterizedSplineTest, IterativeXVA) {
  for (double v : trajectory_->plan()) {
    EXPECT_TRUE(::std::isfinite(v));
  }
  ::Eigen::Matrix<double, 2, 1> state = ::Eigen::Matrix<double, 2, 1>::Zero();
  for (size_t i = 1; i < length_plan_xva_.size(); ++i) {
    ::Eigen::Matrix<double, 3, 1> xva =
        finished_trajectory_->GetNextXVA(dt_config_.dt, &state);
    EXPECT_LT((length_plan_xva_[i] - xva).norm(), 1e-2);
  }
}

TEST_P(ParameterizedSplineTest, PathRelativeMathTest) {
  Eigen::Matrix<double, 5, 1> absolute_state =
      Eigen::Matrix<double, 5, 1>::Zero();
  Eigen::Matrix<double, 5, 1> relative_state =
      Eigen::Matrix<double, 5, 1>::Zero();
  std::vector<Eigen::Matrix<double, 5, 5>> A_discretes;
  std::vector<Eigen::Matrix<double, 5, 2>> B_discretes;
  // Test that the path relative coordinate conversion is mathematically
  // consistent with the absolute coordinate system.
  for (size_t i = 0; i < length_plan_xva_.size(); ++i) {
    std::stringstream msg;
    msg << i << " of " << length_plan_xva_.size();
    SCOPED_TRACE(msg.str());
    const double distance = length_plan_xva_[i](0);
    const double velocity = length_plan_xva_[i](1);
    Eigen::Matrix<double, 5, 5> A_continuous;
    Eigen::Matrix<double, 5, 2> B_continuous;
    trajectory_->PathRelativeContinuousSystem(distance, &A_continuous,
                                              &B_continuous);
    Eigen::Matrix<double, 5, 5> A_discrete;
    Eigen::Matrix<double, 5, 2> B_discrete;
    controls::C2D(A_continuous, B_continuous, dt_config_.dt, &A_discrete,
                  &B_discrete);

    // The below code exists to check that the transform to path-relative
    // coordinates doesn't result in significantly different results from the
    // normal math. However, numerical differences can explode after enough
    // iterations of integration, so stop checking after a certain number of
    // iterations.
    if (i >= GetParam().valid_path_relative_iterations) {
      continue;
    }

    const Eigen::Matrix<double, 2, 1> U_ff =
        finished_trajectory_->FFVoltage(distance);
    const Eigen::Matrix<double, 2, 1> U = U_ff;

    absolute_state = RungeKuttaU(
        [this](const Eigen::Matrix<double, 5, 1> &X,
               const Eigen::Matrix<double, 2, 1> &U) {
          return ContinuousDynamics(
              finished_trajectory_->velocity_drivetrain().plant(),
              dt_config_.Tlr_to_la(), X, U);
        },
        absolute_state, U, aos::time::DurationInSeconds(dt_config_.dt));
    const Eigen::Matrix<double, 5, 1> goal_absolute_state =
        finished_trajectory_->GoalState(distance, velocity);
    const Eigen::Matrix<double, 5, 1> goal_relative_state =
        finished_trajectory_->StateToPathRelativeState(
            distance, goal_absolute_state, false);
    ASSERT_EQ(distance, goal_relative_state(0));
    ASSERT_EQ(0.0, goal_relative_state(1));
    ASSERT_NEAR(goal_absolute_state(2), goal_relative_state(2), 1e-2);
    ASSERT_EQ(goal_absolute_state(3), goal_relative_state(3));
    ASSERT_EQ(goal_absolute_state(4), goal_relative_state(4));

    relative_state = A_discrete * relative_state + B_discrete * U;

    ASSERT_LT((relative_state - goal_relative_state).norm(), 1e-2)
        << ": Goal\n"
        << goal_relative_state << " Integrated\n"
        << relative_state;
  }

  const size_t path_length = length_plan_xva_.size();

  // Test that if we run path-relative feedback controller then we end up
  // tracking with reasonably low error.
  absolute_state = Eigen::Matrix<double, 5, 1>::Zero();
  double initial_error = 0;
  for (size_t i = 0; i < path_length; ++i) {
    std::stringstream msg;
    msg << i << " of " << length_plan_xva_.size();
    SCOPED_TRACE(msg.str());
    const double distance = length_plan_xva_[i](0);
    const double velocity = length_plan_xva_[i](1);

    const Eigen::Matrix<double, 5, 1> goal_absolute_state =
        finished_trajectory_->GoalState(distance, velocity);
    const Eigen::Matrix<double, 5, 1> goal_relative_state =
        finished_trajectory_->StateToPathRelativeState(
            distance, goal_absolute_state, false);
    const Eigen::Matrix<double, 5, 1> current_relative_state =
        finished_trajectory_->StateToPathRelativeState(distance, absolute_state,
                                                       false);

    const Eigen::Matrix<double, 2, 1> U_ff =
        finished_trajectory_->FFVoltage(distance);
    Eigen::Matrix<double, 2, 1> U_fb =
        finished_trajectory_->GainForDistance(distance) *
        (goal_relative_state - current_relative_state);
    if (i == 0) {
      initial_error = (goal_relative_state - current_relative_state).norm();
    }
    const Eigen::Matrix<double, 2, 1> U = U_ff + U_fb;

    absolute_state = RungeKuttaU(
        [this](const Eigen::Matrix<double, 5, 1> &X,
               const Eigen::Matrix<double, 2, 1> &U) {
          return ContinuousDynamics(trajectory_->velocity_drivetrain().plant(),
                                    dt_config_.Tlr_to_la(), X, U);
        },
        absolute_state, U, aos::time::DurationInSeconds(dt_config_.dt));
  }

  EXPECT_LT((absolute_state - finished_trajectory_->GoalState(
                                  finished_trajectory_->length(), 0.0))
                .norm(),
            4e-2 + initial_error * 0.5);
}

SplineTestParams MakeSplineTestParams(struct SplineTestParams params) {
  return params;
}

void LimitMiddleOfPathTrajectoryModificationFunction(Trajectory *trajectory) {
  trajectory->LimitVelocity(1.0, 2.0, 0.5);
}

void ShortLimitMiddleOfPathTrajectoryModificationFunction(
    Trajectory *trajectory) {
  trajectory->LimitVelocity(1.5, 1.5, 0.5);
}

INSTANTIATE_TEST_SUITE_P(
    SplineTest, ParameterizedSplineTest,
    ::testing::Values(
        MakeSplineTestParams(
            {Spline4To6((::Eigen::Matrix<double, 2, 4>() << 0.0, 1.2, -0.2, 1.0,
                         0.0, 0.0, 1.0, 1.0)
                            .finished()),
             2.0 /*lateral acceleration*/, 1.0 /*longitudinal acceleration*/,
             10.0 /* velocity limit */, 12.0 /* volts */,
             NullTrajectoryModificationFunction, 40}),
        // Be velocity limited.
        MakeSplineTestParams(
            {Spline4To6((::Eigen::Matrix<double, 2, 4>() << 0.0, 6.0, -1.0, 5.0,
                         0.0, 0.0, 1.0, 1.0)
                            .finished()),
             2.0 /*lateral acceleration*/, 1.0 /*longitudinal acceleration*/,
             0.5 /* velocity limit */, 12.0 /* volts */,
             NullTrajectoryModificationFunction, 40}),
        // Hit the voltage limit.
        MakeSplineTestParams(
            {Spline4To6((::Eigen::Matrix<double, 2, 4>() << 0.0, 6.0, -1.0, 5.0,
                         0.0, 0.0, 1.0, 1.0)
                            .finished()),
             2.0 /*lateral acceleration*/, 3.0 /*longitudinal acceleration*/,
             10.0 /* velocity limit */, 5.0 /* volts */,
             NullTrajectoryModificationFunction, 0}),
        // Hit the curvature limit.
        MakeSplineTestParams(
            {Spline4To6((::Eigen::Matrix<double, 2, 4>() << 0.0, 1.2, -0.2, 1.0,
                         0.0, 0.0, 1.0, 1.0)
                            .finished()),
             1.0 /*lateral acceleration*/, 3.0 /*longitudinal acceleration*/,
             10.0 /* velocity limit */, 12.0 /* volts */,
             NullTrajectoryModificationFunction, 0}),
        // Add an artifical velocity limit in the middle.
        MakeSplineTestParams(
            {Spline4To6((::Eigen::Matrix<double, 2, 4>() << 0.0, 6.0, -1.0, 5.0,
                         0.0, 0.0, 1.0, 1.0)
                            .finished()),
             2.0 /*lateral acceleration*/, 3.0 /*longitudinal acceleration*/,
             10.0 /* velocity limit */, 12.0 /* volts */,
             LimitMiddleOfPathTrajectoryModificationFunction, 0}),
        // Add a really short artifical velocity limit in the middle.
        MakeSplineTestParams(
            {Spline4To6((::Eigen::Matrix<double, 2, 4>() << 0.0, 6.0, -1.0, 5.0,
                         0.0, 0.0, 1.0, 1.0)
                            .finished()),
             2.0 /*lateral acceleration*/, 3.0 /*longitudinal acceleration*/,
             10.0 /* velocity limit */, 12.0 /* volts */,
             ShortLimitMiddleOfPathTrajectoryModificationFunction, 0}),
        // Spline known to have caused issues in the past.
        MakeSplineTestParams(
            {(::Eigen::Matrix<double, 2, 6>() << 0.5f, 3.5f, 4.0f, 8.0f, 10.0f,
              10.2f, 1.0f, 1.0f, -3.0f, -2.0f, -3.5f, -3.65f)
                 .finished(),
             2.0 /*lateral acceleration*/, 3.0 /*longitudinal acceleration*/,
             200.0 /* velocity limit */, 12.0 /* volts */,
             NullTrajectoryModificationFunction, 0}),
        // Perfectly straight line (to check corner cases).
        MakeSplineTestParams(
            {Spline4To6((::Eigen::Matrix<double, 2, 4>() << 0.0, 1.0, 2.0, 3.0,
                         0.0, 0.0, 0.0, 0.0)
                            .finished()),
             2.0 /*lateral acceleration*/, 3.0 /*longitudinal acceleration*/,
             200.0 /* velocity limit */, 12.0 /* volts */,
             NullTrajectoryModificationFunction, 0})));

// TODO(austin): Handle saturation.  254 does this by just not going that
// fast...  We want to maybe replan when we get behind, or something.  Maybe
// stop moving the setpoint like our 2018 arm?

}  // namespace frc::control_loops::drivetrain::testing
