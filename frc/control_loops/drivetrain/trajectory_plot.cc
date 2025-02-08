#include <chrono>

#include "absl/flags/flag.h"
#include "third_party/matplotlib-cpp/matplotlibcpp.h"

#include "aos/init.h"
#include "aos/logging/implementations.h"
#include "aos/network/team_number.h"
#include "aos/time/time.h"
#include "frc/control_loops/dlqr.h"
#include "frc/control_loops/drivetrain/test_robot/drivetrain_base.h"
#include "frc/control_loops/drivetrain/trajectory.h"

// Notes:
//   Basic ideas from spline following are from Jared Russell and
//   http://msc.fe.uni-lj.si/Papers/Chapter10_MobileRobotsNewResearch_Lepetic2005.pdf
//
// For the future, I'd like to use the following to measure distance to the
// path.
//   http://home.eps.hw.ac.uk/~ab226/papers/dist.pdf
//
// LQR controller was inspired by
// https://calhoun.nps.edu/bitstream/handle/10945/40159/kanayama_a_stable.pdf
//
// I ended up just taking the jacobian of the dynamics.  That gives me a tangent
// plane to design a LQR controller around.  That works because we have a good
// feed forwards and a good idea where the robot will be next time so we only
// need to handle disturbances.
//
// https://photos.google.com/share/AF1QipPl34MOTPem2QmmTC3B21dL7GV2_HjxnseRrqxgR60TUasyIPliIuWmnH3yxuSNZw?key=cVhZLUYycXBIZlNTRy10cjZlWm0tcmlqQl9MTE13

ABSL_FLAG(bool, plot, true, "If true, plot");

ABSL_FLAG(double, dx, 0.0, "Amount to disturb x at the start");
ABSL_FLAG(double, dy, 0.0, "Amount to disturb y at the start");
ABSL_FLAG(double, dt, 0.0, "Amount to disturb theta at the start");
ABSL_FLAG(double, dvl, 0.0, "Amount to disturb vl at the start");
ABSL_FLAG(double, dvr, 0.0, "Amount to disturb vr at the start");

ABSL_FLAG(double, forward, 1.0, "Amount to drive forwards");

namespace chrono = ::std::chrono;

namespace frc::control_loops::drivetrain {

void Main() {
  const DrivetrainConfig<double> config = test_robot::GetDrivetrainConfig();
  Trajectory trajectory(DistanceSpline(Spline(Spline4To6(
                            (::Eigen::Matrix<double, 2, 4>() << 0.0,
                             1.2 * absl::GetFlag(FLAGS_forward),
                             -0.2 * absl::GetFlag(FLAGS_forward),
                             absl::GetFlag(FLAGS_forward), 0.0, 0.0, 1.0, 1.0)
                                .finished()))),
                        &config, nullptr);
  trajectory.set_lateral_acceleration(2.0);
  trajectory.set_longitudinal_acceleration(1.0);

  // Grab the spline.
  ::std::vector<double> distances = trajectory.Distances();
  ::std::vector<double> spline_x;
  ::std::vector<double> spline_y;
  ::std::vector<double> spline_theta;

  for (const double distance : distances) {
    const ::Eigen::Matrix<double, 2, 1> point =
        trajectory.spline().XY(distance);
    const double theta = trajectory.spline().Theta(distance);
    spline_x.push_back(point(0));
    spline_y.push_back(point(1));
    spline_theta.push_back(theta);
  }

  // Compute the velocity plan.
  ::aos::monotonic_clock::time_point start_time = ::aos::monotonic_clock::now();
  ::std::vector<double> initial_plan = trajectory.plan();
  trajectory.VoltageFeasibilityPass(Trajectory::VoltageLimit::kConservative);
  ::std::vector<double> voltage_plan = trajectory.plan();
  trajectory.LateralAccelPass();
  ::std::vector<double> curvature_plan = trajectory.plan();
  trajectory.ForwardPass();
  ::std::vector<double> forward_plan = trajectory.plan();
  trajectory.BackwardPass();

  ::aos::monotonic_clock::time_point end_time = ::aos::monotonic_clock::now();

  ::std::vector<double> plan_segment_center_distance;
  ::std::vector<double> plan_type;
  for (fb::SegmentConstraint segment_type : trajectory.plan_segment_type()) {
    plan_type.push_back(static_cast<int>(segment_type));
  }
  for (size_t i = 0; i < distances.size() - 1; ++i) {
    plan_segment_center_distance.push_back((distances[i] + distances[i + 1]) /
                                           2.0);
  }

  ::std::vector<double> backward_plan = trajectory.plan();

  AOS_LOG(INFO, "Took %fms to plan\n",
          chrono::duration_cast<chrono::duration<double, ::std::milli>>(
              end_time - start_time)
              .count());

  // Now, compute the xva as a function of time.
  ::std::vector<double> length_plan_t;
  ::std::vector<double> length_plan_x;
  ::std::vector<double> length_plan_v;
  ::std::vector<double> length_plan_a;
  ::std::vector<double> length_plan_vl;
  ::std::vector<double> length_plan_vr;
  const chrono::nanoseconds kDt = chrono::microseconds(5050);
  const double kDtDouble = ::aos::time::DurationInSeconds(kDt);
  {
    ::std::vector<::Eigen::Matrix<double, 3, 1>> length_plan_xva =
        trajectory.PlanXVA(kDt);
    for (size_t i = 0; i < length_plan_xva.size(); ++i) {
      length_plan_t.push_back(i * kDtDouble);
      length_plan_x.push_back(length_plan_xva[i](0));
      length_plan_v.push_back(length_plan_xva[i](1));
      length_plan_a.push_back(length_plan_xva[i](2));

      ::Eigen::Matrix<double, 2, 1> U =
          trajectory.FFVoltage(length_plan_xva[i](0));
      length_plan_vl.push_back(U(0));
      length_plan_vr.push_back(U(1));
    }
  }

  flatbuffers::FlatBufferBuilder fbb;

  fbb.Finish(trajectory.Serialize(&fbb));

  aos::FlatbufferDetachedBuffer<fb::Trajectory> trajectory_buffer(
      fbb.Release());

  FinishedTrajectory finished_trajectory(&config, &trajectory_buffer.message());

  ::Eigen::Matrix<double, 5, 1> state = ::Eigen::Matrix<double, 5, 1>::Zero();
  state(0, 0) = absl::GetFlag(FLAGS_dx);
  state(1, 0) = absl::GetFlag(FLAGS_dy);
  state(2, 0) = absl::GetFlag(FLAGS_dt);
  state(3, 0) = absl::GetFlag(FLAGS_dvl);
  state(4, 0) = absl::GetFlag(FLAGS_dvr);
  ::std::vector<double> simulation_t = length_plan_t;
  ::std::vector<double> simulation_x;
  ::std::vector<double> error_x;
  ::std::vector<double> simulation_y;
  ::std::vector<double> error_y;
  ::std::vector<double> simulation_theta;
  ::std::vector<double> error_theta;
  ::std::vector<double> simulation_velocity_l;
  ::std::vector<double> error_velocity_l;
  ::std::vector<double> simulation_velocity_r;
  ::std::vector<double> error_velocity_r;
  ::std::vector<double> simulation_ul;
  ::std::vector<double> simulation_ur;
  for (size_t i = 0; i < length_plan_t.size(); ++i) {
    const double distance = length_plan_x[i];
    const double velocity = length_plan_v[i];
    const ::Eigen::Matrix<double, 5, 1> goal_state =
        trajectory.GoalState(distance, velocity);

    const ::Eigen::Matrix<double, 5, 1> state_error = goal_state - state;

    simulation_x.push_back(state(0));
    simulation_y.push_back(state(1));
    simulation_theta.push_back(state(2));
    simulation_velocity_l.push_back(state(3));
    simulation_velocity_r.push_back(state(4));

    error_x.push_back(state_error(0));
    error_y.push_back(state_error(1));
    error_theta.push_back(state_error(2));
    error_velocity_l.push_back(state_error(3));
    error_velocity_r.push_back(state_error(4));

    const ::Eigen::Matrix<double, 2, 5> K =
        finished_trajectory.GainForDistance(distance);

    const ::Eigen::Matrix<double, 2, 1> U_ff = trajectory.FFVoltage(distance);
    const ::Eigen::Matrix<double, 2, 1> U_fb = K * state_error;
    const ::Eigen::Matrix<double, 2, 1> U = U_ff + U_fb;
    state = RungeKuttaU(
        [&trajectory, &config](const ::Eigen::Matrix<double, 5, 1> &X,
                               const ::Eigen::Matrix<double, 2, 1> &U) {
          return ContinuousDynamics(trajectory.velocity_drivetrain().plant(),
                                    config.Tlr_to_la(), X, U);
        },
        state, U, kDtDouble);

    simulation_ul.push_back(U(0));
    simulation_ur.push_back(U(1));
  }

  if (absl::GetFlag(FLAGS_plot)) {
    matplotlibcpp::figure();
    matplotlibcpp::plot(plan_segment_center_distance, plan_type,
                        {{"label", "plan_type"}});
    matplotlibcpp::plot(distances, backward_plan, {{"label", "backward"}});
    matplotlibcpp::plot(distances, forward_plan, {{"label", "forward"}});
    matplotlibcpp::plot(distances, curvature_plan, {{"label", "lateral"}});
    matplotlibcpp::plot(distances, initial_plan, {{"label", "initial"}});
    matplotlibcpp::legend();

    matplotlibcpp::figure();
    matplotlibcpp::plot(length_plan_t, length_plan_x, {{"label", "x"}});
    matplotlibcpp::plot(length_plan_t, length_plan_v, {{"label", "v"}});
    matplotlibcpp::plot(length_plan_t, length_plan_a, {{"label", "a"}});
    matplotlibcpp::plot(length_plan_t, length_plan_vl, {{"label", "Vl"}});
    matplotlibcpp::plot(length_plan_t, length_plan_vr, {{"label", "Vr"}});
    matplotlibcpp::legend();

    matplotlibcpp::figure();
    matplotlibcpp::plot(length_plan_t, error_x, {{"label", "x error"}});
    matplotlibcpp::plot(length_plan_t, error_y, {{"label", "y error"}});
    matplotlibcpp::plot(length_plan_t, error_theta, {{"label", "theta error"}});
    matplotlibcpp::plot(length_plan_t, error_velocity_l,
                        {{"label", "velocityl error"}});
    matplotlibcpp::plot(length_plan_t, error_velocity_r,
                        {{"label", "velocityr error"}});
    matplotlibcpp::legend();

    matplotlibcpp::figure();
    matplotlibcpp::plot(spline_x, spline_y, {{"label", "spline"}});
    matplotlibcpp::plot(simulation_x, simulation_y, {{"label", "robot"}});
    matplotlibcpp::legend();

    matplotlibcpp::show();
  }
}

}  // namespace frc::control_loops::drivetrain

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  ::aos::network::OverrideTeamNumber(1868);
  ::frc::control_loops::drivetrain::Main();
  return 0;
}
