#include "frc/control_loops/drivetrain/splinedrivetrain.h"

#include <numbers>

#include "Eigen/Dense"
#include "absl/log/absl_check.h"

#include "aos/json_to_flatbuffer.h"
#include "aos/realtime.h"
#include "aos/util/math.h"
#include "frc/control_loops/control_loops_generated.h"
#include "frc/control_loops/drivetrain/drivetrain_config.h"
#include "frc/control_loops/drivetrain/drivetrain_goal_generated.h"
#include "frc/control_loops/drivetrain/drivetrain_output_generated.h"
#include "frc/control_loops/drivetrain/drivetrain_status_generated.h"

namespace numbers = std::numbers;

namespace frc::control_loops::drivetrain {

SplineDrivetrain::SplineDrivetrain(const DrivetrainConfig<double> &dt_config)
    : dt_config_(dt_config),
      velocity_drivetrain_(
          std::make_shared<StateFeedbackLoop<2, 2, 2, double,
                                             StateFeedbackHybridPlant<2, 2, 2>,
                                             HybridKalman<2, 2, 2>>>(
              dt_config_.make_hybrid_drivetrain_velocity_loop())),
      current_xva_(Eigen::Vector3d::Zero()),
      next_xva_(Eigen::Vector3d::Zero()),
      next_U_(Eigen::Vector2d::Zero()),
      uncapped_U_(Eigen::Vector2d::Zero()),
      last_state_error_(Eigen::Matrix<double, 5, 1>::Zero()),
      last_U_components_(Eigen::Matrix<double, 2, 5>::Zero()),
      last_U_ff_(Eigen::Vector2d::Zero()) {}

void SplineDrivetrain::ScaleCapU(Eigen::Matrix<double, 2, 1> *U) {
  output_was_capped_ =
      ::std::abs((*U)(0, 0)) > 12.0 || ::std::abs((*U)(1, 0)) > 12.0;

  if (output_was_capped_) {
    *U *= 12.0 / U->lpNorm<Eigen::Infinity>();
  }
}

void SplineDrivetrain::SetGoal(
    const ::frc::control_loops::drivetrain::Goal *goal) {
  UpdateSplineHandles(goal->has_spline_handle()
                          ? std::make_optional<int>(goal->spline_handle())
                          : std::nullopt);
}

bool SplineDrivetrain::IsCurrentTrajectory(
    const fb::Trajectory *trajectory) const {
  const FinishedTrajectory *current = current_trajectory();
  return (current != nullptr &&
          current->spline_handle() == trajectory->handle());
}

bool SplineDrivetrain::HasTrajectory(const fb::Trajectory *trajectory) const {
  if (trajectory == nullptr) {
    return false;
  }
  for (size_t ii = 0; ii < trajectories_.size(); ++ii) {
    if (trajectories_[ii].spline_handle() == trajectory->handle()) {
      return true;
    }
  }
  return false;
}

void SplineDrivetrain::DeleteTrajectory(const fb::Trajectory *trajectory) {
  ABSL_CHECK(trajectory != nullptr);

  for (size_t ii = 0; ii < trajectories_.size(); ++ii) {
    if (trajectories_[ii].spline_handle() == trajectory->handle()) {
      trajectories_.erase(trajectories_.begin() + ii);
      return;
    }
  }

  ABSL_LOG(FATAL) << "Trying to remove unknown trajectory "
                  << trajectory->handle();
}

void SplineDrivetrain::AddTrajectory(const fb::Trajectory *trajectory) {
  ABSL_CHECK_LT(trajectories_.size(), trajectories_.capacity());
  trajectories_.emplace_back(&dt_config_, trajectory, velocity_drivetrain_);
  UpdateSplineHandles(commanded_spline_);
}

void SplineDrivetrain::DeleteCurrentSpline() {
  const FinishedTrajectory *const trajectory = current_trajectory();
  ABSL_CHECK(trajectory != nullptr);
  DeleteTrajectory(&trajectory->trajectory());
  executing_spline_ = false;
  commanded_spline_.reset();
  current_xva_.setZero();
}

void SplineDrivetrain::UpdateSplineHandles(
    std::optional<int> commanded_spline) {
  // If we are currently executing a spline and have received a change
  if (executing_spline_) {
    if (!commanded_spline) {
      // We've been told to stop executing a spline; remove it from our queue,
      // and clean up.
      DeleteCurrentSpline();
      return;
    } else {
      if (executing_spline_) {
        const FinishedTrajectory *const trajectory = current_trajectory();
        ABSL_CHECK(trajectory != nullptr);

        if (trajectory->spline_handle() != *commanded_spline) {
          // If we are executing a spline, and the handle has changed, garbage
          // collect the old spline.
          DeleteCurrentSpline();
        }
      }
    }
  }
  commanded_spline_ = commanded_spline;
  // We've now cleaned up the previous state; handle any new commands.
  if (!commanded_spline_) {
    return;
  }
  for (size_t ii = 0; ii < trajectories_.size(); ++ii) {
    if (trajectories_[ii].spline_handle() == *commanded_spline_) {
      executing_spline_ = true;
    }
  }
  // If we didn't find the commanded spline in the list of available splines,
  // that's fine; it just means, it hasn't been fully planned yet.
}

FinishedTrajectory *SplineDrivetrain::current_trajectory() {
  for (size_t ii = 0; ii < trajectories_.size(); ++ii) {
    if (trajectories_[ii].spline_handle() == *commanded_spline_) {
      return &trajectories_[ii];
    }
  }
  return nullptr;
}

const FinishedTrajectory *SplineDrivetrain::current_trajectory() const {
  for (size_t ii = 0; ii < trajectories_.size(); ++ii) {
    if (trajectories_[ii].spline_handle() == *commanded_spline_) {
      return &trajectories_[ii];
    }
  }
  return nullptr;
}

// TODO(alex): Hold position when done following the spline.
void SplineDrivetrain::Update(
    bool enable, const ::Eigen::Matrix<double, 5, 1> &state,
    const ::Eigen::Matrix<double, 2, 1> &voltage_error) {
  next_U_ = ::Eigen::Matrix<double, 2, 1>::Zero();
  enable_ = enable;
  if (enable && executing_spline_) {
    ::Eigen::Matrix<double, 2, 1> U_ff = ::Eigen::Matrix<double, 2, 1>::Zero();
    const FinishedTrajectory *const trajectory = current_trajectory();
    ABSL_CHECK(trajectory != nullptr);
    if (!IsAtEnd() && executing_spline_) {
      // TODO(alex): It takes about a cycle for the outputs to propagate to the
      // motors. Consider delaying the output by a cycle.
      U_ff = trajectory->FFVoltage(current_xva_(0));
    }

    const double current_distance = current_xva_(0);
    ::Eigen::Matrix<double, 2, 5> K =
        trajectory->GainForDistance(current_distance);
    ::Eigen::Matrix<double, 5, 1> goal_state = CurrentGoalState();
    const bool backwards = trajectory->drive_spline_backwards();
    if (backwards) {
      ::Eigen::Matrix<double, 2, 1> swapU(U_ff(1, 0), U_ff(0, 0));
      U_ff = -swapU;
      goal_state(2, 0) += numbers::pi;
      double left_goal = goal_state(3, 0);
      double right_goal = goal_state(4, 0);
      goal_state(3, 0) = -right_goal;
      goal_state(4, 0) = -left_goal;
    }
    const Eigen::Matrix<double, 5, 1> relative_goal =
        trajectory->StateToPathRelativeState(current_distance, goal_state,
                                             backwards);
    const Eigen::Matrix<double, 5, 1> relative_state =
        trajectory->StateToPathRelativeState(current_distance, state,
                                             backwards);
    Eigen::Matrix<double, 5, 1> state_error = relative_goal - relative_state;
    state_error(2, 0) = ::aos::math::NormalizeAngle(state_error(2, 0));
    ::Eigen::Matrix<double, 2, 1> U_fb = K * state_error;
    last_U_components_ = K * Eigen::DiagonalMatrix<double, 5>(state_error);

    if (backwards) {
      Eigen::Matrix<double, 2, 1> swapU(U_fb(1), U_fb(0));
      U_fb = -swapU;
    }

    ::Eigen::Matrix<double, 2, 1> xv_state = current_xva_.block<2, 1>(0, 0);
    next_xva_ = trajectory->GetNextXVA(dt_config_.dt, &xv_state);
    next_U_ = U_ff + U_fb - voltage_error;
    uncapped_U_ = next_U_;
    ScaleCapU(&next_U_);
    last_state_error_ = state_error;
    last_U_ff_ = U_ff;
  }
}

void SplineDrivetrain::SetOutput(
    ::frc::control_loops::drivetrain::OutputT *output) {
  if (!output) {
    return;
  }
  if (executing_spline_) {
    if (!IsAtEnd()) {
      output->left_voltage = next_U_(0);
      output->right_voltage = next_U_(1);
      current_xva_ = next_xva_;
    }
  }
  output->left_voltage = next_U_(0);
  output->right_voltage = next_U_(1);
  if (IsAtEnd()) {
    output->left_voltage = 0.0;
    output->right_voltage = 0.0;
  }
}

void SplineDrivetrain::PopulateStatus(drivetrain::StatusStatic *status) const {
  if (status && enable_) {
    status->set_uncapped_left_voltage(uncapped_U_(0));
    status->set_uncapped_right_voltage(uncapped_U_(1));
    status->set_robot_speed(current_xva_(1));
    status->set_output_was_capped(output_was_capped_);
  }
}

namespace {
template <typename Matrix>
void PopulateSplineState(const Matrix &state, SplineStateStatic *spline_state) {
  spline_state->set_x(state(0));
  spline_state->set_y(state(1));
  spline_state->set_theta(::aos::math::NormalizeAngle(state(2)));
  spline_state->set_left_velocity(state(3));
  spline_state->set_right_velocity(state(4));
}
}  // namespace

void SplineDrivetrain::PopulateTrajectoryLogging(
    drivetrain::TrajectoryLoggingStatic *trajectory_logging) const {
  auto *available_splines = trajectory_logging->add_available_splines();
  ABSL_CHECK(available_splines->reserve(trajectories_.size()));
  for (const auto &trajectory : trajectories_) {
    ABSL_CHECK(available_splines->emplace_back(trajectory.spline_handle()));
  }

  if (executing_spline_) {
    ::Eigen::Matrix<double, 5, 1> goal_state = CurrentGoalState();
    trajectory_logging->set_x(goal_state(0));
    trajectory_logging->set_y(goal_state(1));
    const FinishedTrajectory *const trajectory = current_trajectory();
    ABSL_CHECK(trajectory != nullptr);
    if (trajectory->drive_spline_backwards()) {
      trajectory_logging->set_left_velocity(-goal_state(4));
      trajectory_logging->set_right_velocity(-goal_state(3));
      trajectory_logging->set_theta(
          ::aos::math::NormalizeAngle(goal_state(2) + numbers::pi));
    } else {
      trajectory_logging->set_theta(::aos::math::NormalizeAngle(goal_state(2)));
      trajectory_logging->set_left_velocity(goal_state(3));
      trajectory_logging->set_right_velocity(goal_state(4));
    }
    PopulateSplineState(last_state_error_,
                        trajectory_logging->add_state_error());
    PopulateSplineState(last_U_components_.row(0),
                        trajectory_logging->add_left_voltage_components());
    PopulateSplineState(last_U_components_.row(1),
                        trajectory_logging->add_right_voltage_components());
    trajectory_logging->set_left_ff_voltage(last_U_ff_(0));
    trajectory_logging->set_right_ff_voltage(last_U_ff_(1));
  }
  trajectory_logging->set_is_executing(!IsAtEnd() && executing_spline_);
  trajectory_logging->set_is_executed(executing_spline_ && IsAtEnd());
  if (commanded_spline_) {
    trajectory_logging->set_goal_spline_handle(*commanded_spline_);
    if (executing_spline_) {
      trajectory_logging->set_current_spline_idx(*commanded_spline_);
    }
  }
  if (executing_spline_) {
    const FinishedTrajectory *const trajectory = current_trajectory();
    ABSL_CHECK(trajectory != nullptr);
    trajectory_logging->set_distance_remaining(trajectory->length() -
                                               current_xva_.x());
  } else {
    trajectory_logging->set_distance_remaining(0.0);
  }
  trajectory_logging->set_distance_traveled(executing_spline_ ? current_xva_.x()
                                                              : 0.0);
}

}  // namespace frc::control_loops::drivetrain
