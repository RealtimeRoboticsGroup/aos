#ifndef FRC_CONTROL_LOOPS_DRIVETRAIN_SPLINEDRIVETRAIN_H_
#define FRC_CONTROL_LOOPS_DRIVETRAIN_SPLINEDRIVETRAIN_H_

#include <atomic>
#include <thread>

#include "Eigen/Dense"

#include "aos/condition.h"
#include "aos/mutex/mutex.h"
#include "frc/control_loops/control_loops_generated.h"
#include "frc/control_loops/drivetrain/distance_spline.h"
#include "frc/control_loops/drivetrain/drivetrain_config.h"
#include "frc/control_loops/drivetrain/drivetrain_goal_generated.h"
#include "frc/control_loops/drivetrain/drivetrain_output_generated.h"
#include "frc/control_loops/drivetrain/drivetrain_status_generated.h"
#include "frc/control_loops/drivetrain/spline.h"
#include "frc/control_loops/drivetrain/trajectory.h"

namespace frc::control_loops::drivetrain {

class SplineDrivetrain {
 public:
  static constexpr size_t kMaxTrajectories = 6;
  SplineDrivetrain(const DrivetrainConfig<double> &dt_config);

  void SetGoal(const ::frc::control_loops::drivetrain::Goal *goal);
  // Note that the user maintains ownership of the trajectory flatbuffer for the
  // entire time; once AddTrajectory() is called, the trajectory pointer must
  // stay valid until the spline has finished executing and HasTrajectory()
  // returns false.
  bool HasTrajectory(const fb::Trajectory *trajectory) const;
  void AddTrajectory(const fb::Trajectory *trajectory);
  bool IsCurrentTrajectory(const fb::Trajectory *trajectory) const;
  void DeleteTrajectory(const fb::Trajectory *trajectory);

  void Update(bool enabled, const ::Eigen::Matrix<double, 5, 1> &state,
              const ::Eigen::Matrix<double, 2, 1> &voltage_error);

  void SetOutput(::frc::control_loops::drivetrain::OutputT *output);

  flatbuffers::Offset<TrajectoryLogging> MakeTrajectoryLogging(
      aos::Sender<drivetrain::Status>::Builder *builder) const;
  flatbuffers::Offset<TrajectoryLogging> MakeTrajectoryLogging(
      flatbuffers::FlatBufferBuilder *builder) const;
  void PopulateStatus(drivetrain::Status::Builder *status) const;

  // Accessor for the current goal state, pretty much only present for debugging
  // purposes.
  ::Eigen::Matrix<double, 5, 1> CurrentGoalState() const {
    if (executing_spline_) {
      const FinishedTrajectory *finished = current_trajectory();
      CHECK(finished != nullptr);
      return finished->GoalState(current_xva_(0), current_xva_(1));
    } else {
      return ::Eigen::Matrix<double, 5, 1>::Zero();
    }
  }

  bool IsAtEnd() const {
    if (!executing_spline_) {
      return true;
    }

    const FinishedTrajectory *finished = current_trajectory();
    CHECK(finished != nullptr);
    return finished->is_at_end(current_xva_.block<2, 1>(0, 0));
  }

  size_t trajectory_count() const { return trajectories_.size(); }

  // Returns true if the splinedrivetrain is enabled.
  bool enable() const { return enable_; }

 private:
  void ScaleCapU(Eigen::Matrix<double, 2, 1> *U);

  // This is called to update the internal state for managing all the splines.
  // Calling it redundantly does not cause any issues. It checks the value of
  // commanded_spline to determine whether we are being commanded to run a
  // spline, and if there is any trajectory in the list of trajectories matching
  // the command, we begin/continue executing that spline. If commanded_spline
  // is empty or has changed, we stop executing the previous trajectory and
  // remove it from trajectories_. Then, when the drivetrain code checks
  // HasTrajectory() for the old trajectory, it will return false and the
  // drivetrain can free up the fetcher to get the next trajectory.
  void UpdateSplineHandles(std::optional<int> commanded_spline);

  // Deletes the currently executing trajectory.
  void DeleteCurrentSpline();

  FinishedTrajectory *current_trajectory();
  const FinishedTrajectory *current_trajectory() const;

  const DrivetrainConfig<double> dt_config_;

  std::shared_ptr<
      StateFeedbackLoop<2, 2, 2, double, StateFeedbackHybridPlant<2, 2, 2>,
                        HybridKalman<2, 2, 2>>>
      velocity_drivetrain_;

  bool executing_spline_ = false;

  // TODO(james): Sort out construction to avoid so much dynamic memory
  // allocation...
  aos::SizedArray<FinishedTrajectory, kMaxTrajectories> trajectories_;

  std::optional<int> commanded_spline_;

  // State required to compute the next iteration's output.
  ::Eigen::Matrix<double, 3, 1> current_xva_, next_xva_;
  ::Eigen::Matrix<double, 2, 1> next_U_;

  // Information used for status message.
  ::Eigen::Matrix<double, 2, 1> uncapped_U_;
  ::Eigen::Matrix<double, 5, 1> last_state_error_;
  ::Eigen::Matrix<double, 2, 5> last_U_components_;
  ::Eigen::Matrix<double, 2, 1> last_U_ff_;
  bool enable_ = false;
  bool output_was_capped_ = false;
};

}  // namespace frc::control_loops::drivetrain

#endif  // FRC_CONTROL_LOOPS_DRIVETRAIN_SPLINEDRIVETRAIN_H_
