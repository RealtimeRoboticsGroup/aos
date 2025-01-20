#include "frc/control_loops/drivetrain/trajectory_generator.h"

namespace frc::control_loops::drivetrain {

TrajectoryGenerator::TrajectoryGenerator(aos::EventLoop *event_loop,
                                         const DrivetrainConfig<double> &config)
    : event_loop_(event_loop),
      dt_config_(config),
      trajectory_sender_(
          event_loop_->MakeSender<fb::Trajectory>("/drivetrain")),
      spline_goal_fetcher_(event_loop->MakeFetcher<SplineGoal>("/drivetrain")) {
  event_loop_->MakeWatcher("/drivetrain", [this](const SplineGoal &goal) {
    HandleSplineGoal(goal);
  });
  event_loop_->OnRun([this]() {
    if (spline_goal_fetcher_.Fetch()) {
      HandleSplineGoal(*spline_goal_fetcher_.get());
    }
  });
}

void TrajectoryGenerator::HandleSplineGoal(const SplineGoal &goal) {
  Trajectory trajectory(goal, &dt_config_);
  trajectory.Plan();

  aos::Sender<fb::Trajectory>::Builder builder =
      trajectory_sender_.MakeBuilder();

  CHECK_EQ(builder.Send(trajectory.Serialize(builder.fbb())),
           aos::RawSender::Error::kOk);
}

}  // namespace frc::control_loops::drivetrain
