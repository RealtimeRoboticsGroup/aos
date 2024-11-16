#include "autonomous_pd_loop.h"

#include "frc971/control_loops/swerve/autonomous_pd_loop.h"
#include "frc971/control_loops/swerve/swerve_path_generated.h"

using frc971::control_loops::swerve::AutonomousPDLoop;

AutonomousPDLoop::AutonomousPDLoop(
    aos::EventLoop *event_loop, std::string_view spline_path,
    std::vector<std::pair<double, std::function<void()>>> callbacks)
    : path_index_(std::nullopt),
      swerve_goal_sender_(
          event_loop->MakeSender<frc971::control_loops::swerve::GoalStatic>(
              "/drivetrain")),
      joystick_state_fetcher_(
          event_loop->MakeFetcher<aos::JoystickState>("/aos")),
      callbacks_(callbacks),
      event_loop_(event_loop) {
  auto spline_fbs =
      aos::JsonFileToFlatbuffer<frc971::control_loops::swerve::SwervePath>(
          spline_path);

  // Eigenify
  std::vector<AutonomousPDLoop::SwervePathPoint> path;
  for (size_t i = 0; i < spline_fbs.message().times()->size(); i++) {
    auto position = spline_fbs.message().positions()->Get(i);
    auto velocity = spline_fbs.message().velocities()->Get(i);
    auto acceleration = spline_fbs.message().accelerations()->Get(i);
    auto scoring = spline_fbs.message().scoring()->Get(i);
    auto time = spline_fbs.message().times()->Get(i);
    SwervePathPoint point = {
        Eigen::Matrix<double, 3, 1>(position->x(), position->y(),
                                    position->theta()),
        Eigen::Matrix<double, 3, 1>(velocity->x(), velocity->y(),
                                    velocity->theta()),
        Eigen::Matrix<double, 3, 1>(acceleration->x(), acceleration->y(),
                                    acceleration->theta()),
        static_cast<bool>(scoring), time};
    path.emplace_back(point);
  }

  path_ =
      std::make_shared<std::vector<AutonomousPDLoop::SwervePathPoint>>(path);

  const auto timer = event_loop_->AddTimer([this]() {
    if (path_index_) {
      size_t index = path_index_.value();

      Iterate(&path_->at(index));

      if (index >= (path_->size() - 1)) {
        path_index_ = std::nullopt;
        completed_ = true;
      } else {
        path_index_ = index + 1;
      }

      if (joystick_state_fetcher_.Fetch()) {
        auto joystick_state = joystick_state_fetcher_.get();
        if (!(joystick_state->autonomous() && joystick_state->enabled())) {
          path_index_ = std::nullopt;
          completed_ = true;
        }
      }
    } else if (!completed_) {
      if (joystick_state_fetcher_.Fetch()) {
        auto joystick_state = joystick_state_fetcher_.get();
        if (joystick_state->autonomous() && joystick_state->enabled()) {
          path_index_ = 0;
        }
      }
    }
  });
  event_loop->OnRun([this, timer, event_loop]() {
    timer->Schedule(event_loop->monotonic_now(), std::chrono::milliseconds(5));
  });
};

void AutonomousPDLoop::Iterate(SwervePathPoint *path_point) {
  auto builder = swerve_goal_sender_.MakeStaticBuilder();

  auto joystick_goal = builder->add_joystick_goal();
  joystick_goal->set_vx(path_point->velocity(0));
  joystick_goal->set_vy(path_point->velocity(1));
  joystick_goal->set_omega(path_point->velocity(2));

  auto time = path_point->time;

  for (auto callback : callbacks_) {
    if (time >= callback.first) {
      (callback.second)();
      callback.first = std::numeric_limits<double>::max();
    }
  }

  builder.CheckOk(builder.Send());
};

bool AutonomousPDLoop::Completed() { return completed_; }
