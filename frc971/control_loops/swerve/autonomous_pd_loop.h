#ifndef FRC971_CONTROL_LOOPS_SWERVE
#define FRC971_CONTROL_LOOPS_SWERVE

#include <Eigen/Dense>

#include "aos/events/event_loop.h"
#include "frc971/control_loops/swerve/swerve_drivetrain_goal_static.h"
#include "frc971/control_loops/swerve/swerve_drivetrain_status_generated.h"
#include "frc971/input/joystick_state_generated.h"

namespace frc971::control_loops::swerve {

class AutonomousPDLoop {
 public:
  struct SwervePathPoint {
    Eigen::Matrix<double, 3, 1> position;
    Eigen::Matrix<double, 3, 1> velocity;
    Eigen::Matrix<double, 3, 1> acceleration;
    bool scoring;

    double time;
  };

  AutonomousPDLoop(
      aos::EventLoop *event_loop, std::string_view spline_path,
      std::vector<std::pair<double, std::function<void()>>> callbacks);
  void Iterate(SwervePathPoint *path_point);

  bool Completed();

 private:
  // nullopt when we aren't running, when we're running its the index of the
  // running path
  std::optional<size_t> path_index_;

  std::shared_ptr<std::vector<SwervePathPoint>> path_;

  aos::Sender<GoalStatic> swerve_goal_sender_;
  aos::Fetcher<aos::JoystickState> joystick_state_fetcher_;

  std::vector<std::pair<double, std::function<void()>>> callbacks_;

  bool completed_;

  aos::EventLoop *event_loop_;
};

}  // namespace frc971::control_loops::swerve

#endif  // FRC971_CONTROL_LOOPS_SWERVE
