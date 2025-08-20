#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frc/control_loops/swerve/goal_solver.h"
#include "y2024_swerve/control_loops/parameters.h"

using frc::control_loops::swerve::GoalSolver;

int main(int argc, char **argv) {
  ::aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("aos_config.json");

  ::aos::ShmEventLoop event_loop(&config.message());

  GoalSolver goal_solver(
      &event_loop, y2024_swerve::control_loops::MakeSwerveParameters<float>());

  event_loop.Run();

  return 0;
}
