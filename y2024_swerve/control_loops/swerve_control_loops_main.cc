#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frc/constants/constants_sender_lib.h"
#include "frc/control_loops/swerve/swerve_control_loops.h"
#include "y2024_swerve/constants/constants_generated.h"
#include "y2024_swerve/control_loops/parameters.h"

using frc::control_loops::swerve::SwerveControlLoops;

int main(int argc, char **argv) {
  ::aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("aos_config.json");

  frc::constants::WaitForConstants<y2024_swerve::Constants>(
      &config.message());

  ::aos::ShmEventLoop event_loop(&config.message());

  frc::constants::ConstantsFetcher<y2024_swerve::Constants> constants(
      &event_loop);

  SwerveControlLoops swerve_control_loops(
      &event_loop, constants.constants().common()->rotation(),
      constants.constants().robot()->swerve_zeroing(),
      y2024_swerve::control_loops::MakeSwerveParameters<float>(),
      "/drivetrain");

  event_loop.Run();

  return 0;
}
