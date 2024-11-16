#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frc971/constants/constants_sender_lib.h"
#include "frc971/control_loops/swerve/swerve_control_loops.h"
#include "y2024_swerve/constants/constants_generated.h"
#include "y2024_swerve/control_loops/parameters.h"

using frc971::control_loops::swerve::LinearVelocityController;
using frc971::control_loops::swerve::SwerveControlLoops;

LinearVelocityController::ControllerWeights CreateWeights(
    const y2024_swerve::VelocityControllerWeights *weights) {
  return LinearVelocityController::ControllerWeights{
      .Thetas_q = weights->thetas_q(),
      .Omegas_q = weights->omegas_q(),
      .Is_r = weights->is_r(),
      .Is_t = weights->is_t(),
      .Vel_q = weights->vel_q(),
      .Theta_q = weights->theta_q(),
      .Omega_q = weights->omega_q()};
}

int main(int argc, char **argv) {
  ::aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("aos_config.json");

  frc971::constants::WaitForConstants<y2024_swerve::Constants>(
      &config.message());

  ::aos::ShmEventLoop event_loop(&config.message());

  frc971::constants::ConstantsFetcher<y2024_swerve::Constants> constants(
      &event_loop);

  SwerveControlLoops swerve_control_loops(
      &event_loop, constants.constants().common()->rotation(),
      constants.constants().robot()->swerve_zeroing(),
      y2024_swerve::control_loops::MakeSwerveParameters<float>(),
      CreateWeights(constants.constants().common()->weights()), "/drivetrain");

  event_loop.Run();

  return 0;
}
