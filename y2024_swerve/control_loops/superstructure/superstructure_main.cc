#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "y2024_swerve/control_loops/superstructure/superstructure.h"

using y2024_swerve::control_loops::superstructure::Superstructure;

int main(int argc, char **argv) {
  ::aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("aos_config.json");

  ::aos::ShmEventLoop event_loop(&config.message());

  std::shared_ptr<const y2024_swerve::constants::Values> values =
      std::make_shared<const y2024_swerve::constants::Values>(
          y2024_swerve::constants::MakeValues());
  Superstructure superstructure(&event_loop, values);

  event_loop.Run();

  return 0;
}