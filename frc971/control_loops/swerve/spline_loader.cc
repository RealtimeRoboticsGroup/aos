#include "absl/flags/flag.h"

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "frc971/control_loops/swerve/autonomous_pd_loop.h"

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(
          "/home/max/Documents/971-Robot-Code/auto_pd_loop/bazel-bin/"
          "y2024_swerve/aos_config.stripped.json");

  aos::ShmEventLoop event_loop(&config.message());

  frc971::control_loops::swerve::AutonomousPDLoop pd_loop(
      &event_loop, "/home/max/Downloads/solutions.json(2).json");

  event_loop.Run();
}
