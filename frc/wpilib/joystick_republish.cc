#include <sys/resource.h>
#include <sys/time.h>

#include "absl/flags/flag.h"

#include "aos/configuration.h"
#include "aos/events/shm_event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "aos/init.h"
#include "frc/input/joystick_state_generated.h"

ABSL_FLAG(std::string, config, "aos_config.json", "Config file to use.");

int main(int argc, char *argv[]) {
  aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));
  aos::ShmEventLoop event_loop(&config.message());

  aos::Sender<frc::JoystickState> sender(
      event_loop.MakeSender<frc::JoystickState>("/imu/frc"));

  event_loop.MakeWatcher(
      "/roborio/frc", [&](const frc::JoystickState &joystick_state) {
        auto builder = sender.MakeBuilder();
        flatbuffers::Offset<frc::JoystickState> state_fbs =
            aos::CopyFlatBuffer(&joystick_state, builder.fbb());
        builder.CheckOk(builder.Send(state_fbs));
      });

  event_loop.Run();
  return 0;
}
