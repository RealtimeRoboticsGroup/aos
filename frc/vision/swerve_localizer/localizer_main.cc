#include "absl/flags/flag.h"

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frc/constants/constants_sender_lib.h"
#include "frc/vision/swerve_localizer/localizer.h"

ABSL_FLAG(std::string, config, "aos_config.json",
          "Path to the config file to use.");

int main(int argc, char *argv[]) {
  aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  frc::constants::WaitForConstants<frc::vision::CameraConstants>(
      &config.message());
  frc::constants::WaitForConstants<frc::vision::TargetMap>(&config.message());

  aos::ShmEventLoop event_loop(&config.message());
  frc::vision::swerve_localizer::Localizer localizer(&event_loop);

  event_loop.Run();

  return 0;
}
