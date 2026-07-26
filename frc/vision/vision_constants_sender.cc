#include "absl/flags/flag.h"
#include "absl/log/log.h"
#include "absl/strings/ascii.h"

#include "aos/configuration.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/util/file.h"
#include "frc/constants/constants_sender_lib.h"
#include "frc/vision/camera_constants_generated.h"
#include "frc/vision/camera_constants_list_generated.h"

ABSL_FLAG(std::string, config, "aos_config.json", "Path to the AOS config.");
ABSL_FLAG(std::string, constants_path, "constants.json",
          "Path to the constant file");

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  const auto config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));
  aos::ShmEventLoop event_loop(&config.message());

  std::optional<std::string> robotname =
      aos::util::MaybeReadFileToString("robotname");
  if (!robotname.has_value()) {
    LOG(ERROR) << "Failed to read robotname file 'robotname': ";
    return 1;
  }

  *robotname = std::string(absl::StripAsciiWhitespace(*robotname));
  if (robotname->empty()) {
    LOG(ERROR) << "robotname file was empty/whitespace.";
    return 1;
  }

  frc::constants::NameConstantSender<frc::vision::CameraConstants,
                                     frc::vision::CameraConstantsList>
      constants_sender(&event_loop, absl::GetFlag(FLAGS_constants_path),
                       *robotname);
  return 0;
}
