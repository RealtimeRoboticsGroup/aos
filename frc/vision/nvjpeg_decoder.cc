// NVJPG hardware JPEG decoder service for Jetson Orin; see
// nvjpeg_decoder_service.h.

#include "absl/flags/flag.h"

#include "aos/configuration.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frc/vision/nvjpeg_decoder_service.h"

ABSL_FLAG(std::string, config, "aos_config.json",
          "File path of aos configuration");

namespace frc::vision {

int Main() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  aos::ShmEventLoop event_loop(&config.message());

  event_loop.SetRuntimeRealtimePriority(5);

  NvjpegDecoder nvjpeg_decoder(&event_loop);

  event_loop.Run();

  return 0;
}

}  // namespace frc::vision

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  return frc::vision::Main();
}
