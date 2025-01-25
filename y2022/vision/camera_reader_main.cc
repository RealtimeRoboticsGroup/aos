#include <math.h>

#include "absl/flags/flag.h"

#include "aos/events/shm_event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "aos/init.h"
#include "aos/network/team_number.h"
#include "frc/vision/v4l2_reader.h"
#include "frc/vision/vision_generated.h"

// config used to allow running camera_reader independently.  E.g.,
// bazel run //y2022/vision:camera_reader -- --config y2022/aos_config.json
//   --override_hostname pi-7971-1  --ignore_timestamps true
ABSL_FLAG(bool, use_outdoors, true, "true if outdoors");
ABSL_FLAG(std::string, config, "aos_config.json",
          "Path to the config file to use.");
ABSL_FLAG(std::string, channel, "0", "What camera channel to use.");
ABSL_FLAG(std::string, viddevice, "0", "What video device to use.");
ABSL_FLAG(double, duty_cycle, 0.65, "Duty cycle of the LEDs");
ABSL_FLAG(uint32_t, exposure, 3,
          "Exposure time, in 100us increments; 0 implies auto exposure");
ABSL_FLAG(uint32_t, outdoors_exposure, 2,
          "Exposure time when using --use_outdoors, in 100us increments; 0 "
          "implies auto exposure");

namespace y2022::vision {
namespace {

using namespace frc::vision;

void CameraReaderMain() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  aos::ShmEventLoop event_loop(&config.message());

  // First, log the data for future reference.
  V4L2Reader v4l2_reader(&event_loop, absl::GetFlag(FLAGS_viddevice),
                         absl::GetFlag(FLAGS_channel));
  const uint32_t exposure = (absl::GetFlag(FLAGS_use_outdoors)
                                 ? absl::GetFlag(FLAGS_outdoors_exposure)
                                 : absl::GetFlag(FLAGS_exposure));
  if (exposure > 0) {
    LOG(INFO) << "Setting camera to Manual Exposure mode with exposure = "
              << exposure << " or " << static_cast<double>(exposure) / 10.0
              << " ms";
    v4l2_reader.SetExposure(exposure);
  } else {
    LOG(INFO) << "Setting camera to use Auto Exposure";
    v4l2_reader.UseAutoExposure();
  }

  event_loop.Run();
}

}  // namespace
}  // namespace y2022::vision

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  y2022::vision::CameraReaderMain();
}
