#include "absl/flags/flag.h"

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frc/vision/v4l2_reader.h"

ABSL_FLAG(std::string, config, "aos_config.json",
          "Path to the config file to use.");
ABSL_FLAG(std::string, channel, "/camera", "What camera channel to use.");
ABSL_FLAG(std::string, viddevice, "/dev/video0", "What video device to use.");
ABSL_FLAG(uint32_t, exposure, 100,
          "Exposure time, in 100us increments; 0 implies auto exposure");

namespace frc::vision {

void CameraReaderMain() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  aos::ShmEventLoop event_loop(&config.message());

  // First, log the data for future reference.
  MjpegV4L2Reader v4l2_reader(&event_loop, event_loop.epoll(),
                              absl::GetFlag(FLAGS_viddevice),
                              absl::GetFlag(FLAGS_channel));
  const uint32_t exposure = absl::GetFlag(FLAGS_exposure);
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

}  // namespace frc::vision

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  frc::vision::CameraReaderMain();
}
