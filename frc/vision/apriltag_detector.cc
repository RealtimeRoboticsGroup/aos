#include <string>

#include "absl/flags/flag.h"

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frc/orin/gpu_apriltag.h"
#include "frc/vision/camera_constants_generated.h"

ABSL_FLAG(std::string, channel, "/camera", "Channel name");
ABSL_FLAG(std::string, config, "aos_config.json",
          "Path to the config file to use.");

ABSL_FLAG(uint32_t, width, 1600, "Width of the image in pixels.");
ABSL_FLAG(uint32_t, height, 1304, "Height of the image in pixels.");

namespace frc::vision {

const calibration::CameraCalibration *FindCameraCalibration(
    const CameraConstants &calibration_data, std::string_view node_name,
    int camera_number) {
  CHECK(calibration_data.has_calibration());
  for (const calibration::CameraCalibration *candidate :
       *calibration_data.calibration()) {
    if (candidate->node_name()->string_view() != node_name ||
        candidate->camera_number() != camera_number) {
      continue;
    }
    return candidate;
  }
  LOG(FATAL) << ": Failed to find camera calibration for " << node_name
             << " and camera number " << camera_number;
}

void GpuApriltagDetector() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  frc::constants::WaitForConstants<CameraConstants>(&config.message());

  aos::ShmEventLoop event_loop(&config.message());

  const frc::constants::ConstantsFetcher<CameraConstants> calibration_data(
      &event_loop);

  CHECK_GE(absl::GetFlag(FLAGS_channel).length(), 8u)
      << ": Channel name must be of the form /cameraX*";
  int camera_id = std::stoi(absl::GetFlag(FLAGS_channel).substr(7, 1));
  const frc::vision::calibration::CameraCalibration *calibration =
      FindCameraCalibration(calibration_data.constants(),
                            event_loop.node()->name()->string_view(),
                            camera_id);

  frc::apriltag::ApriltagDetector detector(
      &event_loop, absl::GetFlag(FLAGS_channel), calibration,
      absl::GetFlag(FLAGS_width), absl::GetFlag(FLAGS_height));

  detector.PinMemory(&event_loop);

  // TODO(austin): Figure out our core pinning strategy.
  // event_loop.SetRuntimeAffinity(aos::MakeCpusetFromCpus({5}));

  LOG(INFO) << "Setting scheduler priority";
  struct sched_param param;
  param.sched_priority = 21;
  PCHECK(sched_setscheduler(0, SCHED_FIFO, &param) == 0);

  LOG(INFO) << "Running event loop";
  // TODO(austin): Pre-warm it...
  event_loop.Run();
}

}  // namespace frc::vision

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  frc::vision::GpuApriltagDetector();

  return 0;
}
