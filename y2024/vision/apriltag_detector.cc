
#include <string>

#include "absl/flags/flag.h"

#include "aos/init.h"
#include "frc/orin/gpu_apriltag.h"
#include "y2024/constants/constants_generated.h"
#include "y2024/vision/vision_util.h"

ABSL_FLAG(std::string, channel, "/camera", "Channel name");
ABSL_FLAG(std::string, config, "aos_config.json",
          "Path to the config file to use.");

void GpuApriltagDetector() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  frc::constants::WaitForConstants<y2024::Constants>(&config.message());

  aos::ShmEventLoop event_loop(&config.message());

  const frc::constants::ConstantsFetcher<y2024::Constants> calibration_data(
      &event_loop);

  CHECK(absl::GetFlag(FLAGS_channel).length() == 8);
  int camera_id = std::stoi(absl::GetFlag(FLAGS_channel).substr(7, 1));
  const frc::vision::calibration::CameraCalibration *calibration =
      y2024::vision::FindCameraCalibration(
          calibration_data.constants(),
          event_loop.node()->name()->string_view(), camera_id);

  frc::apriltag::ApriltagDetector detector(
      &event_loop, absl::GetFlag(FLAGS_channel), calibration);

  // TODO(austin): Figure out our core pinning strategy.
  // event_loop.SetRuntimeAffinity(aos::MakeCpusetFromCpus({5}));

  LOG(INFO) << "Setting scheduler priority";
  struct sched_param param;
  param.sched_priority = 21;
  PCHECK(sched_setscheduler(0, SCHED_FIFO, &param) == 0);

  LOG(INFO) << "Running event loop";
  // TODO(austin): Pre-warm it...
  event_loop.Run();
}  // namespace frc::apriltag

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  GpuApriltagDetector();

  return 0;
}
