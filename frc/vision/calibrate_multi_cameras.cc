#include <numeric>

#include "absl/flags/flag.h"

#include "aos/configuration.h"
#include "aos/events/logging/log_reader.h"
#include "aos/init.h"
#include "frc/constants/constants_sender_lib.h"
#include "frc/vision/calibrate_multi_cameras_lib.h"
#include "frc/vision/swerve_localizer/simulated_constants_sender_lib.h"
#include "frc/vision/vision_util.h"
#include "frc/vision/vision_util_lib.h"

ABSL_FLAG(std::string, config, "",
          "If set, override the log's config file with this one.");
ABSL_FLAG(std::string, constants_path, "frc/vision/constants.json",
          "Path to the constant file");
ABSL_FLAG(double, max_pose_error_ratio, 0.4,
          "Throw out target poses with a higher pose error ratio than this");

ABSL_DECLARE_FLAG(int32_t, min_target_id);
ABSL_DECLARE_FLAG(int32_t, max_target_id);
ABSL_DECLARE_FLAG(double, outlier_std_devs);

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  frc::vision::NodeList node_list(frc::vision::CreateNodeList());

  std::map<std::string, int> ordering_map(
      frc::vision::CreateOrderingMap(node_list));

  std::optional<aos::FlatbufferDetachedBuffer<aos::Configuration>> config =
      (absl::GetFlag(FLAGS_config).empty()
           ? std::nullopt
           : std::make_optional(
                 aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config))));

  // open logfiles
  aos::logger::LogReader reader(
      aos::logger::SortParts(aos::logger::FindLogs(argc, argv)),
      config.has_value() ? &config->message() : nullptr);

  reader.RemapLoggedChannel("/constants", "frc.vision.CameraConstants");
  reader.Register();

  frc::vision::swerve_localizer::SimulatedConstantsSender(
      reader.event_loop_factory(), absl::GetFlag(FLAGS_team_number),
      absl::GetFlag(FLAGS_constants_path));

  auto find_calibration = [](aos::EventLoop *const event_loop,
                             std::string node_name, int camera_number)
      -> const frc::vision::calibration::CameraCalibration * {
    frc::constants::ConstantsFetcher<frc::vision::CameraConstants>
        constants_fetcher(event_loop);
    // Get the calibration for this orin/camera pair
    return frc::vision::FindCameraCalibration(constants_fetcher.constants(),
                                              node_name, camera_number);
  };

  frc::vision::ExtrinsicsMain(node_list, find_calibration,
                              frc::vision::kOrinColors, &reader, ordering_map);
}
