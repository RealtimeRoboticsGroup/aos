#include "absl/flags/flag.h"

#include "aos/configuration.h"
#include "aos/events/logging/log_reader.h"
#include "aos/events/logging/log_writer.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/network/team_number.h"
#include "aos/network/web_proxy.h"
#include "aos/util/file.h"
#include "aos/util/mcap_logger.h"
#include "aos/util/simulation_logger.h"
#include "frc/constants/constants_sender_lib.h"
#include "frc/vision/swerve_localizer/localizer.h"
#include "frc/vision/swerve_localizer/simulated_constants_sender_lib.h"

ABSL_FLAG(std::string, config, "frc/vision/aos_config.json",
          "Name of the config file to replay using.");
ABSL_FLAG(std::string, calibration, "frc/vision/constants.json",
          "Name of the config file to replay using.");
ABSL_FLAG(std::optional<std::string>, robot_name, std::nullopt,
          "Name of the robot to use the constants from.");
ABSL_FLAG(bool, override_config, false,
          "If set, override the logged config with --config.");
ABSL_FLAG(bool, rerun_realtime, false, "If set, replay at a realtime rate.");
ABSL_FLAG(bool, override_constants, false,
          "If set, override the logged constants with --calibration.");
ABSL_FLAG(int32_t, team, 4646, "Team number to use for logfile replay.");
ABSL_FLAG(std::string, output_folder, "/tmp/replayed",
          "Name of the folder to write replayed logs to.");
ABSL_FLAG(std::string, field_map_path,
          "../frc2026_field_map_welded/file/frc2026.fmap",
          "Path to the field map file");

ABSL_FLAG(std::string, data_dir,
          "frc/vision/swerve_localizer/www/www_directory",
          "Path to the field html page.");
ABSL_FLAG(int32_t, buffer_size, -1,
          "-1 if infinite, in # of messages / channel.");
ABSL_FLAG(bool, rerun, true, "If true, rerun the localizer.");

using frc::vision::swerve_localizer::SimulatedConstantsSender;
using frc::vision::swerve_localizer::SimulatedFieldMapSender;

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  aos::network::OverrideTeamNumber(absl::GetFlag(FLAGS_team));

  if (absl::GetFlag(FLAGS_override_constants) &&
      !absl::GetFlag(FLAGS_robot_name).has_value()) {
    LOG(ERROR) << "Must supply a --robot_name if using --override_constants.";
    return 1;
  }

  const aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  // sort logfiles
  const std::vector<aos::logger::LogFile> logfiles =
      aos::logger::SortParts(aos::logger::FindLogs(argc, argv));

  // open logfiles
  aos::logger::LogReader reader(logfiles, absl::GetFlag(FLAGS_override_config)
                                              ? &config.message()
                                              : nullptr);

  if (absl::GetFlag(FLAGS_rerun)) {
    reader.RemapLoggedChannel("/localizer",
                              "frc.vision.swerve_localizer.Status");
    reader.RemapLoggedChannel("/localizer", "frc.controls.LocalizerOutput");
    for (const auto camera : {"camera0", "camera1", "camera2", "camera3"}) {
      reader.RemapLoggedChannel(absl::StrCat("/", camera, "/gray"),
                                "frc.vision.swerve_localizer.Visualization");
    }
    reader.RemapLoggedChannel("/constants", "frc.vision.TargetMap");
    if (absl::GetFlag(FLAGS_override_constants)) {
      reader.RemapLoggedChannel("/constants", "frc.vision.CameraConstants");
    }
  }

  auto factory =
      std::make_unique<aos::SimulatedEventLoopFactory>(reader.configuration());

  const aos::Node *node = nullptr;
  if (aos::configuration::MultiNode(reader.configuration())) {
    node = aos::configuration::GetNode(reader.configuration(), "orin");
  }

  reader.RegisterWithoutStarting(factory.get());

  SimulatedFieldMapSender field_map_sender(factory.get(),
                                           absl::GetFlag(FLAGS_field_map_path));
  if (absl::GetFlag(FLAGS_override_constants)) {
    // Note: The constants sender only does anything in its constructor, so we
    // are fine deleting it immediately.
    SimulatedConstantsSender camera_constants_sender(
        factory.get(), absl::GetFlag(FLAGS_team),
        absl::GetFlag(FLAGS_calibration),
        absl::GetFlag(FLAGS_robot_name).value());
  }

  std::unique_ptr<aos::EventLoop> web_proxy_event_loop;
  std::unique_ptr<aos::web_proxy::WebProxy> web_proxy;

  if (absl::GetFlag(FLAGS_rerun_realtime)) {
    reader.set_exit_on_finish(false);
  }

  std::vector<std::unique_ptr<aos::util::LoggerState>> loggers;

  reader.OnStart(node, [&factory, node, &loggers, &web_proxy_event_loop,
                        &web_proxy, &reader]() {
    aos::NodeEventLoopFactory *node_factory =
        factory->GetNodeEventLoopFactory(node);
    if (absl::GetFlag(FLAGS_rerun)) {
      node_factory->AlwaysStart<frc::vision::swerve_localizer::Localizer>(
          "localizer");
    }
    aos::util::MkdirP(absl::GetFlag(FLAGS_output_folder) + "/",
                      static_cast<std::filesystem::perms>(0755));
    loggers.push_back(std::make_unique<aos::util::LoggerState>(
        factory.get(), node, absl::GetFlag(FLAGS_output_folder)));

    node_factory->AlwaysStart<aos::McapLogger>(
        "mcap_logger", absl::GetFlag(FLAGS_output_folder) + "/log.mcap",
        aos::McapLogger::Serialization::kFlatbuffer,
        aos::McapLogger::CanonicalChannelNames::kCanonical,
        aos::McapLogger::Compression::kNone);

    web_proxy_event_loop = factory->MakeEventLoop("localizer", node);
    web_proxy = std::make_unique<aos::web_proxy::WebProxy>(
        web_proxy_event_loop.get(), factory->scheduler_epoll(),
        aos::web_proxy::StoreHistory::kYes, absl::GetFlag(FLAGS_buffer_size));
    if (absl::GetFlag(FLAGS_rerun_realtime)) {
      reader.SetRealtimeReplayRate(0.005);
      LOG(INFO) << "Going slow to wait for the user to connect.";
    }

    web_proxy->SetDataPath(absl::GetFlag(FLAGS_data_dir).c_str());
    if (absl::GetFlag(FLAGS_rerun_realtime)) {
      aos::TimerHandler *timer = web_proxy_event_loop->AddTimer([&reader]() {
        LOG(INFO) << "Replaying";
        reader.SetRealtimeReplayRate(1.0);
      });
      web_proxy_event_loop->OnRun([timer, &web_proxy_event_loop]() {
        timer->Schedule(web_proxy_event_loop->monotonic_now() +
                        std::chrono::milliseconds(10));
      });
    }
  });

  reader.event_loop_factory()->Run();

  reader.Deregister();

  return 0;
}
