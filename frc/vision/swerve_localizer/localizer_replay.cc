#include "absl/flags/flag.h"

#include "aos/configuration.h"
#include "aos/events/logging/log_reader.h"
#include "aos/events/logging/log_writer.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/network/team_number.h"
#include "aos/network/web_proxy.h"
#include "aos/util/simulation_logger.h"
#include "frc/constants/constants_sender_lib.h"
#include "frc/vision/swerve_localizer/localizer.h"
#include "frc/vision/swerve_localizer/simulated_constants_sender_lib.h"

ABSL_FLAG(std::string, config, "frc/vision/aos_config.json",
          "Name of the config file to replay using.");
ABSL_FLAG(std::string, calibration, "frc/vision/constants.json",
          "Name of the config file to replay using.");
ABSL_FLAG(bool, override_config, false,
          "If set, override the logged config with --config.");
ABSL_FLAG(int32_t, team, 4646, "Team number to use for logfile replay.");
ABSL_FLAG(std::string, output_folder, "/tmp/replayed",
          "Name of the folder to write replayed logs to.");
ABSL_FLAG(std::string, field_map_path,
          "../frc2025_field_map_welded/file/frc2025r2.fmap",
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
    reader.RemapLoggedChannel("/constants", "frc.vision.CameraConstants");
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
  SimulatedConstantsSender camera_constants_sender(
      factory.get(), absl::GetFlag(FLAGS_team),
      absl::GetFlag(FLAGS_calibration));

  std::unique_ptr<aos::EventLoop> web_proxy_event_loop;
  std::unique_ptr<aos::web_proxy::WebProxy> web_proxy;

  reader.set_exit_on_finish(false);

  std::vector<std::unique_ptr<aos::util::LoggerState>> loggers;

  reader.OnStart(node, [&factory, node, &loggers, &web_proxy_event_loop,
                        &web_proxy, &reader]() {
    aos::NodeEventLoopFactory *node_factory =
        factory->GetNodeEventLoopFactory(node);
    if (absl::GetFlag(FLAGS_rerun)) {
      node_factory->AlwaysStart<frc::vision::swerve_localizer::Localizer>(
          "localizer");
    }
    loggers.push_back(std::make_unique<aos::util::LoggerState>(
        factory.get(), node, absl::GetFlag(FLAGS_output_folder)));

    web_proxy_event_loop = factory->MakeEventLoop("localizer", node);
    web_proxy = std::make_unique<aos::web_proxy::WebProxy>(
        web_proxy_event_loop.get(), factory->scheduler_epoll(),
        aos::web_proxy::StoreHistory::kYes, absl::GetFlag(FLAGS_buffer_size));
    reader.SetRealtimeReplayRate(0.005);
    LOG(INFO) << "Going slow to wait for the user to connect.";

    web_proxy->SetDataPath(absl::GetFlag(FLAGS_data_dir).c_str());
    aos::TimerHandler *timer = web_proxy_event_loop->AddTimer([&reader]() {
      LOG(INFO) << "Replaying";
      reader.SetRealtimeReplayRate(1.0);
    });
    web_proxy_event_loop->OnRun([timer, &web_proxy_event_loop]() {
      timer->Schedule(web_proxy_event_loop->monotonic_now() +
                      std::chrono::milliseconds(10));
    });
  });

  reader.event_loop_factory()->Run();

  reader.Deregister();

  return 0;
}
