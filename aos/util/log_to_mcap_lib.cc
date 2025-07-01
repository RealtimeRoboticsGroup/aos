#include <algorithm>
#include <memory>
#include <optional>
#include <ostream>
#include <set>
#include <string>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "flatbuffers/reflection_generated.h"

#include "aos/configuration.h"
#include "aos/events/event_loop.h"
#include "aos/events/logging/log_reader.h"
#include "aos/events/logging/logfile_sorting.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/flatbuffers.h"
#include "aos/util/clock_publisher.h"
#include "aos/util/clock_timepoints_schema.h"
#include "aos/util/mcap_logger.h"

ABSL_FLAG(std::string, node, "", "Node to replay from the perspective of.");
ABSL_FLAG(std::string, mode, "flatbuffer", "json or flatbuffer serialization.");
ABSL_FLAG(
    bool, canonical_channel_names, false,
    "If set, use full channel names; by default, will shorten names to be the "
    "shortest possible version of the name (e.g., /aos instead of /pi/aos).");
ABSL_FLAG(bool, compress, true, "Whether to use LZ4 compression in MCAP file.");
ABSL_FLAG(bool, include_clocks, true,
          "Whether to add a /clocks channel that publishes all nodes' clock "
          "offsets.");
ABSL_FLAG(bool, fetch, false,
          "If set, *all* messages in the logfile will be included, including "
          "any that may have occurred prior to the start of the log. This "
          "can be used to see additional data, but given that data may be "
          "incomplete prior to the start of the log, you should be careful "
          "about interpretting data flow when using this flag.");
ABSL_FLAG(std::vector<std::string>, drop_channels, {},
          "A comma-separated list of MCAP topic names to drop. This looks like "
          "so: --drop_channels='/0/foo a.b.Msg1,/0/bar a.c.Msg2'.");

namespace aos::util {

std::function<bool(const Channel *)> GetChannelShouldBeDroppedTester() {
  const std::vector<std::string> &dropped_channel_strings =
      absl::GetFlag(FLAGS_drop_channels);

  // Convert the strings to regex objects.
  std::vector<std::regex> dropped_channels{dropped_channel_strings.begin(),
                                           dropped_channel_strings.end()};

  return [dropped_channels =
              std::move(dropped_channels)](const aos::Channel *channel) {
    // Convert the channel to an MCAP-style topic.
    const std::string topic_name = absl::StrCat(
        channel->name()->string_view(), " ", channel->type()->string_view());
    // Check if the topic matches any of the regexes the user specified.
    const auto topic_matches =
        [topic_name = std::move(topic_name)](const std::regex &regex) {
          return std::regex_match(topic_name, regex);
        };
    return std::ranges::any_of(dropped_channels, topic_matches);
  };
}

int ConvertLogToMcap(const std::vector<std::string> &log_paths,
                     std::string output_path,
                     std::function<void(logger::LogReader &)> setup_callback) {
  const std::vector<logger::LogFile> logfiles =
      logger::SortParts(logger::FindLogs(log_paths));
  CHECK(!logfiles.empty());
  const std::set<std::string> logger_nodes = logger::LoggerNodes(logfiles);
  CHECK_LT(0u, logger_nodes.size());
  const std::string logger_node = *logger_nodes.begin();
  std::string replay_node = absl::GetFlag(FLAGS_node);
  if (replay_node.empty()) {
    if (logger_nodes.size() == 1u) {
      LOG(INFO) << "Guessing \"" << logger_node
                << "\" as node given that --node was not specified.";
      replay_node = logger_node;
    } else {
      LOG(ERROR) << "Must supply a --node for log_to_mcap.";
      return 1;
    }
  }

  std::optional<FlatbufferDetachedBuffer<Configuration>> config;

  if (absl::GetFlag(FLAGS_include_clocks)) {
    logger::LogReader config_reader(logfiles);

    if (configuration::MultiNode(config_reader.configuration())) {
      CHECK(!replay_node.empty()) << ": Must supply a --node.";
    }

    const Configuration *raw_config = config_reader.logged_configuration();
    // The ClockTimepoints message for multiple VPUs is bigger than the default
    // 1000 bytes. So we need to set a bigger size here.
    ChannelT channel_overrides;
    channel_overrides.max_size = 2000;
    config = configuration::AddChannelToConfiguration(
        raw_config, "/clocks",
        FlatbufferSpan<reflection::Schema>(ClockTimepointsSchema()),
        replay_node.empty() ? nullptr
                            : configuration::GetNode(raw_config, replay_node),
        channel_overrides);
  }

  logger::LogReader reader(
      logfiles, config.has_value() ? &config.value().message() : nullptr);
  if (setup_callback) {
    setup_callback(reader);
  }
  SimulatedEventLoopFactory factory(reader.configuration());
  reader.RegisterWithoutStarting(&factory);

  if (configuration::MultiNode(reader.configuration())) {
    CHECK(!replay_node.empty()) << ": Must supply a --node.";
  }

  const Node *node =
      !configuration::MultiNode(reader.configuration())
          ? nullptr
          : configuration::GetNode(reader.configuration(), replay_node);

  std::unique_ptr<EventLoop> clock_event_loop;
  std::unique_ptr<ClockPublisher> clock_publisher;

  std::unique_ptr<EventLoop> mcap_event_loop;
  std::unique_ptr<McapLogger> relogger;
  auto startup_handler = [&relogger, &mcap_event_loop, &reader,
                          &clock_event_loop, &clock_publisher, &factory, node,
                          output_path]() {
    CHECK(!mcap_event_loop) << ": log_to_mcap does not support generating MCAP "
                               "files from multi-boot logs.";
    mcap_event_loop = reader.event_loop_factory()->MakeEventLoop("mcap", node);
    relogger = std::make_unique<McapLogger>(
        mcap_event_loop.get(), output_path,
        absl::GetFlag(FLAGS_mode) == "flatbuffer"
            ? McapLogger::Serialization::kFlatbuffer
            : McapLogger::Serialization::kJson,
        absl::GetFlag(FLAGS_canonical_channel_names)
            ? McapLogger::CanonicalChannelNames::kCanonical
            : McapLogger::CanonicalChannelNames::kShortened,
        absl::GetFlag(FLAGS_compress) ? McapLogger::Compression::kLz4
                                      : McapLogger::Compression::kNone,
        GetChannelShouldBeDroppedTester());
    if (absl::GetFlag(FLAGS_include_clocks)) {
      clock_event_loop =
          reader.event_loop_factory()->MakeEventLoop("clock", node);
      clock_publisher =
          std::make_unique<ClockPublisher>(&factory, clock_event_loop.get());
    }
  };
  if (absl::GetFlag(FLAGS_fetch)) {
    // Note: This condition is subtly different from just calling Fetch() on
    // every channel in OnStart(). Namely, if there is >1 message on a given
    // channel prior to the logfile start, then fetching in the reader OnStart()
    // is insufficient to get *all* log data.
    factory.GetNodeEventLoopFactory(node)->OnStartup(startup_handler);
  } else {
    reader.OnStart(node, startup_handler);
  }
  reader.event_loop_factory()->Run();
  reader.Deregister();

  return 0;
}

}  // namespace aos::util
