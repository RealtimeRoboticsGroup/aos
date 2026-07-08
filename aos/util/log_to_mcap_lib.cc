#include <algorithm>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <ostream>
#include <ranges>
#include <set>
#include <string>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/strings/str_join.h"
#include "flatbuffers/reflection_generated.h"

#include "aos/configuration.h"
#include "aos/events/event_loop.h"
#include "aos/events/logging/log_reader.h"
#include "aos/events/logging/logfile_sorting.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/flatbuffers.h"
#include "aos/macros.h"
#include "aos/util/clock_publisher.h"
#include "aos/util/clock_timepoints_schema.h"
#include "aos/util/mcap_logger.h"

namespace aos::util {

// The enum value for the --fetch flag. See the flag description below for
// more information.
enum class FetchMode {
  kNone,
  kAll,
  kRewrite,
};

// Forward-declare the absl helpers for simplicity.
bool AbslParseFlag(absl::string_view text, FetchMode *flag, std::string *err);
std::string AbslUnparseFlag(const FetchMode &fetch_mode);

}  // namespace aos::util

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
ABSL_FLAG(aos::util::FetchMode, fetch, aos::util::FetchMode::kRewrite,
          "Determines the fetch mode. Valid options are: \"none\", \"all\", "
          "and \"rewrite\". If set to \"none\", will not fetch any messages. "
          "If set to \"all\", *all* messages in the logfile will be included, "
          "including any that may have occurred prior to the start of the log. "
          "This can be used to see additional data, but given that data may be "
          "incomplete prior to the start of the log, you should be careful "
          "about interpretting data flow when using this flag. If set to "
          "\"rewrite\", it behaves the same as \"all\", but with the "
          "additional behavior described in "
          "--rewrite_timestamp_delta_seconds.");
ABSL_FLAG(int, rewrite_timestamp_delta_seconds, 10,
          "When --fetch_and_rewrite_timestamps is used, this determines the "
          "number of seconds by which the latched messages are shifted. The "
          "final timestamp of latched messages will be the log's start time "
          "rounded down to the nearest second and moved back this many "
          "seconds. All latched messages will have this timestamp. If this "
          "timestamp modification would put messages before time 0, then the "
          "latched messages will show up at time zero. Note that if multiple "
          "messages exist on a channel before log start then they will all "
          "have the same timestamp.");
ABSL_FLAG(std::vector<std::string>, include_channels, {".*"},
          "A comma-separated list of MCAP topic names to include. This looks "
          "like so: --include_channels='/0/foo a.b.Msg1,/0/bar a.c.Msg2'. "
          "Only topics in this list will be in the final MCAP. Topics included "
          "by this list can still be dropped via --drop_channels.");
ABSL_FLAG(std::vector<std::string>, drop_channels, {},
          "A comma-separated list of MCAP topic names to drop. This looks like "
          "so: --drop_channels='/0/foo a.b.Msg1,/0/bar a.c.Msg2'. Works in "
          "conjunction with --include_channels. See that help for more "
          "information.");
ABSL_FLAG(int, progress_update_interval_seconds, 5,
          "If set to a positive value, print a conversion progress update "
          "every time this many seconds of real time have passed. If set to "
          "a non-positive value, no progress updates will be printed.");

namespace aos::util {

namespace {

// Use std::less<void> here so that we can use std::string_view for lookup.
// https://en.cppreference.com/w/cpp/utility/functional/less_void.html
using FetchModeLookupMap = std::map<std::string, FetchMode, std::less<void>>;

const FetchModeLookupMap &GetFetchModeMap() {
  static FetchModeLookupMap kFetchModeMap{
      {"none", FetchMode::kNone},
      {"all", FetchMode::kAll},
      {"rewrite", FetchMode::kRewrite},
  };
  return kFetchModeMap;
}

}  // namespace

bool AbslParseFlag(absl::string_view text, FetchMode *flag, std::string *err) {
  const FetchModeLookupMap &fetch_modes = GetFetchModeMap();
  if (auto it = fetch_modes.find(text); it != fetch_modes.end()) {
    *flag = it->second;
    return true;
  }
  *err = "Unknown value. Allowed values are: ";
  *err += absl::StrJoin(fetch_modes | std::views::transform([](auto &element) {
                          return element.first;
                        }),
                        ", ");
  return false;
}

std::string AbslUnparseFlag(const FetchMode &fetch_mode) {
  for (const auto &[flag_value, flag_mode] : GetFetchModeMap()) {
    if (flag_mode == fetch_mode) {
      return flag_value;
    }
  }
  ABSL_LOG(FATAL) << "Unhandled FetchMode: " << static_cast<int>(fetch_mode);
  AOS_UNREACHABLE();
}

namespace {

// Helper class to print progress updates during conversion.
class ProgressUpdatePrinter {
 public:
  ProgressUpdatePrinter(EventLoop *event_loop) : event_loop_(event_loop) {
    // Initialize all the tracking variables.
    last_print_time_ = monotonic_clock::now();
    virtual_start_time_ = event_loop->monotonic_now();

    aos::TimerHandler *timer = event_loop->AddTimer([this]() {
      // Check to see if we should print an update.
      monotonic_clock::time_point now = monotonic_clock::now();
      if (now >
          last_print_time_ + std::chrono::seconds(absl::GetFlag(
                                 FLAGS_progress_update_interval_seconds))) {
        // Calculate how much time of the log we have processed since the last
        // update.
        monotonic_clock::time_point virtual_now = event_loop_->monotonic_now();
        monotonic_clock::duration total_virtual_duration =
            virtual_now - virtual_start_time_;

        LOG(INFO) << "Processed a total of "
                  << std::chrono::duration<double>(total_virtual_duration)
                  << " of the log.";

        // Update the tracking variables for the next iteration.
        last_print_time_ = now;
      }
    });

    event_loop_->OnRun([this, timer] {
      // Check every virtual second.
      timer->Schedule(virtual_start_time_ + std::chrono::seconds(1),
                      std::chrono::seconds(1));
    });
  }

 private:
  EventLoop *event_loop_;
  monotonic_clock::time_point last_print_time_;
  monotonic_clock::time_point virtual_start_time_;
};

using namespace std::literals::chrono_literals;

// Round down to the nearest second and then remove
// --rewrite_timestamp_delta_seconds seconds. The goal here is to make a time
// point that people will perceive as "latched". We remove an extra second just
// to make it a little more obvious in case the log starts on or shortly after
// the top of a second.
template <typename Clock>
auto MakeLatchedTimestamp(typename Clock::time_point time)
    -> Clock::time_point {
  // The duration_cast rounds down.
  const auto seconds =
      std::chrono::duration_cast<std::chrono::seconds>(time.time_since_epoch());
  // Don't go below a timestamp of zero seconds. The mcap CLI tool experiences
  // some underflows otherwise.
  const auto adjusted_seconds = std::max(
      0s, seconds - std::chrono::seconds(
                        absl::GetFlag(FLAGS_rewrite_timestamp_delta_seconds)));
  return Clock::epoch() + adjusted_seconds;
}

}  // namespace

std::function<bool(const Channel *)> GetChannelShouldBeDroppedTester() {
  const std::vector<std::string> &included_channel_strings =
      absl::GetFlag(FLAGS_include_channels);
  const std::vector<std::string> &dropped_channel_strings =
      absl::GetFlag(FLAGS_drop_channels);

  // Convert the strings to regex objects.
  std::vector<std::regex> included_channels{included_channel_strings.begin(),
                                            included_channel_strings.end()};
  std::vector<std::regex> dropped_channels{dropped_channel_strings.begin(),
                                           dropped_channel_strings.end()};

  return [included_channels = std::move(included_channels),
          dropped_channels =
              std::move(dropped_channels)](const aos::Channel *channel) {
    // Convert the channel to an MCAP-style topic.
    const std::string topic_name = absl::StrCat(
        channel->name()->string_view(), " ", channel->type()->string_view());
    // Check if the topic should be included.
    const auto topic_is_included = [&topic_name](const std::regex &regex) {
      return std::regex_match(topic_name, regex);
    };
    // Check if the topic matches any of the to-be-dropped regexes the user
    // specified.
    const auto topic_is_dropped = [&topic_name](const std::regex &regex) {
      return std::regex_match(topic_name, regex);
    };
    return std::ranges::none_of(included_channels, topic_is_included) ||
           std::ranges::any_of(dropped_channels, topic_is_dropped);
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

  std::unique_ptr<EventLoop> progress_update_printer_event_loop;
  std::unique_ptr<ProgressUpdatePrinter> progress_update_printer;

  std::unique_ptr<EventLoop> clock_event_loop;
  std::unique_ptr<ClockPublisher> clock_publisher;

  std::unique_ptr<EventLoop> mcap_event_loop;
  std::unique_ptr<McapLogger> relogger;

  const FetchMode fetch_mode = absl::GetFlag(FLAGS_fetch);

  if (fetch_mode == FetchMode::kRewrite) {
    // Add this early as possible so that it is one of the earliest callbacks
    // actually invoked.
    reader.OnStart(node, [&relogger] {
      ABSL_CHECK(relogger) << ": Internal bug. Startup order violated.";
      // Stop overriding timestamps once the log has started. We just want
      // real timestamps from now on.
      relogger->OverrideMessageTimestamps(std::nullopt, std::nullopt);
    });
  }

  // Handles the logic that needs to happen when we want the MCAP to start. This
  // primarily deals with setting up the McapLogger instance. When exactly this
  // happens depends on the fetch-related command line flags.
  auto startup_handler = [&relogger, &mcap_event_loop, &reader, node,
                          fetch_mode, output_path]() {
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

    if (fetch_mode == FetchMode::kRewrite) {
      // Override the timestamps for all latched messages so that they show up
      // near the beginning of the log. All latched messages will have the same
      // timestamp. Overriding will stop in the `OnStart` callback we added
      // above.
      relogger->OverrideMessageTimestamps(
          MakeLatchedTimestamp<aos::monotonic_clock>(
              reader.monotonic_start_time(node)),
          MakeLatchedTimestamp<aos::realtime_clock>(
              reader.realtime_start_time(node)));
    }
  };

  // Handles the logic that needs to happen to start publishing to the
  // ClockTimepoints channel. When exactly this happens depends on the
  // fetch-related command line flags.
  auto clock_setup_handler = [&clock_event_loop, &clock_publisher, &reader,
                              &factory, node] {
    if (absl::GetFlag(FLAGS_include_clocks)) {
      clock_event_loop =
          reader.event_loop_factory()->MakeEventLoop("clock", node);
      clock_publisher =
          std::make_unique<ClockPublisher>(&factory, clock_event_loop.get());
    }
  };

  switch (fetch_mode) {
    case FetchMode::kAll:
      // Note: This condition is subtly different from just calling Fetch() on
      // every channel in OnStart(). Namely, if there is >1 message on a given
      // channel prior to the logfile start, then fetching in the reader
      // OnStart() is insufficient to get *all* log data.
      factory.GetNodeEventLoopFactory(node)->OnStartup(startup_handler);
      // We don't have a good way of starting the simulation at the first
      // fetched message. But we want ClockTimepoints messages for all fetched
      // messages. So we bite the bullet and start up the clock publisher at
      // time zero.
      factory.GetNodeEventLoopFactory(node)->OnStartup(clock_setup_handler);
      break;
    case FetchMode::kRewrite:
      // Same notes here for startup_handler as for --fetch above.
      factory.GetNodeEventLoopFactory(node)->OnStartup(startup_handler);
      // Since we are rewriting the timestamps for fetched messages to be very
      // close to the beginning of the log, we don't want superfluous
      // ClockTimepoints messages. Start those messages at the beginning of the
      // log.
      reader.OnStart(node, clock_setup_handler);
      break;
    case FetchMode::kNone:
      // No messages are fetched. Start everything at the beginning of the log.
      reader.OnStart(node, startup_handler);
      reader.OnStart(node, clock_setup_handler);
  }

  // Set up the tool to print conversion progress to the terminal.
  if (absl::GetFlag(FLAGS_progress_update_interval_seconds) > 0) {
    reader.OnStart(node, [&progress_update_printer_event_loop,
                          &progress_update_printer, &reader, node] {
      progress_update_printer_event_loop =
          reader.event_loop_factory()->MakeEventLoop("progress_update_printer",
                                                     node);
      progress_update_printer = std::make_unique<ProgressUpdatePrinter>(
          progress_update_printer_event_loop.get());
    });
  }

  reader.event_loop_factory()->Run();
  reader.Deregister();

  return 0;
}

}  // namespace aos::util
