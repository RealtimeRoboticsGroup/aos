#include "aos/events/logging/node_printer.h"

#include <functional>
#include <regex>
#include <string_view>

ABSL_FLAG(
    std::string, name, "",
    "Name to match for printing out channels. Empty means no name filter.");
ABSL_FLAG(std::string, type, "",
          "Channel type to match for printing out channels. Empty means no "
          "type filter.");
ABSL_FLAG(bool, regex_match, false,
          "If true, interpret --name and --type as regular expressions. The "
          "regular expressions are matched against the entire strings. If "
          "false (i.e. default), use substring matching instead.");
ABSL_FLAG(bool, json, false, "If true, print fully valid JSON");
ABSL_FLAG(bool, fetch, false,
          "If true, also print out the messages from before the start of the "
          "log file");

ABSL_FLAG(bool, print, true,
          "If true, actually print the messages.  If false, discard them, "
          "confirming they can be parsed.");
ABSL_FLAG(
    uint64_t, count, 0,
    "If >0, log_cat will exit after printing this many messages.  This "
    "includes messages from before the start of the log if --fetch is set.");

ABSL_FLAG(int64_t, max_vector_size, 100,
          "If positive, vectors longer than this will not be printed");
ABSL_FLAG(bool, pretty, false,
          "If true, pretty print the messages on multiple lines");
ABSL_FLAG(
    bool, pretty_max, false,
    "If true, expand every field to its own line (expands more than -pretty)");
ABSL_FLAG(bool, print_timestamps, true, "If true, timestamps are printed.");
ABSL_FLAG(bool, distributed_clock, false,
          "If true, print out the distributed time");
ABSL_FLAG(double, monotonic_start_time, 0.0,
          "If set, only print messages sent at or after this many seconds "
          "after epoch.");
ABSL_FLAG(double, monotonic_end_time, 0.0,
          "If set, only print messages sent at or before this many seconds "
          "after epoch.");
ABSL_FLAG(bool, hex, false,
          "Are integers in the messages printed in hex notation.");
ABSL_FLAG(bool, flush, false, "If set, flushes stdout after every line.");

namespace aos::logging {

aos::Printer MakePrinter() {
  return aos::Printer(
      {
          .pretty = absl::GetFlag(FLAGS_pretty),
          .max_vector_size =
              static_cast<size_t>(absl::GetFlag(FLAGS_max_vector_size)),
          .pretty_max = absl::GetFlag(FLAGS_pretty_max),
          .print_timestamps = absl::GetFlag(FLAGS_print_timestamps),
          .json = absl::GetFlag(FLAGS_json),
          .distributed_clock = absl::GetFlag(FLAGS_distributed_clock),
          .hex = absl::GetFlag(FLAGS_hex),
      },
      absl::GetFlag(FLAGS_flush));
}

std::function<bool(const aos::Channel *)> GetChannelShouldBePrintedTester() {
  if (absl::GetFlag(FLAGS_regex_match)) {
    // The user requested regex matching.
    std::regex name_regex(absl::GetFlag(FLAGS_name));
    std::regex type_regex(absl::GetFlag(FLAGS_type));
    return [name_regex = std::move(name_regex),
            type_regex = std::move(type_regex)](const aos::Channel *channel) {
      const std::string_view name = channel->name()->string_view();
      const std::string_view type = channel->type()->string_view();
      return std::regex_match(name.begin(), name.end(), name_regex) &&
             std::regex_match(type.begin(), type.end(), type_regex);
    };
  } else {
    // We're using substring matching.
    return [](const aos::Channel *channel) {
      const std::string_view name = channel->name()->string_view();
      const std::string_view type = channel->type()->string_view();
      return name.find(absl::GetFlag(FLAGS_name)) != std::string_view::npos &&
             type.find(absl::GetFlag(FLAGS_type)) != std::string_view::npos;
    };
  }
}

NodePrinter::NodePrinter(aos::EventLoop *event_loop,
                         aos::SimulatedEventLoopFactory *factory,
                         aos::Printer *printer)
    : factory_(factory),
      node_factory_(factory->GetNodeEventLoopFactory(event_loop->node())),
      event_loop_(event_loop),
      node_name_(event_loop_->node() == nullptr
                     ? ""
                     : std::string(event_loop->node()->name()->string_view())),
      printer_(printer) {
  event_loop_->SkipTimingReport();
  event_loop_->SkipAosLog();

  const flatbuffers::Vector<flatbuffers::Offset<aos::Channel>> *channels =
      event_loop_->configuration()->channels();

  const monotonic_clock::time_point start_time =
      (absl::GetFlag(FLAGS_monotonic_start_time) == 0.0
           ? monotonic_clock::min_time
           : monotonic_clock::time_point(
                 std::chrono::duration_cast<monotonic_clock::duration>(
                     std::chrono::duration<double>(
                         absl::GetFlag(FLAGS_monotonic_start_time)))));
  const monotonic_clock::time_point end_time =
      (absl::GetFlag(FLAGS_monotonic_end_time) == 0.0
           ? monotonic_clock::max_time
           : monotonic_clock::time_point(
                 std::chrono::duration_cast<monotonic_clock::duration>(
                     std::chrono::duration<double>(
                         absl::GetFlag(FLAGS_monotonic_end_time)))));

  std::function<bool(const aos::Channel *)> channel_should_be_printed =
      GetChannelShouldBePrintedTester();

  for (const aos::Channel *channel : *channels) {
    if (channel_should_be_printed(channel)) {
      if (!aos::configuration::ChannelIsReadableOnNode(channel,
                                                       event_loop_->node())) {
        continue;
      }
      const flatbuffers::string_view name = channel->name()->string_view();
      const flatbuffers::string_view type = channel->type()->string_view();
      VLOG(1) << "Listening on " << name << " " << type;

      CHECK(channel->schema() != nullptr);
      event_loop_->MakeRawWatcher(
          channel, [this, channel, start_time, end_time](
                       const aos::Context &context, const void * /*message*/) {
            if (!absl::GetFlag(FLAGS_print)) {
              return;
            }
            if (absl::GetFlag(FLAGS_count) > 0 &&
                printer_->message_count() >= absl::GetFlag(FLAGS_count)) {
              return;
            }

            if (!absl::GetFlag(FLAGS_fetch) && !started_) {
              return;
            }

            if (context.monotonic_event_time < start_time ||
                context.monotonic_event_time > end_time) {
              return;
            }

            printer_->PrintMessage(node_name_, node_factory_, channel, context);
            if (absl::GetFlag(FLAGS_count) > 0 &&
                printer_->message_count() >= absl::GetFlag(FLAGS_count)) {
              factory_->Exit();
            }
          });
    }
  }
}

void NodePrinter::SetStarted(bool started,
                             aos::monotonic_clock::time_point monotonic_now,
                             aos::realtime_clock::time_point realtime_now) {
  started_ = started;
  if (absl::GetFlag(FLAGS_json)) {
    return;
  }
  if (started_) {
    std::cout << std::endl;
    std::cout << (event_loop_->node() != nullptr
                      ? (event_loop_->node()->name()->str() + " ")
                      : "")
              << "Log starting at " << realtime_now << " (" << monotonic_now
              << ")";
    std::cout << std::endl << std::endl;
  } else {
    std::cout << std::endl;
    std::cout << (event_loop_->node() != nullptr
                      ? (event_loop_->node()->name()->str() + " ")
                      : "")
              << "Log shutting down at " << realtime_now << " ("
              << monotonic_now << ")";
    std::cout << std::endl << std::endl;
  }
}

}  // namespace aos::logging
