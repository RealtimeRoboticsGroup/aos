#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/configuration.h"
#include "aos/events/logging/log_reader.h"
#include "aos/init.h"
#include "aos/network/message_bridge_statistics_dump_lib.h"

ABSL_FLAG(std::string, sending_node, "",
          "Filter for the node that is sending the forwarded messages (the "
          "node the message bridge server runs on). Empty for no filter.");
ABSL_FLAG(std::string, receiving_node, "",
          "Filter for the node that the forwarded messages are sent to. Empty "
          "for no filter.");
ABSL_FLAG(bool, stream, false,
          "Stream out all the message bridge server statistics in the log.");
ABSL_FLAG(bool, final, true,
          "Display the final server statistics at the end of the log. Since "
          "the counters are cumulative, this summarizes the entire log.");

namespace aos {
struct DumperState {
  std::unique_ptr<EventLoop> event_loop;
  std::unique_ptr<message_bridge::MessageBridgeStatisticsDump> dumper;
};
int Main(int argc, char *argv[]) {
  if (argc < 2) {
    ABSL_LOG(ERROR) << "Expected at least 1 logfile as an argument";
    return 1;
  }
  aos::logger::LogReader reader(
      aos::logger::SortParts(aos::logger::FindLogs(argc, argv)));
  reader.Register();
  {
    std::vector<DumperState> dumpers;
    for (const aos::Node *node : aos::configuration::GetNodes(
             reader.event_loop_factory()->configuration())) {
      if (!absl::GetFlag(FLAGS_sending_node).empty() && node != nullptr &&
          absl::GetFlag(FLAGS_sending_node) != node->name()->string_view()) {
        continue;
      }
      std::unique_ptr<aos::EventLoop> event_loop =
          reader.event_loop_factory()->MakeEventLoop(
              "message_bridge_statistics", node);
      event_loop->SkipTimingReport();
      event_loop->SkipAosLog();
      if (event_loop->GetChannel<message_bridge::ServerStatistics>("/aos") ==
          nullptr) {
        // Not all nodes (e.g., on a single-node system) will have a message
        // bridge running.
        ABSL_LOG(WARNING) << "No ServerStatistics channel"
                          << (node == nullptr
                                  ? std::string("")
                                  : absl::StrCat(" on ",
                                                 node->name()->string_view()))
                          << "; skipping.";
        continue;
      }
      std::unique_ptr<message_bridge::MessageBridgeStatisticsDump> dumper =
          std::make_unique<message_bridge::MessageBridgeStatisticsDump>(
              event_loop.get(),
              absl::GetFlag(FLAGS_final)
                  ? message_bridge::MessageBridgeStatisticsDump::
                        PrintFinalStatistics::kYes
                  : message_bridge::MessageBridgeStatisticsDump::
                        PrintFinalStatistics::kNo,
              absl::GetFlag(FLAGS_stream)
                  ? message_bridge::MessageBridgeStatisticsDump::StreamResults::
                        kYes
                  : message_bridge::MessageBridgeStatisticsDump::StreamResults::
                        kNo);
      if (!absl::GetFlag(FLAGS_receiving_node).empty()) {
        dumper->NodeFilter(absl::GetFlag(FLAGS_receiving_node));
      }
      dumpers.push_back({std::move(event_loop), std::move(dumper)});
    }
    reader.event_loop_factory()->Run();
  }
  reader.Deregister();
  return EXIT_SUCCESS;
}
}  // namespace aos

int main(int argc, char *argv[]) {
  aos::InitGoogle(&argc, &argv);
  return aos::Main(argc, argv);
}
