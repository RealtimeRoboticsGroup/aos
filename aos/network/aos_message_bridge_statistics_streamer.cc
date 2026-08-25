#include "absl/flags/flag.h"

#include "aos/configuration.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/network/message_bridge_statistics_dump_lib.h"

ABSL_FLAG(std::string, config, "aos_config.json",
          "The path to the config to use.");
ABSL_FLAG(std::string, receiving_node, "",
          "Filter for the node that the forwarded messages are sent to. Empty "
          "for no filter.");
ABSL_FLAG(bool, stream, true,
          "Stream out all the message bridge server statistics that we "
          "receive.");
ABSL_FLAG(bool, final, false,
          "Display the last received server statistics when the process is "
          "terminated. Since the counters are cumulative, this summarizes "
          "everything since the message bridge server started.");

namespace aos {
int Main() {
  aos::FlatbufferVector<aos::Configuration> config(
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config)));
  ShmEventLoop event_loop(&config.message());
  message_bridge::MessageBridgeStatisticsDump dumper(
      &event_loop,
      absl::GetFlag(FLAGS_final) ? message_bridge::MessageBridgeStatisticsDump::
                                       PrintFinalStatistics::kYes
                                 : message_bridge::MessageBridgeStatisticsDump::
                                       PrintFinalStatistics::kNo,
      absl::GetFlag(FLAGS_stream)
          ? message_bridge::MessageBridgeStatisticsDump::StreamResults::kYes
          : message_bridge::MessageBridgeStatisticsDump::StreamResults::kNo);
  if (!absl::GetFlag(FLAGS_receiving_node).empty()) {
    dumper.NodeFilter(absl::GetFlag(FLAGS_receiving_node));
  }
  event_loop.Run();
  return EXIT_SUCCESS;
}
}  // namespace aos

int main(int argc, char *argv[]) {
  aos::InitGoogle(&argc, &argv);
  return aos::Main();
}
