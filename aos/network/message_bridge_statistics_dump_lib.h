#ifndef AOS_NETWORK_MESSAGE_BRIDGE_STATISTICS_DUMP_LIB_H_
#define AOS_NETWORK_MESSAGE_BRIDGE_STATISTICS_DUMP_LIB_H_
#include <optional>
#include <string>

#include "aos/configuration.h"
#include "aos/events/event_loop.h"
#include "aos/flatbuffers.h"
#include "aos/network/message_bridge_server_generated.h"

namespace aos::message_bridge {
// A class to handle printing message bridge ServerStatistics in a useful
// format on the command line.
//
// Note that this will provide the statistics for just the server on the node
// for the provided EventLoop (i.e., just messages being *sent* on this node).
//
// Main features:
// * Correlates the channel_index in the per-channel statistics to channel
//   names/types, so that users do not have to interpret the channel_index
//   themselves.
// * Formats the statistics into a human-readable table.
// * Can filter on the node that the connection is to.
// * Can print the final statistics at the end of a log. Since the counters in
//   ServerStatistics are cumulative, the final message summarizes the entire
//   log.
class MessageBridgeStatisticsDump {
 public:
  enum class PrintFinalStatistics { kYes, kNo };
  enum class StreamResults { kYes, kNo };
  MessageBridgeStatisticsDump(aos::EventLoop *event_loop,
                              PrintFinalStatistics print_final,
                              StreamResults stream);
  // The destructor handles the final printout of the last received statistics
  // (if requested), which for log reading should happen after the log has been
  // fully replayed and for live systems will happen when the user Ctrl-C's.
  ~MessageBridgeStatisticsDump();

  // Only output statistics for the specified destination node.
  void NodeFilter(std::string_view node) { node_filter_ = node; }

 private:
  void HandleServerStatistics(const ServerStatistics &statistics);
  const Channel *GetChannel(size_t index);
  void PrintConnection(std::ostream *os, const ServerConnection &connection);
  void PrintStatistics(const ServerStatistics &statistics,
                       aos::monotonic_clock::time_point monotonic_time,
                       aos::realtime_clock::time_point realtime_time);

  aos::EventLoop *event_loop_;
  PrintFinalStatistics print_final_;
  StreamResults stream_;
  std::optional<std::string> node_filter_;
  // The most recent statistics message and its send times, saved so that the
  // destructor can print the final counts. The counters in ServerStatistics
  // are cumulative since boot, so no manual accumulation is required.
  std::optional<FlatbufferDetachedBuffer<ServerStatistics>> latest_statistics_;
  aos::monotonic_clock::time_point latest_monotonic_time_ =
      aos::monotonic_clock::min_time;
  aos::realtime_clock::time_point latest_realtime_time_ =
      aos::realtime_clock::min_time;
};
}  // namespace aos::message_bridge
#endif  // AOS_NETWORK_MESSAGE_BRIDGE_STATISTICS_DUMP_LIB_H_
