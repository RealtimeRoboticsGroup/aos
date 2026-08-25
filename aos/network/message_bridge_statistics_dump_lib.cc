#include "aos/network/message_bridge_statistics_dump_lib.h"

#include <iostream>
#include <sstream>

#include "absl/log/absl_check.h"
#include "absl/strings/str_cat.h"

#include "aos/flatbuffer_merge.h"
#include "aos/time/time.h"
#include "aos/util/print_table.h"

namespace aos::message_bridge {
namespace {
std::string MaybeNodeName(std::string_view prefix_if_node,
                          const aos::Node *node) {
  if (node == nullptr) {
    return "";
  }
  return absl::StrCat(prefix_if_node, node->name()->string_view());
}
}  // namespace

MessageBridgeStatisticsDump::MessageBridgeStatisticsDump(
    aos::EventLoop *event_loop, PrintFinalStatistics print_final,
    StreamResults stream)
    : event_loop_(event_loop), print_final_(print_final), stream_(stream) {
  event_loop_->MakeWatcher("/aos", [this](const ServerStatistics &statistics) {
    HandleServerStatistics(statistics);
  });
}

void MessageBridgeStatisticsDump::HandleServerStatistics(
    const ServerStatistics &statistics) {
  if (stream_ == StreamResults::kYes) {
    PrintStatistics(statistics, event_loop_->context().monotonic_event_time,
                    event_loop_->context().realtime_event_time);
  }
  if (print_final_ == PrintFinalStatistics::kYes) {
    // The counters in ServerStatistics are cumulative, so the last message
    // seen summarizes everything that came before it. Just keep a copy of it
    // around for the destructor to print.
    latest_statistics_ = aos::CopyFlatBuffer(&statistics);
    latest_monotonic_time_ = event_loop_->context().monotonic_event_time;
    latest_realtime_time_ = event_loop_->context().realtime_event_time;
  }
}

void MessageBridgeStatisticsDump::PrintConnection(
    std::ostream *os, const ServerConnection &connection) {
  // Spacing to use for indentation.
  const std::string kIndent = "  ";
  ABSL_CHECK(connection.has_node());
  ABSL_CHECK(connection.node()->has_name());
  *os << kIndent << "Connection to receiving node "
      << connection.node()->name()->string_view() << ": "
      << EnumNameState(connection.state()) << std::endl;
  *os << kIndent << kIndent << "sent_packets: " << connection.sent_packets()
      << " dropped_packets: " << connection.dropped_packets()
      << " retry_count: " << connection.retry_count()
      << " partial_deliveries: " << connection.partial_deliveries()
      << " connection_count: " << connection.connection_count()
      << " invalid_connection_count: " << connection.invalid_connection_count()
      << std::endl;
  if (connection.has_boot_uuid()) {
    *os << kIndent << kIndent
        << "boot_uuid: " << connection.boot_uuid()->string_view()
        << " connected_since: "
        << aos::monotonic_clock::time_point(
               std::chrono::nanoseconds(connection.connected_since_time()))
        << " monotonic_offset: "
        << std::chrono::nanoseconds(connection.monotonic_offset()) << std::endl;
  }
  if (!connection.has_channels() || connection.channels()->size() == 0) {
    return;
  }
  *os << kIndent << kIndent << "Channels (" << connection.channels()->size()
      << "):" << std::endl;
  std::vector<std::array<std::string, 5>> rows;
  rows.push_back(
      {"Channel Name", "Type", "Sent Packets", "Dropped Packets", "Retries"});
  for (const ServerChannelStatistics *channel_statistics :
       *connection.channels()) {
    const Channel *channel = GetChannel(channel_statistics->channel_index());
    rows.push_back({channel->name()->str(), channel->type()->str(),
                    std::to_string(channel_statistics->sent_packets()),
                    std::to_string(channel_statistics->dropped_packets()),
                    std::to_string(channel_statistics->retry_count())});
  }
  util::PrintTable(os, kIndent + kIndent + kIndent, rows);
}

void MessageBridgeStatisticsDump::PrintStatistics(
    const ServerStatistics &statistics,
    aos::monotonic_clock::time_point monotonic_time,
    aos::realtime_clock::time_point realtime_time) {
  std::cout << "ServerStatistics"
            << MaybeNodeName(" from sending node ", event_loop_->node()) << " ("
            << monotonic_time << "," << realtime_time
            << "): timestamp_send_failures: "
            << statistics.timestamp_send_failures()
            << " invalid_connection_count: "
            << statistics.invalid_connection_count() << std::endl;
  if (!statistics.has_connections()) {
    return;
  }
  for (const ServerConnection *connection : *statistics.connections()) {
    if (node_filter_.has_value() &&
        node_filter_.value() != connection->node()->name()->string_view()) {
      continue;
    }
    PrintConnection(&std::cout, *connection);
  }
}

MessageBridgeStatisticsDump::~MessageBridgeStatisticsDump() {
  if (print_final_ == PrintFinalStatistics::kYes &&
      latest_statistics_.has_value()) {
    std::cout << "\nFinal server statistics"
              << MaybeNodeName(" from sending node ", event_loop_->node())
              << ":\n\n";
    PrintStatistics(latest_statistics_->message(), latest_monotonic_time_,
                    latest_realtime_time_);
  }
}

const Channel *MessageBridgeStatisticsDump::GetChannel(size_t index) {
  ABSL_CHECK_GT(event_loop_->configuration()->channels()->size(), index);
  return event_loop_->configuration()->channels()->Get(index);
}

}  // namespace aos::message_bridge
