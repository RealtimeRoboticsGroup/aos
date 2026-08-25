// A tool to calculate message latency statistics for every channel that is
// forwarded by the message bridge in a log.
//
// For every node, this subscribes to every channel that is forwarded to that
// node. For each message, the latency is the time between when the message was
// sent on the sending node and when it was received on the receiving node.
// Since those times are measured on different nodes' clocks, both are
// converted to the distributed clock using the event loop factory before being
// compared.
//
// Dropped messages are detected by looking for gaps in the remote queue index
// of the received messages.
#include <iomanip>
#include <iostream>
#include <optional>

#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"

#include "aos/configuration.h"
#include "aos/events/logging/log_reader.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"
#include "aos/time/time.h"
#include "aos/util/print_table.h"
#include "aos/util/status.h"

ABSL_FLAG(std::string, name, "",
          "Substring filter for the channel names to include. Empty for no "
          "filter.");
ABSL_FLAG(std::string, sending_node, "",
          "Filter for the node that the forwarded messages are sent from. "
          "Empty for no filter.");
ABSL_FLAG(std::string, receiving_node, "",
          "Filter for the node that the forwarded messages are sent to. Empty "
          "for no filter.");

namespace aos {
namespace {
// Accumulates count/min/max/average/standard deviation statistics for a
// series of samples.
class SampleStatistics {
 public:
  void Add(double sample) {
    min_ = (count_ == 0) ? sample : std::min(min_, sample);
    max_ = (count_ == 0) ? sample : std::max(max_, sample);
    ++count_;
    // https://en.wikipedia.org/wiki/Standard_deviation#Rapid_calculation_methods
    const double delta = sample - mean_;
    mean_ += delta / count_;
    Q_ += delta * (sample - mean_);
  }

  size_t count() const { return count_; }
  double min() const { return min_; }
  double max() const { return max_; }
  double average() const { return mean_; }
  double standard_deviation() const {
    return (count_ < 2) ? 0.0 : std::sqrt(Q_ / (count_ - 1));
  }

 private:
  size_t count_ = 0;
  double min_ = 0.0;
  double max_ = 0.0;
  double mean_ = 0.0;
  double Q_ = 0.0;
};

std::string FormatDecimal(double value) {
  std::stringstream ss;
  ss << std::fixed << std::setprecision(3) << value;
  return ss.str();
}

// Formats the configured time_to_live of a connection in milliseconds, or
// "reliable" for a time_to_live of 0.
std::string FormatTimeToLive(const aos::Connection *connection) {
  return (connection->time_to_live() == 0)
             ? "reliable"
             : FormatDecimal(connection->time_to_live() / 1e6);
}

// Formats statistics as "average [min, max] std standard_deviation".
std::string FormatStatistics(const SampleStatistics &statistics) {
  return FormatDecimal(statistics.average()) + " [" +
         FormatDecimal(statistics.min()) + ", " +
         FormatDecimal(statistics.max()) + "] std " +
         FormatDecimal(statistics.standard_deviation());
}

// Watches every channel that is forwarded to the node that the provided event
// loop runs on and accumulates per-channel statistics. The destructor prints
// the statistics, which for log reading should happen after the log has been
// fully replayed.
class NodeLatencyDump {
 public:
  NodeLatencyDump(aos::EventLoop *event_loop,
                  aos::SimulatedEventLoopFactory *factory)
      : event_loop_(event_loop), factory_(factory) {
    event_loop_->SkipTimingReport();
    event_loop_->SkipAosLog();

    for (const aos::Channel *channel :
         *event_loop_->configuration()->channels()) {
      // Only watch channels that are forwarded to this node from another node.
      if (!channel->has_source_node() || !channel->has_destination_nodes() ||
          channel->destination_nodes()->size() == 0) {
        continue;
      }
      if (channel->source_node()->string_view() ==
          event_loop_->node()->name()->string_view()) {
        continue;
      }
      if (!aos::configuration::ChannelIsReadableOnNode(channel,
                                                       event_loop_->node())) {
        continue;
      }
      if (!absl::GetFlag(FLAGS_sending_node).empty() &&
          absl::GetFlag(FLAGS_sending_node) !=
              channel->source_node()->string_view()) {
        continue;
      }
      if (channel->name()->string_view().find(absl::GetFlag(FLAGS_name)) ==
          std::string::npos) {
        continue;
      }

      const aos::Node *sending_node = aos::configuration::GetNode(
          event_loop_->configuration(), channel->source_node()->string_view());
      CHECK(sending_node != nullptr)
          << "Node not in config: " << channel->source_node()->string_view();

      const aos::Connection *connection =
          aos::configuration::ConnectionToNode(channel, event_loop_->node());
      CHECK(connection != nullptr)
          << "No connection to node " << event_loop_->node()->name()->str()
          << " for channel " << channel->name()->str();

      const size_t index = statistics_.size();
      statistics_.push_back({channel, sending_node, connection});
      event_loop_->MakeRawNoArgWatcher(
          channel, [this, index](const aos::Context &context) {
            HandleMessage(context, index);
          });
    }
  }

  ~NodeLatencyDump() {
    std::cout << "Statistics for messages received by node "
              << event_loop_->node()->name()->string_view()
              << " (sent by the nodes listed per channel below):" << std::endl;
    std::vector<std::array<std::string, 10>> rows;
    rows.push_back({"Channel Name", "Type", "Sending Node", "Count", "Dropped",
                    "Frequency (Hz)", "Latency (ms)", "TTL (ms)",
                    "Size (bytes)", "Bandwidth (bytes/s)"});
    for (const ChannelStatistics &channel_statistics : statistics_) {
      if (channel_statistics.received_count == 0) {
        continue;
      }
      rows.push_back({channel_statistics.channel->name()->str(),
                      channel_statistics.channel->type()->str(),
                      channel_statistics.sending_node->name()->str(),
                      std::to_string(channel_statistics.received_count),
                      std::to_string(channel_statistics.dropped_count),
                      FormatDecimal(channel_statistics.Frequency()),
                      FormatStatistics(channel_statistics.latency_ms),
                      FormatTimeToLive(channel_statistics.connection),
                      FormatStatistics(channel_statistics.size),
                      FormatDecimal(channel_statistics.Bandwidth())});
    }
    if (rows.size() > 1u) {
      aos::util::PrintTable(&std::cout, "  ", rows);
    }
  }

  // Returns true if any forwarded channels were found for this node.
  bool has_channels() const { return !statistics_.empty(); }

 private:
  struct ChannelStatistics {
    const aos::Channel *channel;
    const aos::Node *sending_node;
    // The connection that forwards this channel to this node.
    const aos::Connection *connection;

    size_t received_count = 0;
    // The number of messages that were sent on the sending node but never
    // received on this node, as detected by gaps in the remote queue index.
    size_t dropped_count = 0;
    // The remote queue index of the last message received on this channel.
    std::optional<uint32_t> last_remote_queue_index = std::nullopt;

    // Latency statistics, measured on the distributed clock, in milliseconds.
    SampleStatistics latency_ms{};

    // Message payload size statistics, in bytes.
    SampleStatistics size{};
    size_t total_bytes = 0;
    // Receive times of the first and last messages, for the bandwidth
    // calculation.
    aos::monotonic_clock::time_point first_message_time =
        aos::monotonic_clock::min_time;
    aos::monotonic_clock::time_point last_message_time =
        aos::monotonic_clock::min_time;

    // Returns the average bandwidth of the channel in bytes per second over
    // the time span in which messages were received.
    // Note that this does have a fence-post issue, where a message at a
    // consistent period will see the calculated bandwidth go down as we get
    // more samples.
    double Bandwidth() const {
      const double seconds_active =
          aos::time::DurationInSeconds(last_message_time - first_message_time);
      return (seconds_active <= 0.0) ? 0.0 : total_bytes / seconds_active;
    }

    // Returns the average frequency of received messages in hertz over the
    // time span in which messages were received.
    double Frequency() const {
      const double seconds_active =
          aos::time::DurationInSeconds(last_message_time - first_message_time);
      return (seconds_active <= 0.0) ? 0.0
                                     : (received_count - 1) / seconds_active;
    }
  };

  void HandleMessage(const aos::Context &context, size_t index) {
    ChannelStatistics &channel_statistics = statistics_[index];
    ++channel_statistics.received_count;

    // Look for gaps in the remote queue index to detect dropped messages. If
    // the index went backwards, the sending node rebooted; restart tracking
    // without counting drops.
    if (channel_statistics.last_remote_queue_index.has_value() &&
        context.remote_queue_index >
            channel_statistics.last_remote_queue_index.value()) {
      channel_statistics.dropped_count +=
          context.remote_queue_index -
          channel_statistics.last_remote_queue_index.value() - 1;
    }
    channel_statistics.last_remote_queue_index = context.remote_queue_index;

    // Track the payload size and the receive times for the bandwidth
    // calculation.
    channel_statistics.size.Add(context.size);
    channel_statistics.total_bytes += context.size;
    if (channel_statistics.first_message_time ==
        aos::monotonic_clock::min_time) {
      channel_statistics.first_message_time = context.monotonic_event_time;
    }
    channel_statistics.last_message_time = context.monotonic_event_time;

    // Only calculate latency for messages where the remote time was filled in.
    if (context.monotonic_remote_time == context.monotonic_event_time) {
      return;
    }
    // Convert both times to the distributed clock so that they can be compared
    // across nodes.
    const aos::distributed_clock::time_point remote_time = CheckExpected(
        factory_->GetNodeEventLoopFactory(channel_statistics.sending_node)
            ->ToDistributedClock(context.monotonic_remote_time));
    const aos::distributed_clock::time_point event_time =
        CheckExpected(factory_->GetNodeEventLoopFactory(event_loop_->node())
                          ->ToDistributedClock(context.monotonic_event_time));
    channel_statistics.latency_ms.Add(
        std::chrono::duration<double, std::milli>(event_time - remote_time)
            .count());
  }

  aos::EventLoop *event_loop_;
  aos::SimulatedEventLoopFactory *factory_;
  std::vector<ChannelStatistics> statistics_;
};

struct DumperState {
  std::unique_ptr<EventLoop> event_loop;
  std::unique_ptr<NodeLatencyDump> dumper;
};

int Main(int argc, char *argv[]) {
  if (argc < 2) {
    LOG(ERROR) << "Expected at least 1 logfile as an argument";
    return 1;
  }
  aos::logger::LogReader reader(
      aos::logger::SortParts(aos::logger::FindLogs(argc, argv)));
  reader.Register();
  CHECK(aos::configuration::MultiNode(
      reader.event_loop_factory()->configuration()))
      << "Cannot compute forwarding latencies on a single-node log.";
  {
    std::vector<DumperState> dumpers;
    for (const aos::Node *node : aos::configuration::GetNodes(
             reader.event_loop_factory()->configuration())) {
      if (!absl::GetFlag(FLAGS_receiving_node).empty() &&
          absl::GetFlag(FLAGS_receiving_node) != node->name()->string_view()) {
        continue;
      }
      std::unique_ptr<aos::EventLoop> event_loop =
          reader.event_loop_factory()->MakeEventLoop("message_bridge_latency",
                                                     node);
      std::unique_ptr<NodeLatencyDump> dumper =
          std::make_unique<NodeLatencyDump>(event_loop.get(),
                                            reader.event_loop_factory());
      if (!dumper->has_channels()) {
        // Nothing is forwarded to this node; no point in keeping it around.
        continue;
      }
      dumpers.push_back({std::move(event_loop), std::move(dumper)});
    }
    reader.event_loop_factory()->Run();
  }
  reader.Deregister();
  return EXIT_SUCCESS;
}
}  // namespace
}  // namespace aos

int main(int argc, char *argv[]) {
  aos::InitGoogle(&argc, &argv);
  return aos::Main(argc, argv);
}
