#include "aos/ipc_lib/memory_estimation.h"

#include <map>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/events/logging/logfile_utils.h"
#include "aos/ipc_lib/memory_mapped_queue.h"

namespace aos::ipc_lib {
size_t TotalSharedMemoryUsage(const aos::Configuration *config,
                              const aos::Node *node) {
  size_t total_size = 0;
  const flatbuffers::Vector<flatbuffers::Offset<aos::Channel>> *channels =
      config->channels();
  ABSL_CHECK(channels != nullptr);
  // Stores all channels by size, for sorted displays.
  // Indexed by channel memory usage, with elements that are a displayable name
  // of the channel.
  std::multimap<size_t, std::string> channel_sizes;
  for (const aos::Channel *channel : *channels) {
    if (aos::configuration::ChannelIsReadableOnNode(channel, node)) {
      total_size +=
          LocklessQueueMemorySize(MakeQueueConfiguration(config, channel));
      if (ABSL_VLOG_IS_ON(1)) {
        channel_sizes.emplace(
            LocklessQueueMemorySize(MakeQueueConfiguration(config, channel)),
            configuration::CleanedChannelToString(channel));
      }
    }
  }

  if (ABSL_VLOG_IS_ON(1)) {
    for (const auto &pair : channel_sizes) {
      ABSL_LOG(INFO) << "Channel size (bytes): " << pair.first << " "
                     << pair.second;
    }
  }
  return total_size;
}

size_t LogMemoryBufferSizeForChannel(const aos::Channel *channel,
                                     std::chrono::nanoseconds buffer_duration) {
  const size_t buffer_size =
      channel->max_size() *
      (channel->frequency() * aos::time::DurationInSeconds(buffer_duration));
  VLOG(2) << "Allocating " << buffer_size << " bytes for "
          << configuration::CleanedChannelToString(channel);
  return buffer_size;
}

size_t LogMemoryBufferSizeForChannelTimestamps(
    const aos::Channel *channel, std::chrono::nanoseconds buffer_duration) {
  const size_t buffer_size = logger::PackRemoteMessageSize() *
                             channel->frequency() *
                             aos::time::DurationInSeconds(buffer_duration);
  VLOG(2) << "Allocating " << buffer_size << " bytes for "
          << configuration::CleanedChannelToString(channel);
  return buffer_size;
}

size_t TotalLogMemoryBufferSizeForChannels(
    const aos::Configuration *config, const aos::Node *node,
    std::function<bool(const aos::Channel *)> filter,
    std::chrono::nanoseconds buffer_duration) {
  size_t total_size = 0;
  for (const aos::Channel *channel : *config->channels()) {
    if (!filter(channel)) {
      continue;
    }
    if (configuration::ChannelMessageIsLoggedOnNode(channel, node)) {
      total_size += LogMemoryBufferSizeForChannel(channel, buffer_duration);
    }
    if (channel->has_destination_nodes()) {
      for (const aos::Connection *connection : *channel->destination_nodes()) {
        if (configuration::ConnectionDeliveryTimeIsLoggedOnNode(connection,
                                                                node)) {
          total_size +=
              LogMemoryBufferSizeForChannelTimestamps(channel, buffer_duration);
        }
      }
    }
  }
  return total_size;
}
}  // namespace aos::ipc_lib
