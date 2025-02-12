#include "aos/ipc_lib/memory_estimation.h"

#include <map>

#include "aos/ipc_lib/memory_mapped_queue.h"
namespace aos::ipc_lib {
size_t TotalSharedMemoryUsage(const aos::Configuration *config,
                              const aos::Node *node) {
  size_t total_size = 0;
  const flatbuffers::Vector<flatbuffers::Offset<aos::Channel>> *channels =
      config->channels();
  CHECK(channels != nullptr);
  // Stores all channels by size, for sorted displays.
  // Indexed by channel memory usage, with elements that are a displayable name
  // of the channel.
  std::multimap<size_t, std::string> channel_sizes;
  for (const aos::Channel *channel : *channels) {
    if (aos::configuration::ChannelIsReadableOnNode(channel, node)) {
      total_size +=
          LocklessQueueMemorySize(MakeQueueConfiguration(config, channel));
      if (VLOG_IS_ON(1)) {
        channel_sizes.emplace(
            LocklessQueueMemorySize(MakeQueueConfiguration(config, channel)),
            configuration::CleanedChannelToString(channel));
      }
    }
  }

  if (VLOG_IS_ON(1)) {
    for (const auto &pair : channel_sizes) {
      LOG(INFO) << "Channel size (bytes): " << pair.first << " " << pair.second;
    }
  }
  return total_size;
}
}  // namespace aos::ipc_lib
