#include "aos/ipc_lib/memory_mapped_queue.h"

#include <limits>
#include <ostream>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "absl/strings/str_cat.h"
#include "flatbuffers/string.h"

#include "aos/ipc_lib/index.h"

namespace aos::ipc_lib {

std::string ShmFolder(std::string_view shm_base, const Channel *channel) {
  ABSL_CHECK(channel->has_name());
  ABSL_CHECK_EQ(channel->name()->string_view()[0], '/');
  return absl::StrCat(shm_base, channel->name()->string_view(), "/");
}

std::string ShmPath(std::string_view shm_base, const Channel *channel) {
  ABSL_CHECK(channel->has_type());
  return ShmFolder(shm_base, channel) + channel->type()->str() + ".v8";
}

LocklessQueueConfiguration MakeQueueConfiguration(
    const Configuration *configuration, const Channel *channel) {
  LocklessQueueConfiguration config;

  config.num_watchers = channel->num_watchers();
  config.num_senders = channel->num_senders();
  // The value in the channel will default to 0 if readers are configured to
  // copy.
  config.num_pinners = channel->num_readers();
  config.queue_size = configuration::QueueSize(configuration, channel);
  ABSL_CHECK_LT(config.queue_size,
                std::numeric_limits<QueueIndex::PackedIndexType>::max())
      << ": More messages/second configured than the queue can hold on "
      << configuration::CleanedChannelToString(channel) << ", "
      << channel->frequency() << "hz for "
      << std::chrono::duration<double>(
             configuration::ChannelStorageDuration(configuration, channel))
             .count()
      << "sec";
  config.message_data_size = channel->max_size();

  return config;
}

MemoryMappedQueue::MemoryMappedQueue(std::string_view shm_base,
                                     std::filesystem::perms permissions,
                                     const Configuration *config,
                                     const Channel *channel)
    : config_(MakeQueueConfiguration(config, channel)),
      writable_mapping_(ShmPath(shm_base, channel),
                        ipc_lib::LocklessQueueMemorySize(config_), permissions),
      readonly_mapping_(ShmPath(shm_base, channel),
                        ipc_lib::LocklessQueueMemorySize(config_),
                        permissions) {
  ipc_lib::InitializeLocklessQueueMemory(memory(), config_);
}

MemoryMappedQueue::~MemoryMappedQueue() {}

}  // namespace aos::ipc_lib
