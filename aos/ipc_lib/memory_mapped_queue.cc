#include "aos/ipc_lib/memory_mapped_queue.h"

#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <ostream>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "absl/strings/str_cat.h"
#include "flatbuffers/string.h"

#include "aos/ipc_lib/index.h"

namespace aos::ipc_lib {

// Process-global cache of MemoryMappedQueue instances keyed by SHM path, so
// every sender, fetcher and watcher on a channel in this process shares one
// pair of mappings.  Entries are weak_ptrs: a mapping is released as soon as
// its last user drops it and re-created on the next request.
class MemoryMappedQueueCache {
 public:
  // Only ever reached from MemoryMappedQueue construction (never from
  // destruction), so an ordinary function-local static is fine.
  static MemoryMappedQueueCache &GetInstance() {
    static MemoryMappedQueueCache cache;
    return cache;
  }

  std::shared_ptr<MemoryMappedQueue> GetOrCreate(
      std::string_view shm_base, std::filesystem::perms permissions,
      const Configuration *config, const Channel *channel) {
    std::string path = ShmPath(shm_base, channel);
    std::lock_guard<std::mutex> lock(mutex_);
    auto &entry = cache_[path];
    auto existing = entry.lock();
    if (existing) {
      // Every user shares this instance's config, so a second configuration
      // with a different queue layout would otherwise silently get this one.
      ABSL_CHECK(existing->config() == MakeQueueConfiguration(config, channel))
          << ": " << path
          << " is already mapped in this process with a different queue "
             "configuration";
      return existing;
    }
    auto queue = std::make_shared<MemoryMappedQueue>(shm_base, permissions,
                                                     config, channel);
    entry = queue;
    return queue;
  }

 private:
  MemoryMappedQueueCache() = default;

  // Guards cache_.
  std::mutex mutex_;
  std::map<std::string, std::weak_ptr<MemoryMappedQueue>> cache_;
};

std::shared_ptr<MemoryMappedQueue> GetOrCreateMemoryMappedQueue(
    std::string_view shm_base, std::filesystem::perms permissions,
    const Configuration *config, const Channel *channel) {
  return MemoryMappedQueueCache::GetInstance().GetOrCreate(
      shm_base, permissions, config, channel);
}

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
