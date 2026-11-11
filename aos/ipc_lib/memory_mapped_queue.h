#ifndef AOS_IPC_LIB_MEMORY_MAPPED_QUEUE_H_
#define AOS_IPC_LIB_MEMORY_MAPPED_QUEUE_H_

#include <stddef.h>
#include <stdint.h>

#include <filesystem>
#include <memory>
#include <string>
#include <string_view>

#include "aos/configuration.h"
#include "aos/ipc_lib/lockless_queue.h"
#include "aos/ipc_lib/queue_memory.h"
#include "aos/ipc_lib/shm_mapping.h"

namespace aos::ipc_lib {

std::string ShmFolder(std::string_view shm_base, const Channel *channel);

std::string ShmPath(std::string_view shm_base, const Channel *channel);

LocklessQueueConfiguration MakeQueueConfiguration(
    const Configuration *configuration, const Channel *channel);

// Returns the number of huge pages of size huge_page_size needed to back every
// queue readable on node whose memory size is >= threshold.  Used by starterd
// to size vm.nr_hugepages before creating the queues.
size_t HugePagesNeededForNode(const Configuration *config, const Node *node,
                              size_t threshold, size_t huge_page_size);

// Queue memory in a shared-memory file under shm_base, mapped read/write and
// read-only.  Shared with every other process that maps the same file, and it
// outlives all of them; see shm_mapping.h.
//
// Within a process, ShmEventLoop shares one instance per channel between every
// sender, fetcher and watcher on it (see GetOrCreateMemoryMappedQueue), so the
// queue is mapped and pre-faulted once per process rather than once per user.
// That includes sharing across threads when several ShmEventLoops live in one
// process: an instance is immutable once constructed and every public method
// is const, so it may be used concurrently from any number of threads without
// external synchronization.  Concurrent access to the mapped memory itself is
// governed by LocklessQueue.
class MemoryMappedQueue : public QueueMemory {
 public:
  MemoryMappedQueue(std::string_view shm_base,
                    std::filesystem::perms permissions,
                    const Configuration *config, const Channel *channel);
  ~MemoryMappedQueue() override;

  // This class can't be default or copy constructed.
  MemoryMappedQueue() = delete;
  MemoryMappedQueue(const MemoryMappedQueue &other) = delete;
  MemoryMappedQueue &operator=(const MemoryMappedQueue &rhs) = delete;

  LocklessQueueMemory *memory() const override {
    return reinterpret_cast<ipc_lib::LocklessQueueMemory *>(
        writable_mapping_.data());
  }

  const LocklessQueueMemory *const_memory() const override {
    return reinterpret_cast<const LocklessQueueMemory *>(
        readonly_mapping_.data());
  }

  const LocklessQueueConfiguration &config() const override { return config_; }

 private:
  const LocklessQueueConfiguration config_;
  WritableShmMapping writable_mapping_;
  ReadOnlyShmMapping readonly_mapping_;
};

// Returns a shared MemoryMappedQueue for the given channel, creating one if
// none exists yet.  The returned shared_ptr keeps the mapping alive; when the
// last holder drops it the memory is munmap'd and the cache entry expires.
// Thread-safe.
std::shared_ptr<MemoryMappedQueue> GetOrCreateMemoryMappedQueue(
    std::string_view shm_base, std::filesystem::perms permissions,
    const Configuration *config, const Channel *channel);

}  // namespace aos::ipc_lib

#endif  // AOS_IPC_LIB_MEMORY_MAPPED_QUEUE_H_
