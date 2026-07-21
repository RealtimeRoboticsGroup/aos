#ifndef AOS_IPC_LIB_MEMORY_MAPPED_QUEUE_H_
#define AOS_IPC_LIB_MEMORY_MAPPED_QUEUE_H_

#include <stddef.h>
#include <stdint.h>

#include <filesystem>
#include <string>
#include <string_view>

#include "absl/types/span.h"

#include "aos/configuration.h"
#include "aos/ipc_lib/lockless_queue.h"
#include "aos/ipc_lib/shm_mapping.h"

namespace aos::ipc_lib {

std::string ShmFolder(std::string_view shm_base, const Channel *channel);

std::string ShmPath(std::string_view shm_base, const Channel *channel);

LocklessQueueConfiguration MakeQueueConfiguration(
    const Configuration *configuration, const Channel *channel);

class MemoryMappedQueue {
 public:
  MemoryMappedQueue(std::string_view shm_base,
                    std::filesystem::perms permissions,
                    const Configuration *config, const Channel *channel);
  ~MemoryMappedQueue();

  // This class can't be default or copy constructed.
  MemoryMappedQueue() = delete;
  MemoryMappedQueue(const MemoryMappedQueue &other) = delete;
  MemoryMappedQueue &operator=(const MemoryMappedQueue &rhs) = delete;

  LocklessQueueMemory *memory() const {
    return reinterpret_cast<ipc_lib::LocklessQueueMemory *>(
        writable_mapping_.data());
  }

  const LocklessQueueMemory *const_memory() const {
    return reinterpret_cast<const LocklessQueueMemory *>(
        readonly_mapping_.data());
  }

  const LocklessQueueConfiguration &config() const { return config_; }

  LocklessQueue queue() const {
    return LocklessQueue(const_memory(), memory(), config());
  }

  absl::Span<char> GetMutableSharedMemory() const {
    return absl::Span<char>(static_cast<char *>(writable_mapping_.data()),
                            writable_mapping_.size());
  }

  absl::Span<const char> GetConstSharedMemory() const {
    return absl::Span<const char>(
        static_cast<const char *>(readonly_mapping_.data()),
        readonly_mapping_.size());
  }

 private:
  const LocklessQueueConfiguration config_;
  WritableShmMapping writable_mapping_;
  ReadOnlyShmMapping readonly_mapping_;
};

}  // namespace aos::ipc_lib

#endif  //  AOS_IPC_LIB_MEMORY_MAPPED_QUEUE_H_
