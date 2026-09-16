#ifndef AOS_IPC_LIB_MEMORY_MAPPED_QUEUE_H_
#define AOS_IPC_LIB_MEMORY_MAPPED_QUEUE_H_

#include <stddef.h>
#include <stdint.h>

#include <filesystem>
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

// Queue memory in a shared-memory file under shm_base, mapped read/write and
// read-only.  Shared with every other process that maps the same file, and it
// outlives all of them; see shm_mapping.h.
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

}  // namespace aos::ipc_lib

#endif  //  AOS_IPC_LIB_MEMORY_MAPPED_QUEUE_H_
