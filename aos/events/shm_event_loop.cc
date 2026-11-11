#include "aos/events/shm_event_loop.h"

#include <filesystem>
#include <memory>
#include <utility>

#include "absl/flags/flag.h"

#include "aos/ipc_lib/memory_mapped_queue.h"

// This value is affected by the umask of the process which is calling it
// and is set to the user's value by default (check yours running `umask` on
// the command line).
// Any file mode requested is transformed using: mode & ~umask and the default
// umask is 0022 (allow any permissions for the user, dont allow writes for
// groups or others).
// See https://man7.org/linux/man-pages/man2/umask.2.html for more details.
// WITH THE DEFAULT UMASK YOU WONT ACTUALLY GET THESE PERMISSIONS :)
ABSL_FLAG(uint32_t, permissions, 0770,
          "Permissions to make shared memory files and folders, "
          "affected by the process's umask. "
          "See shm_event_loop.cc for more details.");

namespace aos {

ShmEventLoop::ShmEventLoop(const Configuration *configuration)
    : QueueEventLoop(configuration), shm_base_(ShmBase()) {}

ShmEventLoop::ShmEventLoop(const Configuration *configuration, Aio *aio)
    : QueueEventLoop(configuration, aio), shm_base_(ShmBase()) {}

ShmEventLoop::~ShmEventLoop() { CheckNotRunningForDestruction(); }

namespace {

// One user's handle on a MemoryMappedQueue shared through
// GetOrCreateMemoryMappedQueue().  The mapping lives as long as any handle on
// it, in this or any other ShmEventLoop in the process.
class SharedMemoryMappedQueue : public ipc_lib::QueueMemory {
 public:
  explicit SharedMemoryMappedQueue(
      std::shared_ptr<ipc_lib::MemoryMappedQueue> queue)
      : queue_(std::move(queue)) {}

  ipc_lib::LocklessQueueMemory *memory() const override {
    return queue_->memory();
  }
  const ipc_lib::LocklessQueueMemory *const_memory() const override {
    return queue_->const_memory();
  }
  const ipc_lib::LocklessQueueConfiguration &config() const override {
    return queue_->config();
  }

 private:
  const std::shared_ptr<ipc_lib::MemoryMappedQueue> queue_;
};

}  // namespace

std::unique_ptr<ipc_lib::QueueMemory> ShmEventLoop::MakeQueueMemory(
    const Channel *channel) {
  // Every sender, fetcher and watcher on a channel in this process shares one
  // mapping, rather than mapping and pre-faulting the queue once each.
  return std::make_unique<SharedMemoryMappedQueue>(
      ipc_lib::GetOrCreateMemoryMappedQueue(
          shm_base_,
          static_cast<std::filesystem::perms>(absl::GetFlag(FLAGS_permissions)),
          configuration(), channel));
}

}  // namespace aos
