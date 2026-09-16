#include "aos/events/shm_event_loop.h"

#include <filesystem>

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

std::unique_ptr<ipc_lib::QueueMemory> ShmEventLoop::MakeQueueMemory(
    const Channel *channel) {
  return std::make_unique<ipc_lib::MemoryMappedQueue>(
      shm_base_,
      static_cast<std::filesystem::perms>(absl::GetFlag(FLAGS_permissions)),
      configuration(), channel);
}

}  // namespace aos
