#include "aos/events/aio_unix.h"

#include <pthread.h>

#include <atomic>
#include <cerrno>
#include <mutex>

#include "absl/log/absl_log.h"

namespace aos::internal {
namespace {

// Private to this file: reached through the accessors below so no backend can
// take a reference to one and forget which memory order it wanted.
std::atomic<size_t> fork_count{0};
std::atomic<size_t> parent_fork_count{0};
std::once_flag fork_counters_once;

}  // namespace

size_t ForkCount() { return fork_count.load(std::memory_order_relaxed); }

size_t ParentForkCount() {
  return parent_fork_count.load(std::memory_order_relaxed);
}

void RegisterForkCounters() {
  std::call_once(fork_counters_once, []() {
    pthread_atfork(
        // No prepare handler: nothing here needs quiescing before the fork,
        // only fixing up after it.
        nullptr,
        // Parent: runs in the forking process once fork() returns.
        []() { parent_fork_count.fetch_add(1, std::memory_order_relaxed); },
        // Child: runs in the new process once fork() returns.
        []() { fork_count.fetch_add(1, std::memory_order_relaxed); });
  });
}

void CheckRegisteredFdOpenAfterFork(int fd) {
  const int flags = fcntl(fd, F_GETFD);
  ABSL_PCHECK(flags != -1 || errno != EBADF)
      << ": a forked child closed fd " << fd
      << " while it was still registered with this Aio.  Call "
         "ForgetClosedFd() for every registered fd the child closes, before "
         "its next use of the loop.";
}

void CheckOwnFdOpenAfterFork(int fd, const char *owner) {
  const int flags = fcntl(fd, F_GETFD);
  ABSL_PCHECK(flags != -1 || errno != EBADF)
      << ": a forked child closed fd " << fd
      << ", which belongs to this Aio's own " << owner
      << ".  The loop cannot run without it, so the child must leave it open.";
}

void DieOwnFdStillRegistered(int fd, const char *owner) {
  ABSL_LOG(FATAL) << "fd " << fd << ", now this Aio's own " << owner
                  << ", is still registered by the caller: it was closed "
                     "without ForgetClosedFd(), and the kernel reused its "
                     "number.  Call ForgetClosedFd() right after close().";
}

void DieForgettingOwnFd(int fd, const char *owner) {
  ABSL_LOG(FATAL) << "ForgetClosedFd(" << fd << "): fd " << fd
                  << " belongs to this Aio's own " << owner
                  << ", not to a registration made with On*() or "
                     "AsyncRead()/AsyncWrite().";
}

}  // namespace aos::internal
