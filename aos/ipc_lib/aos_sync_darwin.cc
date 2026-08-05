#if !AOS_SYNC_DEBUG
#undef NDEBUG
#define NDEBUG
#endif

// The macOS half of the futex emulation in aos_sync_portable.cc.  These are
// all built on the XNU private os_sync API provided by
// <os/os_sync_wait_on_address.h>.

#include "aos/ipc_lib/aos_sync.h"

#include <os/clock.h>
#include <os/lock.h>
#include <os/os_sync_wait_on_address.h>
#include <os/proc.h>
#include <pthread.h>

#include <atomic>
#include <cerrno>
#include <cstdint>

#include "aos/ipc_lib/aos_sync_internal.h"

// OS_CLOCK_MACH_ABSOLUTE_TIME is only measured in nanoseconds on Apple Silicon;
// on Intel Macs it uses host-specific mach tick units that require conversion
// via mach_timebase_info(). We don't bother supporting Intel here.
#if !defined(__aarch64__) && !defined(__arm64__)
#error \
    "aos_sync OSX support requires Apple Silicon (arm64); " \
    "OS_CLOCK_MACH_ABSOLUTE_TIME is not nanoseconds on Intel Macs."
#endif

namespace aos::ipc_lib::sync {

pid_t do_get_tid() {
  uint64_t tid;
  pthread_threadid_np(NULL, &tid);
  return static_cast<pid_t>(tid) & FUTEX_TID_MASK;
}

void InstallAtforkHook() {
  ABSL_PCHECK(pthread_atfork(NULL, NULL, &atfork_child) == 0)
      << ": pthread_atfork(NULL, NULL, "
      << reinterpret_cast<void *>(&atfork_child) << ") failed";
}

int wait_on_address(aos_futex *addr1, int val1,
                    const struct timespec *timeout) {
  int ret;
  if (timeout != nullptr) {
    // Match Linux's FUTEX_WAIT semantics: the timespec is a *relative* timeout.
    // Callers that want an absolute deadline (e.g. the OSX condition_wait
    // path) must convert to a relative duration before calling.
    //
    // os_sync_wait_on_address_with_timeout takes its timeout in
    // OS_CLOCK_MACH_ABSOLUTE_TIME units; on Apple Silicon those are
    // nanoseconds (enforced by the arm64 #error check at the top of the file).
    int64_t timeout_ns = static_cast<int64_t>(timeout->tv_sec) * 1000000000LL +
                         static_cast<int64_t>(timeout->tv_nsec);
    if (timeout_ns <= 0) {
      return -ETIMEDOUT;
    }
    ret = os_sync_wait_on_address_with_timeout(
        addr1, val1, sizeof(*addr1), OS_SYNC_WAIT_ON_ADDRESS_SHARED,
        OS_CLOCK_MACH_ABSOLUTE_TIME, static_cast<uint64_t>(timeout_ns));
  } else {
    ret = os_sync_wait_on_address(addr1, val1, sizeof(*addr1),
                                  OS_SYNC_WAIT_ON_ADDRESS_SHARED);
  }

  // Per <os/os_sync_wait_on_address.h>, any non-negative return indicates
  // success -- the value is the number of waiters that remain blocked on the
  // address after this thread was woken. The Linux FUTEX_WAIT contract
  // doesn't expose that count, so we normalize any success to 0.
  if (ret >= 0) return 0;
  // Per <os/os_sync_wait_on_address.h>, EFAULT can be returned as a transient
  // error (e.g. low-memory / page-fault races) even for a valid address, and
  // the documented contract is for the caller to retry. Map it to
  // -EWOULDBLOCK so the existing wait/retry loops handle it like a spurious
  // wake instead of LOG(FATAL)ing.
  if (errno == EFAULT) return -EWOULDBLOCK;
  return -errno;
}

// Used by both normal wake and unlock_pi.
//
// Return value mirrors the Linux FUTEX_WAKE return contract as closely as macOS
// allows:
//   >= 0 : number of waiters that were woken (approximate, see below).
//   <  0 : -errno.
//
// `val1` mirrors the Linux FUTEX_WAKE semantics as closely as macOS allows:
//   1 : wake 1 waiter
//   any other value: wake all waiters
//
// The os_sync_wake_by_address_{any,all} APIs only report success (0) or
// failure (-1 with errno), and in particular ENOENT when there were no
// waiters. They don't tell us how many threads were actually woken, so we
// approximate "success" as 1 -- enough to let callers distinguish
// "woke at least one" from "woke none" from "error".
int sys_futex_wake(aos_futex *addr1, int val1) {
  int ret;
  if (val1 == 1) {
    ret = os_sync_wake_by_address_any(addr1, sizeof(*addr1),
                                      OS_SYNC_WAKE_BY_ADDRESS_SHARED);
  } else {
    ret = os_sync_wake_by_address_all(addr1, sizeof(*addr1),
                                      OS_SYNC_WAKE_BY_ADDRESS_SHARED);
  }
  if (ret == 0) {
    return 1;
  }
  if (errno == ENOENT) {
    // No waiters; nothing was woken. Matches Linux FUTEX_WAKE's return of 0.
    return 0;
  }
  return -errno;
}

int sys_futex_unlock_pi(aos_futex *addr1) {
  // Just wake as we are not using PI locks on macOS.
  // Also clear the lock value because mutex_unlock expects us to do it if it
  // called us.
  std::atomic_ref<uint32_t>(*addr1).store(0, std::memory_order_release);
  const int ret = os_sync_wake_by_address_any(addr1, sizeof(*addr1),
                                              OS_SYNC_WAKE_BY_ADDRESS_SHARED);
  if (ret == 0 || errno == ENOENT) {
    // Success, or there were simply no waiters to wake (which is normal when
    // the last unlock races with an interrupted waiter). Match Linux
    // FUTEX_UNLOCK_PI's 0-on-success contract.
    return 0;
  }
  return -errno;
}

}  // namespace aos::ipc_lib::sync
