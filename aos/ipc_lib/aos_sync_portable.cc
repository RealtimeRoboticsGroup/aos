#if !AOS_SYNC_DEBUG
#undef NDEBUG
#define NDEBUG
#endif

// The futex emulation shared by every OS which doesn't have futexes.
//
// None of these OSes have anything like the kernel's priority-inheritance
// futex operations, or its robust list, so this builds both out of a plain CAS
// on the futex word plus whatever wait-on-address primitive the OS does have
// (wait_on_address(), implemented per-OS).  The behavioral differences from
// Linux which fall out of that are documented on aos_mutex in aos_sync.h.

#include <stdio.h>
#include <time.h>

#include <atomic>
#include <cassert>
#include <cerrno>
#include <cinttypes>
#include <climits>
#include <ostream>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/ipc_lib/aos_sync.h"
#include "aos/ipc_lib/aos_sync_internal.h"
#include "aos/ipc_lib/shm_observers.h"
#include "aos/macros.h"

using ::aos::linux_code::ipc_lib::RunShmObservers;

namespace aos::ipc_lib::sync {

// There is no OS-level robust list to register with; RobustListCleaner below
// walks ours by hand instead.
void RegisterRobustList(void * /*head*/, size_t /*size*/) {}

int sys_futex_wait(int op, aos_futex *addr1, int val1,
                   const struct timespec *timeout) {
  if (op == FUTEX_TRYLOCK_PI) {
    uint32_t tid = get_tid();
    uint32_t val =
        std::atomic_ref<uint32_t>(*addr1).load(std::memory_order_acquire);
    if ((val & FUTEX_TID_MASK) == 0) {
      uint32_t new_val = tid;
      if (val & FUTEX_OWNER_DIED) new_val |= FUTEX_OWNER_DIED;
      if (val & FUTEX_WAITERS) new_val |= FUTEX_WAITERS;
      if (compare_and_swap(addr1, val, new_val)) {
        return 0;
      }
    }
    return -EWOULDBLOCK;
  }

  return wait_on_address(addr1, val1, timeout);
}

namespace my_robust_list {
namespace {

// Userspace fallback for the kernel's robust-futex list.
//
// On Linux the kernel walks robust_head on thread death and marks any
// still-held mutexes with FUTEX_OWNER_DIED, even if the thread died abruptly
// (abort, signal, etc.). The OSes which get here have no equivalent, so we
// piggy-back on thread_local destruction instead. That means:
//   * We only run when the thread exits cleanly through its runtime's TLS
//     teardown. If the whole process is going down, we don't really care.
//   * Destruction order between thread_local objects is unspecified, so in
//     the pathological case where another thread_local destructor is still
//     holding an aos_mutex when this cleaner runs (or vice versa), the
//     cleanup might miss that mutex.
// Note that the kernel-backed path on Linux isn't a complete guarantee
// either -- the OOM killer, in particular, skips robust-futex processing --
// so callers already have to cope with mutexes that never get the
// FUTEX_OWNER_DIED treatment on owner death. In practice thread death on
// macOS almost always means process death, and this codepath is only used
// for intra-process dev/test, so the looser guarantee is acceptable.
struct RobustListCleaner {
  ~RobustListCleaner() {
    while (!next_is_head(robust_head.next)) {
      aos_mutex *m = next_to_mutex(robust_head.next);
      robust_head.next = m->next;

      uint32_t val =
          std::atomic_ref<uint32_t>(m->futex).load(std::memory_order_relaxed);
      uint32_t new_val = FUTEX_OWNER_DIED;
      if (val & FUTEX_WAITERS) new_val |= FUTEX_WAITERS;
      std::atomic_ref<uint32_t>(m->futex).store(new_val,
                                                std::memory_order_seq_cst);
      sys_futex_wake(&m->futex, 1);
    }
  }
  void Touch() const {}
};
thread_local RobustListCleaner robust_list_cleaner;

}  // namespace
}  // namespace my_robust_list

void TouchRobustListCleaner() { my_robust_list::robust_list_cleaner.Touch(); }

int mutex_do_get(aos_mutex *m, bool signals_fail,
                 const struct timespec *timeout, uint32_t tid) {
  RunShmObservers run_observers(m, true);
  if (kPrintOperations) {
    printf("%" PRId32 ": %p do_get\n", tid, m);
  }

  while (true) {
    uint32_t v =
        std::atomic_ref<uint32_t>(m->futex).load(std::memory_order_acquire);

    if ((v & FUTEX_TID_MASK) == tid) {
      ABSL_LOG(FATAL) << "multiple lock of " << m << " by " << tid;
    }

    // If it's unlocked (0) or marked as owner died (but no owner TID), we can
    // take it.
    if ((v & FUTEX_TID_MASK) == 0) {
      uint32_t new_val = tid;
      if (v & FUTEX_OWNER_DIED) new_val |= FUTEX_OWNER_DIED;
      if (v & FUTEX_WAITERS) new_val |= FUTEX_WAITERS;

      if (compare_and_swap(&m->futex, v, new_val)) {
        return mutex_finish_lock(m);
      }
      // Retry if CAS failed.
      continue;
    }

    if (!(v & FUTEX_WAITERS)) {
      if (!compare_and_swap(&m->futex, v, v | FUTEX_WAITERS)) continue;
      v |= FUTEX_WAITERS;
    }

    const int ret = sys_futex_wait(FUTEX_WAIT, &m->futex, v, timeout);
    if (ret != 0) {
      if (ret == -ETIMEDOUT) return 3;
      if (ret == -EINTR) {
        if (signals_fail) return 2;
        continue;
      }
      // EWOULDBLOCK (or similar from os_sync emulation) just means retry
      if (ret == -EWOULDBLOCK) continue;

      errno = -ret;
      ABSL_PLOG(FATAL) << "sys_futex_wait failed";
    }
  }
}

void mutex_do_unlock(aos_mutex *m, uint32_t tid) {
  // If the atomic TID->0 transition fails (ie FUTEX_WAITERS is set),
  if (!compare_and_swap(&m->futex, tid, 0)) {
    // sys_futex_unlock_pi handles waking any waiters.
    const int ret = sys_futex_unlock_pi(&m->futex);
    if (ret != 0) {
      my_robust_list::robust_head.pending_next = 0;
      errno = -ret;
      ABSL_PLOG(FATAL) << "FUTEX_UNLOCK_PI(" << (&m->futex) << ") failed";
    }
  } else {
    // There aren't any waiters, so no need to wake anybody.
  }
}

void condition_wake(aos_condition *c, aos_mutex * /*m*/, int number_requeue) {
  RunShmObservers run_observers(c, true);
  std::atomic_ref<uint32_t>(*c).fetch_add(1, std::memory_order_seq_cst);
  // Simple wake
  sys_futex_wake(c, number_requeue == 0 ? 1 : INT_MAX);
}

}  // namespace aos::ipc_lib::sync

// condition_wait is part of the public interface, which lives at the global
// scope.
using namespace aos::ipc_lib::sync;  // NOLINT(build/namespaces)

int condition_wait(aos_condition *c, aos_mutex *m, struct timespec *end_time) {
  RunShmObservers run_observers(c, false);
  const uint32_t wait_start =
      std::atomic_ref<uint32_t>(*c).load(std::memory_order_seq_cst);

  mutex_unlock(m);

  // Callers (e.g. Condition::WaitTimed) pass an absolute CLOCK_MONOTONIC
  // deadline, but our sys_futex_wait shim here mirrors Linux FUTEX_WAIT
  // and takes a relative timeout. Convert here.
  struct timespec relative_timeout;
  struct timespec *relative_timeout_ptr = nullptr;
  if (end_time != nullptr) {
    struct timespec now;
    ABSL_PCHECK(clock_gettime(CLOCK_MONOTONIC, &now) == 0);
    int64_t ns =
        static_cast<int64_t>(end_time->tv_sec - now.tv_sec) * 1000000000LL +
        static_cast<int64_t>(end_time->tv_nsec - now.tv_nsec);
    if (ns < 0) ns = 0;
    relative_timeout.tv_sec = static_cast<time_t>(ns / 1000000000LL);
    relative_timeout.tv_nsec = static_cast<long>(ns % 1000000000LL);
    relative_timeout_ptr = &relative_timeout;
  }

  // Wait
  int ret = sys_futex_wait(FUTEX_WAIT, c, wait_start, relative_timeout_ptr);

  // Re-lock. mutex_lock returns 1 if the previous owner died while holding
  // the mutex; match the Linux condition_wait path and propagate that (it
  // takes priority over a timeout, since the caller needs to know it now
  // owns recovery of whatever the dead owner was protecting).
  const int relock = mutex_lock(m);
  assert(relock == 0 || relock == 1);

  if (relock == 1) {
    return 1;
  }
  if (ret == -ETIMEDOUT) {
    return -1;
  }
  return 0;
}
