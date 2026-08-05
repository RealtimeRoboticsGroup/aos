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
//
// An OS may also back some mutexes with a real kernel object instead of the
// futex word (see aos_sync_windows.cc for the only one that does today); its
// mutex_do_* dispatch to these portable_mutex_do_* fallbacks for the mutexes
// that stay on the futex word.

#include <stdio.h>
#include <time.h>

#include <atomic>
#include <cassert>
#include <cerrno>
#include <chrono>
#include <cinttypes>
#include <climits>
#include <ostream>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/ipc_lib/aos_sync.h"
#include "aos/ipc_lib/aos_sync_internal.h"
#include "aos/ipc_lib/shm_observers.h"
#include "aos/macros.h"
#include "aos/time/time.h"

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
// (abort, signal, etc.). Neither Windows nor macOS has an equivalent, so we
// piggy-back on thread_local destruction instead. That means:
//   * We only run when the thread tears its thread_locals down: pthread's TLS
//     destructors on macOS, the CRT's TLS callback on Windows. A thread that
//     goes away without that happening -- TerminateThread, or any thread other
//     than the one calling TerminateProcess/ExitProcess -- leaves its mutexes
//     held with no notification. If the whole process is going down cleanly,
//     we don't really care.
//   * Destruction order between thread_local objects is unspecified, so in
//     the pathological case where another thread_local destructor is still
//     holding an aos_mutex when this cleaner runs (or vice versa), the
//     cleanup might miss that mutex.
// Note that the kernel-backed path on Linux isn't a complete guarantee
// either -- the OOM killer, in particular, skips robust-futex processing --
// so callers already have to cope with mutexes that never get the
// FUTEX_OWNER_DIED treatment on owner death. macOS only uses this for
// intra-process dev/test, where thread death almost always means process
// death anyway. On Windows this walk only carries the notification for
// process-private mutexes and for death-notification words (whose backstop
// is RobustOwnershipTracker's liveness probe); shared-memory mutexes get a
// stronger, kernel-enforced version regardless, because the named kernel
// mutex backing them is abandoned on any thread termination -- including
// TerminateProcess, which never runs this walk.  See the design comment at
// the top of aos_sync_windows.cc.
struct RobustListCleaner {
  // CONTRACT: this destructor must be safe to run in ANY exiting thread, at
  // any point, against nothing but zero-initialized thread_local state --
  // whether or not the thread ever used aos_sync.
  //
  // That is forced by the weakest platform and adopted everywhere: MSVC has
  // no lazy thread_local construction, so it registers this destructor
  // eagerly in every thread at thread attach (the CRT TLS callback) and runs
  // it in every thread at exit.  The Itanium ABI (Linux/macOS) only registers
  // it in threads that odr-use the object (the Touch() call in
  // initialize_in_new_thread), which would let this code assume an
  // initialized robust_head -- but depending on that would leave the
  // stronger assumption silently broken on Windows, so we write to the
  // weaker guarantee on every platform.
  ~RobustListCleaner() {
    ThreadState *state = thread_state();
    // A thread which never called initialize_in_new_thread() has a
    // zero-initialized robust_head; walking that would dereference garbage.
    if (state->robust_head.next == 0) {
      return;
    }
    while (!next_is_head(&state->robust_head, state->robust_head.next)) {
      aos_mutex *m =
          next_to_mutex(&state->robust_head, state->robust_head.next);
      // Note: if we are walking a mutex which is inside a mapping that another
      // thread is tearing down, we could be valid, but then segfault on the
      // load below.  The likelihood of that is low enough, and enough would
      // have to go wrong first that this isn't worrying about.
      if (!IsValidAddress(m, sizeof(aos_mutex))) {
        break;
      }
      state->robust_head.next = m->next;
      m->next = 0;
      m->previous = nullptr;

      // Mirror the kernel's handle_futex_death(): only mark a mutex this
      // thread still owns, and do it with a compare-and-swap loop rather than
      // a blind store.  The TID check keeps us from clobbering a slot that a
      // racing recoverer (RobustOwnershipTracker's liveness probe can declare
      // this thread dead before this destructor finishes) already handed to a
      // new owner, and the CAS keeps us from erasing a FUTEX_WAITERS bit a
      // waiter sets concurrently.
      uint32_t val =
          std::atomic_ref<uint32_t>(m->futex).load(std::memory_order_relaxed);
      while ((val & FUTEX_TID_MASK) == static_cast<uint32_t>(state->tid)) {
        const uint32_t new_val = FUTEX_OWNER_DIED | (val & FUTEX_WAITERS);
        const uint32_t prev = compare_and_swap_val(&m->futex, val, new_val);
        if (prev == val) {
          RobustListCleanerWake(&m->futex);
          break;
        }
        val = prev;
      }
    }
  }
  void Touch() const {}
};
thread_local RobustListCleaner robust_list_cleaner;

}  // namespace
}  // namespace my_robust_list

void TouchRobustListCleaner() { my_robust_list::robust_list_cleaner.Touch(); }

int portable_mutex_do_get(aos_mutex *m, bool signals_fail, uint32_t tid) {
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

    const int ret = sys_futex_wait(FUTEX_WAIT, &m->futex, v, nullptr);
    if (ret != 0) {
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

void portable_mutex_do_unlock(aos_mutex *m, uint32_t tid) {
  futex_mutex_do_unlock(m, tid);
}

int portable_mutex_do_trylock(aos_mutex *m, uint32_t tid,
                              my_robust_list::Adder *adder) {
  return futex_mutex_do_trylock(m, tid, adder);
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
  // deadline.  On Linux that deadline goes straight to the kernel, which waits
  // on an absolute timeout (FUTEX_WAIT_REQUEUE_PI, like the PI mutexes'
  // FUTEX_LOCK_PI).  We have no such primitive here: the OS waits we build on
  // (WaitForSingleObject / WaitOnAddress on Windows, os_sync_wait_on_address on
  // macOS) only take a *relative* timeout, so we reduce the deadline to a
  // relative interval and re-wait for the remaining time until the absolute
  // deadline actually passes.  Those primitives round up to the system timer
  // tick and can fire before the requested interval, so a single relative wait
  // may report a timeout early; looping against the deadline avoids that.
  namespace chrono = ::std::chrono;
  int ret;
  while (true) {
    struct timespec relative_timeout;
    struct timespec *relative_timeout_ptr = nullptr;
    if (end_time != nullptr) {
      const chrono::nanoseconds end_ns = chrono::seconds(end_time->tv_sec) +
                                         chrono::nanoseconds(end_time->tv_nsec);
      const chrono::nanoseconds relative_ns =
          end_ns - aos::monotonic_clock::now().time_since_epoch();
      if (relative_ns <= chrono::nanoseconds::zero()) {
        // Deadline reached.
        ret = -ETIMEDOUT;
        break;
      }
      relative_timeout = aos::time::to_timespec(relative_ns);
      relative_timeout_ptr = &relative_timeout;
    }

    ret = sys_futex_wait(FUTEX_WAIT, c, wait_start, relative_timeout_ptr);
    // Anything other than an early timeout (a wake, a value change, an error,
    // or no deadline at all) is final.  Only a timeout gets re-checked against
    // the absolute deadline above.
    if (ret != -ETIMEDOUT || end_time == nullptr) {
      break;
    }
  }

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
