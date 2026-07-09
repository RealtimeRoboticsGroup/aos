#if !AOS_SYNC_DEBUG
#undef NDEBUG
#define NDEBUG
#endif

#include "aos/ipc_lib/aos_sync.h"

#include <stdio.h>

#ifdef _MSC_VER
#include <basetsd.h>
using ssize_t = SSIZE_T;
#endif

#include <atomic>
#include <cassert>
#include <cerrno>
#include <cinttypes>
#include <climits>
#include <ostream>

#include "absl/base/call_once.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/ipc_lib/aos_sync_internal.h"
#include "aos/ipc_lib/shm_observers.h"
#include "aos/macros.h"

using ::aos::linux_code::ipc_lib::RunShmObservers;

// This code was originally based on
// <https://www.akkadia.org/drepper/futex.pdf>, but is has since evolved a lot.
// However, that still has useful information.
//
// Finding information about actually using futexes is really REALLY hard, so
//   here's a list of the stuff that I've used:
// futex(7) has a really high-level overview.
// <http://locklessinc.com/articles/futex_cheat_sheet/> describes some of the
//   operations in a bit more detail than most places.
// <http://locklessinc.com/articles/mutex_cv_futex/> is the basis of our
//   implementations (before PI).
// <http://lwn.net/Articles/360699/> has a nice overview of futexes in late 2009
//   (fairly recent compared to everything else...).
// <https://www.kernel.org/doc/Documentation/pi-futex.txt>,
//   <https://www.kernel.org/doc/Documentation/futex-requeue-pi.txt>,
//   <https://www.kernel.org/doc/Documentation/robust-futexes.txt>,
//   and <https://www.kernel.org/doc/Documentation/robust-futex-ABI.txt> are all
//   useful references.
// The kernel source (kernel/futex.c) has some useful comments about what the
//   various operations do (except figuring out which argument goes where in the
//   syscall is still confusing).
// futex(2) is basically useless except for describing the order of the
//   arguments (it only has high-level descriptions of what some of the
//   operations do, and some of them are wrong in Wheezy).
// glibc's nptl pthreads implementation is the intended user of most of these
//   things, so it is also a good place to look for examples. However, it is all
//   very hard to read because it supports ~20 different kinds of mutexes and
//   several variations of condition variables, and some of the pieces of code
//   are only written in assembly.
// set_robust_list(2) is wrong in Wheezy (it doesn't actually take a TID
//   argument).
//
// Can't use PRIVATE futex operations because they use the pid (or something) as
//   part of the hash.
//
// ThreadSanitizer understands how these mutexes etc work. It appears to be able
// to figure out the happens-before relationship from the __ATOMIC_SEQ_CST
// atomic primitives.
//
// Remember that EAGAIN and EWOUDBLOCK are the same! (ie if you get EAGAIN from
// FUTEX_WAIT, the docs call it EWOULDBLOCK...)

// Values for an aos_mutex.futex (kernel-mandated):
// 0 = unlocked
// TID = locked, not contended
// |FUTEX_WAITERS = there are waiters (aka contended)
// |FUTEX_OWNER_DIED = old owner died
//
// Values for an aos_futex being used directly:
// 0 = unset
// 1 = set
//
// The value of an aos_condition is just a generation counter.

#ifdef AOS_SANITIZER_thread
extern "C" void AnnotateHappensBefore(const char *file, int line,
                                      uintptr_t addr);
extern "C" void AnnotateHappensAfter(const char *file, int line,
                                     uintptr_t addr);
#define ANNOTATE_HAPPENS_BEFORE(address)    \
  AnnotateHappensBefore(__FILE__, __LINE__, \
                        reinterpret_cast<uintptr_t>(address))
#define ANNOTATE_HAPPENS_AFTER(address) \
  AnnotateHappensAfter(__FILE__, __LINE__, reinterpret_cast<uintptr_t>(address))
#else
#define ANNOTATE_HAPPENS_BEFORE(address)
#define ANNOTATE_HAPPENS_AFTER(address)
#endif

namespace aos::ipc_lib::sync {
namespace {

// This gets called by functions before LOG(FATAL)ing with error messages
// that would be incorrect if the error was caused by a process forking without
// initialize_in_new_thread getting called in the fork.
void check_cached_tid(pid_t tid) {
  pid_t actual = do_get_tid();
  if (tid != actual) {
    ABSL_LOG(FATAL)
        << "task " << static_cast<intmax_t>(tid) << " forked into "
        << static_cast<intmax_t>(actual)
        << " without letting aos_sync know so we're not really sure "
           "what's going on";
  }
}

// The common implementation for everything that wants to lock a mutex.
// If signals_fail is false, the function will try again if the wait syscall is
// interrupted by a signal.
inline int mutex_get(aos_mutex *m, bool signals_fail) {
  const uint32_t tid = get_tid();
  my_robust_list::Adder adder(m);
  const int r = mutex_do_get(m, signals_fail, tid);
  if (r == 0 || r == 1) adder.Add();
  return r;
}

}  // namespace

void atfork_child() {
  // The next time get_tid() is called, it will set everything up again.
  thread_state()->tid = 0;
}

namespace {
// TU-local so ELF can use a cheap TLS model; see the ThreadState comment in
// aos_sync_internal.h.  Zero-initialized, which is the state every consumer
// other than RobustListCleaner treats as "initialize me first".
thread_local ThreadState g_thread_state;
}  // namespace

ThreadState *thread_state() { return &g_thread_state; }

namespace my_robust_list {

uintptr_t robust_list_offset = 0;

void Init() {
  aos_robust_list_head *const head = &thread_state()->robust_head;
  // It starts out just pointing back to itself.
  head->next = robust_head_next_value(head);
  head->futex_offset = static_cast<ssize_t>(offsetof(aos_mutex, futex)) -
                       static_cast<ssize_t>(offsetof(aos_mutex, next));
  head->pending_next = 0;
  RegisterRobustList(reinterpret_cast<void *>(robust_head_next_value(head)),
                     sizeof(*head));
  if (kRobustListDebug) {
    printf("%" PRId32 ": init done\n", get_tid());
  }
}

void SetRobustListOffset(uintptr_t offset) {
  // Calling this from a thread which has never used aos_sync would walk a
  // zero-initialized robust_head off into garbage.  Only the destructor in
  // aos_sync_portable.cc is written to tolerate the uninitialized state (it
  // has to be; see the contract there); everything else must come through
  // get_tid()'s lazy initialization first.
  aos_robust_list_head *const head = &thread_state()->robust_head;
  ABSL_CHECK(head->next != 0);
  const uintptr_t offset_change = offset - robust_list_offset;
  robust_list_offset = offset;
  aos_mutex *m = robust_head_mutex(head);
  // Update the offset contained in each of the mutexes which is already locked.
  while (!next_is_head(head, m->next)) {
    m->next += offset_change;
    m = next_to_mutex(head, m->next);
  }
}

bool HaveLockedMutexes() {
  // A zero-initialized (never-used-aos_sync) thread's robust_head would
  // compare as "locked mutexes exist", which is the wrong answer.
  aos_robust_list_head *const head = &thread_state()->robust_head;
  ABSL_CHECK(head->next != 0);
  return head->next != robust_head_next_value(head);
}

}  // namespace my_robust_list

int futex_mutex_do_trylock(aos_mutex *m, uint32_t tid,
                           my_robust_list::Adder *adder) {
  // Try an atomic 0->TID transition.
  uint32_t c = compare_and_swap_val(&m->futex, 0, tid);

  if (c != 0) {
    if (AOS_LIKELY((c & FUTEX_OWNER_DIED) == 0)) {
      // Somebody else had it locked; we failed.
      return 4;
    }
    // FUTEX_OWNER_DIED was set, so we have to call into the kernel (or its
    // userspace emulation) to deal with resetting it.
    const int ret = sys_futex_wait(FUTEX_TRYLOCK_PI, &m->futex, 0, NULL);
    if (ret == 0) {
      adder->Add();
      // Only clear the owner died if somebody else didn't do the recovery
      // and then unlock before our TRYLOCK happened.
      return mutex_finish_lock(m);
    }
    // EWOULDBLOCK means that somebody else beat us to it.
    if (AOS_LIKELY(ret == -EWOULDBLOCK)) {
      return 4;
    }
    thread_state()->robust_head.pending_next = 0;
    errno = -ret;
    ABSL_PLOG(FATAL) << "FUTEX_TRYLOCK_PI(" << (&m->futex)
                     << ", 0, NULL) failed";
  }

  lock_pthread_mutex(m);
  adder->Add();
  return 0;
}

void futex_mutex_do_unlock(aos_mutex *m, uint32_t tid) {
  // If the atomic TID->0 transition fails (ie FUTEX_WAITERS is set),
  if (!compare_and_swap(&m->futex, tid, 0)) {
    // sys_futex_unlock_pi wakes a waiter for us.
    const int ret = sys_futex_unlock_pi(&m->futex);
    if (ret != 0) {
      thread_state()->robust_head.pending_next = 0;
      errno = -ret;
      ABSL_PLOG(FATAL) << "FUTEX_UNLOCK_PI(" << (&m->futex) << ") failed";
    }
  } else {
    // There aren't any waiters, so no need to wake anybody.
  }
}

void initialize_in_new_thread() {
  // No synchronization necessary in most of this because it's all thread-local!

  thread_state()->tid = do_get_tid();

  static absl::once_flag once;
  absl::call_once(once, InstallAtforkHook);

  my_robust_list::Init();
  TouchRobustListCleaner();
}

}  // namespace aos::ipc_lib::sync

// Everything below here is the public interface, which lives at the global
// scope.  Pull the implementation namespace in rather than qualifying every
// single use of it.
using namespace aos::ipc_lib::sync;  // NOLINT(build/namespaces)

int mutex_lock(aos_mutex *m) { return mutex_get(m, true); }
int mutex_grab(aos_mutex *m) { return mutex_get(m, false); }

void mutex_unlock(aos_mutex *m) {
  RunShmObservers run_observers(m, true);
  const uint32_t tid = get_tid();
  if (kPrintOperations) {
    printf("%" PRId32 ": %p unlock\n", tid, m);
  }

  const uint32_t value =
      std::atomic_ref<uint32_t>(m->futex).load(std::memory_order_seq_cst);
  if (AOS_UNLIKELY((value & FUTEX_TID_MASK) != tid)) {
    thread_state()->robust_head.pending_next = 0;
    check_cached_tid(tid);
    if ((value & FUTEX_TID_MASK) == 0) {
      ABSL_LOG(FATAL) << "multiple unlock of aos_mutex " << m << " by " << tid;
    } else {
      ABSL_LOG(FATAL) << "aos_mutex " << m << " is locked by "
                      << (value & FUTEX_TID_MASK) << ", not " << tid;
    }
  }

  my_robust_list::Remover remover(m);
  unlock_pthread_mutex(m);

  mutex_do_unlock(m, tid);
}

int mutex_trylock(aos_mutex *m) {
  RunShmObservers run_observers(m, true);
  const uint32_t tid = get_tid();
  if (kPrintOperations) {
    printf("%" PRId32 ": %p trylock\n", tid, m);
  }
  my_robust_list::Adder adder(m);

  return mutex_do_trylock(m, tid, &adder);
}

bool mutex_islocked(const aos_mutex *m) {
  const uint32_t tid = get_tid();

  const uint32_t value =
      std::atomic_ref<uint32_t>(const_cast<uint32_t &>(m->futex))
          .load(std::memory_order_relaxed);
  return (value & FUTEX_TID_MASK) == tid;
}

bool mutex_owner_is_dead_from_value(uint32_t value) {
  return (value & FUTEX_OWNER_DIED) != 0;
}

uint32_t mutex_load_value(const aos_mutex *m) {
  return std::atomic_ref<uint32_t>(const_cast<uint32_t &>(m->futex))
      .load(std::memory_order_acquire);
}

uint32_t mutex_owner(const aos_mutex *m) {
  return mutex_owner_from_value(mutex_load_value(m));
}

bool mutex_owner_is_dead(const aos_mutex *m) {
  const uint32_t value =
      std::atomic_ref<uint32_t>(const_cast<uint32_t &>(m->futex))
          .load(std::memory_order_relaxed);
  return mutex_owner_is_dead_from_value(value);
}

bool mutex_pretend_owner_died_for_testing(aos_mutex *m, uint32_t tid) {
  // This is only used in tests, so use seq_cst rather than reasoning about
  // what weaker orderings would be safe.
  std::atomic_ref<uint32_t> ref(m->futex);
  uint32_t value = ref.load(std::memory_order_seq_cst);
  while (mutex_owner_from_value(value) == tid) {
    const uint32_t new_value = FUTEX_OWNER_DIED | (value & FUTEX_WAITERS);
    if (ref.compare_exchange_strong(value, new_value,
                                    std::memory_order_seq_cst)) {
      return true;
    }
    // The compare failed and reloaded value, so recheck the owner.
  }
  return false;
}

void death_notification_init(aos_mutex *m) {
  const uint32_t tid = get_tid();
  if (kPrintOperations) {
    printf("%" PRId32 ": %p death_notification start\n", tid, m);
  }
  my_robust_list::Adder adder(m);
  {
    RunShmObservers run_observers(m, true);
    ABSL_CHECK(compare_and_swap(&m->futex, 0, tid));
  }
  adder.Add();
}

void death_notification_release(aos_mutex *m) {
  RunShmObservers run_observers(m, true);

#ifndef NDEBUG
  // Verify it's "locked", like it should be.
  {
    const uint32_t tid = get_tid();
    if (kPrintOperations) {
      printf("%" PRId32 ": %p death_notification release\n", tid, m);
    }
    const uint32_t value =
        std::atomic_ref<uint32_t>(m->futex).load(std::memory_order_seq_cst);
    assert((value & ~FUTEX_WAITERS) == tid);
  }
#endif

  my_robust_list::Remover remover(m);
  ANNOTATE_HAPPENS_BEFORE(m);
  const int ret = sys_futex_unlock_pi(&m->futex);
  if (ret != 0) {
    thread_state()->robust_head.pending_next = 0;
    errno = -ret;
    ABSL_PLOG(FATAL) << "FUTEX_UNLOCK_PI(" << &m->futex << ") failed";
  }
}

void condition_signal(aos_condition *c, aos_mutex *m) {
  condition_wake(c, m, 0);
}

void condition_broadcast(aos_condition *c, aos_mutex *m) {
  condition_wake(c, m, INT_MAX);
}

int futex_wait_timeout(aos_futex *m, const struct timespec *timeout) {
  RunShmObservers run_observers(m, false);
  const int ret = sys_futex_wait(FUTEX_WAIT, m, 0, timeout);
  if (ret != 0) {
    if (ret == -EINTR) {
      return 1;
    } else if (ret == -ETIMEDOUT) {
      return 2;
    } else if (ret != -EWOULDBLOCK) {
      errno = -ret;
      return -1;
    }
  }
  ANNOTATE_HAPPENS_AFTER(m);
  return 0;
}

int futex_wait(aos_futex *m) { return futex_wait_timeout(m, NULL); }

int futex_set_value(aos_futex *m, uint32_t value) {
  RunShmObservers run_observers(m, false);
  ANNOTATE_HAPPENS_BEFORE(m);
  std::atomic_ref<uint32_t>(*m).store(value, std::memory_order_seq_cst);
  const int r = sys_futex_wake(m, INT_MAX - 4096);
  if (AOS_UNLIKELY(static_cast<unsigned int>(r) >
                   static_cast<unsigned int>(-4096))) {
    errno = -r;
    return -1;
  } else {
    return r;
  }
}

int futex_set(aos_futex *m) { return futex_set_value(m, 1); }

int futex_unset(aos_futex *m) {
  return !std::atomic_ref<uint32_t>(*m).exchange(0, std::memory_order_seq_cst);
}

namespace aos::linux_code::ipc_lib {

// Sets an extra offset between mutexes and the value we use for them in the
// robust list (only the forward pointers). This is used to work around a kernel
// bug by keeping a second set of mutexes which is always writable so the kernel
// won't go into an infinite loop when trying to unlock them.
void SetRobustListOffset(ptrdiff_t offset) {
  my_robust_list::SetRobustListOffset(offset);
}

// Returns true iff there are any mutexes locked by the current thread.
// This is mainly useful for testing.
bool HaveLockedMutexes() { return my_robust_list::HaveLockedMutexes(); }

}  // namespace aos::linux_code::ipc_lib
