#ifndef AOS_IPC_LIB_AOS_SYNC_INTERNAL_H_
#define AOS_IPC_LIB_AOS_SYNC_INTERNAL_H_

// The interface between the OS-independent half of aos_sync (aos_sync.cc: the
// robust list, the futex word encoding, and the public API) and the per-OS
// backends which do the actual blocking and waking.
//
// The backends are:
//   * aos_sync_linux.cc: real futexes, including the PI operations.
//   * aos_sync_portable.cc: the futex emulation every OS without futexes
//     shares (a CAS on the word plus an OS wait-on-address primitive).
//   * aos_sync_darwin.cc: the macOS half of that emulation.
//
// Nothing outside those files should include this header.
//
// Everything in here assumes the NDEBUG handling at the top of aos_sync.cc has
// already happened, so every file which includes it has to start with that
// same block (assert() is used both here and in the backends).

#include <stddef.h>
#include <stdio.h>
#include <sys/types.h>
#include <time.h>

#include <atomic>
#include <cassert>
#include <cerrno>
#include <cinttypes>
#include <cstdint>

#ifdef __linux__
#include <linux/futex.h>
#endif

#ifdef AOS_SANITIZER_thread
#include <pthread.h>

#include <sanitizer/tsan_interface_atomic.h>
#endif

#include "aos/ipc_lib/aos_sync.h"
#include "aos/macros.h"
#include "aos/util/compiler_memory_barrier.h"

#ifndef __linux__
// The futex operations and the layout of the futex word are part of the
// interface we emulate everywhere else, so define them for the platforms with
// no <linux/futex.h> to get them from.
#define FUTEX_WAIT 0
#define FUTEX_WAKE 1
#define FUTEX_FD 2
#define FUTEX_REQUEUE 3
#define FUTEX_CMP_REQUEUE 4
#define FUTEX_WAKE_OP 5
#define FUTEX_LOCK_PI 6
#define FUTEX_UNLOCK_PI 7
#define FUTEX_TRYLOCK_PI 8
#define FUTEX_WAIT_BITSET 9
#define FUTEX_WAKE_BITSET 10
#define FUTEX_WAIT_REQUEUE_PI 11
#define FUTEX_CMP_REQUEUE_PI 12

#define FUTEX_PRIVATE_FLAG 128
#define FUTEX_CLOCK_REALTIME 256
#define FUTEX_CMD_MASK ~(FUTEX_PRIVATE_FLAG | FUTEX_CLOCK_REALTIME)

#define FUTEX_WAIT_PRIVATE (FUTEX_WAIT | FUTEX_PRIVATE_FLAG)
#define FUTEX_WAKE_PRIVATE (FUTEX_WAKE | FUTEX_PRIVATE_FLAG)
#define FUTEX_REQUEUE_PRIVATE (FUTEX_REQUEUE | FUTEX_PRIVATE_FLAG)
#define FUTEX_CMP_REQUEUE_PRIVATE (FUTEX_CMP_REQUEUE | FUTEX_PRIVATE_FLAG)
#define FUTEX_WAKE_OP_PRIVATE (FUTEX_WAKE_OP | FUTEX_PRIVATE_FLAG)
#define FUTEX_LOCK_PI_PRIVATE (FUTEX_LOCK_PI | FUTEX_PRIVATE_FLAG)
#define FUTEX_UNLOCK_PI_PRIVATE (FUTEX_UNLOCK_PI | FUTEX_PRIVATE_FLAG)
#define FUTEX_TRYLOCK_PI_PRIVATE (FUTEX_TRYLOCK_PI | FUTEX_PRIVATE_FLAG)
#define FUTEX_WAIT_BITSET_PRIVATE (FUTEX_WAIT_BITSET | FUTEX_PRIVATE_FLAG)
#define FUTEX_WAKE_BITSET_PRIVATE (FUTEX_WAKE_BITSET | FUTEX_PRIVATE_FLAG)
#define FUTEX_WAIT_REQUEUE_PI_PRIVATE \
  (FUTEX_WAIT_REQUEUE_PI | FUTEX_PRIVATE_FLAG)
#define FUTEX_CMP_REQUEUE_PI_PRIVATE (FUTEX_CMP_REQUEUE_PI | FUTEX_PRIVATE_FLAG)

#define FUTEX_WAITERS 0x80000000
#define FUTEX_OWNER_DIED 0x40000000
#define FUTEX_TID_MASK 0x3fffffff
#endif  // !__linux__

namespace aos::ipc_lib::sync {

const bool kRobustListDebug = false;
const bool kLockDebug __attribute__((unused)) = false;
const bool kPrintOperations = false;

#ifdef AOS_SANITIZER_thread

// Simple macro for checking something which should always be true.
// Using the standard CHECK macro isn't safe because failures often result in
// reentering the mutex locking code, which doesn't work.
#define SIMPLE_CHECK(expr)                                                   \
  do {                                                                       \
    if (!(expr)) {                                                           \
      fprintf(stderr, "%s: %d: SIMPLE_CHECK(" #expr ") failed!\n", __FILE__, \
              __LINE__);                                                     \
      abort();                                                               \
    }                                                                        \
  } while (false)

// Forcibly initializes the pthread mutex for *m.
// This sequence of operations is only safe for the simpler kinds of mutexes in
// glibc's pthreads implementation on Linux.
inline void init_pthread_mutex(aos_mutex *m) {
  // Re-initialize the mutex so the destroy won't fail if it's locked.
  // tsan ignores this.
  SIMPLE_CHECK(0 == pthread_mutex_init(&m->pthread_mutex, nullptr));
  // Destroy the mutex so tsan will forget about it if some now-dead thread
  // locked it.
  SIMPLE_CHECK(0 == pthread_mutex_destroy(&m->pthread_mutex));

  // Now actually initialize it, making sure it's process-shareable so it works
  // correctly across shared memory.
  pthread_mutexattr_t attr;
  SIMPLE_CHECK(0 == pthread_mutexattr_init(&attr));
  SIMPLE_CHECK(0 == pthread_mutexattr_setpshared(&attr, true));
  SIMPLE_CHECK(0 == pthread_mutex_init(&m->pthread_mutex, &attr));
  SIMPLE_CHECK(0 == pthread_mutexattr_destroy(&attr));
}

// Locks the pthread mutex for *m.
// If a stack trace ever reveals the pthread_mutex_lock call in here blocking,
// there is a bug in our mutex code or the way somebody is calling it.
inline void lock_pthread_mutex(aos_mutex *m) {
  if (!m->pthread_mutex_init) {
    init_pthread_mutex(m);
    m->pthread_mutex_init = true;
  }
  SIMPLE_CHECK(0 == pthread_mutex_lock(&m->pthread_mutex));
}

// Forcibly locks the pthread mutex for *m.
// This will (somewhat hackily) rip the lock out from underneath somebody else
// who is already holding it.
inline void force_lock_pthread_mutex(aos_mutex *m) {
  if (!m->pthread_mutex_init) {
    init_pthread_mutex(m);
    m->pthread_mutex_init = true;
  }
  const int trylock_result = pthread_mutex_trylock(&m->pthread_mutex);
  SIMPLE_CHECK(trylock_result == 0 || trylock_result == EBUSY);
  if (trylock_result == 0) {
    // We're good, so unlock it and then go for a real lock down below.
    SIMPLE_CHECK(0 == pthread_mutex_unlock(&m->pthread_mutex));
  } else {
    // Somebody (should always be somebody else who died with it held) already
    // has it, so make tsan forget about that.
    init_pthread_mutex(m);
  }
  lock_pthread_mutex(m);
}

// Unlocks the pthread mutex for *m.
inline void unlock_pthread_mutex(aos_mutex *m) {
  assert(m->pthread_mutex_init);
  SIMPLE_CHECK(0 == pthread_mutex_unlock(&m->pthread_mutex));
}

#else

// Empty implementations of all these so the code below doesn't need #ifdefs.
inline void lock_pthread_mutex(aos_mutex *) {}
inline void force_lock_pthread_mutex(aos_mutex *) {}
inline void unlock_pthread_mutex(aos_mutex *) {}

#endif

// Returns the previous value of f.
inline uint32_t compare_and_swap_val(aos_futex *f, uint32_t before,
                                     uint32_t after) {
#ifdef AOS_SANITIZER_thread
  // This is a workaround for <https://llvm.org/bugs/show_bug.cgi?id=23176>.
  // Basically, most of the atomic operations are broken under tsan, but this
  // particular one isn't.
  // TODO(Brian): Remove this #ifdef (and the one in compare_and_swap) once we
  // don't have to worry about tsan with this bug any more.
  uint32_t before_value = before;
  __tsan_atomic32_compare_exchange_strong(
      reinterpret_cast<int32_t *>(f),
      reinterpret_cast<int32_t *>(&before_value), after,
      __tsan_memory_order_seq_cst, __tsan_memory_order_seq_cst);
  return before_value;
#else
  std::atomic_ref<uint32_t> ref(*f);
  uint32_t expected = before;
  ref.compare_exchange_strong(expected, after, std::memory_order_seq_cst,
                              std::memory_order_seq_cst);
  return expected;
#endif
}

// Returns true if it succeeds and false if it fails.
inline bool compare_and_swap(aos_futex *f, uint32_t before, uint32_t after) {
#ifdef AOS_SANITIZER_thread
  return compare_and_swap_val(f, before, after) == before;
#else
  std::atomic_ref<uint32_t> ref(*f);
  uint32_t expected = before;
  return ref.compare_exchange_strong(expected, after, std::memory_order_seq_cst,
                                     std::memory_order_seq_cst);
#endif
}

// Returns the current thread's ID, in whatever encoding the futex word uses on
// this OS.  Implemented by each backend.
pid_t do_get_tid();

// Starts off at 0 in each new thread (because that's what it gets initialized
// to in most of them or it gets to reset to 0 after a fork by atfork_child()).
extern thread_local pid_t my_tid;

// This gets called to set everything up in a new thread by get_tid().
void initialize_in_new_thread();

// Gets called before the fork(2) wrapper function returns in the child.
void atfork_child();

// Arranges for atfork_child() to run in the child of a fork(2).  Does nothing
// on the OSes which have no fork.
void InstallAtforkHook();

// Gets the current thread's TID and does all of the 1-time initialization the
// first time it's called in a given thread.
inline uint32_t get_tid() {
  if (AOS_UNLIKELY(my_tid == 0)) {
    initialize_in_new_thread();
  }
  static_assert(sizeof(my_tid) <= sizeof(uint32_t), "pid_t is too big");
  return static_cast<uint32_t>(my_tid);
}

// Contains all of the stuff for dealing with the robust list. Nothing outside
// this namespace should touch anything inside it except Init, Adder, and
// Remover.
namespace my_robust_list {

static_assert(offsetof(aos_mutex, next) == 0,
              "Our math all assumes that the beginning of a mutex and its next "
              "pointer are at the same place in memory.");

// Our version of robust_list_head.
// This is copied from the kernel header because that's a pretty stable ABI (and
// any changes will be backwards compatible anyways) and we want ours to have
// different types.
// The uintptr_ts are &next of the elements in the list (with stuff |ed in).
struct aos_robust_list_head {
  uintptr_t next;
  long futex_offset;
  uintptr_t pending_next;
};

#ifdef __linux__
static_assert(offsetof(aos_robust_list_head, next) ==
                  offsetof(robust_list_head, list),
              "Our aos_robust_list_head doesn't match the kernel's");
static_assert(offsetof(aos_robust_list_head, futex_offset) ==
                  offsetof(robust_list_head, futex_offset),
              "Our aos_robust_list_head doesn't match the kernel's");
static_assert(offsetof(aos_robust_list_head, pending_next) ==
                  offsetof(robust_list_head, list_op_pending),
              "Our aos_robust_list_head doesn't match the kernel's");
static_assert(sizeof(aos_robust_list_head) == sizeof(robust_list_head),
              "Our aos_robust_list_head doesn't match the kernel's");
#endif

extern thread_local aos_robust_list_head robust_head;

// Extra offset between mutex values and where we point to for their robust list
// entries (from SetRobustListOffset).
extern uintptr_t robust_list_offset;

// The value to OR each pointer's value with whenever putting it into the robust
// list (technically only if it's PI, but all of ours are, so...).
static const uintptr_t kRobustListOr = 1;

// Returns the value which goes into a next variable to represent the head.
inline uintptr_t robust_head_next_value() {
  return reinterpret_cast<uintptr_t>(&robust_head.next);
}
// Returns true iff next represents the head.
inline bool next_is_head(uintptr_t next) {
  return next == robust_head_next_value();
}
// Returns the (psuedo-)mutex corresponding to the head.
// This does NOT have a previous pointer, so be careful with the return value.
inline aos_mutex *robust_head_mutex() {
  return reinterpret_cast<aos_mutex *>(robust_head_next_value());
}

inline uintptr_t mutex_to_next(aos_mutex *m) {
  return (reinterpret_cast<uintptr_t>(&m->next) + robust_list_offset) |
         kRobustListOr;
}
inline aos_mutex *next_to_mutex(uintptr_t next) {
  if (AOS_UNLIKELY(robust_list_offset != 0) && next_is_head(next)) {
    // We don't offset the head pointer, so be careful.
    return reinterpret_cast<aos_mutex *>(next);
  }
  return reinterpret_cast<aos_mutex *>((next & ~kRobustListOr) -
                                       robust_list_offset);
}

// Sets up the robust list for each thread.
void Init();

// Updating the offset with locked mutexes is important during robustness
// testing, because there are mutexes which are locked before this is set to a
// non-0 value and then unlocked after it is changed back. However, to make sure
// the code works correctly when manipulating the next pointer of the last of
// those mutexes, all of their next values have to be adjusted appropriately.
void SetRobustListOffset(uintptr_t offset);

bool HaveLockedMutexes();

// Handles adding a mutex to the robust list.
// The idea is to create one of these at the beginning of a function that needs
// to do this and then call Add() iff it should actually be added.
class Adder {
 public:
  Adder(aos_mutex *m) : m_(m) {
    assert(robust_head.pending_next == 0);
    if (kRobustListDebug) {
      printf("%" PRId32 ": maybe add %p\n", get_tid(), m_);
    }
    robust_head.pending_next = mutex_to_next(m);
    aos_compiler_memory_barrier();
  }
  ~Adder() {
    assert(robust_head.pending_next == mutex_to_next(m_));
    if (kRobustListDebug) {
      printf("%" PRId32 ": done maybe add %p, n=%p p=%p\n", get_tid(), m_,
             next_to_mutex(m_->next), m_->previous);
    }
    aos_compiler_memory_barrier();
    robust_head.pending_next = 0;
  }

  void Add() {
    assert(robust_head.pending_next == mutex_to_next(m_));
    if (kRobustListDebug) {
      printf("%" PRId32 ": adding %p\n", get_tid(), m_);
    }
    const uintptr_t old_head_next_value = robust_head.next;

    m_->next = old_head_next_value;
    aos_compiler_memory_barrier();
    robust_head.next = mutex_to_next(m_);

    m_->previous = robust_head_mutex();
    if (!next_is_head(old_head_next_value)) {
      // robust_head's psuedo-mutex doesn't have a previous pointer to update.
      next_to_mutex(old_head_next_value)->previous = m_;
    }
    aos_compiler_memory_barrier();
    if (kRobustListDebug) {
      printf("%" PRId32 ": done adding %p\n", get_tid(), m_);
    }
  }

 private:
  aos_mutex *const m_;

  DISALLOW_COPY_AND_ASSIGN(Adder);
};

// Handles removing a mutex from the robust list.
// The idea is to create one of these at the beginning of a function that needs
// to do this.
class Remover {
 public:
  Remover(aos_mutex *m) {
    assert(robust_head.pending_next == 0);
    if (kRobustListDebug) {
      printf("%" PRId32 ": beginning to remove %p, n=%p p=%p\n", get_tid(), m,
             next_to_mutex(m->next), m->previous);
    }
    robust_head.pending_next = mutex_to_next(m);
    aos_compiler_memory_barrier();

    aos_mutex *const previous = m->previous;
    const uintptr_t next_value = m->next;

    previous->next = m->next;
    if (!next_is_head(next_value)) {
      // robust_head's psuedo-mutex doesn't have a previous pointer to update.
      next_to_mutex(next_value)->previous = previous;
    }

    if (kRobustListDebug) {
      printf("%" PRId32 ": done removing %p\n", get_tid(), m);
    }
  }
  ~Remover() {
    assert(robust_head.pending_next != 0);
    aos_compiler_memory_barrier();
    robust_head.pending_next = 0;
    if (kRobustListDebug) {
      printf("%" PRId32 ": done with removal\n", get_tid());
    }
  }

 private:
  DISALLOW_COPY_AND_ASSIGN(Remover);
};

}  // namespace my_robust_list

// Registers this thread's robust list with the OS, so that it can mark any
// mutexes this thread still holds when it dies.  Only Linux has such a thing;
// the other backends emulate the walk in userspace and do nothing here.
void RegisterRobustList(void *head, size_t size);

// Makes sure the userspace emulation of the kernel's robust list walk is armed
// for this thread.  A no-op on the backends where the OS does it for us.
void TouchRobustListCleaner();

// These sys_futex_* functions are the OS primitives everything else is built
// on.  On Linux they are wrappers around syscall(SYS_futex), and each of them
// takes the specific set of arguments for a given futex operation.  The other
// backends emulate the same contract: they return the result or a negated
// errno value.  -1..-4095 mean errors and not successful results, which is
// guaranteed by the kernel.

// Used for FUTEX_WAIT, FUTEX_LOCK_PI, and FUTEX_TRYLOCK_PI.
int sys_futex_wait(int op, aos_futex *addr1, int val1,
                   const struct timespec *timeout);

int sys_futex_wake(aos_futex *addr1, int val1);

int sys_futex_unlock_pi(aos_futex *addr1);

#ifndef __linux__
// Blocks until the value at addr1 stops being val1, somebody wakes it, or (if
// timeout isn't nullptr) the relative timeout expires.  Returns 0 or a negated
// errno, just like FUTEX_WAIT does.
//
// This is the one piece of aos_sync_portable.cc's emulation which every OS has
// to provide for itself; everything else it needs is above.
int wait_on_address(aos_futex *addr1, int val1, const struct timespec *timeout);
#endif

// Finishes the locking of a mutex by potentially clearing FUTEX_OWNER_DIED in
// the futex and returning the correct value.
inline int mutex_finish_lock(aos_mutex *m) {
  const uint32_t value =
      std::atomic_ref<uint32_t>(m->futex).load(std::memory_order_acquire);
  if (AOS_UNLIKELY((value & FUTEX_OWNER_DIED) != 0)) {
    std::atomic_ref<uint32_t>(m->futex).fetch_and(~FUTEX_OWNER_DIED,
                                                  std::memory_order_relaxed);
    force_lock_pthread_mutex(m);
    return 1;
  } else {
    lock_pthread_mutex(m);
    return 0;
  }
}

// Split out separately from mutex_get so condition_wait can call it and use its
// own my_robust_list::Adder.
int mutex_do_get(aos_mutex *m, bool signals_fail,
                 const struct timespec *timeout, uint32_t tid);

// Releases *m, which the caller has already checked is locked by tid and taken
// off the robust list.
void mutex_do_unlock(aos_mutex *m, uint32_t tid);

// The common implementation for broadcast and signal.
// number_requeue is the number of waiters to requeue (probably INT_MAX or 0). 1
// will always be woken.
void condition_wake(aos_condition *c, aos_mutex *m, int number_requeue);

}  // namespace aos::ipc_lib::sync

#endif  // AOS_IPC_LIB_AOS_SYNC_INTERNAL_H_
