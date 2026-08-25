#if !AOS_SYNC_DEBUG
#undef NDEBUG
#define NDEBUG
#endif

// The Linux backend for aos_sync.  Everything in here is a thin wrapper around
// the kernel's futex support, including the priority-inheritance operations
// which none of the other backends have anything like.

#include "aos/ipc_lib/aos_sync.h"

#include <linux/futex.h>
#include <pthread.h>
#include <stdio.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <atomic>
#include <cassert>
#include <cerrno>
#include <cinttypes>
#include <ostream>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/ipc_lib/aos_sync_internal.h"
#include "aos/ipc_lib/shm_observers.h"
#include "aos/macros.h"

using ::aos::linux_code::ipc_lib::RunShmObservers;

namespace aos::ipc_lib::sync {

// The sys_futex_* functions below each have optimized versions for some
// architectures which don't go through syscall(2) or errno. These use register
// variables to get the values in the right registers to actually make the
// syscall.

// The actual macros that we key off of to use the inline versions or not.
#if defined(__ARM_EABI__)
// The syscall interface is different for non-EABI ARM, so we test specifically
// for EABI.
#define ARM_EABI_INLINE_SYSCALL 1
#define AARCH64_INLINE_SYSCALL 0
#elif defined(__aarch64__)
// Linux only has one supported syscall ABI on aarch64, which is the one we
// support.
#define ARM_EABI_INLINE_SYSCALL 0
#define AARCH64_INLINE_SYSCALL 1
#else
#define ARM_EABI_INLINE_SYSCALL 0
#define AARCH64_INLINE_SYSCALL 0
#endif

namespace {

inline int sys_futex_cmp_requeue_pi(aos_futex *addr1, int num_wake,
                                    int num_requeue, aos_futex *m,
                                    uint32_t val) {
#if ARM_EABI_INLINE_SYSCALL
  register aos_futex *addr1_reg __asm__("r0") = addr1;
  register int op_reg __asm__("r1") = FUTEX_CMP_REQUEUE_PI;
  register int num_wake_reg __asm__("r2") = num_wake;
  register int num_requeue_reg __asm__("r3") = num_requeue;
  register aos_futex *m_reg __asm__("r4") = m;
  register uint32_t val_reg __asm__("r5") = val;
  register int syscall_number __asm__("r7") = SYS_futex;
  register int result __asm__("r0");
  __asm__ volatile("swi #0"
                   : "=r"(result)
                   : "r"(addr1_reg), "r"(op_reg), "r"(num_wake_reg),
                     "r"(num_requeue_reg), "r"(m_reg), "r"(val_reg),
                     "r"(syscall_number)
                   : "memory");
  return result;
#elif AARCH64_INLINE_SYSCALL
  register aos_futex *addr1_reg __asm__("x0") = addr1;
  register int op_reg __asm__("x1") = FUTEX_CMP_REQUEUE_PI;
  register int num_wake_reg __asm__("x2") = num_wake;
  register int num_requeue_reg __asm__("x3") = num_requeue;
  register aos_futex *m_reg __asm__("x4") = m;
  register uint32_t val_reg __asm__("x5") = val;
  register int syscall_number __asm__("x8") = SYS_futex;
  register int result __asm__("x0");
  __asm__ volatile("svc #0"
                   : "=r"(result)
                   : "r"(addr1_reg), "r"(op_reg), "r"(num_wake_reg),
                     "r"(num_requeue_reg), "r"(m_reg), "r"(val_reg),
                     "r"(syscall_number)
                   : "memory");
  return result;
#else
  const int r = syscall(SYS_futex, addr1, FUTEX_CMP_REQUEUE_PI, num_wake,
                        num_requeue, m, val);
  if (r == -1) return -errno;
  return r;
#endif
}

inline int sys_futex_wait_requeue_pi(aos_condition *addr1, uint32_t start_val,
                                     const struct timespec *timeout,
                                     aos_futex *m) {
#if ARM_EABI_INLINE_SYSCALL
  register aos_condition *addr1_reg __asm__("r0") = addr1;
  register int op_reg __asm__("r1") = FUTEX_WAIT_REQUEUE_PI;
  register uint32_t start_val_reg __asm__("r2") = start_val;
  register const struct timespec *timeout_reg __asm__("r3") = timeout;
  register aos_futex *m_reg __asm__("r4") = m;
  register int syscall_number __asm__("r7") = SYS_futex;
  register int result __asm__("r0");
  __asm__ volatile("swi #0"
                   : "=r"(result)
                   : "r"(addr1_reg), "r"(op_reg), "r"(start_val_reg),
                     "r"(timeout_reg), "r"(m_reg), "r"(syscall_number)
                   : "memory");
  return result;
#elif AARCH64_INLINE_SYSCALL
  register aos_condition *addr1_reg __asm__("x0") = addr1;
  register int op_reg __asm__("x1") = FUTEX_WAIT_REQUEUE_PI;
  register uint32_t start_val_reg __asm__("x2") = start_val;
  register const struct timespec *timeout_reg __asm__("x3") = timeout;
  register aos_futex *m_reg __asm__("x4") = m;
  register int syscall_number __asm__("x8") = SYS_futex;
  register int result __asm__("x0");
  __asm__ volatile("svc #0"
                   : "=r"(result)
                   : "r"(addr1_reg), "r"(op_reg), "r"(start_val_reg),
                     "r"(timeout_reg), "r"(m_reg), "r"(syscall_number)
                   : "memory");
  return result;
#else
  const int r =
      syscall(SYS_futex, addr1, FUTEX_WAIT_REQUEUE_PI, start_val, timeout, m);
  if (r == -1) return -errno;
  return r;
#endif
}

}  // namespace

pid_t do_get_tid() {
  pid_t r = syscall(SYS_gettid);
  assert(r > 0);
  return r;
}

void InstallAtforkHook() {
  ABSL_PCHECK(pthread_atfork(NULL, NULL, &atfork_child) == 0)
      << ": pthread_atfork(NULL, NULL, "
      << reinterpret_cast<void *>(&atfork_child) << ") failed";
}

void RegisterRobustList(void *head, size_t size) {
  ABSL_PCHECK(syscall(SYS_set_robust_list, head, size) == 0)
      << ": set_robust_list(" << head << ", " << size << ") failed";
}

// The kernel walks the robust list for us, so there is nothing to arm here.
void TouchRobustListCleaner() {}

int sys_futex_wait(int op, aos_futex *addr1, int val1,
                   const struct timespec *timeout) {
#if ARM_EABI_INLINE_SYSCALL
  register aos_futex *addr1_reg __asm__("r0") = addr1;
  register int op_reg __asm__("r1") = op;
  register int val1_reg __asm__("r2") = val1;
  register const struct timespec *timeout_reg __asm__("r3") = timeout;
  register int syscall_number __asm__("r7") = SYS_futex;
  register int result __asm__("r0");
  __asm__ volatile("swi #0"
                   : "=r"(result)
                   : "r"(addr1_reg), "r"(op_reg), "r"(val1_reg),
                     "r"(timeout_reg), "r"(syscall_number)
                   : "memory");
  return result;
#elif AARCH64_INLINE_SYSCALL
  register aos_futex *addr1_reg __asm__("x0") = addr1;
  register int op_reg __asm__("x1") = op;
  register int val1_reg __asm__("x2") = val1;
  register const struct timespec *timeout_reg __asm__("x3") = timeout;
  register int syscall_number __asm__("x8") = SYS_futex;
  register int result __asm__("x0");
  __asm__ volatile("svc #0"
                   : "=r"(result)
                   : "r"(addr1_reg), "r"(op_reg), "r"(val1_reg),
                     "r"(timeout_reg), "r"(syscall_number)
                   : "memory");
  return result;
#else
  const int r = syscall(SYS_futex, addr1, op, val1, timeout);
  if (r == -1) return -errno;
  return r;
#endif
}

int sys_futex_wake(aos_futex *addr1, int val1) {
#if ARM_EABI_INLINE_SYSCALL
  register aos_futex *addr1_reg __asm__("r0") = addr1;
  register int op_reg __asm__("r1") = FUTEX_WAKE;
  register int val1_reg __asm__("r2") = val1;
  register int syscall_number __asm__("r7") = SYS_futex;
  register int result __asm__("r0");
  __asm__ volatile("swi #0"
                   : "=r"(result)
                   : "r"(addr1_reg), "r"(op_reg), "r"(val1_reg),
                     "r"(syscall_number)
                   : "memory");
  return result;
#elif AARCH64_INLINE_SYSCALL
  register aos_futex *addr1_reg __asm__("x0") = addr1;
  register int op_reg __asm__("x1") = FUTEX_WAKE;
  register int val1_reg __asm__("x2") = val1;
  register int syscall_number __asm__("x8") = SYS_futex;
  register int result __asm__("x0");
  __asm__ volatile("svc #0"
                   : "=r"(result)
                   : "r"(addr1_reg), "r"(op_reg), "r"(val1_reg),
                     "r"(syscall_number)
                   : "memory");
  return result;
#else
  const int r = syscall(SYS_futex, addr1, FUTEX_WAKE, val1);
  if (r == -1) return -errno;
  return r;
#endif
}

int sys_futex_unlock_pi(aos_futex *addr1) {
#if ARM_EABI_INLINE_SYSCALL
  register aos_futex *addr1_reg __asm__("r0") = addr1;
  register int op_reg __asm__("r1") = FUTEX_UNLOCK_PI;
  register int syscall_number __asm__("r7") = SYS_futex;
  register int result __asm__("r0");
  __asm__ volatile("swi #0"
                   : "=r"(result)
                   : "r"(addr1_reg), "r"(op_reg), "r"(syscall_number)
                   : "memory");
  return result;
#elif AARCH64_INLINE_SYSCALL
  register aos_futex *addr1_reg __asm__("x0") = addr1;
  register int op_reg __asm__("x1") = FUTEX_UNLOCK_PI;
  register int syscall_number __asm__("x8") = SYS_futex;
  register int result __asm__("x0");
  __asm__ volatile("svc #0"
                   : "=r"(result)
                   : "r"(addr1_reg), "r"(op_reg), "r"(syscall_number)
                   : "memory");
  return result;
#else
  const int r = syscall(SYS_futex, addr1, FUTEX_UNLOCK_PI);
  if (r == -1) return -errno;
  return r;
#endif
}

int mutex_do_get(aos_mutex *m, bool signals_fail, uint32_t tid) {
  RunShmObservers run_observers(m, true);
  if (kPrintOperations) {
    printf("%" PRId32 ": %p do_get\n", tid, m);
  }

  while (true) {
    // If the atomic 0->TID transition fails.
    if (!compare_and_swap(&m->futex, 0, tid)) {
      // Wait in the kernel, which handles atomically ORing in FUTEX_WAITERS
      // before actually sleeping.
      const int ret = sys_futex_wait(FUTEX_LOCK_PI, &m->futex, 1, nullptr);
      if (ret != 0) {
        if (AOS_LIKELY(ret == -EINTR)) {
          if (signals_fail) {
            return 2;
          } else {
            continue;
          }
        }
        thread_state()->robust_head.pending_next = 0;
        ABSL_CHECK_NE(ret, -EDEADLK)
            << ": multiple lock of " << m << " by " << tid;

        errno = -ret;
        ABSL_PLOG(FATAL) << "FUTEX_LOCK_PI(" << &m->futex << "(="
                         << std::atomic_ref<uint32_t>(m->futex).load(
                                std::memory_order_seq_cst)
                         << "), 1, nullptr) failed";
      } else {
        if (kLockDebug) {
          printf("%" PRId32 ": %p kernel lock done\n", tid, m);
        }
        // The kernel already handled setting the value to our TID (ish).
        break;
      }
    } else {
      if (kLockDebug) {
        printf("%" PRId32 ": %p fast lock done\n", tid, m);
      }
      lock_pthread_mutex(m);
      // Fastpath succeeded, so no need to call into the kernel.
      // Because this is the fastpath, it's a good idea to avoid even having to
      // load the value again down below.
      return 0;
    }
  }

  return mutex_finish_lock(m);
}

int mutex_do_trylock(aos_mutex *m, uint32_t tid, my_robust_list::Adder *adder) {
  return futex_mutex_do_trylock(m, tid, adder);
}

void mutex_do_unlock(aos_mutex *m, uint32_t tid) {
  futex_mutex_do_unlock(m, tid);
}

void condition_wake(aos_condition *c, aos_mutex *m, int number_requeue) {
  RunShmObservers run_observers(c, true);
  // Make it so that anybody just going to sleep won't.
  // This is where we might accidentally wake more than just 1 waiter with 1
  // signal():
  //   1 already sleeping will be woken but n might never actually make it to
  //     sleep in the kernel because of this.
  uint32_t new_value =
      std::atomic_ref<uint32_t>(*c).fetch_add(1, std::memory_order_seq_cst) + 1;

  while (true) {
    // This really wants to be FUTEX_REQUEUE_PI, but the kernel doesn't have
    // that... However, the code to support that is in the kernel, so it might
    // be a good idea to patch it to support that and use it iff it's there.
    const int ret =
        sys_futex_cmp_requeue_pi(c, 1, number_requeue, &m->futex, new_value);
    if (ret < 0) {
      // If the value got changed out from under us (aka somebody else did a
      // condition_wake).
      if (AOS_LIKELY(ret == -EAGAIN)) {
        // If we're doing a broadcast, the other guy might have done a signal
        // instead, so we have to try again.
        // If we're doing a signal, we have to go again to make sure that 2
        // signals wake 2 processes.
        new_value =
            std::atomic_ref<uint32_t>(*c).load(std::memory_order_relaxed);
        continue;
      }
      thread_state()->robust_head.pending_next = 0;
      errno = -ret;
      ABSL_PLOG(FATAL) << "FUTEX_CMP_REQUEUE_PI(" << c << ", 1, "
                       << number_requeue << ", " << &m->futex << ", *" << c
                       << ") failed";
    } else {
      return;
    }
  }
}

}  // namespace aos::ipc_lib::sync

// The rest of this is part of the public interface, which lives at the global
// scope.
using namespace aos::ipc_lib::sync;  // NOLINT(build/namespaces)

uint32_t mutex_owner_from_value(uint32_t value) {
  return value & FUTEX_TID_MASK;
}

int condition_wait(aos_condition *c, aos_mutex *m, struct timespec *end_time) {
  RunShmObservers run_observers(c, false);
  const uint32_t tid = get_tid();
  const uint32_t wait_start =
      std::atomic_ref<uint32_t>(*c).load(std::memory_order_seq_cst);

  mutex_unlock(m);

  my_robust_list::Adder adder(m);

  while (true) {
    // Wait in the kernel iff the value of it doesn't change (ie somebody else
    // does a wake) from before we unlocked the mutex.
    int ret = sys_futex_wait_requeue_pi(c, wait_start, end_time, &m->futex);

    if (ret != 0) {
      // Timed out waiting.  Signal that back up to the user.
      if (AOS_LIKELY(ret == -ETIMEDOUT)) {
        // We have to relock it ourself because the kernel didn't do it.
        const int r = mutex_do_get(m, false, tid);
        assert(AOS_LIKELY(r == 0 || r == 1));
        adder.Add();

        // OWNER_DIED takes priority.  Pass it on if we found it.
        if (r == 1) return r;
        // Otherwise communicate that we were interrupted.
        return -1;
      }

      // If it failed because somebody else did a wake and changed the value
      // before we actually made it to sleep.
      if (AOS_LIKELY(ret == -EAGAIN)) {
        // There's no need to unconditionally set FUTEX_WAITERS here if we're
        // using REQUEUE_PI because the kernel automatically does that in the
        // REQUEUE_PI iff it requeued anybody.
        // If we're not using REQUEUE_PI, then everything is just normal locks
        // etc, so there's no need to do anything special there either.

        // We have to relock it ourself because the kernel didn't do it.
        const int r = mutex_do_get(m, false, tid);
        assert(AOS_LIKELY(r == 0 || r == 1));
        adder.Add();
        return r;
      }
      // Try again if it was because of a signal.
      if (AOS_LIKELY(ret == -EINTR)) {
        continue;
      }
      thread_state()->robust_head.pending_next = 0;
      errno = -ret;
      ABSL_PLOG(FATAL) << "FUTEX_WAIT_REQUEUE_PI(" << c << ", " << wait_start
                       << ", " << (&m->futex) << ") failed";
    } else {
      // Record that the kernel relocked it for us.
      lock_pthread_mutex(m);

      // We succeeded in waiting, and the kernel took care of locking the
      // mutex
      // for us and setting FUTEX_WAITERS iff it needed to (for REQUEUE_PI).

      adder.Add();

      const uint32_t value =
          std::atomic_ref<uint32_t>(m->futex).load(std::memory_order_relaxed);
      if (AOS_UNLIKELY((value & FUTEX_OWNER_DIED) != 0)) {
        std::atomic_ref<uint32_t>(m->futex).fetch_and(
            ~FUTEX_OWNER_DIED, std::memory_order_relaxed);
        return 1;
      } else {
        return 0;
      }
    }
  }
}
