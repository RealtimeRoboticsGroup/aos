#include "aos/ipc_lib/robust_ownership_tracker.h"

#include <assert.h>
#include <errno.h>
#include <libproc.h>
#include <pthread.h>
#include <stdint.h>
#include <sys/proc_info.h>
#include <unistd.h>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

namespace aos::ipc_lib {
namespace {

// PROC_PIDUNIQIDENTIFIERINFO and its struct live in XNU's bsd/sys/proc_info.h
// but are not exported to the macOS SDK (the same situation as
// PROC_PIDTHREADID64INFO below), so define them to their known values.  The
// kernel returns 56 bytes: three reserved words, not the four some copies of
// the header grew later.
#ifndef PROC_PIDUNIQIDENTIFIERINFO
#define PROC_PIDUNIQIDENTIFIERINFO 17
struct proc_uniqidentifierinfo {
  uint8_t p_uuid[16];   /* UUID of the main executable */
  uint64_t p_uniqueid;  /* 64 bit unique identifier for process */
  uint64_t p_puniqueid; /* unique identifier for process's parent */
  uint64_t p_reserve2;
  uint64_t p_reserve3;
  uint64_t p_reserve4;
};
#endif

// Fetches pid's p_uniqueid: a per-boot monotonic 64-bit process id that is
// never recycled.  Unlike the thread-id flavor, XNU puts this flavor in the
// bucket that skips the effective-uid check (alongside PROC_PIDT_SHORTBSDINFO
// and friends), so it works across euid boundaries.  Returns 0 on failure
// with errno set; real p_uniqueids are nonzero (launchd's is 1).
uint64_t ProcessUniqueId(pid_t pid) {
  struct proc_uniqidentifierinfo uniqinfo;
  const int result = proc_pidinfo(pid, PROC_PIDUNIQIDENTIFIERINFO, 0, &uniqinfo,
                                  sizeof(uniqinfo));
  if (result != sizeof(uniqinfo)) {
    return 0;
  }
  return uniqinfo.p_uniqueid;
}

}  // namespace

bool RobustOwnershipTracker::OwnerIsDefinitelyAbsolutelyDead() const {
  auto loaded = LoadAcquire();
  if (loaded.OwnerIsDead()) {
    return true;
  }
  if (loaded.IsUnclaimed()) {
    return false;
  }
  const pid_t pid = owner_pid_.load(std::memory_order_relaxed);
  const uint64_t thread_id = owner_thread_id_.load(std::memory_order_relaxed);
  if (pid == 0 || thread_id == 0) {
    return true;
  }
  if ((static_cast<pid_t>(thread_id) & FUTEX_TID_MASK) != loaded.tid()) {
    ABSL_LOG(ERROR) << "Detected that owner changed (TID mismatch: expected "
                    << (static_cast<pid_t>(thread_id) & FUTEX_TID_MASK)
                    << ", got " << loaded.tid() << ").";
    return true;
  }

  // On macOS, the 64-bit thread ID (returned by pthread_threadid_np) is
  // system-wide unique and monotonically increasing, meaning it will not
  // recycle during the lifetime of the system.  Consequently, checking the
  // thread's existence using proc_pidinfo with the recorded PID and thread ID
  // is 100% robust against both thread death and PID recycling (since a
  // recycled PID will not contain a thread with this unique ID).  We do not
  // need a process start time check.
#ifndef PROC_PIDTHREADID64INFO
#define PROC_PIDTHREADID64INFO 15
#endif
  struct proc_threadinfo threadinfo;
  errno = 0;
  int result = proc_pidinfo(pid, PROC_PIDTHREADID64INFO, thread_id, &threadinfo,
                            sizeof(threadinfo));
  if (result == sizeof(threadinfo)) {
    return false;
  }
  // Only ESRCH is the kernel saying "no such process or thread".  Other
  // failures mean the question could not be asked, and most likely EPERM:
  // XNU gates this proc_pidinfo flavor behind an *effective*-uid match
  // (proc_security_policy with CHECK_SAME_USER; root and
  // PRIV_GLOBAL_PROC_INFO override), while the queue's uid gate
  // (InitializeLocklessQueueMemory) compares GetUserId() -- the *real* uid
  // on Darwin, picked for signal deliverability -- so processes can share a
  // queue and still be forbidden from inspecting each other.  A sandbox MAC
  // hook can refuse for reasons of its own, too.
  if (errno == ESRCH) {
    ABSL_LOG(ERROR) << "Detected that TID " << loaded.tid()
                    << " died (proc_pidinfo: ESRCH).";
    return true;
  }
  const int threadinfo_errno = errno;

  // The question was refused at thread granularity, but process granularity
  // is enough: on Darwin a thread cannot die abruptly without taking its
  // process along (there is no TerminateThread), and a thread that exits
  // normally runs the TLS cleanup walk in aos_sync_portable.cc, which marks
  // the futex FUTEX_OWNER_DIED -- a case the OwnerIsDead() check above
  // already caught.  So ask the ungated process-identity flavor instead and
  // compare against the p_uniqueid recorded at Acquire() time; since
  // p_uniqueid is never recycled, a match proves this is still the very
  // process that took ownership, no matter who owns the pid now or how many
  // times it was reused.
  errno = 0;
  const uint64_t unique_id = ProcessUniqueId(pid);
  if (unique_id != 0) {
    const uint64_t owner_unique_id =
        owner_unique_id_.load(std::memory_order_relaxed);
    if (unique_id != owner_unique_id) {
      ABSL_LOG(ERROR) << "Detected that TID " << loaded.tid() << " died (PID "
                      << pid << " was recycled: p_uniqueid " << unique_id
                      << " != recorded " << owner_unique_id << ").";
      return true;
    }
    return false;
  }
  if (errno == ESRCH) {
    ABSL_LOG(ERROR) << "Detected that TID " << loaded.tid()
                    << " died (proc_pidinfo: ESRCH).";
    return true;
  }
  // Both flavors failed with something other than ESRCH.  The uniqid flavor
  // has no uid gate, so this is a MAC/sandbox policy denying process-info
  // outright (or a programming error: EINVAL/EFAULT).  Either way it is
  // deterministic, not transient -- every probe of every slot will fail the
  // same way, which means dead owners can never be detected and their slots
  // never recovered.  That environment cannot support the queue's recovery
  // guarantee at all, so die at the first probe rather than degrade silently
  // and permanently.
  ABSL_LOG(FATAL) << "proc_pidinfo(" << pid
                  << ") failed at both granularities (PROC_PIDTHREADID64INFO "
                  << thread_id << ": result=" << result
                  << ", errno=" << threadinfo_errno
                  << "; PROC_PIDUNIQIDENTIFIERINFO: errno=" << errno
                  << "); cannot tell whether the owner died, so dead-owner "
                     "recovery cannot work in this environment.";
}

void RobustOwnershipTracker::Acquire() {
  // There is a subtle ordering of operations here: the metadata
  // (start_time_ticks_) must be fully written and visible BEFORE the futex is
  // claimed by death_notification_init. Otherwise, another process inspecting
  // the tracker concurrently could see that the futex is claimed but find the
  // metadata is still unset (or contains stale values), incorrectly concluding
  // that the owner is dead.
  //
  // Note that if two processes concurrently attempt to call Acquire() on the
  // same tracker, they could overwrite each other's metadata before either
  // claims the futex, leading to state corruption.  Callers must serialize
  // calls to Acquire() (e.g., using a higher-level lock, as is done in the
  // lockless queue implementation via the queue setup lock).
  uint64_t mac_tid;
  pthread_threadid_np(NULL, &mac_tid);
  ABSL_CHECK_GT(mac_tid, 0u);
  // Self-inspection with this flavor cannot be permission-refused (our euid
  // trivially matches our own), so failure here means something is very
  // wrong.
  const uint64_t unique_id = ProcessUniqueId(getpid());
  ABSL_PCHECK(unique_id != 0)
      << ": proc_pidinfo(self, PROC_PIDUNIQIDENTIFIERINFO) failed";

  owner_pid_ = getpid();
  owner_thread_id_ = mac_tid;
  owner_unique_id_ = unique_id;
  death_notification_init(&mutex_);
}

}  // namespace aos::ipc_lib
