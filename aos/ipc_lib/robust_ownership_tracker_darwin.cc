#include <assert.h>
#include <errno.h>
#include <libproc.h>
#include <pthread.h>
#include <sys/proc_info.h>
#include <unistd.h>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/ipc_lib/robust_ownership_tracker.h"

namespace aos::ipc_lib {

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
  int result = proc_pidinfo(pid, PROC_PIDTHREADID64INFO, thread_id, &threadinfo,
                            sizeof(threadinfo));
  if (result != sizeof(threadinfo)) {
    ABSL_LOG(ERROR) << "Detected that TID " << loaded.tid()
                    << " died (proc_pidinfo failed: result=" << result
                    << ", errno=" << errno << ").";
    return true;
  }
  return false;
}

void RobustOwnershipTracker::Acquire() {
  uint64_t mac_tid;
  pthread_threadid_np(NULL, &mac_tid);
  ABSL_CHECK_GT(mac_tid, 0u);

  owner_pid_ = getpid();
  owner_thread_id_ = mac_tid;
  death_notification_init(&mutex_);
}

}  // namespace aos::ipc_lib
