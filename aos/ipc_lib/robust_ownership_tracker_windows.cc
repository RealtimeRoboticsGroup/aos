#include <assert.h>
#include <process.h>
#include <windows.h>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/ipc_lib/robust_ownership_tracker.h"

namespace aos::ipc_lib {

namespace {

uint64_t ReadStartTimeTicks(pid_t tid) {
  if (tid == 0) {
    return RobustOwnershipTracker::kNoStartTimeTicks;
  }
  HANDLE hThread = OpenThread(THREAD_QUERY_LIMITED_INFORMATION, FALSE,
                              static_cast<DWORD>(tid));
  if (hThread == NULL) {
    return RobustOwnershipTracker::kNoStartTimeTicks;
  }
  FILETIME creation_time, exit_time, kernel_time, user_time;
  if (!GetThreadTimes(hThread, &creation_time, &exit_time, &kernel_time,
                      &user_time)) {
    CloseHandle(hThread);
    return RobustOwnershipTracker::kNoStartTimeTicks;
  }
  CloseHandle(hThread);
  ULARGE_INTEGER li;
  li.LowPart = creation_time.dwLowDateTime;
  li.HighPart = creation_time.dwHighDateTime;
  return li.QuadPart;
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
  const uint64_t proc_start_time_ticks = ReadStartTimeTicks(loaded.tid());
  if (proc_start_time_ticks == kNoStartTimeTicks) {
    ABSL_LOG(ERROR) << "Detected that PID " << loaded.tid() << " died.";
    return true;
  }

  if (proc_start_time_ticks != start_time_ticks_) {
    ABSL_LOG(ERROR) << "Detected that PID " << loaded.tid()
                    << " died from a starttime missmatch.";
    return true;
  }
  return false;
}

void RobustOwnershipTracker::Acquire() {
  pid_t tid = static_cast<pid_t>(GetCurrentThreadId());
  assert(tid > 0);
  const uint64_t proc_start_time_ticks = ReadStartTimeTicks(tid);
  ABSL_CHECK_NE(proc_start_time_ticks, kNoStartTimeTicks);

  start_time_ticks_ = proc_start_time_ticks;
  death_notification_init(&mutex_);
}

}  // namespace aos::ipc_lib
