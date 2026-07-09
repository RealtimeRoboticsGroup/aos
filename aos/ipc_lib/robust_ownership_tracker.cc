#include "aos/ipc_lib/robust_ownership_tracker.h"

#include <assert.h>

#include <atomic>
#include <limits>
#include <optional>
#include <ostream>
#include <sstream>

#ifdef _WIN32
#include <windows.h>
#else
#include <linux/futex.h>
#include <sys/syscall.h>
#include <unistd.h>

#include "aos/util/proc_stat.h"
#endif

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

namespace aos::ipc_lib {

#ifdef _WIN32
uint64_t RobustOwnershipTracker::ReadStartTimeTicks(pid_t tid) {
  if (tid == 0) {
    return kNoStartTimeTicks;
  }
  HANDLE hThread = OpenThread(THREAD_QUERY_LIMITED_INFORMATION, FALSE,
                              static_cast<DWORD>(tid));
  if (hThread == NULL) {
    return kNoStartTimeTicks;
  }
  FILETIME creation_time, exit_time, kernel_time, user_time;
  if (!GetThreadTimes(hThread, &creation_time, &exit_time, &kernel_time,
                      &user_time)) {
    CloseHandle(hThread);
    return kNoStartTimeTicks;
  }
  CloseHandle(hThread);
  ULARGE_INTEGER li;
  li.LowPart = creation_time.dwLowDateTime;
  li.HighPart = creation_time.dwHighDateTime;
  return li.QuadPart;
}
#else
uint64_t RobustOwnershipTracker::ReadStartTimeTicks(pid_t tid) {
  if (tid == 0) {
    return kNoStartTimeTicks;
  }
  std::optional<aos::util::ProcStat> proc_stat = util::ReadProcStat(tid);
  if (!proc_stat.has_value()) {
    return kNoStartTimeTicks;
  }
  return proc_stat->start_time_ticks;
}
#endif

ThreadOwnerStatusSnapshot RobustOwnershipTracker::LoadAcquire() const {
  return ThreadOwnerStatusSnapshot(
      std::atomic_ref<uint32_t>(const_cast<uint32_t &>(mutex_.futex.value))
          .load(std::memory_order_acquire));
}

ThreadOwnerStatusSnapshot RobustOwnershipTracker::LoadRelaxed() const {
  return ThreadOwnerStatusSnapshot(
      std::atomic_ref<uint32_t>(const_cast<uint32_t &>(mutex_.futex.value))
          .load(std::memory_order_relaxed));
}

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

void RobustOwnershipTracker::ForceClear() {
  std::atomic_ref<uint32_t>(mutex_.futex.value)
      .store(0, std::memory_order_release);
  start_time_ticks_ = kNoStartTimeTicks;
}

bool RobustOwnershipTracker::IsHeldBySelf() {
  return death_notification_is_held(&mutex_);
}

bool RobustOwnershipTracker::IsHeldBy(pid_t tid) {
  return LoadRelaxed().tid() == tid;
}

void RobustOwnershipTracker::Acquire() {
#ifdef _WIN32
  pid_t tid = static_cast<pid_t>(GetCurrentThreadId());
#else
  pid_t tid = syscall(SYS_gettid);
#endif
  assert(tid > 0);
  const uint64_t proc_start_time_ticks = ReadStartTimeTicks(tid);
  ABSL_CHECK_NE(proc_start_time_ticks, kNoStartTimeTicks);

  start_time_ticks_ = proc_start_time_ticks;
  death_notification_init(&mutex_);
}

void RobustOwnershipTracker::Release() {
  death_notification_release(&mutex_);
  start_time_ticks_ = kNoStartTimeTicks;
}

::std::string RobustOwnershipTracker::DebugString() const {
  ::std::stringstream s;
  const uint32_t futex =
      std::atomic_ref<uint32_t>(const_cast<uint32_t &>(mutex_.futex.value))
          .load(std::memory_order_relaxed);
  s << "{.tid=aos_mutex(" << ::std::hex << futex;

  if (futex != 0) {
    s << ":";
    if (mutex_owner_is_dead_from_value(futex)) {
      s << "FUTEX_OWNER_DIED|";
    }
    s << "tid=" << mutex_owner_from_value(futex);
  }

  s << "),}";
  return s.str();
}

}  // namespace aos::ipc_lib
