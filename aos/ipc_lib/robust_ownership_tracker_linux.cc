#include <assert.h>
#include <linux/futex.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <optional>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/ipc_lib/robust_ownership_tracker.h"
#include "aos/util/proc_stat.h"

namespace aos::ipc_lib {
namespace {

uint64_t ReadStartTimeTicks(pid_t tid) {
  if (tid == 0) {
    return RobustOwnershipTracker::kNoStartTimeTicks;
  }
  std::optional<aos::util::ProcStat> proc_stat = util::ReadProcStat(tid);
  if (!proc_stat.has_value()) {
    return RobustOwnershipTracker::kNoStartTimeTicks;
  }
  return proc_stat->start_time_ticks;
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
  pid_t tid = syscall(SYS_gettid);
  assert(tid > 0);
  const uint64_t proc_start_time_ticks = ReadStartTimeTicks(tid);
  ABSL_CHECK_NE(proc_start_time_ticks, kNoStartTimeTicks);

  start_time_ticks_ = proc_start_time_ticks;
  death_notification_init(&mutex_);
}

}  // namespace aos::ipc_lib
