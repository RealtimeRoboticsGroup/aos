#include "aos/ipc_lib/robust_ownership_tracker.h"

#include <assert.h>
#include <linux/futex.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <optional>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

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
  pid_t tid = syscall(SYS_gettid);
  assert(tid > 0);
  const uint64_t proc_start_time_ticks = ReadStartTimeTicks(tid);
  ABSL_CHECK_NE(proc_start_time_ticks, kNoStartTimeTicks);

  start_time_ticks_ = proc_start_time_ticks;
  death_notification_init(&mutex_);
}

}  // namespace aos::ipc_lib
