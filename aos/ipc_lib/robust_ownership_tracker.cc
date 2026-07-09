#include "aos/ipc_lib/robust_ownership_tracker.h"

#include <assert.h>

#include <atomic>
#include <limits>
#include <ostream>
#include <sstream>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

namespace aos::ipc_lib {

ThreadOwnerStatusSnapshot RobustOwnershipTracker::LoadAcquire() const {
  return ThreadOwnerStatusSnapshot(mutex_load_value(&mutex_));
}

ThreadOwnerStatusSnapshot RobustOwnershipTracker::LoadRelaxed() const {
  return ThreadOwnerStatusSnapshot(mutex_load_value(&mutex_));
}

void RobustOwnershipTracker::ForceClear() {
  std::atomic_ref<uint32_t>(mutex_.futex).store(0, std::memory_order_release);
#ifdef __APPLE__
  owner_pid_ = 0;
  owner_thread_id_ = 0;
  owner_unique_id_ = 0;
#else
  start_time_ticks_ = kNoStartTimeTicks;
#endif
}

bool RobustOwnershipTracker::IsHeldBySelf() {
  return death_notification_is_held(&mutex_);
}

bool RobustOwnershipTracker::IsHeldBy(pid_t tid) {
  return LoadRelaxed().tid() == tid;
}

void RobustOwnershipTracker::Release() {
  // Must be called with the same higher-level lock held as Acquire() (the queue
  // setup lock in the lockless queue).  That serialization is what makes it
  // safe to clear the metadata below: with no concurrent Acquire() re-taking
  // the slot and no concurrent reader (OwnerIsDefinitelyAbsolutelyDead), there
  // is no window where a claimed futex is paired with cleared/stale metadata.
  // Without the lock, a thread that Acquired the freed slot between the futex
  // release and this clear would have its fresh metadata clobbered.
  death_notification_release(&mutex_);
#ifdef __APPLE__
  owner_pid_ = 0;
  owner_thread_id_ = 0;
  owner_unique_id_ = 0;
#else
  start_time_ticks_ = kNoStartTimeTicks;
#endif
}

::std::string RobustOwnershipTracker::DebugString() const {
  ::std::stringstream s;
  const uint32_t futex = mutex_load_value(&mutex_);
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
