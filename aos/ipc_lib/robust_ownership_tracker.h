#ifndef AOS_IPC_LIB_ROBUST_OWNERSHIP_TRACKER_H_
#define AOS_IPC_LIB_ROBUST_OWNERSHIP_TRACKER_H_

#include <stdint.h>

#include <atomic>
#include <string>

#include "aos/ipc_lib/aos_sync.h"
#include "aos/realtime.h"

namespace aos::ipc_lib {
namespace testing {
class RobustOwnershipTrackerTest;
}  // namespace testing

// Results of atomically loading the ownership state via RobustOwnershipTracker
// below. This allows the state to be compared and queried later.
class ThreadOwnerStatusSnapshot {
 public:
  ThreadOwnerStatusSnapshot() : futex_(0) {}
  ThreadOwnerStatusSnapshot(uint32_t futex) : futex_(futex) {}
  ThreadOwnerStatusSnapshot(const ThreadOwnerStatusSnapshot &) = default;
  ThreadOwnerStatusSnapshot &operator=(const ThreadOwnerStatusSnapshot &) =
      default;
  ThreadOwnerStatusSnapshot(ThreadOwnerStatusSnapshot &&) = default;
  ThreadOwnerStatusSnapshot &operator=(ThreadOwnerStatusSnapshot &&) = default;

  // Returns if the owner died as noticed by the robust futex using Acquire
  // memory ordering.
  bool OwnerIsDead() const { return mutex_owner_is_dead_from_value(futex_); }

  // Returns true if no one has claimed ownership.
  bool IsUnclaimed() const {
    return mutex_owner_from_value(futex_) == 0 && !OwnerIsDead();
  }

  // Returns the thread ID (a.k.a. "tid") of the owning thread. Use this when
  // trying to access the /proc entry that corresponds to the owning thread for
  // example. Do not use the futex value directly.
  pid_t tid() const { return mutex_owner_from_value(futex_); }

  bool operator==(const ThreadOwnerStatusSnapshot &other) const {
    return other.futex_ == futex_;
  }

 private:
  uint32_t futex_;
};

// This object reliably tracks a thread owning a resource.
class RobustOwnershipTracker {
 public:
  static constexpr uint64_t kNoStartTimeTicks =
      std::numeric_limits<uint64_t>::max();

  // Loads the realtime-compatible contents of the ownership tracker with
  // Acquire memory ordering.
  ThreadOwnerStatusSnapshot LoadAcquire() const;

  // Loads all the realtime-compatible contents of the ownership tracker with
  // Relaxed memory order.
  ThreadOwnerStatusSnapshot LoadRelaxed() const;

  // Checks both the robust futex and process/thread start times to see if the
  // thread is alive.
  bool OwnerIsDefinitelyAbsolutelyDead() const;

  // Clears all ownership state.
  //
  // This should only really be called if you are 100% certain that the owner is
  // dead. Use `LoadAquire().OwnerIsDead()` to determine this.
  void ForceClear();

  // Returns true if this thread holds ownership.
  bool IsHeldBySelf();

  // Returns true if the mutex is held by the provided tid.
  bool IsHeldBy(pid_t tid);

  // Acquires ownership.
  //
  // There is a subtle ordering of operations here: the metadata
  // (start_time_ticks_, owner_pid_, owner_thread_id_) must be fully written
  // and visible BEFORE the futex is claimed by death_notification_init.
  // Otherwise, another process inspecting the tracker concurrently could see
  // that the futex is claimed but find the metadata is still unset (or
  // contains stale values), incorrectly concluding that the owner is dead.
  //
  // Note that if two processes concurrently attempt to call Acquire() on the
  // same tracker, they could overwrite each other's metadata before either
  // claims the futex, leading to state corruption.  Callers must serialize
  // calls to Acquire() (e.g., using a higher-level lock, as is done in the
  // lockless queue implementation via the queue setup lock).
  void Acquire();

  // Releases ownership.
  void Release();

  // Returns a string representing this object.
  std::string DebugString() const;

 private:
  friend class testing::RobustOwnershipTrackerTest;

  aos_mutex mutex_;
#ifdef __APPLE__
  std::atomic<pid_t> owner_pid_;
  std::atomic<uint64_t> owner_thread_id_;
#else
  std::atomic<uint64_t> start_time_ticks_;
#endif
};

}  // namespace aos::ipc_lib

#endif  // AOS_IPC_LIB_ROBUST_OWNERSHIP_TRACKER_H_
