#include "aos/ipc_lib/robust_ownership_tracker.h"

#include <assert.h>
#include <process.h>
#include <windows.h>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

namespace aos::ipc_lib {

namespace {

// Returns true if the thread behind this handle has already exited.
//
// can_wait says whether the handle carries SYNCHRONIZE, which makes for the
// better of the two tests: a thread handle becomes signaled when, and only
// when, the thread terminates.  Without it we have to read the exit code,
// which reports STILL_ACTIVE for a running thread -- and also for the
// pathological thread which exited with 259 as its code.
bool ThreadHasExited(HANDLE thread, bool can_wait) {
  if (can_wait) {
    return WaitForSingleObject(thread, 0) == WAIT_OBJECT_0;
  }
  DWORD exit_code;
  return GetExitCodeThread(thread, &exit_code) && exit_code != STILL_ACTIVE;
}

// Returns the creation time of tid in ticks, or kNoStartTimeTicks if there is
// no live thread with that id.
//
// Opening the thread is not enough to know that it is running.  A Windows
// thread object outlives the thread itself for as long as anyone holds a
// handle to it, so OpenThread against a thread which exited while someone
// still held a handle succeeds, and reports the creation time it always had.
// Handing that back would tell OwnerIsDefinitelyAbsolutelyDead() the owner is
// alive and well, and a slot whose owner died holding it would never be
// reclaimed -- which is the whole job of this file.  So ask whether the thread
// has exited as well.
//
// GetThreadTimes' exit_time cannot answer that question: it is explicitly
// undefined for a thread which has not exited.
uint64_t ReadStartTimeTicks(pid_t tid) {
  if (tid == 0) {
    return RobustOwnershipTracker::kNoStartTimeTicks;
  }

  // Ask for SYNCHRONIZE so ThreadHasExited() can use the handle's signaled
  // state, but fall back to the query right on its own rather than failing the
  // open: a thread we cannot open is reported dead below, and wrongly
  // declaring a live owner dead is far worse than the leaked slot being fixed
  // here.
  bool can_wait = true;
  HANDLE thread = OpenThread(THREAD_QUERY_LIMITED_INFORMATION | SYNCHRONIZE,
                             FALSE, static_cast<DWORD>(tid));
  if (thread == NULL) {
    can_wait = false;
    thread = OpenThread(THREAD_QUERY_LIMITED_INFORMATION, FALSE,
                        static_cast<DWORD>(tid));
  }
  if (thread == NULL) {
    return RobustOwnershipTracker::kNoStartTimeTicks;
  }

  uint64_t result = RobustOwnershipTracker::kNoStartTimeTicks;
  FILETIME creation_time, exit_time, kernel_time, user_time;
  if (!ThreadHasExited(thread, can_wait) &&
      GetThreadTimes(thread, &creation_time, &exit_time, &kernel_time,
                     &user_time)) {
    ULARGE_INTEGER li;
    li.LowPart = creation_time.dwLowDateTime;
    li.HighPart = creation_time.dwHighDateTime;
    result = li.QuadPart;
  }
  CloseHandle(thread);
  return result;
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
  pid_t tid = static_cast<pid_t>(GetCurrentThreadId());
  assert(tid > 0);
  const uint64_t proc_start_time_ticks = ReadStartTimeTicks(tid);
  ABSL_CHECK_NE(proc_start_time_ticks, kNoStartTimeTicks);

  start_time_ticks_ = proc_start_time_ticks;
  death_notification_init(&mutex_);
}

}  // namespace aos::ipc_lib
