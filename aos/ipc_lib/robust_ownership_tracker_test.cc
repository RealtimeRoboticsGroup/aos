#include "aos/ipc_lib/robust_ownership_tracker.h"

#include <errno.h>
#include <stdlib.h>
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <atomic>
#include <cstdint>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "gtest/gtest.h"

#include "aos/testing/test_child.h"
#include "aos/testing/test_shm.h"

namespace aos::ipc_lib::testing {
namespace {

// NoMatchingPID stores this into the futex's TID field and expects
// OwnerIsDefinitelyAbsolutelyDead() to report the owner gone.  It must be a PID
// that can't name a live process, so the platform's lookup of that TID fails --
// and each platform looks the TID up differently.
#if defined(__linux__)
uint32_t NonexistentPid() {
  // PID_MAX_LIMIT comes from the kernel's include/linux/threads.h, which isn't
  // exported to userspace, so we spell it out here.
  return 1 << 22;
}
#elif defined(__APPLE__)
// Darwin caps PIDs at PID_MAX, so PID_MAX + 1 can't name a live process.
// PID_MAX lives in the kernel-only header bsd/sys/proc_internal.h and isn't in
// the userspace SDK, so define it to its known value if it isn't already.  (The
// darwin check actually returns on a "futex TID != recorded owner thread id"
// mismatch before it ever does the liveness lookup, so any value other than our
// owner works here; PID_MAX + 1 just keeps the intent clear.)
#ifndef PID_MAX
#define PID_MAX 99999
#endif
uint32_t NonexistentPid() { return PID_MAX + 1; }
#else  // Windows
uint32_t NonexistentPid() {
  // Windows looks the PID up via the process API.  999999 is never a valid
  // Windows PID (process ids are always multiples of 4), so the lookup fails.
  return 999999;
}
#endif

}  // namespace

// Capture RobustOwnershipTracker in shared memory so it is shared across a
// fork (on Linux) or simply allocated/shared (on Windows).
//
// Ownership taken by the *test* thread has to be released before the block goes
// away.  Acquire() links the tracker's mutex into this thread's robust list,
// and that list is plain userspace memory on every platform: the next
// death_notification_init() on this thread writes the new mutex's address
// through the old head's `previous` pointer (my_robust_list::Adder::Add), and
// on the platforms without a kernel-walked list RobustListCleaner walks the
// whole thing at thread exit.  Abandoning the entry leaves both of those
// reading and writing freed memory; that it happens to survive today only means
// the allocator handed the page back.
//
// Ownership abandoned by a *child* is a different story and is what the tests
// are actually about: that entry is on the child's list, which the kernel (or,
// where there is no kernel list, RobustListCleaner) takes apart when the child
// dies.  That is how a lockless-queue slot is abandoned when its owner dies.
class SharedRobustOwnershipTracker {
 public:
  SharedRobustOwnershipTracker() : block_(sizeof(RobustOwnershipTracker)) {
    tracker_ = new (block_.get()) RobustOwnershipTracker();
  }

  // Unlinks the tracker from this thread's robust list if this thread is the
  // one holding it.  A tracker abandoned by a dead child is left alone: its
  // futex names the child, not us, and nothing of ours points at it.
  //
  // Tests which fake owner death by overwriting the futex's TID have to put the
  // real value back before this runs, or the entry stays on the list.
  ~SharedRobustOwnershipTracker() {
    if (tracker_->IsHeldBySelf()) {
      tracker_->Release();
    }
  }

  RobustOwnershipTracker &tracker() const { return *tracker_; }

 private:
  aos::testing::SharedMemoryBlock block_;
  RobustOwnershipTracker *tracker_;
};

class RobustOwnershipTrackerTest : public ::testing::Test {
 public:
  // Runs a function in a child (a forked process, or a thread on platforms
  // without fork) and waits for it to finish before resuming.  The tests use
  // this to make a *different* owner claim the tracker and then die: process
  // death on POSIX, thread death on Windows -- each being the granularity at
  // which that platform detects owner death.
  template <typename T>
  void RunInChildAndBlockUntilComplete(T fn) {
    aos::testing::TestChild child;
    child.Start(std::move(fn));
    child.Join();
  }

  // Returns the robust mutex.
  aos_mutex &GetMutex(RobustOwnershipTracker &tracker) {
    return tracker.mutex_;
  }

#ifndef __APPLE__
  // Returns the current start time in ticks.
  uint64_t GetStartTimeTicks(RobustOwnershipTracker &tracker) {
    return tracker.start_time_ticks_.load();
  }

  // Sets the current start time in ticks.
  void SetStartTimeTicks(RobustOwnershipTracker &tracker, uint64_t start_time) {
    tracker.start_time_ticks_ = start_time;
  }
#else
  // Redirects the recorded owner pid, leaving the recorded thread id and
  // p_uniqueid alone.  RecycledPidNoticed uses this to simulate the owner's
  // pid having been recycled to an unrelated process.
  void SetOwnerPid(RobustOwnershipTracker &tracker, pid_t pid) {
    tracker.owner_pid_ = pid;
  }
#endif
};

// Tests that acquiring the futex doesn't erroneously report the owner (i.e.
// "us") as dead.
TEST_F(RobustOwnershipTrackerTest, AcquireWorks) {
  SharedRobustOwnershipTracker shared_tracker;

  EXPECT_FALSE(shared_tracker.tracker().OwnerIsDefinitelyAbsolutelyDead());

  // Run acquire in this process, and expect it should not be dead until
  // after the test finishes.
  shared_tracker.tracker().Acquire();

  // We have ownership. Since we are alive, the owner should not be marked as
  // dead. We can use relaxed ordering since we are the only ones touching the
  // data here.
  EXPECT_FALSE(shared_tracker.tracker().LoadRelaxed().OwnerIsDead());
  EXPECT_FALSE(shared_tracker.tracker().OwnerIsDefinitelyAbsolutelyDead());
}

// Tests that child death without unlocking results in the futex being marked as
// dead, and the owner being very dead.
TEST_F(RobustOwnershipTrackerTest, FutexRecovers) {
  SharedRobustOwnershipTracker shared_tracker;

  RunInChildAndBlockUntilComplete(
      [&]() { shared_tracker.tracker().Acquire(); });

  // Since the child that took ownership died, we expect that death to be
  // reported.
  EXPECT_TRUE(shared_tracker.tracker().LoadRelaxed().OwnerIsDead());
  EXPECT_TRUE(shared_tracker.tracker().OwnerIsDefinitelyAbsolutelyDead());
}

#ifndef _WIN32
// Tests that an owner killed outright -- no cleanup of any kind having run --
// is noticed by the platform's liveness probe.  This is the only test which
// reaches the probe's "dead" verdict with a tid that really lived and really
// died (Darwin's proc_pidinfo lookup, Linux's /proc start-time read); the
// other tests either get the futex marked for them by cleanup, or fake death
// with a tid that never existed.
//
// POSIX-only because it turns on killing the child, and only a forked process
// can be killed; the Windows analog (where the child is an unkillable thread)
// is ExitedOwnerWithLiveHandle below.
TEST_F(RobustOwnershipTrackerTest, KilledOwnerNoticed) {
  SharedRobustOwnershipTracker shared_tracker;

  aos::testing::TestChild child;
  child.Start([&]() {
    shared_tracker.tracker().Acquire();
    while (true) {
      pause();
    }
  });
  // Acquire() writes its metadata before claiming the futex, so once the claim
  // is visible the tracker is fully filled in and safe to kill.
  while (shared_tracker.tracker().LoadRelaxed().IsUnclaimed()) {
    usleep(1000);
  }
  const uint32_t held_futex =
      std::atomic_ref<uint32_t>(GetMutex(shared_tracker.tracker()).futex)
          .load(std::memory_order_relaxed);
  child.Terminate();

  // On Linux the kernel walks the robust list even for SIGKILL and replaces
  // the tid with FUTEX_OWNER_DIED, which would short-circuit the probe.  Put
  // the claimed value back so the probe has to decide for itself; on the
  // platforms where nothing ran (Darwin's cleaner is userspace and SIGKILL
  // skips it), this store is a no-op.
  std::atomic_ref<uint32_t>(GetMutex(shared_tracker.tracker()).futex)
      .store(held_futex, std::memory_order_relaxed);
  EXPECT_FALSE(shared_tracker.tracker().LoadRelaxed().OwnerIsDead());

  EXPECT_TRUE(shared_tracker.tracker().OwnerIsDefinitelyAbsolutelyDead());
}
#endif

#ifdef __APPLE__
// Tests that an owner whose pid now names a different process is noticed as
// dead.  Acquire in this process, then redirect the recorded pid at launchd
// (pid 1) -- a real, live process that is not the recorded owner, exactly
// what a recycled pid looks like.
//
// This is also the only reachable test of the probe's euid-refused fallback:
// run unprivileged, proc_pidinfo(1, PROC_PIDTHREADID64INFO, ...) fails the
// effective-uid gate with EPERM (launchd is root's), which forces the probe
// onto the ungated PROC_PIDUNIQIDENTIFIERINFO path, where launchd's
// p_uniqueid (1) mismatches the recorded one.  Run *as* root there is no
// EPERM: the thread lookup itself fails with ESRCH (our thread id is not one
// of launchd's).  Dead either way, just via different branches.
TEST_F(RobustOwnershipTrackerTest, RecycledPidNoticed) {
  SharedRobustOwnershipTracker shared_tracker;

  shared_tracker.tracker().Acquire();
  EXPECT_FALSE(shared_tracker.tracker().OwnerIsDefinitelyAbsolutelyDead());

  SetOwnerPid(shared_tracker.tracker(), 1);
  EXPECT_TRUE(shared_tracker.tracker().OwnerIsDefinitelyAbsolutelyDead());

  // Put the real pid back so the tracker we still hold is coherent when the
  // destructor Release()s it.
  SetOwnerPid(shared_tracker.tracker(), getpid());
}
#endif

#ifdef _WIN32
// Tests that an owner which exited is noticed as dead even while its thread
// object is still around.
//
// Windows keeps that object alive for as long as anyone holds a handle to it,
// which also keeps its thread id from being reused, so the liveness probe can
// open the dead owner's thread and read back the creation time it always had.
// Reporting that as "still running" leaves the slot unreclaimable forever.
TEST_F(RobustOwnershipTrackerTest, ExitedOwnerWithLiveHandle) {
  SharedRobustOwnershipTracker shared_tracker;

  HANDLE owner_handle = NULL;
  uint32_t held_futex = 0;
  RunInChildAndBlockUntilComplete([&]() {
    shared_tracker.tracker().Acquire();
    // The futex names its owner in the platform's own encoding (Windows
    // stores the thread id shifted, see do_get_tid()), so capture the exact
    // value Acquire() wrote rather than re-deriving it here.
    held_futex =
        std::atomic_ref<uint32_t>(GetMutex(shared_tracker.tracker()).futex)
            .load(std::memory_order_relaxed);
    // Held past the child's death on purpose: that is what keeps the thread
    // object, and the thread id naming it, alive to be looked up.
    owner_handle = OpenThread(THREAD_QUERY_LIMITED_INFORMATION | SYNCHRONIZE,
                              FALSE, GetCurrentThreadId());
  });
  ASSERT_TRUE(owner_handle != NULL);
  ASSERT_NE(held_futex, 0u);

  // The child exited cleanly, so RobustListCleaner marked the futex on its way
  // out, and that mark short-circuits OwnerIsDefinitelyAbsolutelyDead() before
  // the liveness probe ever runs.  Put the futex back the way a thread killed
  // outright leaves it -- still naming its owner, with no cleanup having run --
  // since that is the case the probe exists for.
  std::atomic_ref<uint32_t>(GetMutex(shared_tracker.tracker()).futex)
      .store(held_futex, std::memory_order_relaxed);
  EXPECT_FALSE(shared_tracker.tracker().LoadRelaxed().OwnerIsDead());

  EXPECT_TRUE(shared_tracker.tracker().OwnerIsDefinitelyAbsolutelyDead());

  CloseHandle(owner_handle);
}
#endif

// Tests that a PID which doesn't exist results in the process being noticed as
// dead when we inspect /proc.
TEST_F(RobustOwnershipTrackerTest, NoMatchingPID) {
  SharedRobustOwnershipTracker shared_tracker;

  shared_tracker.tracker().Acquire();
  EXPECT_FALSE(shared_tracker.tracker().LoadRelaxed().OwnerIsDead());
  EXPECT_FALSE(shared_tracker.tracker().OwnerIsDefinitelyAbsolutelyDead());
  std::atomic_ref<uint32_t> futex(GetMutex(shared_tracker.tracker()).futex);
  const uint32_t owner = futex.load(std::memory_order_relaxed);
  futex.store(NonexistentPid(), std::memory_order_relaxed);

  // Since we're only pretending that the owner died (by changing the TID in the
  // futex), we only notice that the owner is dead when spending the time
  // walking through /proc.
  EXPECT_FALSE(shared_tracker.tracker().LoadRelaxed().OwnerIsDead());
  EXPECT_TRUE(shared_tracker.tracker().OwnerIsDefinitelyAbsolutelyDead());

  // Stop pretending: we really do still own this, and the fixture has to be
  // able to see that to unlink it from our robust list.
  futex.store(owner, std::memory_order_relaxed);
}

#ifndef __APPLE__
// Tests that a mismatched start time results in the process being marked as
// dead.
TEST_F(RobustOwnershipTrackerTest, NoMatchingStartTime) {
  SharedRobustOwnershipTracker shared_tracker;

  shared_tracker.tracker().Acquire();
  EXPECT_FALSE(shared_tracker.tracker().LoadRelaxed().OwnerIsDead());
  EXPECT_FALSE(shared_tracker.tracker().OwnerIsDefinitelyAbsolutelyDead());

  EXPECT_NE(GetStartTimeTicks(shared_tracker.tracker()), 0);
  EXPECT_NE(GetStartTimeTicks(shared_tracker.tracker()),
            RobustOwnershipTracker::kNoStartTimeTicks);
  SetStartTimeTicks(shared_tracker.tracker(), 1);

  // Since we're only pretending that the owner died (by changing the tracked
  // start time ticks in the tracker), we only notice that the owner is dead
  // when spending the time walking through /proc.
  EXPECT_FALSE(shared_tracker.tracker().LoadRelaxed().OwnerIsDead());
  EXPECT_TRUE(shared_tracker.tracker().OwnerIsDefinitelyAbsolutelyDead());
}
#endif

}  // namespace aos::ipc_lib::testing
