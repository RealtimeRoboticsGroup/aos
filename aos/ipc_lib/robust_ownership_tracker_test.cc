#include "aos/ipc_lib/robust_ownership_tracker.h"

#include <errno.h>
#include <stdlib.h>
#ifndef _WIN32
#include <sys/wait.h>
#else
#include <windows.h>

#include <thread>
#endif

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "gtest/gtest.h"

#include "aos/testing/test_shm.h"

namespace aos::ipc_lib::testing {

// Capture RobustOwnershipTracker in shared memory so it is shared across a
// fork (on Linux) or simply allocated/shared (on Windows).
class SharedRobustOwnershipTracker {
 public:
  SharedRobustOwnershipTracker() : block_(sizeof(RobustOwnershipTracker)) {
    tracker_ = new (block_.get()) RobustOwnershipTracker();
  }
  ~SharedRobustOwnershipTracker() {
    if (acquired_) {
      tracker_->Release();
    }
  }

  RobustOwnershipTracker &tracker() const { return *tracker_; }

  void Acquire() {
    tracker_->Acquire();
    acquired_ = true;
  }

  void PreventRelease() { acquired_ = false; }

 private:
  aos::testing::SharedMemoryBlock block_;
  RobustOwnershipTracker *tracker_;
  bool acquired_ = false;
};

class RobustOwnershipTrackerTest : public ::testing::Test {
 public:
#ifdef _WIN32
  template <typename T>
  void RunInChildAndBlockUntilComplete(T fn) {
    std::thread thread(fn);
    thread.join();
  }
#else
  // Runs a function in a child process, and then exits afterwards.  Waits for
  // the child to finish before resuming.
  template <typename T>
  void RunInChildAndBlockUntilComplete(T fn) {
    pid_t pid = fork();
    if (pid == 0) {
      fn();
      ABSL_LOG(INFO) << "Child exiting normally.";
      exit(0);
      return;
    }

    ABSL_LOG(INFO) << "Child has pid " << pid;

    while (true) {
      ABSL_LOG(INFO) << "Waiting for child.";
      int status;
      const pid_t waited_on = waitpid(pid, &status, 0);
      // Check for failure.
      if (waited_on == -1) {
        if (errno == EINTR) continue;
        ABSL_PLOG(FATAL) << ": waitpid(" << pid << ", " << &status
                         << ", 0) failed";
      }
      ABSL_CHECK_EQ(waited_on, pid)
          << ": waitpid() got child " << waited_on << " instead of " << pid;
      ABSL_CHECK(WIFEXITED(status));
      ABSL_LOG(INFO) << "Status " << WEXITSTATUS(status);
      ABSL_CHECK(WEXITSTATUS(status) == 0);
      return;
    }
  }
#endif

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
#endif
};

// Tests that acquiring the futex doesn't erroneously report the owner (i.e.
// "us") as dead.
TEST_F(RobustOwnershipTrackerTest, AcquireWorks) {
  SharedRobustOwnershipTracker shared_tracker;

  EXPECT_FALSE(shared_tracker.tracker().OwnerIsDefinitelyAbsolutelyDead());

  // Run acquire in this process, and expect it should not be dead until
  // after the test finishes.
  shared_tracker.Acquire();

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

// Tests that a PID which doesn't exist results in the process being noticed as
// dead when we inspect /proc.
TEST_F(RobustOwnershipTrackerTest, NoMatchingPID) {
  SharedRobustOwnershipTracker shared_tracker;

  shared_tracker.Acquire();
  EXPECT_FALSE(shared_tracker.tracker().LoadRelaxed().OwnerIsDead());
  EXPECT_FALSE(shared_tracker.tracker().OwnerIsDefinitelyAbsolutelyDead());
  std::atomic_ref<uint32_t>(GetMutex(shared_tracker.tracker()).futex.value)
      .store(999999, std::memory_order_relaxed);

  // Since we're only pretending that the owner died (by changing the TID in the
  // futex), we only notice that the owner is dead when spending the time
  // walking through /proc.
  EXPECT_FALSE(shared_tracker.tracker().LoadRelaxed().OwnerIsDead());
  EXPECT_TRUE(shared_tracker.tracker().OwnerIsDefinitelyAbsolutelyDead());

  // We have corrupted the futex. Prevent the RAII destructor from calling
  // Release() on it, which would crash with EPERM under Linux.
  shared_tracker.PreventRelease();
}

#ifndef __APPLE__
// Tests that a mismatched start time results in the process being marked as
// dead.
TEST_F(RobustOwnershipTrackerTest, NoMatchingStartTime) {
  SharedRobustOwnershipTracker shared_tracker;

  shared_tracker.Acquire();
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
