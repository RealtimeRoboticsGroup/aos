#include "aos/realtime.h"

#include <chrono>
#include <semaphore>
#include <thread>

#include "absl/base/internal/raw_logging.h"
#include "absl/flags/declare.h"
#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/synchronization/mutex.h"
#include "gtest/gtest.h"

#include "aos/init.h"
#include "aos/sanitizers.h"

ABSL_DECLARE_FLAG(bool, die_on_malloc);

namespace aos::testing {

// Tests that ScopedRealtime handles the simple case.
TEST(RealtimeTest, ScopedRealtime) {
  CheckNotRealtime();
  {
    ScopedRealtime rt;
    CheckRealtime();
  }
  CheckNotRealtime();
}

// Tests that ScopedRealtime handles nesting.
TEST(RealtimeTest, DoubleScopedRealtime) {
  CheckNotRealtime();
  {
    ScopedRealtime rt;
    CheckRealtime();
    {
      ScopedRealtime rt2;
      CheckRealtime();
    }
    CheckRealtime();
  }
  CheckNotRealtime();
}

// Tests that ScopedRealtime handles nesting with ScopedNotRealtime.
TEST(RealtimeTest, ScopedNotRealtime) {
  CheckNotRealtime();
  {
    ScopedRealtime rt;
    CheckRealtime();
    {
      ScopedNotRealtime nrt;
      CheckNotRealtime();
    }
    CheckRealtime();
  }
  CheckNotRealtime();
}

// Tests that ScopedRealtimeRestorer works both when starting RT and nonrt.
TEST(RealtimeTest, ScopedRealtimeRestorer) {
  CheckNotRealtime();
  {
    ScopedRealtime rt;
    CheckRealtime();
    {
      ScopedRealtimeRestorer restore;
      CheckRealtime();

      MarkRealtime(false);
      CheckNotRealtime();
    }
    CheckRealtime();
  }
  CheckNotRealtime();

  {
    ScopedRealtimeRestorer restore;
    CheckNotRealtime();

    MarkRealtime(true);
    CheckRealtime();
  }
  CheckNotRealtime();
}

// Tests that getters and setters properly interact with thread realtime
// priority.
TEST(RealtimeTest, GetSetRealtimePriority) {
  UnsetCurrentThreadRealtimePriority();
  EXPECT_EQ(GetCurrentThreadRealtimePriority(), 0);
  SetCurrentThreadRealtimePriority(30);
  EXPECT_EQ(GetCurrentThreadRealtimePriority(), 30);
  UnsetCurrentThreadRealtimePriority();
}

// Tests that getters and setters properly interact with thread scheduling
// policy.
TEST(RealtimeTest, GetSetSchedulingPolicy) {
  UnsetCurrentThreadRealtimePriority();
  EXPECT_EQ(GetCurrentThreadSchedulingPolicy(), SCHED_OTHER);
  SetCurrentThreadRealtimePriority(1, SCHED_FIFO);
  EXPECT_EQ(GetCurrentThreadSchedulingPolicy(), SCHED_FIFO);
  SetCurrentThreadRealtimePriority(1, SCHED_RR);
  EXPECT_EQ(GetCurrentThreadSchedulingPolicy(), SCHED_RR);
  UnsetCurrentThreadRealtimePriority();
}

// Malloc hooks don't work with asan/msan.
#if !defined(AOS_SANITIZE_MEMORY) && !defined(AOS_SANITIZE_ADDRESS)

// Tests that CHECK statements give real error messages rather than die on
// malloc.
TEST(RealtimeDeathTest, Check) {
  EXPECT_DEATH(
      {
        ScopedRealtime rt;
        CHECK_EQ(1, 2) << ": Numbers aren't equal.";
      },
      "Numbers aren't equal");
  EXPECT_DEATH(
      {
        ScopedRealtime rt;
        CHECK_GT(1, 2) << ": Cute error message";
      },
      "Cute error message");
}

// Tests that CHECK statements give real error messages rather than die on
// malloc.
TEST(RealtimeDeathTest, Fatal) {
  EXPECT_DEATH(
      {
        ScopedRealtime rt;
        LOG(FATAL) << "Cute message here";
      },
      "Cute message here");
}

TEST(RealtimeDeathTest, Malloc) {
  EXPECT_DEATH(
      {
        ScopedRealtime rt;
        volatile int *a = reinterpret_cast<volatile int *>(malloc(sizeof(int)));
        *a = 5;
        EXPECT_EQ(*a, 5);
      },
      "RAW: Malloced");
}

// The first *contended* absl::Mutex lock in a process runs a one-time
// initialization (absl::base_internal::NumCPUs(), to size the spin loop) that
// mallocs on Windows.  If that first contention lands on a realtime thread,
// the malloc hook kills the process.  MarkRealtime() forces the
// initialization before the thread goes realtime; this pins that.
//
// A death test because the property is per-process: the child is a fresh
// process, so the lock below really is its first contended one.  Everything
// before the realtime section is std:: rather than absl:: on purpose -- an
// absl::Notification would take an absl::Mutex of its own and could warm
// the initialization early, and the test would prove nothing.
TEST(RealtimeDeathTest, FirstContendedMutexLockDoesNotMalloc) {
  EXPECT_EXIT(
      {
        absl::Mutex mu;
        std::binary_semaphore holder_has_lock(0);
        std::binary_semaphore main_is_locking(0);
        std::thread holder([&]() {
          mu.lock();
          holder_has_lock.release();
          main_is_locking.acquire();
          // Keep the lock long enough for the main thread to get from its
          // release() into lock() and fail the fast path.  Microseconds
          // against 50ms; and too short would only make this test vacuous,
          // never wrong.  The holder cannot wait for the main thread instead,
          // because the main thread is about to block on it.
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
          mu.unlock();
        });
        holder_has_lock.acquire();
        {
          ScopedRealtime rt;
          main_is_locking.release();
          // Held by the other thread, so this is the contended slow path.
          mu.lock();
          mu.unlock();
        }
        holder.join();
        exit(0);
      },
      ::testing::ExitedWithCode(0), "");
}

TEST(RealtimeDeathTest, Realloc) {
  EXPECT_DEATH(
      {
        void *a = malloc(sizeof(int));
        ScopedRealtime rt;
        volatile int *b =
            reinterpret_cast<volatile int *>(realloc(a, sizeof(int) * 2));
        *b = 5;
        EXPECT_EQ(*b, 5);
      },
      "RAW: Malloced");
}

TEST(RealtimeDeathTest, Calloc) {
  EXPECT_DEATH(
      {
        ScopedRealtime rt;
        volatile int *a =
            reinterpret_cast<volatile int *>(calloc(1, sizeof(int)));
        *a = 5;
        EXPECT_EQ(*a, 5);
      },
      "RAW: Malloced");
}

TEST(RealtimeDeathTest, New) {
  EXPECT_DEATH(
      {
        ScopedRealtime rt;
        volatile int *a = new int;
        *a = 5;
        EXPECT_EQ(*a, 5);
      },
      "RAW: Malloced");
}

TEST(RealtimeDeathTest, NewArray) {
  EXPECT_DEATH(
      {
        ScopedRealtime rt;
        volatile int *a = new int[3];
        *a = 5;
        EXPECT_EQ(*a, 5);
      },
      "RAW: Malloced");
}

#ifndef _WIN32
// Tests that the signal handler drops RT permission and prints out a real
// backtrace instead of crashing on the resulting mallocs.
TEST(RealtimeDeathTest, SignalHandler) {
  EXPECT_DEATH(
      {
        ScopedRealtime rt;
        int x = reinterpret_cast<const volatile int *>(0)[0];
        LOG(INFO) << x;
      },
      "\\*\\*\\* SIGSEGV received at .*");
}
#endif

// Tests that ABSL_RAW_LOG(FATAL) explodes properly.
TEST(RealtimeDeathTest, RawFatal) {
  EXPECT_DEATH(
      {
        ScopedRealtime rt;
        ABSL_RAW_LOG(FATAL, "Cute message here\n");
      },
      "Cute message here");
}

#endif  // !defined(AOS_SANITIZE_MEMORY) && !defined(AOS_SANITIZE_ADDRESS)

#if !defined(__APPLE__) && !defined(_WIN32)
// Tests that we see which CPUs we tried to set when it fails. This can be
// useful for debugging.
TEST(RealtimeDeathTest, SetAffinityErrorMessage) {
  EXPECT_DEATH(
      { SetCurrentThreadAffinity(MakeCpusetFromCpus({1000})); },
      "sched_setaffinity\\(0, sizeof\\(cpu_set_t\\), "
      "cpuset\\.native_handle\\(\\)\\) == 0 "
      "\\{CPUs 1000\\}: Invalid argument");
  EXPECT_DEATH(
      { SetCurrentThreadAffinity(MakeCpusetFromCpus({1000, 1001})); },
      "sched_setaffinity\\(0, sizeof\\(cpu_set_t\\), "
      "cpuset\\.native_handle\\(\\)\\) == 0 "
      "\\{CPUs 1000, 1001\\}: Invalid argument");
}
#endif

// Tests CpuSet functionality.
TEST(CpuSetTest, BasicFunctionality) {
  CpuSet s;
  EXPECT_TRUE(s.Empty());
  for (int i = 0; i < static_cast<int>(CpuSet::kSize); ++i) {
    EXPECT_FALSE(s.IsSet(i));
  }

  s.Set(1);
  EXPECT_FALSE(s.Empty());
  EXPECT_TRUE(s.IsSet(1));
  EXPECT_FALSE(s.IsSet(0));

  s.Set(10);
  EXPECT_TRUE(s.IsSet(1));
  EXPECT_TRUE(s.IsSet(10));
  EXPECT_FALSE(s.IsSet(9));

  s.Clear(1);
  EXPECT_FALSE(s.IsSet(1));
  EXPECT_TRUE(s.IsSet(10));
  EXPECT_FALSE(s.Empty());

  s.Clear();
  EXPECT_TRUE(s.Empty());
  EXPECT_FALSE(s.IsSet(10));
}

TEST(CpuSetTest, Equality) {
  CpuSet s1;
  CpuSet s2;

  EXPECT_EQ(s1, s2);

  s1.Set(1);
  EXPECT_NE(s1, s2);

  s2.Set(1);
  EXPECT_EQ(s1, s2);

  s1.Set(2);
  EXPECT_NE(s1, s2);
}

TEST(CpuSetTest, Stringify) {
  CpuSet s;
  EXPECT_EQ(absl::StrFormat("%v", s), "{CPUs }");
  s.Set(1);
  EXPECT_EQ(absl::StrFormat("%v", s), "{CPUs 1}");
  s.Set(3);
  // Iteration order usually 0..N
  EXPECT_EQ(absl::StrFormat("%v", s), "{CPUs 1, 3}");
}

TEST(CpuSetTest, MakeCpusetFromCpus) {
  CpuSet s = MakeCpusetFromCpus({1, 3});
  EXPECT_TRUE(s.IsSet(1));
  EXPECT_TRUE(s.IsSet(3));
  EXPECT_FALSE(s.IsSet(2));
  EXPECT_EQ(absl::StrFormat("%v", s), "{CPUs 1, 3}");
}

}  // namespace aos::testing

// We need a special gtest main to force die_on_malloc support on.  Otherwise
// we can't test CHECK statements before turning die_on_malloc on globally.
GTEST_API_ int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

#if !defined(AOS_SANITIZE_MEMORY) && !defined(AOS_SANITIZE_ADDRESS)
  absl::SetFlag(&FLAGS_die_on_malloc, true);
#endif

  aos::InitGoogle(&argc, &argv);

  return RUN_ALL_TESTS();
}
