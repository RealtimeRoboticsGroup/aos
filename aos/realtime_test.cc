#include "aos/realtime.h"

#include "absl/base/internal/raw_logging.h"
#include "absl/flags/declare.h"
#include <mach/mach.h>
#include <mach/thread_policy.h>
#include <malloc/malloc.h>
#include <pthread.h>
#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
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
    vm_address_t *zones = nullptr;
    unsigned int count = 0;
    kern_return_t kr = malloc_get_all_zones(mach_task_self(), nullptr, &zones, &count);
    
    if (kr != KERN_SUCCESS) {
        perror("malloc_get_all_zones failed");
        return;
    }

    printf("Found %u active malloc zones\n", count);

    for (unsigned int i = 0; i < count; ++i) {
        malloc_zone_t *zone = (malloc_zone_t *)zones[i];
        const char *name = zone->version >= 4 ? zone->zone_name : "unknown";
        
        printf("Zone [%u]: %s at %p\n", i, name, zone);
        printf("Zone [%u]: %s->malloc %p\n", i, name, zone->malloc);

        // If you want to trap, you have to patch EACH zone's function table
        // because the OS will cycle between them or use specific ones for 
        // different allocation sizes (e.g., NanoZone for < 256 bytes).
    }
  EXPECT_DEATH(
      {
        ScopedRealtime rt;
        volatile int *a = reinterpret_cast<volatile int *>(malloc(sizeof(int)));
        *a = 5;
        EXPECT_EQ(*a, 5);
      },
      "RAW: Malloced");
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

// Tests that ABSL_RAW_LOG(FATAL) explodes properly.
TEST(RealtimeDeathTest, RawFatal) {
  EXPECT_DEATH(
      {
        ScopedRealtime rt;
        ABSL_RAW_LOG(FATAL, "Cute message here\n");
      },
      "Cute message here");
}

#endif

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
