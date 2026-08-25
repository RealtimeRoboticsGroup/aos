// Cross-platform tests for aos_sync.  The OS-specific halves live in their
// own files and are conditionally compiled in by the build file
// (aos_sync_windows_test.cc today); this file holds what is true everywhere.

#include "aos/ipc_lib/aos_sync.h"

#include "gtest/gtest.h"

namespace aos::testing {

// The futex word is the ABI everything else builds on: exactly the kernel's
// 32-bit futex on Linux, and the same layout driven by the backends
// elsewhere.
static_assert(sizeof(aos_futex) == sizeof(uint32_t));
static_assert(alignof(aos_futex) >= alignof(uint32_t));
static_assert(sizeof(aos_condition) == sizeof(uint32_t));

// The TID mask and flag bits must tile the word exactly (kernel-mandated
// values on Linux; everything decodes futex snapshots with them).
static_assert((FUTEX_TID_MASK | FUTEX_WAITERS | FUTEX_OWNER_DIED) ==
              0xffffffff);
static_assert((FUTEX_TID_MASK & (FUTEX_WAITERS | FUTEX_OWNER_DIED)) == 0);

// Basic set/unset semantics of the futex API, no blocking involved.
TEST(AosSyncTest, FutexSetUnset) {
  aos_futex futex{};
  EXPECT_EQ(1, futex_unset(&futex));  // Wasn't set.
  EXPECT_GE(futex_set(&futex), 0);
  EXPECT_EQ(0, futex_unset(&futex));  // Was set.
  EXPECT_EQ(1, futex_unset(&futex));
}

}  // namespace aos::testing
