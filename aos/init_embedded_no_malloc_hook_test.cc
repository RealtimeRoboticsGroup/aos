// Tests that going realtime dies after InitEmbedded() with register_malloc_hook
// off and --die_on_malloc on, rather than letting an allocation through.

#include "gtest/gtest.h"

#include "aos/init.h"
#include "aos/realtime.h"
#include "aos/sanitizers.h"

// The check it dies on is compiled out under the sanitizers.
#if !(defined(AOS_SANITIZE_ADDRESS) || defined(AOS_SANITIZE_MEMORY) || \
      defined(AOS_SANITIZE_THREAD))
TEST(InitEmbeddedNoMallocHookDeathTest, GoingRealtimeDies) {
  EXPECT_DEATH({ aos::ScopedRealtime realtime; }, "Disable --die_on_malloc");
}
#endif

int main(int argc, char **argv) {
  aos::InitOptions options = aos::kEmbeddedInitOptions;
  options.register_malloc_hook = false;
  aos::InitEmbedded(options);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
