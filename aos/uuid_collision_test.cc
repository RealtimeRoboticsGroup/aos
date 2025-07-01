#include "absl/container/btree_set.h"
#include "absl/log/log.h"
#include "gtest/gtest.h"

#include "aos/uuid.h"

namespace aos::testing {

// Tests that modest numbers of UUID::Random() calls cannot create UUID
// collisions (to test that we have not *completely* messed up the random number
// generation).
TEST(UUIDTest, CollisionTest) {
  absl::btree_set<UUID> uuids;
  // When we only had ~32 bits of randomness in our UUIDs, we could generate
  // issues with only ~sqrt(2 ** 32) (aka 2 ** 16) UUIDs.
  // Just go up to 2 ** 24, since too much longer just makes this test take
  // obnoxiously long.
  size_t ii_max = (1UL << 24);
  for (size_t ii = 0; ii < ii_max; ++ii) {
    UUID uuid = UUID::Random();
    ASSERT_FALSE(uuids.count(uuid) > 0) << ii;
    uuids.insert(uuid);

    if ((ii % (ii_max / 64)) == 0 && ii != 0) {
      LOG(INFO) << static_cast<double>(ii) / static_cast<double>(ii_max) * 100.
                << "%";
    }
  }
}

// Tests that our random seed generation for the mt19937 does not trivially
// collide.
TEST(UUIDTest, SeedInitializationTest) {
  std::uniform_int_distribution<uint64_t> distribution(0);
  absl::btree_set<uint64_t> values;
  // This test takes longer than the above due to needing to query randomness
  // from the OS substantially. However, covering a range of 2 ** 18 should
  // readily catch things if we are accidentally using 32-bit seeds.
  size_t ii_max = 1UL << 18;
  for (size_t ii = 0; ii < ii_max; ++ii) {
    std::mt19937 twister = internal::FullySeededRandomGenerator();
    const uint64_t value = distribution(twister);
    ASSERT_FALSE(values.count(value) > 0) << ii;
    values.insert(value);
    if ((ii % (ii_max / 64)) == 0 && ii != 0) {
      LOG(INFO) << static_cast<double>(ii) / static_cast<double>(ii_max) * 100.0
                << "%";
    }
  }
}

}  // namespace aos::testing
