#include <array>
#include <cstdlib>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "absl/flags/declare.h"
#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "gtest/gtest.h"

#include "aos/uuid.h"

ABSL_DECLARE_FLAG(int64_t, deterministic_uuid_seed);

namespace aos::testing {
namespace {

// The UUIDs UUID::Random() hands out, in order, for each seed we test with.
// Checked in rather than computed, so that any change to how a seeded run
// generates UUIDs shows up here as a failure rather than going unnoticed.
//
// Each thread has its own generator.  The sequences below are for the first
// thread (number 0) to generate a UUID, which is what the sequence test runs.
// The BUILD file runs this test once per seed, either passing
// --deterministic_uuid_seed or setting ${TEST_RANDOM_SEED}, and each run looks
// up its own row below.
struct SeededSequence {
  int64_t seed;
  std::array<const char *, 4> uuids;
};

constexpr SeededSequence kSeededSequences[] = {
    {0,
     {"f645627a-cacf-477a-8764-ecd6125616a5",
      "055ed5f5-c723-4ede-8e79-cccd7685c7ad",
      "1eb8a394-2489-41c2-811b-6a79432fc6bc",
      "74379122-0452-4e26-8e38-9d62f1e7ae73"}},
    {1,
     {"65ef89d8-6b50-4f86-8f71-e03a0788abe9",
      "6a758f6e-23f0-42c7-82b7-f7cd5017b184",
      "e0dde5d4-15d4-4945-890f-e0ab19976bab",
      "f5698832-b14a-4024-80c8-d5690408c09f"}},
    {42,
     {"9924b5a6-050e-48b8-88f0-36002efe2e9e",
      "4d9c8601-6e05-4a86-8a66-230b4af95d3b",
      "7417c99e-3361-43fb-8377-0bdc9bae2b73",
      "1003f2f1-f790-4e62-8e04-193baf3d70ae"}},
};

// Returns the seed this binary was configured with, from whichever of the two
// places asked for it.
int64_t ConfiguredSeed() {
  const int64_t flag_seed = absl::GetFlag(FLAGS_deterministic_uuid_seed);
  if (flag_seed >= 0) {
    return flag_seed;
  }
  const char *const test_seed = getenv("TEST_RANDOM_SEED");
  ABSL_CHECK(test_seed != nullptr)
      << ": run this test with --deterministic_uuid_seed or TEST_RANDOM_SEED";
  return atoi(test_seed);
}

const SeededSequence &ExpectedSequence() {
  const int64_t seed = ConfiguredSeed();
  for (const SeededSequence &sequence : kSeededSequences) {
    if (sequence.seed == seed) {
      return sequence;
    }
  }
  ABSL_LOG(FATAL) << ": no expected UUIDs checked in for seed " << seed;
}

// Tests that a seeded run produces the UUIDs it produced when these were
// recorded, which is what lets a tool seeded this way write the same bytes on
// every run and on any machine.
TEST(UUIDDeterministicTest, MatchesCheckedInSequence) {
  const SeededSequence &expected = ExpectedSequence();
  for (size_t i = 0; i < expected.uuids.size(); ++i) {
    EXPECT_EQ(UUID::Random().ToString(), expected.uuids[i])
        << ": UUID " << i << " for seed " << expected.seed;
  }
}

// Tests that seeding does not cost uniqueness: callers are entitled to assume
// two UUIDs differ, whether or not the run repeats.  Each thread uses its own
// generator, so there should still be no collisions across threads.
TEST(UUIDDeterministicTest, StillUniqueAcrossThreads) {
  constexpr int kThreads = 4;
  constexpr int kPerThread = 64;

  // Each thread fills a vector of its own, out of an array that cannot grow,
  // so the threads share nothing but UUID::Random() itself.
  std::array<std::vector<UUID>, kThreads> generated;
  std::vector<std::thread> threads;
  for (int i = 0; i < kThreads; ++i) {
    threads.emplace_back([&generated, i]() {
      for (int j = 0; j < kPerThread; ++j) {
        generated[i].push_back(UUID::Random());
      }
    });
  }
  for (std::thread &thread : threads) {
    thread.join();
  }

  std::set<std::string> unique;
  for (const std::vector<UUID> &per_thread : generated) {
    for (const UUID &uuid : per_thread) {
      unique.insert(uuid.ToString());
    }
  }
  EXPECT_EQ(unique.size(), kThreads * kPerThread);
}

}  // namespace
}  // namespace aos::testing
