#include "gtest/gtest.h"

#include "aos/configuration.h"
#include "aos/testing/path.h"

namespace aos::configuration::testing {

// Tests that duplicate channel hashes are rejected during config validation.
// This test relies on the AOS_CONFIGURATION_TEST_COLLISION preprocessor macro
// to force all channel hashes to collide (return UUID::Zero()).
TEST(ConfigurationCollisionDeathTest, ChannelHashCollision) {
  EXPECT_DEATH(
      { ReadConfig(aos::testing::ArtifactPath("aos/testdata/config1.json")); },
      "Duplicate channel hash");
}

}  // namespace aos::configuration::testing
