#include "aos/ipc_lib/memory_estimation.h"

#include "gtest/gtest.h"

#include "aos/json_to_flatbuffer.h"

namespace aos::ipc_lib::testing {
class MemoryEstimationTest : public ::testing::Test {
 protected:
  static constexpr size_t kFirstSize = 100;
  static constexpr size_t kFirstFrequency = 50;
  static constexpr size_t kForwardedSize = 300;
  static constexpr size_t kForwardedFrequency = 100;
  static constexpr size_t kUnloggedSize = 1000000000UL;
  static constexpr size_t kUnloggedFrequency = 100;
  static constexpr size_t kTimestampSize = 96;
  MemoryEstimationTest()
      : fbs_config_(aos::JsonToFlatbuffer<Configuration>(absl::StrFormat(
            R"json({
  "channel_storage_duration": 1000000000,
  "channels": [
    {
      "name": "/first",
      "type": ".aos.test",
      "source_node": "node1",
      "max_size": %d,
      "frequency": %d
    },
    {
      "name": "/unlogged",
      "type": ".aos.test",
      "source_node": "node1",
      "logger": "NOT_LOGGED",
      "max_size": %d,
      "frequency": %d
    },
    {
      "name": "/forwarded",
      "type": ".aos.test",
      "source_node": "node2",
      "max_size": %d,
      "frequency": %d,
      "logger": "LOCAL_LOGGER",
      "destination_nodes": [
        {
          "name": "node1",
          "timestamp_logger": "LOCAL_AND_REMOTE_LOGGER",
          "timestamp_logger_nodes": [ "node2" ]
        }
      ]
    }
  ],
  "nodes": [
    { "name": "node1" },
    { "name": "node2" }
  ]
})json",
            kFirstSize, kFirstFrequency, kUnloggedSize, kUnloggedFrequency,
            kForwardedSize, kForwardedFrequency))),
        config_(&fbs_config_.message()),
        node1_(configuration::GetNode(config_, "node1")),
        node2_(configuration::GetNode(config_, "node2")) {}
  FlatbufferDetachedBuffer<Configuration> fbs_config_;
  const Configuration *config_;
  const aos::Node *node1_;
  const aos::Node *node2_;
};

// Validates that the total shared memory usage numbers are reasonable.
TEST_F(MemoryEstimationTest, TotalMemoryUsage) {
  // Just hard-code these; we want to be able to visually check that they look
  // sane, and they can be updated when needed.
#ifdef _WIN32
  EXPECT_EQ(110000120720, TotalSharedMemoryUsage(config_, node1_));
#else
  EXPECT_EQ(110000121200, TotalSharedMemoryUsage(config_, node1_));
#endif

#ifdef __APPLE__
  EXPECT_EQ(65280, TotalSharedMemoryUsage(config_, node2_));
#else
  EXPECT_EQ(65120, TotalSharedMemoryUsage(config_, node2_));
#endif
}

// Validates that we calculate the appropriate buffer size for each individual
// channel.
TEST_F(MemoryEstimationTest, ChannelBufferUsage) {
  EXPECT_EQ(
      kFirstSize * kFirstFrequency * 1,
      LogMemoryBufferSizeForChannel(
          configuration::GetChannel(config_, "/first", ".aos.test", "", node1_),
          std::chrono::seconds(1)));
  EXPECT_EQ(
      kFirstSize * kFirstFrequency * 2,
      LogMemoryBufferSizeForChannel(
          configuration::GetChannel(config_, "/first", ".aos.test", "", node1_),
          std::chrono::seconds(2)));
  EXPECT_EQ(
      kTimestampSize * kFirstFrequency * 1,
      LogMemoryBufferSizeForChannelTimestamps(
          configuration::GetChannel(config_, "/first", ".aos.test", "", node1_),
          std::chrono::seconds(1)));
  EXPECT_EQ(kUnloggedSize * kUnloggedFrequency * 1,
            LogMemoryBufferSizeForChannel(
                configuration::GetChannel(config_, "/unlogged", ".aos.test", "",
                                          node1_),
                std::chrono::seconds(1)));
}

// Validates that when we ask for much buffer we need for a given logfile on a
// given node that we get the correct overall number.
TEST_F(MemoryEstimationTest, WholeConfigurationBufferUsage) {
  // Check the total logged size for node1, which should be logging the data for
  // the /first channel and the timestamp data for the /forwarded channel.
  EXPECT_EQ((/* Per-second size of /first */ kFirstSize * kFirstFrequency +
             /* Per-second size of timestamps for /forwarded */ kTimestampSize *
                 kForwardedFrequency) *
                1.5,
            TotalLogMemoryBufferSizeForChannels(
                config_, node1_, [](const aos::Channel *) { return true; },
                std::chrono::milliseconds(1500)));
  // Check the total logged size for node1, which should be logging the data and
  // timestamps for the /forwarded channel.
  EXPECT_EQ((/* Per-second size of /forwarded */ kForwardedSize *
                 kForwardedFrequency +
             /* Per-second size of timestamps for /forwarded */ kTimestampSize *
                 kForwardedFrequency) *
                1,
            TotalLogMemoryBufferSizeForChannels(
                config_, node2_, [](const aos::Channel *) { return true; },
                std::chrono::seconds(1)));
  // Validate that if we have a channel filter that always returns false then we
  // don't need any space for logging.
  EXPECT_EQ(0, TotalLogMemoryBufferSizeForChannels(
                   config_, node1_, [](const aos::Channel *) { return false; },
                   std::chrono::seconds(1)));
}
}  // namespace aos::ipc_lib::testing
