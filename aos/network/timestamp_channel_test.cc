#include "aos/network/timestamp_channel.h"

#include "absl/flags/declare.h"
#include "absl/flags/flag.h"
#include "gtest/gtest.h"

#include "aos/configuration.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/testing/path.h"
#include "aos/testing/ping_pong/ping_generated.h"
#include "aos/testing/tmpdir.h"

namespace aos::message_bridge::testing {

class TimestampChannelTest : public ::testing::Test {
 protected:
  TimestampChannelTest()
      : config_(aos::configuration::ReadConfig(aos::testing::ArtifactPath(
            "aos/network/timestamp_channel_test_config.json"))) {}
  aos::FlatbufferDetachedBuffer<aos::Configuration> config_;
};

// Tests that creating a SimulatedEventLoopFactory with invalid remote timestamp
// channel frequencies fails.
TEST_F(TimestampChannelTest, SimulatedNetworkBridgeFrequencyMismatch) {
  SimulatedEventLoopFactory factory(&config_.message());
  EXPECT_DEATH(factory.RunFor(std::chrono::seconds(1)),
               "rate is lower than the source channel");
}

class TimestampChannelParamTest
    : public TimestampChannelTest,
      public ::testing::WithParamInterface<
          std::tuple<std::string, std::optional<std::string>>> {};

// Tests whether we can or can't retrieve a timestamp channel depending on
// whether it has a valid max frequency configured.
TEST_P(TimestampChannelParamTest, ChannelFrequency) {
  SimulatedEventLoopFactory factory(&config_.message());
  std::unique_ptr<aos::EventLoop> event_loop = factory.MakeEventLoop(
      "test", aos::configuration::GetNode(&config_.message(), "pi1"));
  ChannelTimestampSender timestamp_sender(event_loop.get());
  const aos::Channel *channel =
      event_loop->GetChannel<aos::examples::Ping>(std::get<0>(GetParam()));
  const std::optional<std::string> error_message = std::get<1>(GetParam());
  if (error_message.has_value()) {
    EXPECT_DEATH(timestamp_sender.SenderForChannel(
                     channel, channel->destination_nodes()->Get(0)),
                 error_message.value());
  } else {
    aos::Sender<RemoteMessage> *sender = timestamp_sender.SenderForChannel(
        channel, channel->destination_nodes()->Get(0));
    ASSERT_TRUE(sender != nullptr);
    EXPECT_EQ(absl::StrCat("/pi1/aos/remote_timestamps/pi2",
                           std::get<0>(GetParam()), "/aos-examples-Ping"),
              sender->channel()->name()->string_view());
  }
}

std::tuple<std::string, std::optional<std::string>> MakeParams(
    std::string channel, std::optional<std::string> error) {
  return std::make_tuple(channel, error);
}

INSTANTIATE_TEST_SUITE_P(
    ChannelFrequencyTest, TimestampChannelParamTest,
    ::testing::Values(MakeParams("/nominal", std::nullopt),
                      MakeParams("/timestamps_too_fast", std::nullopt),
                      MakeParams("/timestamps_too_slow",
                                 "rate is lower than the source channel")));
}  // namespace aos::message_bridge::testing
