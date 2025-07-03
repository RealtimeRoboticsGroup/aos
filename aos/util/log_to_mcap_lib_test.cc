#include "aos/util/log_to_mcap_lib.h"

#include <string>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/flags/reflection.h"
#include "absl/log/log.h"
#include "gtest/gtest.h"

#include "aos/configuration_static.h"

ABSL_DECLARE_FLAG(std::vector<std::string>, include_channels);
ABSL_DECLARE_FLAG(std::vector<std::string>, drop_channels);

namespace aos::util::testing {

// The parameters for the ChannelDroppingTest below.
struct ChannelDroppingTestArgs {
  // The value of --include_channels for this test.
  std::vector<std::string> include_channels_list;
  // The value of --drop_channels for this test.
  std::vector<std::string> drop_channels_list;
  // The name of the channel that we're testing.
  std::string channel_name;
  // The type of the channel that we're testing.
  std::string channel_type;
  // Whether or not we expect the channel to be dropped.
  bool should_be_dropped;
};

using ChannelDroppingTest = ::testing::TestWithParam<ChannelDroppingTestArgs>;

// Validates that channels are dropped as specified in --include_channels and
// --drop_channels.
TEST_P(ChannelDroppingTest, ChannelsAreDroppedAsExpected) {
  const ChannelDroppingTestArgs &args = GetParam();

  // Set up the command line flags and instantiate the tester.
  absl::FlagSaver flag_saver;
  absl::SetFlag(&FLAGS_include_channels, args.include_channels_list);
  absl::SetFlag(&FLAGS_drop_channels, args.drop_channels_list);
  std::function<bool(const Channel *)> drop_tester =
      GetChannelShouldBeDroppedTester();

  // Build the channel object that we're testing against.
  fbs::Builder<ChannelStatic> channel_builder;
  ChannelStatic *channel = channel_builder.get();
  fbs::SetStringOrDie(channel->add_name(), args.channel_name);
  fbs::SetStringOrDie(channel->add_type(), args.channel_type);

  // Perform the test.
  EXPECT_EQ(drop_tester(&channel->AsFlatbuffer()), args.should_be_dropped);
}

INSTANTIATE_TEST_SUITE_P(
    ChannelDroppingTests, ChannelDroppingTest,
    ::testing::Values(
        // With a "catch all" regex in --include_channels, no channel should be
        // dropped.
        ChannelDroppingTestArgs{
            .include_channels_list = {".*"},
            .drop_channels_list = {},
            .channel_name = "/aos",
            .channel_type = "foo.bar.Baz",
            .should_be_dropped = false,
        },
        // With an empty --include_channels list, all channels are dropped.
        ChannelDroppingTestArgs{
            .include_channels_list = {},
            .drop_channels_list = {},
            .channel_name = "/aos",
            .channel_type = "foo.bar.Baz",
            .should_be_dropped = true,
        },
        // With a non-empty --include_channels, non-matching channels are
        // dropped.
        ChannelDroppingTestArgs{
            .include_channels_list = {".*.OnlyThisMessageType"},
            .drop_channels_list = {},
            .channel_name = "/aos",
            .channel_type = "foo.bar.Baz",
            .should_be_dropped = true,
        },
        // With a non-empty --include_channels, matching channels are not
        // dropped.
        ChannelDroppingTestArgs{
            .include_channels_list = {".*.OnlyThisMessageType"},
            .drop_channels_list = {},
            .channel_name = "/aos",
            .channel_type = "foo.bar.OnlyThisMessageType",
            .should_be_dropped = false,
        },
        // Channels matching both --include_channels and --drop_channels are
        // dropped.
        ChannelDroppingTestArgs{
            .include_channels_list = {".*.OnlyThisMessageType"},
            .drop_channels_list = {"/aos.*"},
            .channel_name = "/aos",
            .channel_type = "foo.bar.OnlyThisMessageType",
            .should_be_dropped = true,
        }));

}  // namespace aos::util::testing
