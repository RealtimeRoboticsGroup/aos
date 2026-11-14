#include <array>
#include <chrono>
#include <functional>
#include <optional>
#include <thread>
#include <vector>

#include "absl/flags/declare.h"
#include "absl/flags/flag.h"
#include "absl/log/absl_log.h"
#include "gtest/gtest.h"

#include "aos/events/shm_event_loop.h"
#include "aos/ipc_lib/event.h"
#include "aos/network/message_bridge_client_lib.h"
#include "aos/network/message_bridge_server_generated.h"
#include "aos/network/message_bridge_test_lib.h"
#include "aos/testing/ping_pong/ping_generated.h"
#include "aos/testing/ping_pong/pong_generated.h"
#include "aos/util/application_name.h"

ABSL_DECLARE_FLAG(int32_t, rmem);

namespace aos::message_bridge::testing {

namespace {

// Number of low priority messages to queue up ahead of the high priority one.
// This needs to be enough to fill the receiver's window, with the rest waiting
// in the sender's SCTP send queue.
constexpr int kLowMessages = 30;

void SendPing(aos::Sender<examples::Ping> *sender, int value) {
  aos::Sender<examples::Ping>::Builder builder = sender->MakeBuilder();
  // Pad the message out to ~1kB so a handful of them fill the receive window.
  std::array<int32_t, 250> data{};
  builder.fbb()->CreateVector(data.data(), data.size());
  examples::Ping::Builder ping_builder = builder.MakeBuilder<examples::Ping>();
  ping_builder.add_value(value);
  builder.CheckOk(builder.Send(ping_builder.Finish()));
}

// Polls `fetcher` until `done` returns true for the latest message.
template <typename T>
void WaitFor(aos::Fetcher<T> *fetcher, std::function<bool(const T &)> done) {
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (std::chrono::steady_clock::now() < end) {
    if (fetcher->Fetch() && done(*fetcher->get())) {
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  FAIL() << "Timed out waiting on " << fetcher->channel()->name()->str();
}

// Runs a function when it goes out of scope.
class OnScopeExit {
 public:
  explicit OnScopeExit(std::function<void()> fn) : fn_(std::move(fn)) {}
  ~OnScopeExit() { fn_(); }

 private:
  std::function<void()> fn_;
};

const ServerChannelStatistics *FindChannel(const ServerStatistics &stats,
                                           size_t channel_index) {
  for (const ServerChannelStatistics *channel :
       *stats.connections()->Get(0)->channels()) {
    if (channel->channel_index() == channel_index) {
      return channel;
    }
  }
  return nullptr;
}

}  // namespace

// Tests that a message on a high priority channel is forwarded ahead of a
// backlog of messages on a low priority channel.
//
// The message bridge client on pi2 is stalled so it stops reading from its
// socket.  This closes the SCTP receive window and leaves the low priority
// messages queued in pi1's SCTP send queue.  The SCTP_SS_PRIO stream scheduler
// should then send the high priority message ahead of the queued low priority
// ones once pi2 starts reading again.
TEST_P(MessageBridgeParameterizedTest, HighPriorityJumpsQueue) {
  // Shrink the receive buffers so the receive window fills up quickly.
  absl::SetFlag(&FLAGS_rmem, 16384);

  pi1_.OnPi();
  absl::SetFlag(&FLAGS_application_name, "sender");
  aos::ShmEventLoop pi1_event_loop(&config_.message());
  aos::Sender<examples::Ping> low_sender =
      pi1_event_loop.MakeSender<examples::Ping>("/test/low");
  aos::Sender<examples::Ping> high_sender =
      pi1_event_loop.MakeSender<examples::Ping>("/test/high");
  aos::Fetcher<ServerStatistics> pi1_server_statistics_fetcher =
      pi1_event_loop.MakeFetcher<ServerStatistics>("/aos");
  const size_t low_channel_index = configuration::ChannelIndex(
      pi1_event_loop.configuration(), low_sender.channel());
  const size_t high_channel_index = configuration::ChannelIndex(
      pi1_event_loop.configuration(), high_sender.channel());

  pi1_.MakeServer();

  pi2_.OnPi();
  absl::SetFlag(&FLAGS_application_name, "receiver");
  aos::ShmEventLoop pi2_event_loop(&config_.message());
  aos::Sender<examples::Pong> stall_sender =
      pi2_event_loop.MakeSender<examples::Pong>("/pi2/stall");
  aos::Fetcher<examples::Ping> low_fetcher =
      pi2_event_loop.MakeFetcher<examples::Ping>("/test/low");
  aos::Fetcher<examples::Ping> high_fetcher =
      pi2_event_loop.MakeFetcher<examples::Ping>("/test/high");

  pi2_.MakeClient();

  // Stall the pi2 message bridge client when asked to, until released.  This
  // stands in for a receiver which is too busy to keep up.
  aos::Event stalled;
  aos::Event release;
  pi2_.client_event_loop_->MakeWatcher(
      "/pi2/stall", [&stalled, &release](const examples::Pong &) {
        stalled.Set();
        release.Wait();
      });

  pi1_.StartServer();
  pi2_.StartClient();

  // Make sure the client is released and both threads are stopped before the
  // events go out of scope, even if an assertion fails.
  OnScopeExit stop([this, &release]() {
    release.Set();
    pi2_.StopClient();
    pi1_.StopServer();
  });

  WaitFor<ServerStatistics>(
      &pi1_server_statistics_fetcher, [](const ServerStatistics &stats) {
        return stats.connections()->Get(0)->state() == State::CONNECTED;
      });
  ASSERT_FALSE(::testing::Test::HasFailure());

  // The client's connect timer can't fire while it is stalled.  If the stall
  // lasts longer than kReconnectTimeout, the overdue timer fires as soon as the
  // client is released and aborts the connection, dropping everything queued.
  const auto stall_start = std::chrono::steady_clock::now();
  {
    aos::Sender<examples::Pong>::Builder builder = stall_sender.MakeBuilder();
    builder.CheckOk(
        builder.Send(builder.MakeBuilder<examples::Pong>().Finish()));
  }
  ASSERT_TRUE(stalled.WaitTimeout(std::chrono::seconds(10)));

  for (int i = 0; i < kLowMessages; ++i) {
    SendPing(&low_sender, i);
  }
  SendPing(&high_sender, kLowMessages);

  // Wait until the message bridge server has handed every message to the
  // kernel.  No retries means none of them were left waiting in the message
  // bridge itself, so the order they leave in is up to SCTP.
  WaitFor<ServerStatistics>(
      &pi1_server_statistics_fetcher,
      [low_channel_index, high_channel_index](const ServerStatistics &stats) {
        const ServerChannelStatistics *low =
            FindChannel(stats, low_channel_index);
        const ServerChannelStatistics *high =
            FindChannel(stats, high_channel_index);
        return low != nullptr && high != nullptr &&
               low->sent_packets() == kLowMessages && high->sent_packets() == 1;
      });
  ASSERT_FALSE(::testing::Test::HasFailure());
  EXPECT_EQ(pi1_server_statistics_fetcher->connections()->Get(0)->retry_count(),
            0u);
  ASSERT_LT(std::chrono::steady_clock::now() - stall_start,
            SctpClientConnection::kReconnectTimeout)
      << "Client was stalled long enough that it will reconnect when released";

  release.Set();

  // Collect the time each message was received on pi2.
  std::vector<aos::monotonic_clock::time_point> low_times;
  std::optional<aos::monotonic_clock::time_point> high_time;
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while ((low_times.size() < kLowMessages || !high_time) &&
         std::chrono::steady_clock::now() < end) {
    while (low_fetcher.FetchNext()) {
      low_times.push_back(low_fetcher.context().monotonic_event_time);
    }
    if (high_fetcher.FetchNext()) {
      high_time = high_fetcher.context().monotonic_event_time;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(low_times.size(), kLowMessages);
  ASSERT_TRUE(high_time.has_value());

  int position = 0;
  for (const aos::monotonic_clock::time_point low_time : low_times) {
    if (low_time < *high_time) {
      ++position;
    }
  }
  ABSL_LOG(INFO) << "High priority message arrived at position " << position
                 << " of " << kLowMessages + 1;
  // With first-come-first-served scheduling, the high priority message waits
  // behind every low priority message and arrives last.
  EXPECT_LT(position, kLowMessages)
      << "High priority message waited behind every low priority message";
}

INSTANTIATE_TEST_SUITE_P(MessageBridgeTests, MessageBridgeParameterizedTest,
                         ::testing::Values(Param{
                             "message_bridge_priority_test_config.json",
                             false}));

}  // namespace aos::message_bridge::testing
