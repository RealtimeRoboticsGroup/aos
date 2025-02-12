#include "aos/util/foxglove_websocket_lib.h"

#include <chrono>
#include <compare>
#include <string>
#include <utility>

#include "absl/container/btree_set.h"
#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/strings/escaping.h"
#include "absl/types/span.h"
#include "flatbuffers/reflection_generated.h"
#include "flatbuffers/string.h"
#include "flatbuffers/vector.h"
#include "nlohmann/json.hpp"
#include <foxglove/websocket/websocket_server.hpp>

#include "aos/configuration.h"
#include "aos/events/context.h"
#include "aos/flatbuffer_merge.h"
#include "aos/flatbuffers.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/time/time.h"
#include "aos/util/mcap_logger.h"

ABSL_FLAG(uint32_t, sorting_buffer_ms, 100,
          "Amount of time to buffer messages to sort them before sending "
          "them to foxglove.");

namespace {

// Period at which to poll the fetchers for all the channels.
constexpr std::chrono::milliseconds kPollPeriod{50};

void PrintFoxgloveMessage(foxglove::WebSocketLogLevel log_level,
                          char const *message) {
  switch (log_level) {
    case foxglove::WebSocketLogLevel::Debug:
      VLOG(1) << message;
      break;
    case foxglove::WebSocketLogLevel::Info:
      LOG(INFO) << message;
      break;
    case foxglove::WebSocketLogLevel::Warn:
      LOG(WARNING) << message;
      break;
    case foxglove::WebSocketLogLevel::Error:
      LOG(ERROR) << message;
      break;
    case foxglove::WebSocketLogLevel::Critical:
      LOG(FATAL) << message;
      break;
  }
}

}  // namespace

namespace aos {
FoxgloveWebsocketServer::FoxgloveWebsocketServer(
    aos::EventLoop *event_loop, uint32_t port, Serialization serialization,
    FetchPinnedChannels fetch_pinned_channels,
    CanonicalChannelNames canonical_channels)
    : event_loop_(event_loop),
      serialization_(serialization),
      fetch_pinned_channels_(fetch_pinned_channels),
      canonical_channels_(canonical_channels),
      server_(
          "aos_foxglove", &PrintFoxgloveMessage,
          {
              .capabilities =
                  {
                      // Specify server capabilities here.
                      // https://github.com/foxglove/ws-protocol/blob/main/docs/spec.md#fields
                  },
          }) {
  for (const aos::Channel *channel :
       *event_loop_->configuration()->channels()) {
    const bool is_pinned = (channel->read_method() == ReadMethod::PIN);
    if (aos::configuration::ChannelIsReadableOnNode(channel,
                                                    event_loop_->node()) &&
        (!is_pinned || fetch_pinned_channels_ == FetchPinnedChannels::kYes)) {
      const FlatbufferDetachedBuffer<reflection::Schema> schema =
          RecursiveCopyFlatBuffer(channel->schema());
      const std::string shortest_name =
          ShortenedChannelName(event_loop_->configuration(), channel,
                               event_loop_->name(), event_loop_->node());
      std::string name_to_send;
      switch (canonical_channels_) {
        case CanonicalChannelNames::kCanonical:
          name_to_send = channel->name()->string_view();
          break;
        case CanonicalChannelNames::kShortened:
          name_to_send = shortest_name;
          break;
      }
      // TODO(philsc): Add all the channels at once instead of individually.
      const std::vector<ChannelId> ids =
          (serialization_ == Serialization::kJson)
              ? server_.addChannels({foxglove::ChannelWithoutId{
                    .topic = name_to_send + " " + channel->type()->str(),
                    .encoding = "json",
                    .schemaName = channel->type()->str(),
                    .schema =
                        JsonSchemaForFlatbuffer({channel->schema()}).dump()}})
              : server_.addChannels({foxglove::ChannelWithoutId{
                    .topic = name_to_send + " " + channel->type()->str(),
                    .encoding = "flatbuffer",
                    .schemaName = channel->type()->str(),
                    .schema = absl::Base64Escape(
                        {reinterpret_cast<const char *>(schema.span().data()),
                         schema.span().size()})}});
      CHECK_EQ(ids.size(), 1u);
      const ChannelId id = ids[0];
      CHECK(fetchers_.count(id) == 0);
      fetchers_[id] =
          FetcherState{.fetcher = event_loop_->MakeRawFetcher(channel)};
    }
  }

  foxglove::ServerHandlers<foxglove::ConnHandle> handlers;
  handlers.subscribeHandler = [this](ChannelId channel,
                                     foxglove::ConnHandle client_handle) {
    if (fetchers_.count(channel) == 0) {
      return;
    }
    if (!active_channels_.contains(channel)) {
      // Catch up to the latest message on the requested channel, then subscribe
      // to it.
      fetchers_[channel].fetcher->Fetch();
    }
    // Take note that this client is now listening on this channel.
    active_channels_[channel].insert(client_handle);
  };
  handlers.unsubscribeHandler = [this](ChannelId channel,
                                       foxglove::ConnHandle client_handle) {
    auto it = active_channels_.find(channel);
    if (it == active_channels_.end()) {
      // As far as we're aware, no one is listening on this channel. This might
      // be a bogus request from the client. Either way, ignore it.
      return;
    }

    // Remove the client from the list of clients that receive new messages on
    // this channel.
    it->second.erase(client_handle);
    if (it->second.empty()) {
      // If this was the last client for this channel, then we don't need to
      // fetch from this channel anymore.
      active_channels_.erase(it);
    }
  };
  server_.setHandlers(std::move(handlers));

  aos::TimerHandler *timer = event_loop_->AddTimer([this]() {
    // In order to run the websocket server, we just let it spin every cycle for
    // a bit. This isn't great for integration, but lets us stay in control and
    // until we either have (a) a chance to locate a file descriptor to hand
    // epoll; or (b) rewrite the foxglove websocket server to use seasocks
    // (which we know how to integrate), we'll just function with this.
    // TODO(james): Tighter integration into our event loop structure.
    server_.run_for(kPollPeriod / 2);

    // Unfortunately, we can't just push out all the messages as they come in.
    // Foxglove expects that the timestamps associated with each message to be
    // monotonic, and if you send things out of order then it will clear the
    // state of the visualization entirely, which makes viewing plots
    // impossible. If the user only accesses a single channel, that is fine, but
    // as soon as they try to use multiple channels, you encounter interleaving.
    // To resolve this, we specify a buffer (--sorting_buffer_ms), and only send
    // out messages older than that time, sorting everything before we send it
    // out.
    const aos::monotonic_clock::time_point sort_until =
        event_loop_->monotonic_now() -
        std::chrono::milliseconds(absl::GetFlag(FLAGS_sorting_buffer_ms));

    // Pair of <send_time, channel id>.
    absl::btree_set<std::pair<aos::monotonic_clock::time_point, ChannelId>>
        fetcher_times;

    // Go through and seed fetcher_times with the first message on each channel.
    for (const auto &[channel, _connections] : active_channels_) {
      FetcherState *fetcher = &fetchers_[channel];
      if (fetcher->sent_current_message) {
        if (fetcher->fetcher->FetchNext()) {
          fetcher->sent_current_message = false;
        }
      }
      if (!fetcher->sent_current_message) {
        const aos::monotonic_clock::time_point send_time =
            fetcher->fetcher->context().monotonic_event_time;
        if (send_time <= sort_until) {
          fetcher_times.insert(std::make_pair(send_time, channel));
        }
      }
    }

    // Send the oldest message continually until we run out of messages to send.
    while (!fetcher_times.empty()) {
      const ChannelId channel = fetcher_times.begin()->second;
      FetcherState *fetcher = &fetchers_[channel];
      for (foxglove::ConnHandle connection : active_channels_[channel]) {
        switch (serialization_) {
          case Serialization::kJson: {
            const std::string json = aos::FlatbufferToJson(
                fetcher->fetcher->channel()->schema(),
                static_cast<const uint8_t *>(fetcher->fetcher->context().data));
            server_.sendMessage(
                connection, channel,
                fetcher_times.begin()->first.time_since_epoch().count(),
                reinterpret_cast<const uint8_t *>(json.data()), json.size());
            break;
          }
          case Serialization::kFlatbuffer:
            server_.sendMessage(
                connection, channel,
                fetcher_times.begin()->first.time_since_epoch().count(),
                static_cast<const uint8_t *>(fetcher->fetcher->context().data),
                fetcher->fetcher->context().size);
        }
      }
      fetcher_times.erase(fetcher_times.begin());
      fetcher->sent_current_message = true;
      if (fetcher->fetcher->FetchNext()) {
        fetcher->sent_current_message = false;
        const aos::monotonic_clock::time_point send_time =
            fetcher->fetcher->context().monotonic_event_time;
        if (send_time <= sort_until) {
          fetcher_times.insert(std::make_pair(send_time, channel));
        }
      }
    }
  });

  event_loop_->OnRun([timer, this]() {
    timer->Schedule(event_loop_->monotonic_now(), kPollPeriod);
  });

  server_.start("0.0.0.0", port);
}
FoxgloveWebsocketServer::~FoxgloveWebsocketServer() { server_.stop(); }
}  // namespace aos
