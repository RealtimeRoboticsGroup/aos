#include "aos/util/foxglove_websocket_lib.h"

#include <cctype>
#include <chrono>
#include <compare>
#include <ranges>
#include <regex>
#include <string>
#include <string_view>
#include <utility>

#include "absl/container/btree_set.h"
#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/die_if_null.h"
#include "absl/log/log.h"
#include "absl/strings/escaping.h"
#include "absl/strings/str_split.h"
#include "absl/types/span.h"
#include "flatbuffers/reflection_generated.h"
#include "flatbuffers/string.h"
#include "flatbuffers/vector.h"
#include "nlohmann/json.hpp"
#include <foxglove/websocket/websocket_server.hpp>

#include "aos/configuration.h"
#include "aos/configuration_schema.h"
#include "aos/events/context.h"
#include "aos/flatbuffer_merge.h"
#include "aos/flatbuffers.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/time/time.h"
#include "aos/util/live_metadata_schema.h"
#include "aos/util/mcap_logger.h"

ABSL_FLAG(uint32_t, sorting_buffer_ms, 100,
          "Amount of time to buffer messages to sort them before sending "
          "them to foxglove.");

ABSL_FLAG(uint32_t, poll_period_ms, 50,
          "Period to poll channels at and push messages into the websocket.");

ABSL_FLAG(int64_t, max_lossless_channel_size, 1024 * 1024,
          "Max message size to send without skipping messages.  Any messages "
          "sent faster than --poll_period_ms and bigger than this threshold "
          "will get rate limited with Fetch().");

namespace {

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
    CanonicalChannelNames canonical_channels,
    std::vector<std::regex> client_topic_patterns)
    : event_loop_(event_loop),
      stripped_configuration_(
          configuration::StripConfiguration(event_loop_->configuration())),
      live_metadata_([this] {
        fbs::Builder<LiveMetadataStatic> builder;
        fbs::String<0> *foo = ABSL_DIE_IF_NULL(builder->add_node());
        const std::string_view node_name =
            event_loop_->node()->name()->string_view();
        CHECK(foo->reserve(node_name.size()));
        foo->SetString(node_name);
        return builder;
      }()),
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
                      "clientPublish",
                  },
              .supportedEncodings =
                  {
                      // We accept JSON input from the client when they want to
                      // publish a message to an AOS channel. We then parse the
                      // JSON into a flatbuffer.
                      "json",
                  },
              .metadata = {},
              .sessionId = aos::UUID::Random().ToString(),
              .clientTopicWhitelistPatterns = client_topic_patterns,
          }) {
  const auto get_name_to_send = [this](const Channel *channel) {
    switch (canonical_channels_) {
      case CanonicalChannelNames::kCanonical:
        return channel->name()->str();
      case CanonicalChannelNames::kShortened:
        return ShortenedChannelName(event_loop_->configuration(), channel,
                                    event_loop_->name(), event_loop_->node());
    }
    LOG(FATAL) << "Unreachable";
  };

  // Add some special channels that are not real channels on the system.
  // The current motivation here is to make a live system more similar to
  // looking at an MCAP file.
  if (serialization_ == Serialization::kFlatbuffer) {
    {
      // Add the AOS configuration under the "configuration" channel. This
      // matches what the McapLogger does.
      absl::Span<const uint8_t> configuration_schema = ConfigurationSchema();
      const std::vector<ChannelId> ids =
          server_.addChannels({foxglove::ChannelWithoutId{
              .topic = "configuration",
              .encoding = "flatbuffer",
              .schemaName = Configuration::GetFullyQualifiedName(),
              .schema = absl::Base64Escape(
                  {reinterpret_cast<const char *>(configuration_schema.data()),
                   configuration_schema.size()}),
              .schemaEncoding = std::nullopt}});
      CHECK_EQ(ids.size(), 1u);
      special_channels_.emplace(ids[0],
                                SpecialChannelState{
                                    .message = stripped_configuration_.span(),
                                    .pending_sends = {},
                                });
    }
    {
      // Add the channel that tells the client about the system it connected to.
      absl::Span<const uint8_t> live_metadata_schema = LiveMetadataSchema();
      const std::vector<ChannelId> ids =
          server_.addChannels({foxglove::ChannelWithoutId{
              .topic = "live_metadata",
              .encoding = "flatbuffer",
              .schemaName = LiveMetadata::GetFullyQualifiedName(),
              .schema = absl::Base64Escape(
                  {reinterpret_cast<const char *>(live_metadata_schema.data()),
                   live_metadata_schema.size()}),
              .schemaEncoding = std::nullopt}});
      CHECK_EQ(ids.size(), 1u);
      special_channels_.emplace(
          ids[0],
          SpecialChannelState{
              .message = static_cast<const FlatbufferSpan<LiveMetadata> &>(
                             live_metadata_.AsFlatbufferSpan())
                             .span(),
              .pending_sends = {},
          });
    }
  }

  // Add all the channels that we want foxglove to be able to look at.
  for (const aos::Channel *channel :
       *event_loop_->configuration()->channels()) {
    const bool is_pinned = (channel->read_method() == ReadMethod::PIN);
    const std::string name_to_send = get_name_to_send(channel);
    const std::string topic = name_to_send + " " + channel->type()->str();

    if (aos::configuration::ChannelIsReadableOnNode(channel,
                                                    event_loop_->node()) &&
        (!is_pinned || fetch_pinned_channels_ == FetchPinnedChannels::kYes)) {
      const FlatbufferDetachedBuffer<reflection::Schema> schema =
          RecursiveCopyFlatBuffer(channel->schema());
      // TODO(philsc): Add all the channels at once instead of individually.
      const std::vector<ChannelId> ids =
          (serialization_ == Serialization::kJson)
              ? server_.addChannels({foxglove::ChannelWithoutId{
                    .topic = topic,
                    .encoding = "json",
                    .schemaName = channel->type()->str(),
                    .schema =
                        JsonSchemaForFlatbuffer({channel->schema()}).dump(),
                    .schemaEncoding = std::nullopt,
                }})
              : server_.addChannels({foxglove::ChannelWithoutId{
                    .topic = topic,
                    .encoding = "flatbuffer",
                    .schemaName = channel->type()->str(),
                    .schema = absl::Base64Escape(
                        {reinterpret_cast<const char *>(schema.span().data()),
                         schema.span().size()}),
                    .schemaEncoding = std::nullopt,
                }});
      CHECK_EQ(ids.size(), 1u);
      const ChannelId id = ids[0];
      CHECK(fetchers_.count(id) == 0);
      fetchers_[id] = FetcherState{
          .fetcher = event_loop_->MakeRawFetcher(channel),
          .fetch_next = channel->max_size() <=
                        absl::GetFlag(FLAGS_max_lossless_channel_size),
      };
    }
    if (aos::configuration::ChannelIsSendableOnNode(channel,
                                                    event_loop_->node())) {
      if (std::ranges::any_of(client_topic_patterns,
                              [&topic](const std::regex &regex) {
                                return std::regex_match(topic, regex);
                              })) {
        // This is a topic that we want foxglove to send on. Create a sender
        // pre-emptively for this. We cannot create senders dynamically at
        // runtime.
        const auto [it, success] =
            senders_.emplace(topic, event_loop_->MakeRawSender(channel));
        CHECK(success) << "Internal assumption was broken.";
        CHECK(it->second.get() != nullptr)
            << "MakeRawSender failed for some reason on channel {"
            << aos::FlatbufferToJson(channel) << "}";
      }
    }
  }

  foxglove::ServerHandlers<foxglove::ConnHandle> handlers;
  handlers.subscribeHandler = [this](ChannelId channel,
                                     foxglove::ConnHandle client_handle) {
    if (fetchers_.count(channel) == 0) {
      if (special_channels_.count(channel) == 0) {
        return;
      }
      // Note down that this client wants to receive a message from this special
      // channel.
      special_channels_[channel].pending_sends.insert(client_handle);
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
    auto special_channel_it = special_channels_.find(channel);
    if (it == active_channels_.end() &&
        special_channel_it == special_channels_.end()) {
      // As far as we're aware, no one is listening on this channel. This might
      // be a bogus request from the client. Either way, ignore it.
      return;
    }

    if (special_channel_it != special_channels_.end()) {
      CHECK(it == active_channels_.end())
          << ": Somehow allowed a channel to be both a real channel and a "
             "special channel.";

      // Remove the client from the list of clients that need to receive this
      // message.
      special_channel_it->second.pending_sends.erase(client_handle);
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
  handlers.clientAdvertiseHandler =
      [this](foxglove::ClientAdvertisement client_advertisement,
             foxglove::ConnHandle /*conn_handle*/) {
        const std::string &topic = client_advertisement.topic;
        LOG(INFO) << "Client wants to publish to topic " << topic
                  << " with channelId " << client_advertisement.channelId;
        if (!senders_.contains(topic)) {
          LOG(ERROR) << "Topic " << topic << " has no senders pre-configured.";
        }
      };
  handlers.clientUnadvertiseHandler =
      [](foxglove::ClientChannelId client_channel_id,
         foxglove::ConnHandle /*conn_handle*/) {
        LOG(INFO) << "Client stopped publishing to channel with channelId "
                  << client_channel_id;
      };
  handlers.clientMessageHandler =
      [this](const foxglove::ClientMessage &client_message,
             foxglove::ConnHandle /*conn_handle*/) {
        const std::string &topic = client_message.advertisement.topic;
        auto it = senders_.find(topic);
        if (it == senders_.end()) {
          LOG(ERROR) << "Lacking sender for topic " << topic;
          return;
        }
        if (VLOG_IS_ON(1)) {
          LOG(INFO) << "Got " << client_message.data.size()
                    << " bytes from client: ";
          for (uint8_t byte : client_message.data) {
            std::cerr << std::hex << std::setw(2) << static_cast<int>(byte);
          }
          std::cerr << "\n";
          for (uint8_t byte : client_message.data) {
            if (isprint(byte)) {
              std::cerr << static_cast<char>(byte);
            } else {
              std::cerr << " ";
            }
          }
          std::cerr << "\n";
          LOG(INFO) << "Trying to parse it as a flatbuffer.";
        }

        // Validate the header as per:
        // https://github.com/foxglove/ws-protocol/blob/main/docs/spec.md#client-message-data
        constexpr int kNumHeaderBytes = 5;

        if (client_message.data.size() < kNumHeaderBytes) {
          LOG(ERROR) << "Expected at least 5 bytes from the client. Got only "
                     << client_message.data.size() << " bytes.";
          return;
        } else if (client_message.data[0] != foxglove::OpCode::TEXT) {
          LOG(ERROR) << "Got unexpected opcode from client: "
                     << static_cast<int>(client_message.data[0]);
          return;
        }

        RawSender *sender = it->second.get();

        const Channel *channel = ABSL_DIE_IF_NULL(sender->channel());
        // Skip the header when parsing the message.
        const std::string_view data(
            reinterpret_cast<const char *>(client_message.data.data()) +
                kNumHeaderBytes,
            client_message.data.size() - kNumHeaderBytes);

        flatbuffers::FlatBufferBuilder fbb(sender->fbb_allocator()->size(),
                                           sender->fbb_allocator());
        flatbuffers::Offset<flatbuffers::Table> msg_offset =
            aos::JsonToFlatbuffer(data, channel->schema(), &fbb);
        if (msg_offset.IsNull()) {
          LOG(ERROR) << "Failed to parse client message as a flatbuffer.";
          return;
        }
        fbb.Finish(msg_offset);

        RawSender::Error error = sender->Send(fbb.GetSize());
        if (error != RawSender::Error::kOk) {
          LOG(ERROR) << "Failed to send message on " << topic << ": "
                     << static_cast<int>(error);
          return;
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
    server_.run_for(
        std::chrono::milliseconds(absl::GetFlag(FLAGS_poll_period_ms)) / 2);

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

    // Go through the special channels and service those.
    for (auto &channel_and_state : special_channels_) {
      const ChannelId channel = channel_and_state.first;
      SpecialChannelState &state = channel_and_state.second;
      const int64_t timestamp =
          event_loop_->monotonic_now().time_since_epoch().count();

      for (foxglove::ConnHandle connection : state.pending_sends) {
        server_.sendMessage(connection, channel, timestamp,
                            state.message.data(), state.message.size());
      }
      // Take note that we sent out all the special messages.
      state.pending_sends.clear();
    }

    // Go through and seed fetcher_times with the first message on each channel.
    for (const auto &[channel, _connections] : active_channels_) {
      FetcherState *fetcher = &fetchers_[channel];
      if (fetcher->sent_current_message) {
        bool fetched;
        if (fetcher->fetch_next) {
          fetched = fetcher->fetcher->FetchNext();
        } else {
          fetched = fetcher->fetcher->Fetch();
        }
        if (fetched) {
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
      if (fetcher->fetch_next) {
        if (fetcher->fetcher->FetchNext()) {
          fetcher->sent_current_message = false;
          const aos::monotonic_clock::time_point send_time =
              fetcher->fetcher->context().monotonic_event_time;
          if (send_time <= sort_until) {
            fetcher_times.insert(std::make_pair(send_time, channel));
          }
        }
      }
    }
  });

  event_loop_->OnRun([timer, this]() {
    timer->Schedule(
        event_loop_->monotonic_now(),
        std::chrono::milliseconds(absl::GetFlag(FLAGS_poll_period_ms)));
  });

  server_.start("0.0.0.0", port);
}
FoxgloveWebsocketServer::~FoxgloveWebsocketServer() { server_.stop(); }
}  // namespace aos
