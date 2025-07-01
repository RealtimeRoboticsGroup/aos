#ifndef AOS_UTIL_FOXGLOVE_WEBSOCKET_LIB_H_
#define AOS_UTIL_FOXGLOVE_WEBSOCKET_LIB_H_
#include <stdint.h>

#include <map>
#include <memory>
#include <regex>
#include <set>

#include "foxglove/websocket/websocket_notls.hpp"
#include "foxglove/websocket/websocket_server.hpp"

#include "aos/events/event_loop.h"
#include "aos/util/live_metadata_static.h"

namespace aos {
// This class implements a live AOS -> Foxglove Websocket Protocol connection,
// making use of the implementation at
// https://github.com/foxglove/ws-protocol/tree/main/cpp to send JSON messages
// to a foxglove studio client.
// See foxglove_websocket.cc for some usage notes.
class FoxgloveWebsocketServer {
 public:
  // Whether to serialize the messages into the MCAP file as JSON or
  // flatbuffers.
  enum class Serialization {
    kJson,
    kFlatbuffer,
  };
  enum class FetchPinnedChannels {
    kYes,
    kNo,
  };
  // Whether to attempt to shorten channel names.
  enum class CanonicalChannelNames {
    // Just use the full, unambiguous, channel names.
    kCanonical,
    // Use GetChannelAliases() to determine the shortest possible name for the
    // channel for the current node, and use that in the MCAP file. This makes
    // it so that the channels in the resulting file are more likely to match
    // the channel names that are used in "real" applications.
    kShortened,
  };

  // The client_topic_patterns is a list of regexes that are tested against
  // foxglove topics (channel name plus type). When a topic matches, the
  // foxglove client is allowed to send messages on that topic. Those messages
  // will be be sent to the corresponding AOS channel.
  FoxgloveWebsocketServer(aos::EventLoop *event_loop, uint32_t port,
                          Serialization serialization,
                          FetchPinnedChannels fetch_pinned_channels,
                          CanonicalChannelNames canonical_channels,
                          std::vector<std::regex> client_topic_patterns);
  ~FoxgloveWebsocketServer();

 private:
  typedef foxglove::ChannelId ChannelId;

  struct FetcherState {
    std::unique_ptr<aos::RawFetcher> fetcher;
    // Whether the current message in the fetcher has been sent to the client.
    // Starts as true because the fetcher starts with no data.
    // This is necessary because we have to send all of our messages out
    // in order, which we can only do once we know the timestamp of each
    // message. And we can only know the timestamp after having called Fetch()
    // on the fetcher. Once we get around to actually sending the data, we can
    // set this to true so that we know it is safe to call FetchNext() again.
    bool sent_current_message = true;

    // If false, throttle message rates using Fetch instead of using FetchNext
    // to get all of them.  This helps with large messages like images.
    bool fetch_next = true;
  };

  // A special channel is a channel that doesn't exist in the AOS configuration.
  // It's a purely virtual channel that is useful for conveying metadata to the
  // client. For example, there is no real channel that contains the AOS
  // configuration. If a client wants to use the configuration for something,
  // however, then they need access to it somehow. That's where special channels
  // come in. Each special channel has a unique ChannelId, separate from the
  // ChannelId's of the real channels.
  //
  // At the moment special channels only contain a single message. We have no
  // use case for special channels to contain more than a single message.
  struct SpecialChannelState {
    // Non-owning reference to the special message.
    absl::Span<const uint8_t> message;
    // A list of connections that still need the special message.
    std::set<foxglove::ConnHandle, std::owner_less<>> pending_sends;
  };

  aos::EventLoop *event_loop_;
  FlatbufferDetachedBuffer<Configuration> stripped_configuration_;
  fbs::Builder<LiveMetadataStatic> live_metadata_;
  const Serialization serialization_;
  const FetchPinnedChannels fetch_pinned_channels_;
  const CanonicalChannelNames canonical_channels_;
  foxglove::Server<foxglove::WebSocketNoTls> server_;
  // A map of the state for every special channel.
  std::map<ChannelId, SpecialChannelState> special_channels_;
  // A map of fetchers for every single channel that could be subscribed to.
  std::map<ChannelId, FetcherState> fetchers_;
  // The set of channels that we have clients actively subscribed to. Each
  // channel can have an arbitrary number of clients subscribed.
  std::unordered_map<ChannelId,
                     std::set<foxglove::ConnHandle, std::owner_less<>>>
      active_channels_;

  // The sender for a specific topic.
  std::map<std::string, std::unique_ptr<RawSender>> senders_;
};
}  // namespace aos
#endif  // AOS_UTIL_FOXGLOVE_WEBSOCKET_LIB_H_
