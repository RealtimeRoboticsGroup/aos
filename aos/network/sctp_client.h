#ifndef AOS_NETWORK_SCTP_CLIENT_H_
#define AOS_NETWORK_SCTP_CLIENT_H_

#include <cstdio>
#include <cstdlib>
#include <string_view>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/types/span.h"

#include "aos/network/sctp_lib.h"
#include "aos/unique_malloc_ptr.h"

namespace aos::message_bridge {

// Class to encapsulate everything needed to be a SCTP client.
class SctpClient {
 public:
  SctpClient(std::string_view remote_host, int remote_port, int streams,
             std::string_view local_host = "0.0.0.0", int local_port = 4646,
             SctpAuthMethod requested_authentication = SctpAuthMethod::kNoAuth);

  ~SctpClient() {}

  // Receives the next packet from the remote.
  aos::unique_c_ptr<Message> Read() { return sctp_.ReadMessage(); }

  // Sends a block of data on a stream with a TTL.
  // TODO(austin): time_to_live should be a chrono::duration
  bool Send(int stream, std::string_view data, int time_to_live) {
    return sctp_.SendMessage(stream, data, time_to_live, sockaddr_remote_,
                             sac_assoc_id_);
  }

  // Aborts a connection.  Returns true on success.
  bool Abort() { return sctp_.Abort(sac_assoc_id_); }

  int fd() { return sctp_.fd(); }

  // Enables the priority scheduler.  This is a SCTP feature which lets us
  // configure the priority per stream so that higher priority packets don't get
  // backed up behind lower priority packets in the networking queues.
  void SetPriorityScheduler(sctp_assoc_t assoc_id);

  // Remote to send to.
  struct sockaddr_storage sockaddr_remote() const { return sockaddr_remote_; }

  void LogSctpStatus(sctp_assoc_t assoc_id);

  void SetMaxReadSize(size_t max_size) { sctp_.SetMaxReadSize(max_size); }
  void SetMaxWriteSize(size_t max_size) { sctp_.SetMaxWriteSize(max_size); }
  void SetPoolSize(size_t pool_size) { sctp_.SetPoolSize(pool_size); }

  void SetAssociationId(sctp_assoc_t sac_assoc_id) {
    sac_assoc_id_ = sac_assoc_id;
  }

  void FreeMessage(aos::unique_c_ptr<Message> &&message) {
    sctp_.FreeMessage(std::move(message));
  }

  void SetAuthKey(absl::Span<const uint8_t> auth_key) {
    sctp_.SetAuthKey(auth_key);
  }

 private:
  struct sockaddr_storage sockaddr_remote_;
  struct sockaddr_storage sockaddr_local_;
  SctpReadWrite sctp_;

  // Valid if != 0.
  sctp_assoc_t sac_assoc_id_ = 0;
};

}  // namespace aos::message_bridge

#endif  //  AOS_NETWORK_SCTP_CLIENT_H_
