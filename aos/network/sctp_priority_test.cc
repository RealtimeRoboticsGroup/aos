#include <poll.h>

#include <string>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "gtest/gtest.h"

#include "aos/network/sctp_client.h"
#include "aos/network/sctp_server.h"

namespace aos::message_bridge::testing {

namespace {

constexpr int kStreams = 3;
constexpr int kLowStream = 1;
constexpr int kHighStream = 2;
// Lower values are higher priority with SCTP_SS_PRIO.
constexpr uint16_t kLowPriority = 200;
constexpr uint16_t kHighPriority = 10;

constexpr size_t kMessageSize = 1000;
constexpr int kLowMessages = 30;

int LocalPort(int fd) {
  struct sockaddr_storage addr;
  socklen_t len = sizeof(addr);
  ABSL_PCHECK(
      getsockname(fd, reinterpret_cast<struct sockaddr *>(&addr), &len) == 0);
  if (addr.ss_family == AF_INET6) {
    return ntohs(reinterpret_cast<struct sockaddr_in6 *>(&addr)->sin6_port);
  }
  return ntohs(reinterpret_cast<struct sockaddr_in *>(&addr)->sin_port);
}

template <typename T>
aos::unique_c_ptr<Message> ReadBlocking(T *sctp) {
  while (true) {
    struct pollfd pfd = {.fd = sctp->fd(), .events = POLLIN, .revents = 0};
    const int result = poll(&pfd, 1, 5000);
    ABSL_PCHECK(result >= 0);
    ABSL_CHECK_GT(result, 0) << ": Timed out waiting for SCTP data";
    aos::unique_c_ptr<Message> message = sctp->Read();
    if (message) {
      return message;
    }
  }
}

template <typename T>
sctp_assoc_t WaitForCommUp(T *sctp) {
  while (true) {
    aos::unique_c_ptr<Message> message = ReadBlocking(sctp);
    if (message->message_type != Message::kNotification) {
      continue;
    }
    const union sctp_notification *snp =
        reinterpret_cast<const union sctp_notification *>(message->data());
    if (snp->sn_header.sn_type == SCTP_ASSOC_CHANGE &&
        snp->sn_assoc_change.sac_state == SCTP_COMM_UP) {
      return snp->sn_assoc_change.sac_assoc_id;
    }
  }
}

}  // namespace

// Tests that SetPriorityScheduler enables SCTP_SS_PRIO.  Queues up a backlog of
// low priority messages behind a receiver which isn't reading, then sends a
// single high priority message.  With SCTP_SS_PRIO, the high priority message
// jumps the queue of low priority messages which haven't been sent yet.
// Without it, the high priority message waits behind every one of them.
TEST(SctpPriorityTest, PrioritySchedulerReordersMessages) {
  SctpServer server(kStreams, "127.0.0.1", 0);
  server.SetMaxReadSize(kMessageSize + 100);
  server.SetMaxWriteSize(kMessageSize + 100);

  SctpClient client("127.0.0.1", LocalPort(server.fd()), kStreams, "127.0.0.1",
                    0);
  client.SetMaxReadSize(kMessageSize + 100);
  client.SetMaxWriteSize(kMessageSize + 100);
  {
    // Shrink the receive window so the server's sends back up in its send
    // queue, where the stream scheduler decides what goes out next.
    int rcvbuf = 16384;
    ABSL_PCHECK(setsockopt(client.fd(), SOL_SOCKET, SO_RCVBUF, &rcvbuf,
                           sizeof(rcvbuf)) == 0);
  }

  // Like the message bridge client, send the first message to establish the
  // association.
  ABSL_CHECK(client.Send(0, "connect", 0));
  const sctp_assoc_t assoc_id = WaitForCommUp(&server);
  client.SetAssociationId(WaitForCommUp(&client));

  // Mirror MessageBridgeServer: NodeConnected() sets up the scheduler, and
  // each channel then sets the priority of its stream.
  server.SetPriorityScheduler(assoc_id);
  ABSL_CHECK(server.SetStreamPriority(assoc_id, kLowStream, kLowPriority));
  ABSL_CHECK(server.SetStreamPriority(assoc_id, kHighStream, kHighPriority));

  const std::string data(kMessageSize, 'x');
  for (int i = 0; i < kLowMessages; ++i) {
    ABSL_CHECK(server.Send(data, assoc_id, kLowStream, 0))
        << ": Send buffer filled up after " << i << " messages";
  }
  ABSL_CHECK(server.Send(data, assoc_id, kHighStream, 0));

  // Count the low priority messages received before the high priority one.
  int position = 0;
  while (true) {
    aos::unique_c_ptr<Message> message = ReadBlocking(&client);
    const bool is_data = message->message_type == Message::kMessage;
    const int stream = message->header.rcvinfo.rcv_sid;
    client.FreeMessage(std::move(message));
    if (!is_data) {
      continue;
    }
    if (stream == kHighStream) {
      break;
    }
    ++position;
  }
  ABSL_LOG(INFO) << "High priority message arrived at position " << position
                 << " of " << kLowMessages + 1;
  EXPECT_LT(position, kLowMessages)
      << "Stream priority had no effect; SetPriorityScheduler did not enable "
         "SCTP_SS_PRIO";
}

}  // namespace aos::message_bridge::testing
