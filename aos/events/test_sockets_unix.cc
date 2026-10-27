#include "aos/events/test_sockets.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "absl/log/absl_check.h"

namespace aos::testing {
namespace {

// Non-blocking and close-on-exec, like every descriptor a Pipe hands out.
void SetUp(int fd) {
  ABSL_PCHECK(fcntl(fd, F_SETFL, O_NONBLOCK) == 0);
  // Set after the fact rather than at creation, which macOS cannot do.
  ABSL_PCHECK(fcntl(fd, F_SETFD, FD_CLOEXEC) == 0);
}

}  // namespace

TcpPair::TcpPair() {
  const int listener = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  ABSL_PCHECK(listener >= 0);
  sockaddr_in address = {};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  address.sin_port = 0;
  ABSL_PCHECK(bind(listener, reinterpret_cast<sockaddr *>(&address),
                   sizeof(address)) == 0);
  socklen_t length = sizeof(address);
  ABSL_PCHECK(getsockname(listener, reinterpret_cast<sockaddr *>(&address),
                          &length) == 0);
  ABSL_PCHECK(listen(listener, 1) == 0);

  // Still blocking here, so the connect and the accept can just be waited on;
  // both ends go non-blocking once the connection is up.
  client_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  ABSL_PCHECK(client_ >= 0);
  ABSL_PCHECK(connect(client_, reinterpret_cast<sockaddr *>(&address),
                      sizeof(address)) == 0);
  server_ = accept(listener, nullptr, nullptr);
  ABSL_PCHECK(server_ >= 0);
  ABSL_PCHECK(close(listener) == 0);
  SetUp(client_);
  SetUp(server_);
}

TcpPair::~TcpPair() {
  if (client_ != kInvalidFd) ABSL_PCHECK(close(client_) == 0);
  if (server_ != kInvalidFd) ABSL_PCHECK(close(server_) == 0);
}

void TcpPair::ResetClient() {
  const struct linger abortive = {1, 0};
  ABSL_PCHECK(setsockopt(client_, SOL_SOCKET, SO_LINGER, &abortive,
                         sizeof(abortive)) == 0);
  ABSL_PCHECK(close(client_) == 0);
  client_ = kInvalidFd;
}

void SendOutOfBand(FileDescriptor fd, char byte) {
  ABSL_PCHECK(send(fd, &byte, 1, MSG_OOB) == 1);
}

bool ReceiveOutOfBand(FileDescriptor fd, char *byte) {
  const ssize_t res = recv(fd, byte, 1, MSG_OOB);
  if (res == 1) {
    return true;
  }
  // 0, on Linux, for an urgent pointer seen on a socket that has since been
  // shut down, EAGAIN while the urgent byte is still on its way, and EINVAL
  // once it has been read or when there never was one.
  ABSL_PCHECK(res == 0 ||
              (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINVAL));
  return false;
}
bool LastReadWasReset() { return errno == ECONNRESET; }

DatagramPair::DatagramPair() {
  ABSL_PCHECK(socketpair(AF_UNIX, SOCK_DGRAM, 0, fds_) == 0);
  SetUp(fds_[0]);
  SetUp(fds_[1]);
}

DatagramPair::~DatagramPair() {
  ABSL_PCHECK(close(fds_[0]) == 0);
  ABSL_PCHECK(close(fds_[1]) == 0);
}

}  // namespace aos::testing
