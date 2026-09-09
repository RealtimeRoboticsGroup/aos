#include "aos/events/test_sockets.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/select.h>
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

void TcpPair::ShutdownClient() { ABSL_PCHECK(shutdown(client_, SHUT_WR) == 0); }

void TcpPair::CloseServer() {
  ABSL_PCHECK(close(server_) == 0);
  server_ = kInvalidFd;
}

TcpListener::TcpListener() {
  listener_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  ABSL_PCHECK(listener_ >= 0);
  sockaddr_in address = {};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  address.sin_port = 0;
  ABSL_PCHECK(bind(listener_, reinterpret_cast<sockaddr *>(&address),
                   sizeof(address)) == 0);
  ABSL_PCHECK(listen(listener_, 4) == 0);
  SetUp(listener_);
}

TcpListener::~TcpListener() {
  for (int i = 0; i < other_count_; ++i) {
    ABSL_PCHECK(close(others_[i]) == 0);
  }
  ABSL_PCHECK(close(listener_) == 0);
}

FileDescriptor TcpListener::Connect() {
  ABSL_CHECK_LT(other_count_, 8);
  sockaddr_in address = {};
  socklen_t length = sizeof(address);
  ABSL_PCHECK(getsockname(listener_, reinterpret_cast<sockaddr *>(&address),
                          &length) == 0);
  // Blocking for the connect, which a loopback listener with room in its
  // backlog completes before accept().
  const int client = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  ABSL_PCHECK(client >= 0);
  ABSL_PCHECK(connect(client, reinterpret_cast<sockaddr *>(&address),
                      sizeof(address)) == 0);
  SetUp(client);
  others_[other_count_++] = client;
  return client;
}

FileDescriptor TcpListener::Accept() {
  const int accepted = accept(listener_, nullptr, nullptr);
  if (accepted < 0) {
    ABSL_PCHECK(errno == EAGAIN || errno == EWOULDBLOCK);
    return kInvalidFd;
  }
  ABSL_CHECK_LT(other_count_, 8);
  SetUp(accepted);
  others_[other_count_++] = accepted;
  return accepted;
}

void TcpListener::Reset(FileDescriptor fd) {
  int i = 0;
  while (i < other_count_ && others_[i] != fd) ++i;
  ABSL_CHECK_LT(i, other_count_)
      << ": fd " << fd << " is not one of this listener's";
  const struct linger abortive = {1, 0};
  ABSL_PCHECK(
      setsockopt(fd, SOL_SOCKET, SO_LINGER, &abortive, sizeof(abortive)) == 0);
  ABSL_PCHECK(close(fd) == 0);
  others_[i] = others_[--other_count_];
}

int TcpListener::port() const {
  sockaddr_in address = {};
  socklen_t length = sizeof(address);
  ABSL_PCHECK(getsockname(listener_, reinterpret_cast<sockaddr *>(&address),
                          &length) == 0);
  return ntohs(address.sin_port);
}

int UnusedLoopbackPort() {
  const int s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  ABSL_PCHECK(s >= 0);
  sockaddr_in address = {};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  address.sin_port = 0;
  ABSL_PCHECK(
      bind(s, reinterpret_cast<sockaddr *>(&address), sizeof(address)) == 0);
  socklen_t length = sizeof(address);
  ABSL_PCHECK(getsockname(s, reinterpret_cast<sockaddr *>(&address), &length) ==
              0);
  ABSL_PCHECK(close(s) == 0);
  return ntohs(address.sin_port);
}

ConnectingSocket::ConnectingSocket() {
  sockaddr_in address = {};
  address.sin_family = AF_INET;
  address.sin_port = htons(9);
  ABSL_CHECK_EQ(inet_pton(AF_INET, "192.0.2.1", &address.sin_addr), 1);
  Start(address);
}

ConnectingSocket::ConnectingSocket(int loopback_port) {
  sockaddr_in address = {};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  address.sin_port = htons(static_cast<uint16_t>(loopback_port));
  Start(address);
}

void ConnectingSocket::Start(const sockaddr_in &address) {
  fd_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  ABSL_PCHECK(fd_ >= 0);
  SetUp(fd_);
  const int res = connect(fd_, reinterpret_cast<const sockaddr *>(&address),
                          sizeof(address));
  in_progress_ = res != 0 && errno == EINPROGRESS;
}

ConnectingSocket::~ConnectingSocket() { ABSL_PCHECK(close(fd_) == 0); }

bool ConnectingSocket::StillConnecting() const {
  // A connect that finishes, either way, makes the socket writable.
  pollfd entry = {fd_, POLLOUT, 0};
  const int res = poll(&entry, 1, 0);
  ABSL_PCHECK(res >= 0);
  return res == 0;
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

bool OutOfBandPending(FileDescriptor fd) {
  // A connected socket in select()'s exception set has unread out-of-band
  // data, on Linux and macOS both.  poll()'s POLLPRI would say so on Linux,
  // but macOS never reports urgent data that way.
  fd_set exceptional;
  FD_ZERO(&exceptional);
  FD_SET(fd, &exceptional);
  timeval immediately = {0, 0};
  const int res = select(fd + 1, nullptr, nullptr, &exceptional, &immediately);
  ABSL_PCHECK(res >= 0);
  return res == 1;
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
