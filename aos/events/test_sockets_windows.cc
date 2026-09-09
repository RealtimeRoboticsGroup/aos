#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

// winsock2.h has to come before windows.h, which otherwise pulls in the
// incompatible winsock 1 declarations.
// clang-format off
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
// clang-format on

#include "aos/events/test_sockets.h"

#include "absl/log/absl_check.h"

#include "aos/events/winsock_init.h"

namespace aos::testing {
namespace {

SOCKET ToSocket(FileDescriptor fd) { return reinterpret_cast<SOCKET>(fd); }
FileDescriptor ToFd(SOCKET s) { return reinterpret_cast<FileDescriptor>(s); }

// Overlapped, so the IOCP backend can use it, and non-blocking, like every
// socket the tests hand to an Aio.
SOCKET MakeSocket(int type, int protocol) {
  const SOCKET s =
      WSASocket(AF_INET, type, protocol, nullptr, 0, WSA_FLAG_OVERLAPPED);
  ABSL_CHECK_NE(s, INVALID_SOCKET)
      << ": WSASocket failed: " << WSAGetLastError();
  return s;
}

// Non-blocking and not inherited, like every socket a Pipe hands out.
void SetUp(SOCKET s) {
  u_long non_blocking = 1;
  ABSL_CHECK_EQ(ioctlsocket(s, FIONBIO, &non_blocking), 0)
      << ": ioctlsocket(FIONBIO) failed: " << WSAGetLastError();
  ABSL_CHECK(
      SetHandleInformation(reinterpret_cast<HANDLE>(s), HANDLE_FLAG_INHERIT, 0))
      << ": SetHandleInformation failed: " << GetLastError();
}

// Binds s to an ephemeral loopback port, and returns the address it got.
sockaddr_in BindLoopback(SOCKET s) {
  sockaddr_in address = {};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  address.sin_port = 0;
  ABSL_CHECK_EQ(
      bind(s, reinterpret_cast<sockaddr *>(&address), sizeof(address)), 0)
      << ": bind failed: " << WSAGetLastError();
  int length = sizeof(address);
  ABSL_CHECK_EQ(getsockname(s, reinterpret_cast<sockaddr *>(&address), &length),
                0)
      << ": getsockname failed: " << WSAGetLastError();
  return address;
}

}  // namespace

TcpPair::TcpPair() {
  EnsureWinsockInitialized();
  const SOCKET listener = MakeSocket(SOCK_STREAM, IPPROTO_TCP);
  sockaddr_in address = BindLoopback(listener);
  ABSL_CHECK_EQ(listen(listener, 1), 0)
      << ": listen failed: " << WSAGetLastError();

  // Still blocking here, so the connect and the accept can just be waited on;
  // both ends go non-blocking once the connection is up.
  const SOCKET client = MakeSocket(SOCK_STREAM, IPPROTO_TCP);
  ABSL_CHECK_EQ(
      connect(client, reinterpret_cast<sockaddr *>(&address), sizeof(address)),
      0)
      << ": connect failed: " << WSAGetLastError();
  const SOCKET server = accept(listener, nullptr, nullptr);
  ABSL_CHECK_NE(server, INVALID_SOCKET)
      << ": accept failed: " << WSAGetLastError();
  ABSL_CHECK_EQ(closesocket(listener), 0);
  SetUp(client);
  SetUp(server);
  client_ = ToFd(client);
  server_ = ToFd(server);
}

TcpPair::~TcpPair() {
  if (client_ != kInvalidFd) ABSL_CHECK_EQ(closesocket(ToSocket(client_)), 0);
  if (server_ != kInvalidFd) ABSL_CHECK_EQ(closesocket(ToSocket(server_)), 0);
}

void TcpPair::ResetClient() {
  const struct linger abortive = {1, 0};
  ABSL_CHECK_EQ(
      setsockopt(ToSocket(client_), SOL_SOCKET, SO_LINGER,
                 reinterpret_cast<const char *>(&abortive), sizeof(abortive)),
      0)
      << ": setsockopt(SO_LINGER) failed: " << WSAGetLastError();
  ABSL_CHECK_EQ(closesocket(ToSocket(client_)), 0);
  client_ = kInvalidFd;
}

void TcpPair::ShutdownClient() {
  ABSL_CHECK_EQ(shutdown(ToSocket(client_), SD_SEND), 0)
      << ": shutdown(SD_SEND) failed: " << WSAGetLastError();
}

void TcpPair::CloseServer() {
  ABSL_CHECK_EQ(closesocket(ToSocket(server_)), 0);
  server_ = kInvalidFd;
}

TcpListener::TcpListener() {
  EnsureWinsockInitialized();
  const SOCKET listener = MakeSocket(SOCK_STREAM, IPPROTO_TCP);
  BindLoopback(listener);
  ABSL_CHECK_EQ(listen(listener, 4), 0)
      << ": listen failed: " << WSAGetLastError();
  SetUp(listener);
  listener_ = ToFd(listener);
}

TcpListener::~TcpListener() {
  for (int i = 0; i < other_count_; ++i) {
    ABSL_CHECK_EQ(closesocket(ToSocket(others_[i])), 0);
  }
  ABSL_CHECK_EQ(closesocket(ToSocket(listener_)), 0);
}

FileDescriptor TcpListener::Connect() {
  ABSL_CHECK_LT(other_count_, 8);
  sockaddr_in address = {};
  int length = sizeof(address);
  ABSL_CHECK_EQ(getsockname(ToSocket(listener_),
                            reinterpret_cast<sockaddr *>(&address), &length),
                0)
      << ": getsockname failed: " << WSAGetLastError();
  // Blocking for the connect, which a loopback listener with room in its
  // backlog completes before accept().
  const SOCKET client = MakeSocket(SOCK_STREAM, IPPROTO_TCP);
  ABSL_CHECK_EQ(
      connect(client, reinterpret_cast<sockaddr *>(&address), sizeof(address)),
      0)
      << ": connect failed: " << WSAGetLastError();
  SetUp(client);
  others_[other_count_++] = ToFd(client);
  return ToFd(client);
}

FileDescriptor TcpListener::Accept() {
  const SOCKET accepted = accept(ToSocket(listener_), nullptr, nullptr);
  if (accepted == INVALID_SOCKET) {
    ABSL_CHECK_EQ(WSAGetLastError(), WSAEWOULDBLOCK)
        << ": accept failed: " << WSAGetLastError();
    return kInvalidFd;
  }
  ABSL_CHECK_LT(other_count_, 8);
  SetUp(accepted);
  others_[other_count_++] = ToFd(accepted);
  return ToFd(accepted);
}

void TcpListener::Reset(FileDescriptor fd) {
  int i = 0;
  while (i < other_count_ && others_[i] != fd) ++i;
  ABSL_CHECK_LT(i, other_count_)
      << ": fd " << fd << " is not one of this listener's";
  const struct linger abortive = {1, 0};
  ABSL_CHECK_EQ(
      setsockopt(ToSocket(fd), SOL_SOCKET, SO_LINGER,
                 reinterpret_cast<const char *>(&abortive), sizeof(abortive)),
      0)
      << ": setsockopt(SO_LINGER) failed: " << WSAGetLastError();
  ABSL_CHECK_EQ(closesocket(ToSocket(fd)), 0);
  others_[i] = others_[--other_count_];
}

int TcpListener::port() const {
  sockaddr_in address = {};
  int length = sizeof(address);
  ABSL_CHECK_EQ(getsockname(ToSocket(listener_),
                            reinterpret_cast<sockaddr *>(&address), &length),
                0)
      << ": getsockname failed: " << WSAGetLastError();
  return ntohs(address.sin_port);
}

int UnusedLoopbackPort() {
  EnsureWinsockInitialized();
  const SOCKET s = MakeSocket(SOCK_STREAM, IPPROTO_TCP);
  const sockaddr_in address = BindLoopback(s);
  ABSL_CHECK_EQ(closesocket(s), 0);
  return ntohs(address.sin_port);
}

ConnectingSocket::ConnectingSocket() {
  EnsureWinsockInitialized();
  sockaddr_in address = {};
  address.sin_family = AF_INET;
  address.sin_port = htons(9);
  ABSL_CHECK_EQ(inet_pton(AF_INET, "192.0.2.1", &address.sin_addr), 1);
  Start(address);
}

ConnectingSocket::ConnectingSocket(int loopback_port) {
  EnsureWinsockInitialized();
  sockaddr_in address = {};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  address.sin_port = htons(static_cast<u_short>(loopback_port));
  Start(address);
}

void ConnectingSocket::Start(const sockaddr_in &address) {
  const SOCKET s = MakeSocket(SOCK_STREAM, IPPROTO_TCP);
  SetUp(s);
  const int res =
      connect(s, reinterpret_cast<const sockaddr *>(&address), sizeof(address));
  in_progress_ = res != 0 && WSAGetLastError() == WSAEWOULDBLOCK;
  fd_ = ToFd(s);
}

ConnectingSocket::~ConnectingSocket() {
  ABSL_CHECK_EQ(closesocket(ToSocket(fd_)), 0);
}

bool ConnectingSocket::StillConnecting() const {
  // A connect that succeeds makes the socket writable, and one that fails
  // puts it in the exception set.
  fd_set writable;
  fd_set failed;
  FD_ZERO(&writable);
  FD_ZERO(&failed);
  FD_SET(ToSocket(fd_), &writable);
  FD_SET(ToSocket(fd_), &failed);
  TIMEVAL immediately = {0, 0};
  const int res = select(0, nullptr, &writable, &failed, &immediately);
  ABSL_CHECK_GE(res, 0) << ": select failed: " << WSAGetLastError();
  return res == 0;
}

void SendOutOfBand(FileDescriptor fd, char byte) {
  ABSL_CHECK_EQ(send(ToSocket(fd), &byte, 1, MSG_OOB), 1)
      << ": send(MSG_OOB) failed: " << WSAGetLastError();
}

bool ReceiveOutOfBand(FileDescriptor fd, char *byte) {
  const int res = recv(ToSocket(fd), byte, 1, MSG_OOB);
  if (res == 1) {
    return true;
  }
  // 0 after the peer's graceful close, and WSAECONNRESET after its reset,
  // where POSIX gives EINVAL.
  const int error = res == 0 ? 0 : WSAGetLastError();
  ABSL_CHECK(res == 0 || error == WSAEWOULDBLOCK || error == WSAEINVAL ||
             error == WSAECONNRESET)
      << ": recv(MSG_OOB) failed: " << error;
  return false;
}

bool OutOfBandPending(FileDescriptor fd) {
  // A connected socket in select()'s exception set has unread out-of-band
  // data; nothing else puts it there.
  fd_set exceptional;
  FD_ZERO(&exceptional);
  FD_SET(ToSocket(fd), &exceptional);
  TIMEVAL immediately = {0, 0};
  return select(0, nullptr, nullptr, &exceptional, &immediately) == 1;
}

bool LastReadWasReset() { return WSAGetLastError() == WSAECONNRESET; }

DatagramPair::DatagramPair() {
  EnsureWinsockInitialized();
  const SOCKET first = MakeSocket(SOCK_DGRAM, IPPROTO_UDP);
  const SOCKET second = MakeSocket(SOCK_DGRAM, IPPROTO_UDP);
  sockaddr_in first_address = BindLoopback(first);
  sockaddr_in second_address = BindLoopback(second);
  // Connected both ways, so each end sends to and hears only the other, as a
  // socketpair(2)'s ends do.
  ABSL_CHECK_EQ(connect(first, reinterpret_cast<sockaddr *>(&second_address),
                        sizeof(second_address)),
                0)
      << ": connect failed: " << WSAGetLastError();
  ABSL_CHECK_EQ(connect(second, reinterpret_cast<sockaddr *>(&first_address),
                        sizeof(first_address)),
                0)
      << ": connect failed: " << WSAGetLastError();
  SetUp(first);
  SetUp(second);
  fds_[0] = ToFd(first);
  fds_[1] = ToFd(second);
}

DatagramPair::~DatagramPair() {
  ABSL_CHECK_EQ(closesocket(ToSocket(fds_[0])), 0);
  ABSL_CHECK_EQ(closesocket(ToSocket(fds_[1])), 0);
}

}  // namespace aos::testing
