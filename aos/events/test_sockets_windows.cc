#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

// winsock2.h has to come before windows.h, which otherwise pulls in the
// incompatible winsock 1 declarations.
// clang-format off
#include <winsock2.h>
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
