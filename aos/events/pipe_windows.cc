#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include "aos/events/pipe.h"

// winsock2.h has to come first: windows.h otherwise pulls in the
// incompatible winsock 1 declarations, and afunix.h's sockaddr_un needs
// ADDRESS_FAMILY, which arrives via ws2def.h from here.  Kept in a block of
// its own so the include sort cannot hoist either of the two below it -- the
// alphabetical order is exactly wrong.
// clang-format off
#include <winsock2.h>

#include <afunix.h>
#include <windows.h>
// clang-format on

#include <atomic>
#include <cstring>
#include <string>
#include <string_view>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "absl/strings/str_cat.h"

#include "aos/events/winsock_init.h"

namespace aos {

namespace {

// A FileDescriptor is an opaque handle on Windows which holds the SOCKET
// directly -- no int<->SOCKET registry needed.
SOCKET ToSocket(FileDescriptor fd) { return reinterpret_cast<SOCKET>(fd); }

}  // namespace

// Windows has no pollable anonymous pipe (anonymous pipes don't support
// overlapped I/O, and named pipes aren't SOCKETs, which is what the Aio backend
// speaks), so emulate one with a connected AF_UNIX socket pair.  Unlike a
// loopback TCP pair, a unix socket moves data with a kernel buffer copy: a
// completed send is immediately visible at the other end, exactly like a pipe,
// with no in-flight window where a write has happened but the reader can't see
// it yet.
Pipe::Pipe() {
  // A Pipe can outlive every Aio, or be built before the first one, so it
  // cannot rely on Aio having started Winsock.
  EnsureWinsockInitialized();

  SOCKET listener =
      WSASocket(AF_UNIX, SOCK_STREAM, 0, nullptr, 0, WSA_FLAG_OVERLAPPED);
  ABSL_CHECK_NE(listener, INVALID_SOCKET)
      << ": Failed to create listener socket: " << WSAGetLastError();
  SetHandleInformation(reinterpret_cast<HANDLE>(listener), HANDLE_FLAG_INHERIT,
                       0);

  // AF_UNIX sockets bind to a filesystem path, so make up a unique one under
  // the temp directory.  It only exists to rendezvous the two ends; it is
  // deleted again as soon as the connection is up.
  static std::atomic<uint64_t> counter{0};
  char temp_path[MAX_PATH];
  const DWORD temp_length = GetTempPathA(sizeof(temp_path), temp_path);
  ABSL_CHECK(temp_length > 0 && temp_length < sizeof(temp_path))
      << ": GetTempPathA failed: " << GetLastError();
  const std::string socket_path =
      absl::StrCat(temp_path, "aos_pipe_", GetCurrentProcessId(), "_",
                   counter.fetch_add(1), ".sock");

  sockaddr_un addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.sun_family = AF_UNIX;
  ABSL_CHECK_LT(socket_path.size(), sizeof(addr.sun_path))
      << ": temp directory path is too long for sun_path: " << socket_path;
  std::memcpy(addr.sun_path, socket_path.c_str(), socket_path.size() + 1);

  // A stale file from a crashed run at the same path would make bind fail.
  DeleteFileA(socket_path.c_str());

  ABSL_CHECK_EQ(
      bind(listener, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)), 0)
      << ": Failed to bind " << socket_path << ": " << WSAGetLastError();
  ABSL_CHECK_EQ(listen(listener, 1), 0)
      << ": Failed to listen: " << WSAGetLastError();

  SOCKET client =
      WSASocket(AF_UNIX, SOCK_STREAM, 0, nullptr, 0, WSA_FLAG_OVERLAPPED);
  ABSL_CHECK_NE(client, INVALID_SOCKET)
      << ": Failed to create client socket: " << WSAGetLastError();
  SetHandleInformation(reinterpret_cast<HANDLE>(client), HANDLE_FLAG_INHERIT,
                       0);

  ABSL_CHECK_EQ(
      connect(client, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)), 0)
      << ": Failed to connect: " << WSAGetLastError();

  SOCKET server = accept(listener, nullptr, nullptr);
  ABSL_CHECK_NE(server, INVALID_SOCKET)
      << ": Failed to accept: " << WSAGetLastError();
  SetHandleInformation(reinterpret_cast<HANDLE>(server), HANDLE_FLAG_INHERIT,
                       0);

  closesocket(listener);
  DeleteFileA(socket_path.c_str());

  // A pipe has a small fixed capacity, and callers rely on being able to fill
  // it with a bounded number of writes.  Bound the buffering the same way.
  constexpr int kSocketBufferSize = 4096;
  setsockopt(server, SOL_SOCKET, SO_RCVBUF,
             reinterpret_cast<const char *>(&kSocketBufferSize),
             sizeof(kSocketBufferSize));
  setsockopt(client, SOL_SOCKET, SO_SNDBUF,
             reinterpret_cast<const char *>(&kSocketBufferSize),
             sizeof(kSocketBufferSize));

  // Non-blocking, like the fcntl(O_NONBLOCK) on the other platforms.
  u_long mode = 1;
  ioctlsocket(server, FIONBIO, &mode);
  ioctlsocket(client, FIONBIO, &mode);

  fds_[0] = reinterpret_cast<FileDescriptor>(server);
  fds_[1] = reinterpret_cast<FileDescriptor>(client);
}

Pipe::~Pipe() {
  if (fds_[0] != nullptr) {
    closesocket(ToSocket(fds_[0]));
  }
  if (fds_[1] != nullptr) {
    closesocket(ToSocket(fds_[1]));
  }
}

void Pipe::close_read_fd() {
  closesocket(ToSocket(fds_[0]));
  fds_[0] = nullptr;
}

void Pipe::close_write_fd() {
  closesocket(ToSocket(fds_[1]));
  fds_[1] = nullptr;
}

void Pipe::Write(std::string_view data) {
  ABSL_CHECK_EQ(
      send(ToSocket(write_fd()), data.data(), static_cast<int>(data.size()), 0),
      static_cast<int>(data.size()));
}

std::string Pipe::Read(size_t size) {
  std::string result;
  result.resize(size);
  ABSL_CHECK_EQ(
      recv(ToSocket(read_fd()), result.data(), static_cast<int>(size), 0),
      static_cast<int>(size));
  return result;
}

bool Pipe::write_ready() {
  // WSAPoll rather than select(): select()/WSAAsyncSelect() leave persistent
  // notification state armed on the socket, which conflicts with outstanding
  // overlapped I/O on the same socket and can wedge it so neither ever
  // completes.  WSAPoll is a stateless one-shot check.
  WSAPOLLFD poll_fd;
  poll_fd.fd = ToSocket(write_fd());
  poll_fd.events = POLLWRNORM;
  poll_fd.revents = 0;
  const int ret = WSAPoll(&poll_fd, 1, 0);
  return ret > 0 && (poll_fd.revents & POLLWRNORM) != 0;
}

}  // namespace aos
