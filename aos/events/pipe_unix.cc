#include "aos/events/pipe.h"

#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>

#include "absl/log/absl_check.h"

namespace aos {
namespace {

// Non-blocking and close-on-exec, like every descriptor these classes hand
// out.
void SetUp(int fd) {
  ABSL_PCHECK(fcntl(fd, F_SETFL, O_NONBLOCK) == 0);
  // Set after the fact rather than at creation, which macOS cannot do.
  ABSL_PCHECK(fcntl(fd, F_SETFD, FD_CLOEXEC) == 0);
}

bool WouldBlock() { return errno == EAGAIN || errno == EWOULDBLOCK; }

}  // namespace

Pipe::Pipe() {
  ABSL_PCHECK(pipe(fds_) == 0);
  // Close-on-exec matters here: an event loop's wakeup pipe must not leak
  // into the children a process spawns (starterd fork()+exec()s constantly).
  SetUp(fds_[0]);
  SetUp(fds_[1]);
}

Pipe::~Pipe() {
  if (fds_[0] >= 0) {
    ABSL_PCHECK(close(fds_[0]) == 0);
  }
  if (fds_[1] >= 0) {
    ABSL_PCHECK(close(fds_[1]) == 0);
  }
}

void Pipe::close_read_fd() {
  ABSL_PCHECK(close(fds_[0]) == 0);
  fds_[0] = -1;
}

void Pipe::close_write_fd() {
  ABSL_PCHECK(close(fds_[1]) == 0);
  fds_[1] = -1;
}

void Pipe::Write(std::string_view data) {
  ABSL_CHECK_EQ(write(write_fd(), data.data(), data.size()),
                static_cast<ssize_t>(data.size()));
}

std::string Pipe::Read(size_t size) {
  std::string result;
  result.resize(size);
  ABSL_CHECK_EQ(read(read_fd(), result.data(), size),
                static_cast<ssize_t>(size));
  return result;
}

bool Pipe::write_ready() {
  pollfd poll_fd;
  poll_fd.fd = write_fd();
  poll_fd.events = POLLWRNORM;
  poll_fd.revents = 0;
  const int ret = poll(&poll_fd, 1, 0);
  return ret > 0 && (poll_fd.revents & POLLWRNORM) != 0;
}

SocketPair::SocketPair() {
  ABSL_PCHECK(socketpair(AF_UNIX, SOCK_STREAM, 0, fds_) == 0);
  SetUp(fds_[0]);
  SetUp(fds_[1]);
}

SocketPair::~SocketPair() {
  ABSL_PCHECK(close(fds_[0]) == 0);
  ABSL_PCHECK(close(fds_[1]) == 0);
}

int ReadSome(FileDescriptor fd, char *buffer, size_t size) {
  return static_cast<int>(read(fd, buffer, size));
}

int WriteSome(FileDescriptor fd, const char *buffer, size_t size) {
  return static_cast<int>(write(fd, buffer, size));
}

size_t FillUntilBlocked(FileDescriptor fd) {
  const char chunk[4096] = {};
  size_t total = 0;
  int written;
  while ((written = WriteSome(fd, chunk, sizeof(chunk))) > 0) {
    total += written;
  }
  ABSL_PCHECK(WouldBlock()) << ": filling fd " << fd;
  return total;
}

size_t DrainUntilBlocked(FileDescriptor fd) {
  char chunk[4096];
  size_t total = 0;
  int got;
  while ((got = ReadSome(fd, chunk, sizeof(chunk))) > 0) {
    total += got;
  }
  ABSL_CHECK_NE(got, 0) << ": fd " << fd << " reached end of file";
  ABSL_PCHECK(WouldBlock()) << ": draining fd " << fd;
  return total;
}

}  // namespace aos
