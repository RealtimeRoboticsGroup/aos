#ifndef AOS_EVENTS_TEST_SOCKETS_H_
#define AOS_EVENTS_TEST_SOCKETS_H_

#include "aos/events/file_descriptor.h"

// Connected sockets the Aio tests need beyond a Pipe or a SocketPair, built
// the same way on every platform.
namespace aos::testing {

// Both ends of a connected loopback TCP connection, for the tests that need
// one taken down abruptly: a reset is something only a TCP connection sends.
// Both ends are non-blocking.
class TcpPair {
 public:
  TcpPair();
  ~TcpPair();

  TcpPair(const TcpPair &) = delete;
  TcpPair &operator=(const TcpPair &) = delete;

  FileDescriptor client_fd() const { return client_; }
  FileDescriptor server_fd() const { return server_; }

  // Closes the client end abruptly: a zero linger turns the close into a
  // reset, where a plain close would be a graceful shutdown the peer sees as
  // end-of-file.
  void ResetClient();

 private:
  FileDescriptor client_ = kInvalidFd;
  FileDescriptor server_ = kInvalidFd;
};

// Whether the ReadSome() that just returned -1 failed because the peer reset
// the connection: ECONNRESET, or WSAECONNRESET on Windows.  A graceful close
// reads 0 instead.
bool LastReadWasReset();

// Two connected datagram sockets, for the tests that need message
// boundaries.  Both ends are non-blocking.  A socketpair(2) where there is
// one, and two loopback UDP sockets connected to each other on Windows, which
// has no datagram socket pair.  On Windows a datagram can take a moment to
// become readable after the send that carried it has completed, so a test
// that reads one right after sending it waits for it, bounded.
class DatagramPair {
 public:
  DatagramPair();
  ~DatagramPair();

  DatagramPair(const DatagramPair &) = delete;
  DatagramPair &operator=(const DatagramPair &) = delete;

  FileDescriptor first_fd() const { return fds_[0]; }
  FileDescriptor second_fd() const { return fds_[1]; }

 private:
  FileDescriptor fds_[2] = {kInvalidFd, kInvalidFd};
};

}  // namespace aos::testing

#endif  // AOS_EVENTS_TEST_SOCKETS_H_
