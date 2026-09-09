#ifndef AOS_EVENTS_TEST_SOCKETS_H_
#define AOS_EVENTS_TEST_SOCKETS_H_

#include "aos/events/file_descriptor.h"

// Each platform's socket headers define it; the header needs only the name.
struct sockaddr_in;

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
  // Shuts down the client end's writing, so the peer sees its FIN: end of
  // file, with the connection still open the other way.
  void ShutdownClient();
  // Closes the server end, as a caller does before ForgetClosedFd().
  void CloseServer();

 private:
  FileDescriptor client_ = kInvalidFd;
  FileDescriptor server_ = kInvalidFd;
};

// A listening loopback TCP socket, for the tests that watch one for incoming
// connections.  Non-blocking, as is every socket it hands out; it closes all
// of them when it goes.
class TcpListener {
 public:
  TcpListener();
  ~TcpListener();
  TcpListener(const TcpListener &) = delete;
  TcpListener &operator=(const TcpListener &) = delete;

  FileDescriptor fd() const { return listener_; }
  // The loopback port it listens on.
  int port() const;

  // Connects a new client to the listener, which then has a connection
  // waiting to be accepted.  Returns the client's end.
  FileDescriptor Connect();
  // Accepts a waiting connection, or returns kInvalidFd if there is none.
  FileDescriptor Accept();

  // Closes fd, one of the sockets Connect() or Accept() handed out, abruptly,
  // as TcpPair::ResetClient() does, so its peer sees a reset.
  void Reset(FileDescriptor fd);

 private:
  FileDescriptor listener_ = kInvalidFd;
  // Everything Connect() and Accept() handed out, closed with the listener.
  FileDescriptor others_[8];
  int other_count_ = 0;
};

// A non-blocking TCP socket whose connect() has not finished, for the tests
// that watch one the way a caller waits for its connect.  It connects to
// 192.0.2.1 (TEST-NET-1, RFC 5737), which nothing answers, so the connect
// stays in progress until the stack gives up, which takes far longer than a
// test.  Where there is no route there either, the connect fails at once and
// in_progress() says so.
class ConnectingSocket {
 public:
  ConnectingSocket();
  // Connects to this port on the loopback address instead, where the connect
  // finishes, one way or the other, soon after it starts.
  explicit ConnectingSocket(int loopback_port);
  ~ConnectingSocket();

  ConnectingSocket(const ConnectingSocket &) = delete;
  ConnectingSocket &operator=(const ConnectingSocket &) = delete;

  FileDescriptor fd() const { return fd_; }

  // Whether connect() left the connection in progress, rather than failing.
  bool in_progress() const { return in_progress_; }
  // Whether it is still in progress now: neither connected nor failed.
  bool StillConnecting() const;

 private:
  void Start(const sockaddr_in &address);

  FileDescriptor fd_ = kInvalidFd;
  bool in_progress_ = false;
};

// A loopback port nothing listens on, for a connect that is refused.  Bound
// and then given up, so it is free, though nothing stops another process from
// taking it.
int UnusedLoopbackPort();

// Sends one byte of out-of-band (TCP urgent) data on a connected stream
// socket, for the tests of kPriorityEvent.
void SendOutOfBand(FileDescriptor fd, char byte);
// Receives the out-of-band byte waiting on fd.  Returns false if there is
// none.
bool ReceiveOutOfBand(FileDescriptor fd, char *byte);
// Whether an out-of-band byte is waiting on fd, a connected stream socket.
// Asks the kernel directly, outside of any Aio.
bool OutOfBandPending(FileDescriptor fd);

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
