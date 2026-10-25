#ifndef AOS_EVENTS_PIPE_H_
#define AOS_EVENTS_PIPE_H_

#include <cstddef>
#include <string>
#include <string_view>

#include "aos/events/file_descriptor.h"

namespace aos {

// A simple wrapper around both ends of a pipe along with some helpers to easily
// read/write data through it.
//
// Both ends are non-blocking.  On Windows, where there is no pollable pipe, the
// two ends are a connected AF_UNIX socket pair with pipe-sized buffers; see
// pipe_windows.cc.
class Pipe {
 public:
  Pipe();
  ~Pipe();

  Pipe(const Pipe &) = delete;
  Pipe &operator=(const Pipe &) = delete;

  FileDescriptor read_fd() const { return fds_[0]; }
  FileDescriptor write_fd() const { return fds_[1]; }
  void close_read_fd();
  void close_write_fd();

  // Writes all of data, checking that none of it was dropped.
  void Write(std::string_view data);

  // Reads exactly size bytes.
  std::string Read(size_t size);

  // Returns whether the write end could accept data right now, without
  // blocking, writing, or leaving any notification state armed on the fd.
  bool write_ready();

 private:
  FileDescriptor fds_[2];
};

// Both ends of a connected stream socket pair.  Unlike a Pipe, each end both
// reads and writes.  Both ends are non-blocking.  On Windows the buffers are
// pipe-sized, as a Pipe's are, so filling one takes a bounded number of
// writes.  The pair owns both ends for its whole lifetime and closes them when
// it goes, so a caller must not close either one itself.
class SocketPair {
 public:
  SocketPair();
  ~SocketPair();

  SocketPair(const SocketPair &) = delete;
  SocketPair &operator=(const SocketPair &) = delete;

  FileDescriptor first_fd() const { return fds_[0]; }
  FileDescriptor second_fd() const { return fds_[1]; }

 private:
  FileDescriptor fds_[2];
};

// Reads what a non-blocking pipe or socket end has right now: the byte count,
// 0 at end of file, or -1 if nothing could be read, including because nothing
// is there yet.
int ReadSome(FileDescriptor fd, char *buffer, size_t size);

// Writes what fits in a non-blocking pipe or socket end right now: the byte
// count, or -1 if nothing could be written, including because nothing fits.
int WriteSome(FileDescriptor fd, const char *buffer, size_t size);

// Writes into a non-blocking end until nothing more fits, and returns how
// much went in.  Dies if it stops for any other reason.
size_t FillUntilBlocked(FileDescriptor fd);

// Reads from a non-blocking end until nothing is left, and returns how much
// came out.  Dies at end of file, or if it stops for any other reason.
size_t DrainUntilBlocked(FileDescriptor fd);

}  // namespace aos

#endif  // AOS_EVENTS_PIPE_H_
