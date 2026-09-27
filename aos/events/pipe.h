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
// two ends are a connected loopback socket pair with pipe-sized buffers; see
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

}  // namespace aos

#endif  // AOS_EVENTS_PIPE_H_
