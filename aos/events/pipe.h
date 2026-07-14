#ifndef AOS_EVENTS_PIPE_H_
#define AOS_EVENTS_PIPE_H_

#include <fcntl.h>
#include <unistd.h>

#include <string>

#include "absl/log/absl_check.h"

namespace aos {

// A simple wrapper around both ends of a pipe along with some helpers to easily
// read/write data through it.
class Pipe {
 public:
  Pipe() {
    ABSL_PCHECK(pipe(fds_) == 0);
    ABSL_PCHECK(fcntl(fds_[0], F_SETFL, O_NONBLOCK) == 0);
    ABSL_PCHECK(fcntl(fds_[1], F_SETFL, O_NONBLOCK) == 0);
  }
  ~Pipe() {
    if (fds_[0] >= 0) {
      ABSL_PCHECK(close(fds_[0]) == 0);
    }
    if (fds_[1] >= 0) {
      ABSL_PCHECK(close(fds_[1]) == 0);
    }
  }

  int read_fd() { return fds_[0]; }
  int write_fd() { return fds_[1]; }
  void close_read_fd() {
    ABSL_PCHECK(close(fds_[0]) == 0);
    fds_[0] = -1;
  }
  void close_write_fd() {
    ABSL_PCHECK(close(fds_[1]) == 0);
    fds_[1] = -1;
  }

  void Write(const std::string &data) {
    ABSL_CHECK_EQ(write(write_fd(), data.data(), data.size()),
                  static_cast<ssize_t>(data.size()));
  }

  std::string Read(size_t size) {
    std::string result;
    result.resize(size);
    ABSL_CHECK_EQ(read(read_fd(), result.data(), size),
                  static_cast<ssize_t>(size));
    return result;
  }

 private:
  int fds_[2];
};

}  // namespace aos

#endif  // AOS_EVENTS_PIPE_H_
