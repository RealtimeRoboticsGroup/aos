#ifndef AOS_IPC_LIB_LATENCY_LIB_H_
#define AOS_IPC_LIB_LATENCY_LIB_H_

#include <fcntl.h>
#include <string.h>
#include <unistd.h>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/time/time.h"

namespace aos {

void TimerThread(monotonic_clock::time_point end_time, int timer_priority);

class Tracing {
 public:
  Tracing() {
    SetContentsOrDie("/sys/kernel/debug/tracing/events/enable", "1\n");
    fd_ = open("/sys/kernel/debug/tracing/tracing_on",
               O_WRONLY | O_TRUNC | O_CLOEXEC, 0);
    ABSL_PCHECK(fd_ != -1);
  }

  ~Tracing() { close(fd_); }

  void Start() { ABSL_PCHECK(write(fd_, "1\n", 2) == 2); }

  void Stop() { ABSL_PCHECK(write(fd_, "0\n", 2) == 2); }

 private:
  void SetContentsOrDie(const char *filename, const char *data) {
    int fd = open(filename, O_WRONLY | O_TRUNC | O_CLOEXEC);
    ABSL_PCHECK(fd != -1);
    size_t len = strlen(data);
    ABSL_PCHECK(write(fd, data, len) == static_cast<int>(len));
    ABSL_PCHECK(close(fd) == 0);
  }

  int fd_;
};

}  // namespace aos

#endif  // AOS_IPC_LIB_LATENCY_LIB_H_
