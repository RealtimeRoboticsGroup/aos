#include <fcntl.h>
#include <sys/timerfd.h>
#include <unistd.h>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/events/epoll.h"

namespace aos {
namespace internal {

TimerFd::TimerFd()
    : fd_(timerfd_create(CLOCK_MONOTONIC, TFD_CLOEXEC | TFD_NONBLOCK)) {
  ABSL_PCHECK(fd_ != -1);
  Disable();
}

TimerFd::~TimerFd() { ABSL_PCHECK(close(fd_) == 0); }

void TimerFd::SetTime(monotonic_clock::time_point start,
                      monotonic_clock::duration interval) {
  ABSL_CHECK_GE(start, monotonic_clock::epoch());
  struct itimerspec new_value;
  new_value.it_interval = ::aos::time::to_timespec(interval);
  new_value.it_value = ::aos::time::to_timespec(start);
  ABSL_PCHECK(timerfd_settime(fd_, TFD_TIMER_ABSTIME, &new_value, nullptr) ==
              0);
}

uint64_t TimerFd::Read() {
  uint64_t buf;
  ssize_t result = read(fd_, &buf, sizeof(buf));
  if (result == -1) {
    if (errno == EAGAIN) {
      return 0;
    }
  }
  ABSL_PCHECK(result != -1);
  ABSL_CHECK_EQ(result, static_cast<int>(sizeof(buf)));

  return buf;
}

}  // namespace internal
}  // namespace aos
