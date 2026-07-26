#ifndef AOS_EVENTS_TIMERFD_H_
#define AOS_EVENTS_TIMERFD_H_

#include <stdint.h>

#include "aos/time/time.h"

namespace aos {
namespace internal {

// Class wrapping up timerfd.
//
// This is Linux-only: it is a thin wrapper around the timerfd_* syscalls, and
// exists for the code which still wants a timer as a pollable file descriptor.
// Prefer Aio::Timer, which is cross-platform and is what the event loops use.
class TimerFd {
 public:
  TimerFd();
  ~TimerFd();

  TimerFd(const TimerFd &) = delete;
  TimerFd &operator=(const TimerFd &) = delete;
  TimerFd(TimerFd &&) = delete;
  TimerFd &operator=(TimerFd &&) = delete;

  // Sets the trigger time and repeat for the timerfd.
  // An interval of 0 results in a single expiration.
  void SetTime(monotonic_clock::time_point start,
               monotonic_clock::duration interval);

  // Disarms the timer.
  void Disable() {
    // Disarm the timer by feeding zero values
    SetTime(monotonic_clock::epoch(), monotonic_clock::zero());
  }

  // Reads the event.  Returns the number of elapsed cycles.
  uint64_t Read();

  // Returns the file descriptor associated with the timerfd.
  int fd() { return fd_; }

 private:
  int fd_ = -1;
};

}  // namespace internal
}  // namespace aos

#endif  // AOS_EVENTS_TIMERFD_H_
