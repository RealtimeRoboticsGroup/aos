#include <dispatch/dispatch.h>
#include <fcntl.h>
#include <mach/mach_time.h>
#include <sys/event.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <mutex>
#include <set>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "absl/numeric/int128.h"

#include "aos/events/epoll.h"

namespace aos {
namespace internal {
namespace {

uint64_t ToMachTicks(monotonic_clock::time_point time) {
  static mach_timebase_info_data_t timebase_info = []() {
    mach_timebase_info_data_t info;
    mach_timebase_info(&info);
    return info;
  }();

  // Convert aos::monotonic_clock to total nanoseconds
  uint64_t nanos = std::chrono::nanoseconds(time.time_since_epoch()).count();

  // Convert nanoseconds to Mach absolute time units (ticks)
  // Formula: ticks = (nanos * denom) / numer
  // Using absl::uint128 to ensure no overflow during the multiplication
  return static_cast<uint64_t>((absl::uint128(nanos) * timebase_info.denom) /
                               timebase_info.numer);
}

}  // namespace

void ResetTimerFdOnFork(TimerFd *timer) { timer->ResetOnFork(); }

}  // namespace internal

namespace internal {
namespace {

// Checks for fork() and resets all registered TimerFd instances in
// the child process.
//
// kqueues are not inherited by child processes (see kqueue(2)). If we fork, any
// existing TimerFd instances in the child will hold file descriptors
// that refer to nothing (or worse, are closed).
//
// This ensures that if a fork happens (e.g. in a death test), the child process
// gets fresh kqueues and can continue to function (or crash cleanly).  This
// matches Linux's overall behavior of continuing to work in the child.
class AtForkHandler {
 public:
  static AtForkHandler *Instance() {
    static AtForkHandler *instance = new AtForkHandler();
    return instance;
  }

  void RegisterTimerFd(TimerFd *timer) {
    std::lock_guard<std::mutex> lock(mutex_);
    timer_fds_.insert(timer);
  }

  void UnregisterTimerFd(TimerFd *timer) {
    std::lock_guard<std::mutex> lock(mutex_);
    timer_fds_.erase(timer);
  }

 private:
  AtForkHandler() {
    pthread_atfork(&AtForkHandler::PrepareFork, &AtForkHandler::ParentFork,
                   &AtForkHandler::ChildFork);
  }

  static void PrepareFork() ABSL_NO_THREAD_SAFETY_ANALYSIS {
    Instance()->mutex_.lock();
  }

  static void ParentFork() ABSL_NO_THREAD_SAFETY_ANALYSIS {
    Instance()->mutex_.unlock();
  }

  static void ChildFork() ABSL_NO_THREAD_SAFETY_ANALYSIS {
    Instance()->ResetAll();
    Instance()->mutex_.unlock();
  }

  void ResetAll() {
    for (TimerFd *timer : timer_fds_) {
      ResetTimerFdOnFork(timer);
    }
  }

  std::mutex mutex_;
  std::set<TimerFd *> timer_fds_;
};

}  // namespace

TimerFd::TimerFd() {
  fd_ = kqueue();
  ABSL_PCHECK(fd_ >= 0);
  ABSL_PCHECK(fcntl(fd_, F_SETFD, FD_CLOEXEC) == 0);
  AtForkHandler::Instance()->RegisterTimerFd(this);
}

TimerFd::~TimerFd() {
  AtForkHandler::Instance()->UnregisterTimerFd(this);
  if (fd_ != -1) {
    close(fd_);
  }
}

void TimerFd::ResetOnFork() {
  int new_fd = kqueue();
  ABSL_PCHECK(new_fd >= 0);
  ABSL_PCHECK(fcntl(new_fd, F_SETFD, FD_CLOEXEC) == 0);

  // We need to preserve the same file descriptor number because EPoll instances
  // (and other consumers) may have registered this TimerFd using its original
  // fd. If we just assigned a new fd, the EPoll would still be trying to poll
  // the old (now closed or reused) fd, leading to "Bad file descriptor" errors
  // during EPoll::ResetOnFork or subsequent Poll calls.
  //
  // Use dup2 to atomically close the old fd_ (if open) and replace it with
  // new_fd, ensuring fd_ now refers to the new kqueue.
  //
  // https://man7.org/linux/man-pages/man2/dup.2.html
  // "dup2() makes newfd be the copy of oldfd, closing newfd first if
  // necessary... The steps of closing and reusing the file descriptor newfd are
  // performed atomically."
  //
  // Note that kqueues are *not* inherited across fork(), so we must create a
  // new one.
  // https://man.freebsd.org/cgi/man.cgi?query=kqueue&sektion=2
  // "The kqueue() system call creates a new kernel event queue and returns a
  // descriptor... The queue is not inherited by a child created with fork(2)."
  if (dup2(new_fd, fd_) == -1) {
    ABSL_PLOG(FATAL) << "dup2 failed";
  }
  close(new_fd);

  // If a timer was active, we must re-arm it in the new kqueue.
  if (next_expiration_ > monotonic_clock::min_time) {
    SetTime(next_expiration_, interval_);
  }
}

void TimerFd::SetTime(monotonic_clock::time_point start,
                      monotonic_clock::duration interval) {
  // If we are disabling the timer.
  if (interval == monotonic_clock::zero() &&
      start == monotonic_clock::epoch()) {
    struct kevent ev;
    EV_SET(&ev, 1, EVFILT_TIMER, EV_DELETE, 0, 0, NULL);
    // Ignore errors if it didn't exist.
    kevent(fd_, &ev, 1, NULL, 0, NULL);
    next_expiration_ = monotonic_clock::min_time;
    return;
  }

  interval_ = interval;
  next_expiration_ = start;

  // We use the absolute mach time for the expiration.
  struct kevent ev;
  uint64_t mach_ticks = ToMachTicks(next_expiration_);
  EV_SET(&ev, 1, EVFILT_TIMER, EV_ADD | EV_ENABLE | EV_ONESHOT,
         NOTE_MACHTIME | NOTE_ABSOLUTE, mach_ticks, NULL);
  ABSL_PCHECK(kevent(fd_, &ev, 1, NULL, 0, NULL) == 0);
}

uint64_t TimerFd::Read() {
  struct kevent ev;
  struct timespec ts = {0, 0};
  int nev = kevent(fd_, NULL, 0, &ev, 1, &ts);
  if (nev == 0) {
    return 0;
  }
  ABSL_PCHECK(nev > 0);

  uint64_t expirations = 0;
  monotonic_clock::time_point now = monotonic_clock::now();

  // If next_expiration_ is valid, we check against it.
  if (next_expiration_ > monotonic_clock::min_time) {
    // If the timer fired (nev > 0), passing the deadline is expected.
    if (now >= next_expiration_) {
      expirations = 1;
      if (interval_ > monotonic_clock::zero()) {
        // Calculate how many intervals have passed.
        int64_t delta_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                               now - next_expiration_)
                               .count();
        int64_t interval_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(interval_)
                .count();

        uint64_t missed = delta_ns / interval_ns;
        expirations += missed;
        next_expiration_ += (missed + 1) * interval_;

        // Re-arm for the next expiration.
        struct kevent arm_ev;
        uint64_t mach_ticks = ToMachTicks(next_expiration_);
        EV_SET(&arm_ev, 1, EVFILT_TIMER, EV_ADD | EV_ENABLE | EV_ONESHOT,
               NOTE_MACHTIME | NOTE_ABSOLUTE, mach_ticks, NULL);
        ABSL_PCHECK(kevent(fd_, &arm_ev, 1, NULL, 0, NULL) == 0);
      } else {
        // One-shot timer expired.
        next_expiration_ = monotonic_clock::min_time;
      }
    }
  }

  // If kqueue reported readiness but we computed no expiration, the timer state
  // is inconsistent.
  if (expirations == 0 && nev > 0) {
    ABSL_LOG(FATAL) << "TimerFd woke up but no expiration calculated. now: "
                    << now << " next: " << next_expiration_;
  }

  return expirations;
}

}  // namespace internal

}  // namespace aos
