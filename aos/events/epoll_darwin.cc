#include <dispatch/dispatch.h>
#include <fcntl.h>
#include <mach/mach_time.h>
#include <sys/event.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <set>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "absl/numeric/int128.h"

#include "aos/events/epoll.h"
#include "aos/mutex/mutex.h"

namespace aos::internal {

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
void ResetEPollOnFork(EPoll *epoll) { epoll->ResetOnFork(); }

namespace {

// Checks for fork() and resets all registered TimerFd and EPoll instances in
// the child process.
//
// kqueues are not inherited by child processes (see kqueue(2)). If we fork, any
// existing EPoll or TimerFd instances in the child will hold file descriptors
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
    MutexLocker lock(&mutex_);
    timer_fds_.insert(timer);
  }

  void UnregisterTimerFd(TimerFd *timer) {
    MutexLocker lock(&mutex_);
    timer_fds_.erase(timer);
  }

  void RegisterEPoll(EPoll *epoll) {
    MutexLocker lock(&mutex_);
    epolls_.insert(epoll);
  }

  void UnregisterEPoll(EPoll *epoll) {
    MutexLocker lock(&mutex_);
    epolls_.erase(epoll);
  }

 private:
  AtForkHandler() {
    pthread_atfork(&AtForkHandler::PrepareFork, &AtForkHandler::ParentFork,
                   &AtForkHandler::ChildFork);
  }

  static void PrepareFork() ABSL_NO_THREAD_SAFETY_ANALYSIS {
    // Acquire the lock in the parent process before querying.
    // This ensures that:
    // 1. No other thread holds the lock when we fork. If another thread held
    //    it, that thread would not validly exist in the child process, leaving
    //    the mutex in a permanently locked/inconsistent state (deadlock).
    // 2. We have a consistent view of the registry (no one is adding/removing
    //    TimerFds/EPolls while we copy/fork).
    ABSL_CHECK(!Instance()->mutex_.Lock());
  }

  static void ParentFork() ABSL_NO_THREAD_SAFETY_ANALYSIS {
    // Release the lock in the parent process. We are done forking.
    Instance()->mutex_.Unlock();
  }

  static void ChildFork() ABSL_NO_THREAD_SAFETY_ANALYSIS {
    // Release the lock in the child process.
    // Since we acquired it in PrepareFork (in the parent), it is copied to the
    // child in a locked state. We must unlock it here so the child's main
    // thread can use it.
    Instance()->ResetAll();
    Instance()->mutex_.Unlock();
  }

  void ResetAll() {
    for (TimerFd *timer : timer_fds_) {
      ResetTimerFdOnFork(timer);
    }
    for (EPoll *epoll : epolls_) {
      ResetEPollOnFork(epoll);
    }
  }

  Mutex mutex_;
  std::set<TimerFd *> timer_fds_;
  std::set<EPoll *> epolls_;
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

EPoll::EPoll() : epoll_fd_(kqueue()) {
  ABSL_PCHECK(epoll_fd_ > 0);

  // Create a pipe for the Quit function.  We want to use a pipe to be async
  // safe so this can be called from signal handlers.
  int pipefd[2];
  ABSL_PCHECK(pipe(pipefd) == 0);
  ABSL_PCHECK(fcntl(pipefd[0], F_SETFL, O_NONBLOCK) == 0);
  ABSL_PCHECK(fcntl(pipefd[1], F_SETFL, O_NONBLOCK) == 0);
  ABSL_PCHECK(fcntl(pipefd[0], F_SETFD, FD_CLOEXEC) == 0);
  ABSL_PCHECK(fcntl(pipefd[1], F_SETFD, FD_CLOEXEC) == 0);

  quit_epoll_fd_ = pipefd[0];
  quit_signal_fd_ = pipefd[1];
  // Read the fd when data is sent and set run_ to false.
  OnReadable(quit_epoll_fd_, [this]() {
    run_ = false;
    char buf[1];
    ABSL_PCHECK(read(quit_epoll_fd_, &buf[0], 1) == 1);
  });

  AtForkHandler::Instance()->RegisterEPoll(this);
}

EPoll::~EPoll() {
  AtForkHandler::Instance()->UnregisterEPoll(this);
  // Clean up the quit pipe and epoll fd.
  DeleteFd(quit_epoll_fd_);
  close(quit_signal_fd_);
  close(quit_epoll_fd_);
  ABSL_CHECK_EQ(fns_.size(), 0u)
      << ": Not all file descriptors were unregistered before shutting down.";
  close(epoll_fd_);
}

void EPoll::ResetOnFork() {
  close(epoll_fd_);
  epoll_fd_ = kqueue();
  ABSL_PCHECK(epoll_fd_ > 0);
  ABSL_PCHECK(fcntl(epoll_fd_, F_SETFD, FD_CLOEXEC) == 0);

  // Re-register all fds.
  for (const auto &event_data : fns_) {
    if (!event_data) {
      continue;
    }
    const uint32_t new_events = event_data->events;
    event_data->events = 0;
    DoEpollCtl(event_data.get(), new_events);
  }
}

bool EPoll::Poll(bool block) {
  for (const std::function<void()> &function : before_epoll_wait_functions_) {
    function();
  }

  struct kevent event;
  struct timespec timeout;
  timeout.tv_sec = 0;
  timeout.tv_nsec = 0;

  int num_events =
      kevent(epoll_fd_, NULL, 0, &event, 1, block ? NULL : &timeout);

  if (num_events == -1) {
    if (errno == EINTR) {
      return false;
    }
    ABSL_PCHECK(num_events != -1)
        << "kevent failed in Poll. epoll_fd: " << epoll_fd_
        << " pid: " << getpid() << ": " << std::strerror(errno);
  }

  if (num_events == 0) {
    return false;
  }

  EventData *const event_data = (EventData *)event.udata;
  uint32_t events = 0;
  if (event.filter == EVFILT_READ) {
    if (event_data->events & (kIn | kPri)) {
      events |= kIn;
    }
  }
  if (event.filter == EVFILT_WRITE) {
    if (event_data->events & kOut) {
      events |= kOut;
    }
    // If the writer side of the pipe is closed (broken pipe), EV_EOF is set.
    // This corresponds to EPOLLERR/EPOLLHUP on Linux.
    if (event.flags & EV_EOF) {
      events |= kErr;
    }
  }
  if (event.flags & EV_ERROR) {
    events |= kErr;
  }

  // If we found an EOF/Error, make sure we report it if kErr was requested,
  // regardless of which filter triggered it.
  if ((event.flags & EV_EOF) || (event.flags & EV_ERROR)) {
    if (event_data->events & kErr) {
      events |= kErr;
    }
  }

  event_data->DoCallbacks(events);
  return true;
}

void EPoll::DoEpollCtl(EventData *event_data, const uint32_t new_events) {
  const uint32_t old_events = event_data->events;
  if (old_events == new_events) {
    return;
  }
  event_data->events = new_events;

  // Handle Read
  // We enable READ if the user wants In/Pri OR if they want Error (to catch
  // EOFs).
  bool old_in = (old_events & (kIn | kPri)) || (old_events & kErr);
  bool new_in = (new_events & (kIn | kPri)) || (new_events & kErr);
  if (old_in != new_in) {
    struct kevent ev;
    if (new_in) {
      // If we only want Error, use EV_CLEAR (Edge Triggered) to avoid
      // continuous "readable" events if we aren't draining the buffer.
      int flags = EV_ADD | EV_ENABLE;
      if (!(new_events & (kIn | kPri)) && (new_events & kErr)) {
        flags |= EV_CLEAR;
      }
      EV_SET(&ev, event_data->fd, EVFILT_READ, flags, 0, 0, event_data);
    } else {
      EV_SET(&ev, event_data->fd, EVFILT_READ, EV_DELETE, 0, 0, NULL);
    }

    ABSL_PCHECK(kevent(epoll_fd_, &ev, 1, NULL, 0, NULL) != -1)
        << "kevent failed in DoEpollCtl (READ). epoll_fd: " << epoll_fd_
        << " target_fd: " << event_data->fd << " pid: " << getpid() << ": "
        << std::strerror(errno);
  }

  // Handle Write
  // We enable WRITE if user wants Out OR if they want Error (to catch broken
  // pipes).
  bool old_out = (old_events & kOut) || (old_events & kErr);
  bool new_out = (new_events & kOut) || (new_events & kErr);
  if (old_out != new_out) {
    struct kevent ev;
    if (new_out) {
      // If we only want Error, use EV_CLEAR (Edge Triggered) to avoid
      // continuous "writable" events.
      int flags = EV_ADD | EV_ENABLE;
      if (!(new_events & kOut) && (new_events & kErr)) {
        flags |= EV_CLEAR;
      }
      EV_SET(&ev, event_data->fd, EVFILT_WRITE, flags, 0, 0, event_data);
    } else {
      EV_SET(&ev, event_data->fd, EVFILT_WRITE, EV_DELETE, 0, 0, NULL);
    }

    ABSL_PCHECK(kevent(epoll_fd_, &ev, 1, NULL, 0, NULL) != -1)
        << "kevent failed in DoEpollCtl (WRITE). epoll_fd: " << epoll_fd_
        << " target_fd: " << event_data->fd << " pid: " << getpid() << ": "
        << std::strerror(errno);
  }
}

void EPoll::DeleteFdFromEpoll(int fd) {
  // Best effort delete both filters.
  struct kevent events[2];
  int n = 0;
  EV_SET(&events[n++], fd, EVFILT_READ, EV_DELETE, 0, 0, NULL);
  EV_SET(&events[n++], fd, EVFILT_WRITE, EV_DELETE, 0, 0, NULL);

  // Swallow errors.
  kevent(epoll_fd_, events, n, NULL, 0, NULL);
}

}  // namespace aos::internal
