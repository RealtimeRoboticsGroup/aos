#include <dispatch/dispatch.h>
#include <fcntl.h>
#include <sys/event.h>
#include <sys/socket.h>
#include <unistd.h>

#include <map>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "absl/synchronization/mutex.h"
#include "aos/events/epoll.h"

namespace aos::internal {

namespace {
class MacTimerFd {
 public:
  MacTimerFd() {
    fd_ = kqueue();
    ABSL_PCHECK(fd_ >= 0);
    ABSL_PCHECK(fcntl(fd_, F_SETFD, FD_CLOEXEC) == 0);
  }

  ~MacTimerFd() { close(fd_); }

  void SetTime(monotonic_clock::time_point start,
               monotonic_clock::duration interval) {
    // If we are disabling the timer.
    if (interval == monotonic_clock::zero() &&
        start == monotonic_clock::epoch()) {
      struct kevent ev;
      EV_SET(&ev, 1, EVFILT_TIMER, EV_DELETE, 0, 0, NULL);
      // Ignore errors if it didn't exist.
      kevent(fd_, &ev, 1, NULL, 0, NULL);
      return;
    }

    interval_ = interval;

    monotonic_clock::time_point now = monotonic_clock::now();
    int64_t delay_ns = 0;
    if (start > now) {
      delay_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(start -
                                                                      now)
                     .count();
      // kqueue treats 0 as "fire immediately" but sometimes minimal delay is safer?
      // NOTE_NSECONDS with 0 is fine.
    } else {
      // If start is in the past, fire immediately.
      delay_ns = 0;
    }

    // We use a one-shot timer for the initial expiration.
    // If there is an interval, we will re-arm it in Read().
    struct kevent ev;
    EV_SET(&ev, 1, EVFILT_TIMER, EV_ADD | EV_ENABLE | EV_ONESHOT, NOTE_NSECONDS,
           delay_ns, NULL);
    ABSL_PCHECK(kevent(fd_, &ev, 1, NULL, 0, NULL) == 0);
  }

  uint64_t Read() {
    struct kevent ev;
    struct timespec ts = {0, 0};
    int nev = kevent(fd_, NULL, 0, &ev, 1, &ts);
    if (nev == 0) {
      return 0;
    }
    ABSL_PCHECK(nev > 0);
    
    // Checks if we need to re-arm for an interval.
    if (interval_ > monotonic_clock::zero()) {
        int64_t interval_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(interval_).count();
        struct kevent arm_ev;
        // Re-add as a repeating timer (remove EV_ONESHOT).
        EV_SET(&arm_ev, 1, EVFILT_TIMER, EV_ADD | EV_ENABLE, NOTE_NSECONDS, interval_ns, NULL);
        ABSL_PCHECK(kevent(fd_, &arm_ev, 1, NULL, 0, NULL) == 0);
        // We set interval_ to zero so we don't keep resetting it every read? 
        // No, if it's repeating, it stays repeating.
        // But wait, if we just set it to repeating, it will fire every X ns from NOW.
        // There will be skew.
        // Ideally we wouldn't re-arm if it's already repeating.
        // But we transitioned from OneShot (initial) to Repeating (interval).
        // Since we can't easily query the state, and we want to switch modes...
        // We can check user_data or flags?
        // Simpler: Just rely on the fact that SetTime calls force a reset anyway.
        // So we only transition ONCE from initial -> repeating.
        // But Read() is called multiple times.
        // We need state to know if we are currently "in initial delay" or "in interval mode".
        // But wait, SetTime sets ONESHOT.
        // Read detects fire.
        // If we simply set REPEATING now, it will continue indefinitely.
        // So we don't need to re-arm every Read(). Just the first time.
        // But how do we distinguish first Time vs subsequent?
        // We can just set it to repeating every time if it was one-shot?
        // kevent flags return EV_ONESHOT?
        if (ev.flags & EV_ONESHOT) {
             // It was the one-shot initial timer. Switch to repeating.
             int64_t interval_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(interval_).count();
             struct kevent arm_ev;
             EV_SET(&arm_ev, 1, EVFILT_TIMER, EV_ADD | EV_ENABLE, NOTE_NSECONDS, interval_ns, NULL);
             ABSL_PCHECK(kevent(fd_, &arm_ev, 1, NULL, 0, NULL) == 0);
        }
    }
    
    return ev.data;
  }

  int fd() { return fd_; }

 private:
  int fd_;
  monotonic_clock::duration interval_ = monotonic_clock::zero();
};
}  // namespace

static absl::Mutex kTimerMapMutex;
static std::map<int, MacTimerFd *> kTimerMap ABSL_GUARDED_BY(kTimerMapMutex);

TimerFd::TimerFd() {
  MacTimerFd *timer = new MacTimerFd();
  fd_ = timer->fd();
  absl::MutexLock lock(&kTimerMapMutex);
  kTimerMap[fd_] = timer;
}

TimerFd::~TimerFd() {
  MacTimerFd *timer = nullptr;
  {
    absl::MutexLock lock(&kTimerMapMutex);
    auto it = kTimerMap.find(fd_);
    if (it != kTimerMap.end()) {
      timer = it->second;
      kTimerMap.erase(it);
    }
  }
  delete timer;
}

void TimerFd::SetTime(monotonic_clock::time_point start,
                      monotonic_clock::duration interval) {
  absl::MutexLock lock(&kTimerMapMutex);
  auto it = kTimerMap.find(fd_);
  ABSL_CHECK(it != kTimerMap.end());
  it->second->SetTime(start, interval);
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

EPoll::EPoll() : epoll_fd_(kqueue()) {
  ABSL_PCHECK(epoll_fd_ > 0);

  // Set CLOEXEC on kqueue fd.
  int flags = fcntl(epoll_fd_, F_GETFD);
  ABSL_PCHECK(flags != -1);
  ABSL_PCHECK(fcntl(epoll_fd_, F_SETFD, flags | FD_CLOEXEC) == 0);

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
}

EPoll::~EPoll() {
  // Clean up the quit pipe and epoll fd.
  DeleteFd(quit_epoll_fd_);
  close(quit_signal_fd_);
  close(quit_epoll_fd_);
  ABSL_CHECK_EQ(fns_.size(), 0u)
      << ": Not all file descriptors were unregistered before shutting down.";
  close(epoll_fd_);
}

bool EPoll::Poll(bool block) {
  for (const std::function<void()> &function : before_epoll_wait_functions_) {
    function();
  }

  struct kevent event;
  struct timespec timeout;
  timeout.tv_sec = 0;
  timeout.tv_nsec = 0;

  int num_events = kevent(epoll_fd_, NULL, 0, &event, 1,
                          block ? NULL : &timeout);

  if (num_events == -1) {
    if (errno == EINTR) {
      return false;
    }
    ABSL_PCHECK(num_events != -1);
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
  // We enable READ if the user wants In/Pri OR if they want Error (to catch EOFs).
  bool old_in = (old_events & (kIn | kPri)) || (old_events & kErr);
  bool new_in = (new_events & (kIn | kPri)) || (new_events & kErr);
  if (old_in != new_in) {
    struct kevent ev;
    if (new_in) {
      // If we only want Error, use EV_CLEAR (Edge Triggered) to avoid continuous
      // "readable" events if we aren't draining the buffer.
      int flags = EV_ADD | EV_ENABLE;
      if (!(new_events & (kIn | kPri)) && (new_events & kErr)) {
        flags |= EV_CLEAR;
      }
      EV_SET(&ev, event_data->fd, EVFILT_READ, flags, 0, 0, event_data);
    } else {
      EV_SET(&ev, event_data->fd, EVFILT_READ, EV_DELETE, 0, 0, NULL);
    }
    
    // We only strictly require success if the user *explicitly* requested READ/PRI.
    // If we are just enabling it for kErr, we tolerate failure (e.g. write-only FD).
    if (kevent(epoll_fd_, &ev, 1, NULL, 0, NULL) == -1) {
      if (new_in && (new_events & (kIn | kPri))) {
        ABSL_PCHECK(false) << ": Failed to update READ events for fd " << event_data->fd;
      }
    }
  }

  // Handle Write
  // We enable WRITE if user wants Out OR if they want Error (to catch broken pipes).
  bool old_out = (old_events & kOut) || (old_events & kErr);
  bool new_out = (new_events & kOut) || (new_events & kErr);
  if (old_out != new_out) {
    struct kevent ev;
    if (new_out) {
      // If we only want Error, use EV_CLEAR (Edge Triggered) to avoid continuous
      // "writable" events.
      int flags = EV_ADD | EV_ENABLE;
      if (!(new_events & kOut) && (new_events & kErr)) {
         flags |= EV_CLEAR;
      }
      EV_SET(&ev, event_data->fd, EVFILT_WRITE, flags, 0, 0, event_data);
    } else {
      EV_SET(&ev, event_data->fd, EVFILT_WRITE, EV_DELETE, 0, 0, NULL);
    }

    // Only strictly require success if kOut explictly requested.
    if (kevent(epoll_fd_, &ev, 1, NULL, 0, NULL) == -1) {
       if (new_out && (new_events & kOut)) {
         ABSL_PCHECK(false) << ": Failed to update WRITE events for fd " << event_data->fd;
       }
    }
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
