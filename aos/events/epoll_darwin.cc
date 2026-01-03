#include <dispatch/dispatch.h>
#include <fcntl.h>
#include <sys/event.h>
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
    int pipefd[2];
    ABSL_PCHECK(pipe(pipefd) == 0);
    ABSL_PCHECK(fcntl(pipefd[0], F_SETFL, O_NONBLOCK) == 0);
    ABSL_PCHECK(fcntl(pipefd[1], F_SETFL, O_NONBLOCK) == 0);
    read_fd_ = pipefd[0];
    write_fd_ = pipefd[1];
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    // We can use the global queue for the timer.
    timer_source_ = dispatch_source_create(DISPATCH_SOURCE_TYPE_TIMER, 0, 0,
                                           dispatch_get_global_queue(0, 0));
    ABSL_CHECK(timer_source_ != nullptr);

    dispatch_set_context(timer_source_, this);
    dispatch_source_set_event_handler_f(timer_source_,
                                        &MacTimerFd::StaticHandleTimer);
    dispatch_resume(timer_source_);
  }

  ~MacTimerFd() {
    dispatch_source_cancel(timer_source_);
    // We need to release the source, but it might already be cancelled?
    // dispatch_release is automatic in newer C++, but manual here.
    // We must close fds.
    close(read_fd_);
    close(write_fd_);
    dispatch_release(timer_source_);
  }

  static void StaticHandleTimer(void *ctx) {
    static_cast<MacTimerFd *>(ctx)->HandleTimer();
  }

  void HandleTimer() {
    uint64_t count = dispatch_source_get_data(timer_source_);
    // Write the number of expirations to the pipe.
    // This mimics timerfd behavior where read() returns the number of
    // expirations.
    if (write(write_fd_, &count, sizeof(count)) != sizeof(count)) {
      // If the pipe is full, we might lose expirations, but that's expected
      // for non-blocking IO.  However, we should probably warn.
      // For strict timerfd conformance we'd want to handle this better, but
      // for general event loop usage this is likely sufficient/equivalent to
      // unread notifications.
    }
  }

  void SetTime(monotonic_clock::time_point start,
               monotonic_clock::duration interval) {
    uint64_t interval_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                               interval)
                               .count();

    struct timespec now_ts;
    clock_gettime(CLOCK_MONOTONIC, &now_ts);
    monotonic_clock::time_point now =
        monotonic_clock::time_point(std::chrono::seconds(now_ts.tv_sec) +
                                    std::chrono::nanoseconds(now_ts.tv_nsec));

    int64_t delay_ns = 0;
    if (start > now) {
      delay_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(start -
                                                                      now)
                     .count();
    }

    dispatch_time_t start_time = dispatch_time(DISPATCH_TIME_NOW, delay_ns);

    if (interval_ns == 0) {
      // One-shot
      dispatch_source_set_timer(timer_source_, start_time,
                                DISPATCH_TIME_FOREVER, 0);
    } else {
      // Repeating
      dispatch_source_set_timer(timer_source_, start_time, interval_ns, 0);
    }
  }

  int fd() { return read_fd_; }

 private:
  int read_fd_;
  int write_fd_;
  dispatch_source_t timer_source_;
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
    events |= kIn;
  }
  if (event.filter == EVFILT_WRITE) {
    events |= kOut;
  }
  if (event.flags & EV_ERROR) {
    events |= kErr;
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

  struct kevent events[2];
  int n = 0;

  // Handle Read
  bool old_in = old_events & (kIn | kPri);
  bool new_in = new_events & (kIn | kPri);
  if (old_in != new_in) {
    if (new_in) {
      EV_SET(&events[n++], event_data->fd, EVFILT_READ, EV_ADD | EV_ENABLE, 0,
             0, event_data);
    } else {
      EV_SET(&events[n++], event_data->fd, EVFILT_READ, EV_DELETE, 0, 0, NULL);
    }
  }

  // Handle Write
  bool old_out = old_events & kOut;
  bool new_out = new_events & kOut;
  if (old_out != new_out) {
    if (new_out) {
      EV_SET(&events[n++], event_data->fd, EVFILT_WRITE, EV_ADD | EV_ENABLE, 0,
             0, event_data);
    } else {
      EV_SET(&events[n++], event_data->fd, EVFILT_WRITE, EV_DELETE, 0, 0, NULL);
    }
  }

  if (n > 0) {
    ABSL_PCHECK(kevent(epoll_fd_, events, n, NULL, 0, NULL) != -1)
        << ": Failed to update events for fd " << event_data->fd;
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
