#include <errno.h>
#include <fcntl.h>
#include <liburing.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <unistd.h>

#include "aos/events/aio.h"

// Fallback definitions for older liburing / kernel headers.
#ifndef IORING_SETUP_COOP_TASKRUN
#define IORING_SETUP_COOP_TASKRUN (1U << 8)
#endif
#ifndef IORING_SETUP_TASKRUN_FLAG
#define IORING_SETUP_TASKRUN_FLAG (1U << 9)
#endif
#ifndef IORING_SETUP_SINGLE_ISSUER
#define IORING_SETUP_SINGLE_ISSUER (1U << 12)
#endif

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <unordered_map>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/ipc_lib/signalfd.h"
#include "aos/libc/aos_strerror.h"
#include "aos/time/time.h"

ABSL_FLAG(uint32_t, aio_queue_depth, 1024,
          "Depth of the io_uring submission and completion queues.");

namespace aos {
namespace {

// RAII wrapper around eventfd file descriptors.
class EventFD {
 public:
  EventFD() {
    fd_ = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
    ABSL_PCHECK(fd_ >= 0) << "Failed to create eventfd";
  }

  ~EventFD() {
    if (fd_ >= 0) {
      close(fd_);
    }
  }

  // Disable copy construction and assignment to prevent duplicate ownership of
  // the descriptor.
  EventFD(const EventFD &) = delete;
  EventFD &operator=(const EventFD &) = delete;

  EventFD(EventFD &&other) noexcept
      : eventfd_buf(other.eventfd_buf),
        wakeup_req(std::move(other.wakeup_req)),
        fd_(other.fd_) {
    other.fd_ = -1;
  }

  EventFD &operator=(EventFD &&other) noexcept {
    if (this != &other) {
      if (fd_ >= 0) {
        close(fd_);
      }
      fd_ = other.fd_;
      eventfd_buf = other.eventfd_buf;
      wakeup_req = std::move(other.wakeup_req);
      other.fd_ = -1;
    }
    return *this;
  }

  int fd() const { return fd_; }

  // Writes to the eventfd to wake it up.
  void Write() {
    uint64_t val = 1;
    ssize_t res = write(fd_, &val, sizeof(val));
    ABSL_CHECK_EQ(res, static_cast<ssize_t>(sizeof(val)))
        << "Failed to write to eventfd: " << aos_strerror(errno);
  }

  uint64_t eventfd_buf = 0;
  AsyncRequest wakeup_req;

 private:
  int fd_ = -1;
};

}  // namespace

struct Aio::TimerState {
  Aio *aio = nullptr;
  AsyncRequest request;
  aos::monotonic_clock::duration interval = aos::monotonic_clock::zero();
  aos::monotonic_clock::time_point deadline = aos::monotonic_clock::epoch();
  CompletionCallback user_callback = nullptr;
  void *user_context = nullptr;
};

struct Aio::Impl {
  // Processes completed CQEs (Completion Queue Events) in the io_uring ring.
  // It iterates through the completion queue, marks each matching AsyncRequest
  // as done, constructs a Completion status, and invokes the associated
  // callback. Returns true if at least one completion event was processed.
  bool ReapCompletions();

  // Scans the completion queue ring for a specific request.  If found,
  // processes it, marks it done, and clears its user data so the normal poll
  // loop ignores it.
  bool ReapSpecificRequest(AsyncRequest *target_req, bool execute_callback);

  // The following methods implement the timer backend and match the behavior of
  // the corresponding Aio::Timer methods documented in aio.h.
  void InitializeTimer(TimerState *state);
  void DestroyTimer(std::unique_ptr<TimerState> state);
  void ScheduleTimer(TimerState *state,
                     aos::monotonic_clock::time_point deadline,
                     aos::monotonic_clock::duration interval,
                     CompletionCallback callback, void *context);
  void CancelTimer(TimerState *state, bool reap);
  bool TimerIsPending(const TimerState *state) const;

  // Synchronously cancels and reaps the target request if reap is true.
  // Otherwise, cancels the request asynchronously.
  void CancelRequest(AsyncRequest *request, bool reap, bool execute_callback);

  // The following methods correspond directly to the public delegate methods
  // documented in Aio in aio.h.
  void AsyncRead(int fd, std::span<char> buffer, AsyncRequest *request);
  void AsyncWrite(int fd, std::span<const char> buffer, AsyncRequest *request);
  void Cancel(AsyncRequest *request);
  void BeforeWait(std::function<void()> function);
  void OnReadable(int fd, std::function<void()> callback);
  void OnError(int fd, std::function<void()> callback);
  void OnWritable(int fd, std::function<void()> callback);
  void OnEvents(int fd, std::function<void(uint32_t)> callback);
  void DeleteFd(int fd);
  void ForgetClosedFd(int fd);
  void EnableWritable(int fd);
  void DisableWritable(int fd);
  void SetEvents(int fd, uint32_t events);
  void RegisterSignalFd(ipc_lib::SignalFd *sfd, std::function<void()> callback);
  void UnregisterSignalFd(ipc_lib::SignalFd *sfd);

  // Schedules a read request on the eventfd descriptor.
  void SubmitWakeupRead();

  // Standard liburing ring structure.
  struct io_uring ring;

  // RAII wrapper around the eventfd descriptor.
  EventFD event_fd;

  // Loop control flags.
  std::atomic<bool> run{true};
  std::atomic<bool> quit_requested{false};

  // Pre-wait callback functions.
  std::vector<std::function<void()>> before_wait_functions;

  // Tracks active legacy readiness poll requests.
  struct LegacyState {
    Aio::Impl *impl = nullptr;
    int fd = -1;
    uint32_t events = 0;
    std::function<void()> in_fn = nullptr;
    std::function<void()> out_fn = nullptr;
    std::function<void()> err_fn = nullptr;
    std::function<void(uint32_t)> events_fn = nullptr;
    AsyncRequest request;

    // Submits a one-shot poll request to the io_uring ring.
    void Submit();
  };
  std::unordered_map<int, std::unique_ptr<LegacyState>> legacy_states;

  // Manages the state for a signal file descriptor registered in Aio.
  // It schedules a persistent multishot poll request (using io_uring's
  // multishot poll support) to monitor the descriptor for incoming signals.
  // When a signal is triggered, the event loop processes the completion event,
  // reads and consumes the signal data from the file descriptor, and executes
  // the registered user callback.
  struct SignalFdState {
    FileDescriptor fd = -1;
    std::function<void()> callback;
    AsyncRequest request;

    // Prepares and submits a persistent poll request to monitor incoming
    // signals.
    void Submit(Aio::Impl *impl);
  };
  std::unordered_map<ipc_lib::SignalFd *, std::unique_ptr<SignalFdState>>
      signalfd_states;
};

void Aio::Impl::LegacyState::Submit() {
  if (!request.done) return;

  uint32_t poll_mask = 0;
  if (events & 0x01) poll_mask |= POLLIN;
  if (events & 0x02) poll_mask |= POLLPRI;
  if (events & 0x04) poll_mask |= POLLOUT;
  if (events & 0x08) poll_mask |= POLLERR | POLLHUP;

  if (poll_mask == 0) {
    request.done = true;
    return;
  }

  struct io_uring_sqe *sqe = io_uring_get_sqe(&impl->ring);
  ABSL_CHECK(sqe != nullptr) << "Out of SQEs";

  io_uring_prep_poll_add(sqe, fd, poll_mask);
  io_uring_sqe_set_data(sqe, &request);

  request.callback = [](Completion completion, void *context) {
    auto state = static_cast<LegacyState *>(context);
    if (aos::IsOk(completion.status)) {
      uint32_t ready_gio = completion.result;
      uint32_t ready_epoll = 0;
      if (ready_gio & POLLIN) ready_epoll |= 0x01;
      if (ready_gio & POLLPRI) ready_epoll |= 0x02;
      if (ready_gio & POLLOUT) ready_epoll |= 0x04;
      if (ready_gio & POLLERR) ready_epoll |= 0x08;
      if (ready_gio & POLLHUP) ready_epoll |= 0x08;

      auto in_fn = state->in_fn;
      auto out_fn = state->out_fn;
      auto err_fn = state->err_fn;
      auto events_fn = state->events_fn;

      // Re-submit the poll request first so it's active.
      state->Submit();
      if (events_fn) {
        events_fn(ready_epoll);
      } else {
        if ((ready_epoll & (0x01 | 0x02)) && in_fn) {
          in_fn();
        }
        if ((ready_epoll & 0x04) && out_fn) {
          out_fn();
        }
        if ((ready_epoll & 0x08) && err_fn) {
          err_fn();
        }
      }
    }
  };
  request.context = this;
  request.user_data = &request;
  request.done = false;
}

void Aio::Impl::SignalFdState::Submit(Aio::Impl *impl) {
  struct io_uring_sqe *sqe = io_uring_get_sqe(&impl->ring);
  ABSL_CHECK(sqe != nullptr) << "Out of SQEs";

  io_uring_prep_poll_multishot(sqe, fd, POLLIN);
  io_uring_sqe_set_data(sqe, &request);

  request.callback = [](Completion completion, void *context) {
    auto state = static_cast<SignalFdState *>(context);
    if (aos::IsOk(completion.status)) {
      struct signalfd_siginfo siginfo;
      ssize_t res = read(state->fd, &siginfo, sizeof(siginfo));
      ABSL_CHECK_EQ(res, static_cast<ssize_t>(sizeof(siginfo)))
          << "Failed to read from signalfd: " << aos_strerror(errno);
      if (state->callback) {
        state->callback();
      }
    }
  };
  request.context = this;
  request.user_data = &request;
  request.done = false;
}

bool Aio::Impl::ReapCompletions() {
  struct io_uring_cqe *cqe = nullptr;
  unsigned head;
  unsigned count = 0;
  bool processed = false;

  io_uring_for_each_cqe(&ring, head, cqe) {
    processed = true;
    count++;
    AsyncRequest *req = (AsyncRequest *)io_uring_cqe_get_data(cqe);
    int32_t res = cqe->res;

    if (req) {
      if (!(cqe->flags & IORING_CQE_F_MORE)) {
        req->done = true;
      }
      Completion completion;
      completion.user_data = req->user_data;

      if (res == -ECANCELED) {
        completion.status = aos::MakeError("Canceled");
        completion.result = 0;
      } else if (res == -ETIME) {
        completion.status = aos::Ok();
        completion.result = 0;
      } else if (res < 0) {
        completion.status = aos::MakeError("io_uring error");
        completion.result = -res;
      } else {
        completion.status = aos::Ok();
        completion.result = res;
      }

      if (req->callback) {
        req->callback(completion, req->context);
      }
    }
  }

  if (count > 0) {
    io_uring_cq_advance(&ring, count);
  }
  return processed;
}

bool Aio::Impl::ReapSpecificRequest(AsyncRequest *target_req,
                                    bool execute_callback) {
  unsigned head;
  struct io_uring_cqe *cqe = nullptr;
  io_uring_for_each_cqe(&ring, head, cqe) {
    if (io_uring_cqe_get_data(cqe) == target_req) {
      int32_t res = cqe->res;
      target_req->done = true;
      cqe->user_data = 0;

      if (execute_callback && target_req->callback) {
        Completion completion;
        completion.user_data = target_req->user_data;
        if (res == -ECANCELED) {
          completion.status = aos::MakeError("Canceled");
          completion.result = 0;
        } else if (res == -ETIME) {
          completion.status = aos::Ok();
          completion.result = 0;
        } else if (res < 0) {
          completion.status = aos::MakeError("io_uring error");
          completion.result = -res;
        } else {
          completion.status = aos::Ok();
          completion.result = res;
        }
        target_req->callback(completion, target_req->context);
      }
      return true;
    }
  }
  return false;
}

void Aio::Impl::InitializeTimer(TimerState *state) {
  state->request.done = true;
}

void Aio::Impl::DestroyTimer(std::unique_ptr<TimerState> state) {
  state->request.callback = nullptr;
  CancelTimer(state.get(), true);
}

void Aio::Impl::ScheduleTimer(TimerState *state,
                              aos::monotonic_clock::time_point deadline,
                              aos::monotonic_clock::duration interval,
                              CompletionCallback callback, void *context) {
  CancelTimer(state, true);

  state->interval = interval;
  state->deadline = deadline;
  state->user_callback = callback;
  state->user_context = context;
  state->request.user_data = state;

  state->request.callback = [](Completion completion, void *ctx) {
    auto *tstate = static_cast<TimerState *>(ctx);
    auto user_cb = tstate->user_callback;
    auto user_ctx = tstate->user_context;

    if (aos::IsOk(completion.status) &&
        tstate->interval > aos::monotonic_clock::zero()) {
      tstate->deadline += tstate->interval;
      auto *impl = tstate->aio->impl_.get();
      tstate->request.done = false;

      struct __kernel_timespec *ts =
          reinterpret_cast<struct __kernel_timespec *>(&tstate->request.tv_sec);
      auto duration = tstate->deadline.time_since_epoch();
      auto secs = std::chrono::duration_cast<std::chrono::seconds>(duration);
      auto nsecs =
          std::chrono::duration_cast<std::chrono::nanoseconds>(duration - secs);
      ts->tv_sec = secs.count();
      ts->tv_nsec = nsecs.count();

      struct io_uring_sqe *sqe = io_uring_get_sqe(&impl->ring);
      ABSL_CHECK(sqe != nullptr) << "Out of SQEs";
      io_uring_prep_timeout(sqe, ts, 0, IORING_TIMEOUT_ABS);
      io_uring_sqe_set_data(sqe, &tstate->request);
      io_uring_submit(&impl->ring);
    }

    if (user_cb) {
      user_cb(completion, user_ctx);
    }
  };
  state->request.context = state;
  state->request.done = false;

  ABSL_CHECK_GE(deadline, aos::monotonic_clock::epoch());
  auto duration = deadline.time_since_epoch();
  auto secs = std::chrono::duration_cast<std::chrono::seconds>(duration);
  auto nsecs =
      std::chrono::duration_cast<std::chrono::nanoseconds>(duration - secs);

  struct __kernel_timespec *ts =
      reinterpret_cast<struct __kernel_timespec *>(&state->request.tv_sec);
  ts->tv_sec = secs.count();
  ts->tv_nsec = nsecs.count();

  struct io_uring_sqe *sqe = io_uring_get_sqe(&ring);
  ABSL_CHECK(sqe != nullptr) << "Out of SQEs";

  io_uring_prep_timeout(sqe, ts, 0, IORING_TIMEOUT_ABS);
  io_uring_sqe_set_data(sqe, &state->request);
}

void Aio::Impl::CancelRequest(AsyncRequest *request, bool reap,
                              bool execute_callback) {
  if (!request->done) {
    if (reap) {
      // First, submit any pending SQEs to ensure the request is in the kernel.
      io_uring_submit(&ring);

      struct io_uring_sync_cancel_reg reg;
      std::memset(&reg, 0, sizeof(reg));
      reg.addr = reinterpret_cast<uint64_t>(request);
      reg.flags = 0;

      int ret = io_uring_register_sync_cancel(&ring, &reg);
      if (ret == 0) {
        ABSL_CHECK(ReapSpecificRequest(request, execute_callback));
      } else {
        // If the cancel failed or was already in flight, wait for the kernel to
        // generate a completion event for this request.
        while (!ReapSpecificRequest(request, execute_callback)) {
          struct io_uring_cqe *cqe = nullptr;
          int wait_ret = io_uring_wait_cqe(&ring, &cqe);
          ABSL_CHECK_EQ(wait_ret, 0)
              << "io_uring_wait_cqe failed: " << aos_strerror(-wait_ret);
        }
      }
    } else {
      Cancel(request);
    }
  }
}

void Aio::Impl::CancelTimer(TimerState *state, bool reap) {
  CancelRequest(&state->request, reap, true);
}

bool Aio::Impl::TimerIsPending(const TimerState *state) const {
  return !state->request.done;
}

void Aio::Impl::AsyncRead(int fd, std::span<char> buffer,
                          AsyncRequest *request) {
  struct io_uring_sqe *sqe = io_uring_get_sqe(&ring);
  ABSL_CHECK(sqe != nullptr) << "Out of SQEs";

  io_uring_prep_read(sqe, fd, buffer.data(), buffer.size(), -1);
  io_uring_sqe_set_data(sqe, request);
  request->done = false;
}

void Aio::Impl::AsyncWrite(int fd, std::span<const char> buffer,
                           AsyncRequest *request) {
  struct io_uring_sqe *sqe = io_uring_get_sqe(&ring);
  ABSL_CHECK(sqe != nullptr) << "Out of SQEs";

  io_uring_prep_write(sqe, fd, buffer.data(), buffer.size(), -1);
  io_uring_sqe_set_data(sqe, request);
  request->done = false;
}

void Aio::Impl::Cancel(AsyncRequest *request) {
  if (request->done) return;

  struct io_uring_sqe *sqe = io_uring_get_sqe(&ring);
  ABSL_CHECK(sqe != nullptr) << "Out of SQEs";
  io_uring_prep_cancel(sqe, request, 0);
  io_uring_sqe_set_data(sqe, nullptr);
  io_uring_submit(&ring);
}

void Aio::Impl::BeforeWait(std::function<void()> function) {
  before_wait_functions.push_back(std::move(function));
}

void Aio::Impl::OnReadable(int fd, std::function<void()> callback) {
  auto [it, inserted] = legacy_states.try_emplace(fd);
  if (inserted) {
    it->second = std::make_unique<Impl::LegacyState>();
    it->second->impl = this;
    it->second->fd = fd;
  } else {
    ABSL_CHECK(!it->second->in_fn) << "Duplicate in functions for " << fd;
  }
  auto &state = *it->second;
  ABSL_CHECK(!state.events_fn)
      << "Cannot mix OnEvents and OnReadable for fd " << fd;
  state.in_fn = std::move(callback);
  uint32_t new_events = state.events | 0x01 | 0x02 | 0x08;
  if (state.events != new_events) {
    CancelRequest(&state.request, true, false);
    state.events = new_events;
    state.Submit();
  }
}

void Aio::Impl::OnError(int fd, std::function<void()> callback) {
  auto [it, inserted] = legacy_states.try_emplace(fd);
  if (inserted) {
    it->second = std::make_unique<Impl::LegacyState>();
    it->second->impl = this;
    it->second->fd = fd;
  } else {
    ABSL_CHECK(!it->second->err_fn) << "Duplicate error functions for " << fd;
  }
  auto &state = *it->second;
  ABSL_CHECK(!state.events_fn)
      << "Cannot mix OnEvents and OnError for fd " << fd;
  state.err_fn = std::move(callback);
  uint32_t new_events = state.events | 0x08;
  if (state.events != new_events) {
    CancelRequest(&state.request, true, false);
    state.events = new_events;
    state.Submit();
  }
}

void Aio::Impl::OnWritable(int fd, std::function<void()> callback) {
  auto [it, inserted] = legacy_states.try_emplace(fd);
  if (inserted) {
    it->second = std::make_unique<Impl::LegacyState>();
    it->second->impl = this;
    it->second->fd = fd;
  } else {
    ABSL_CHECK(!it->second->out_fn) << "Duplicate out functions for " << fd;
  }
  auto &state = *it->second;
  ABSL_CHECK(!state.events_fn)
      << "Cannot mix OnEvents and OnWritable for fd " << fd;
  state.out_fn = std::move(callback);
  uint32_t new_events = state.events | 0x04;
  if (state.events != new_events) {
    CancelRequest(&state.request, true, false);
    state.events = new_events;
    state.Submit();
  }
}

void Aio::Impl::OnEvents(int fd, std::function<void(uint32_t)> callback) {
  auto [it, inserted] = legacy_states.try_emplace(fd);
  ABSL_CHECK(inserted) << "May not replace OnEvents handlers for fd " << fd;

  it->second = std::make_unique<Impl::LegacyState>();
  it->second->impl = this;
  it->second->fd = fd;
  auto &state = *it->second;
  state.events_fn = std::move(callback);
}

void Aio::Impl::DeleteFd(int fd) {
  auto it = legacy_states.find(fd);
  ABSL_CHECK(it != legacy_states.end()) << "fd " << fd << " not found";

  auto state = std::move(it->second);
  legacy_states.erase(it);

  CancelRequest(&state->request, true, false);
}

void Aio::Impl::ForgetClosedFd(int fd) {
  auto it = legacy_states.find(fd);
  ABSL_CHECK(it != legacy_states.end()) << "fd " << fd << " not found";

  auto state = std::move(it->second);
  legacy_states.erase(it);

  state->request.done = true;
}

void Aio::Impl::EnableWritable(int fd) {
  auto it = legacy_states.find(fd);
  ABSL_CHECK(it != legacy_states.end()) << "fd " << fd << " not found";

  auto &state = *it->second;
  ABSL_CHECK(!state.events_fn) << "EnableWritable is only for fds registered "
                                  "using OnWritable, not OnEvents";
  uint32_t new_events = state.events | 0x04;
  if (state.events != new_events) {
    CancelRequest(&state.request, true, false);
    state.events = new_events;
    state.Submit();
  }
}

void Aio::Impl::DisableWritable(int fd) {
  auto it = legacy_states.find(fd);
  ABSL_CHECK(it != legacy_states.end()) << "fd " << fd << " not found";

  auto &state = *it->second;
  ABSL_CHECK(!state.events_fn) << "DisableWritable is only for fds registered "
                                  "using OnWritable, not OnEvents";
  uint32_t new_events = state.events & ~0x04;
  if (state.events != new_events) {
    CancelRequest(&state.request, true, false);
    state.events = new_events;
    state.Submit();
  }
}

void Aio::Impl::SetEvents(int fd, uint32_t events) {
  auto it = legacy_states.find(fd);
  ABSL_CHECK(it != legacy_states.end()) << "fd " << fd << " not found";

  auto &state = *it->second;
  ABSL_CHECK(state.events_fn)
      << "SetEvents is only for fds registered using OnEvents";
  if (state.events != events) {
    CancelRequest(&state.request, true, false);
    state.events = events;
    state.Submit();
  }
}

void Aio::Impl::SubmitWakeupRead() {
  AsyncRead(event_fd.fd(),
            std::span<char>(reinterpret_cast<char *>(&event_fd.eventfd_buf),
                            sizeof(event_fd.eventfd_buf)),
            &event_fd.wakeup_req);
}

void Aio::Impl::RegisterSignalFd(ipc_lib::SignalFd *sfd,
                                 std::function<void()> callback) {
  auto [it, inserted] = signalfd_states.try_emplace(sfd);
  ABSL_CHECK(inserted) << "Duplicate signalfd registration";

  it->second = std::make_unique<Impl::SignalFdState>();
  it->second->fd = sfd->fd();
  auto &state = *it->second;
  state.callback = std::move(callback);
  state.Submit(this);
}

void Aio::Impl::UnregisterSignalFd(ipc_lib::SignalFd *sfd) {
  auto it = signalfd_states.find(sfd);
  ABSL_CHECK(it != signalfd_states.end()) << "SignalFd not found";

  auto state = std::move(it->second);
  signalfd_states.erase(it);

  state->callback = nullptr;
  CancelRequest(&state->request, true, false);
}

Aio::Aio() : impl_(std::make_unique<Impl>()) {
  // Initialize the io_uring ring with the configured queue depth.
  //
  // We attempt to set the following real-time optimization flags:
  // 1. IORING_SETUP_SINGLE_ISSUER: Informs the kernel that only a single thread
  //    (the event loop thread) will submit and reap requests on this ring. This
  //    allows the kernel to bypass internal mutexes and locks, making the
  //    entire operation path lock-free.
  // 2. IORING_SETUP_COOP_TASKRUN: Prevents the kernel from raising asynchronous
  //    interrupts (like signals or TIF_NOTIFY_SIGNAL) to wake up the thread and
  //    process completion task work while it is busy executing user-space code.
  //    Instead, completion work is deferred to run cooperatively when the
  //    thread naturally enters the kernel (e.g., inside Aio::Poll or
  //    io_uring_enter). This completely eliminates scheduling jitter in the
  //    user-space RT loop.
  // 3. IORING_SETUP_TASKRUN_FLAG: Used with IORING_SETUP_COOP_TASKRUN to expose
  //    a flag indicating when task work is pending, allowing checking for work
  //    efficiently.
  //
  // Note on wakeups: If the loop is asleep waiting for the next event inside
  // io_uring_enter, the timer or signal expiry wakes the thread up immediately
  // and executes the task work as part of the wakeup transition, ensuring zero
  // wakeup latency.
  //
  // If the running kernel does not support these optimizations (returning
  // -EINVAL), we fall back to a standard queue initialization (0 flags).
  const uint32_t queue_depth = ::absl::GetFlag(FLAGS_aio_queue_depth);
  uint32_t flags = IORING_SETUP_SINGLE_ISSUER | IORING_SETUP_COOP_TASKRUN |
                   IORING_SETUP_TASKRUN_FLAG;
  int ret = io_uring_queue_init(queue_depth, &(impl_->ring), flags);
  if (ret == -EINVAL) {
    // Try without IORING_SETUP_SINGLE_ISSUER (requires Linux 6.0+).
    flags = IORING_SETUP_COOP_TASKRUN | IORING_SETUP_TASKRUN_FLAG;
    ret = io_uring_queue_init(queue_depth, &(impl_->ring), flags);
    if (ret == 0) {
      ABSL_LOG(WARNING)
          << "Kernel does not support IORING_SETUP_SINGLE_ISSUER (requires "
             "Linux 6.0+).  Continuing with cooperative taskrun optimizations "
             "only.  Consider using EPoll instead for RT performance.";
    } else if (ret == -EINVAL) {
      // Try standard initialization (cooperative taskrun requires Linux 5.19+).
      ret = io_uring_queue_init(queue_depth, &(impl_->ring), 0);
      if (ret == 0) {
        ABSL_LOG(WARNING)
            << "Kernel does not support IORING_SETUP_COOP_TASKRUN or "
               "IORING_SETUP_TASKRUN_FLAG (require Linux 5.19+).  Continuing "
               "with standard initialization.  Consider using EPoll instead "
               "for RT performance.";
      }
    }
  }
  ABSL_PCHECK(ret == 0) << "io_uring_queue_init failed: " << ret;

  // When wakeup event fd read completes, re-schedule it.
  impl_->event_fd.wakeup_req.callback = [](Completion completion,
                                           void *context) {
    Aio::Impl *impl = (Aio::Impl *)context;
    if (aos::IsOk(completion.status)) {
      impl->SubmitWakeupRead();
    }
  };
  impl_->event_fd.wakeup_req.context = impl_.get();
  impl_->SubmitWakeupRead();
}

Aio::~Aio() {
  std::vector<AsyncRequest *> active_reqs;
  for (auto &pair : impl_->legacy_states) {
    if (!pair.second->request.done) {
      active_reqs.push_back(&pair.second->request);
    }
  }
  for (auto &pair : impl_->signalfd_states) {
    if (!pair.second->request.done) {
      active_reqs.push_back(&pair.second->request);
    }
  }
  for (auto *req : active_reqs) {
    Cancel(req);
  }
  if (!impl_->event_fd.wakeup_req.done) {
    Cancel(&impl_->event_fd.wakeup_req);
  }
  io_uring_queue_exit(&impl_->ring);
}

void Aio::Run() {
  if (impl_->quit_requested) {
    impl_->quit_requested = false;
    return;
  }
  impl_->run = true;
  while (impl_->run) {
    Poll(true);
  }
  impl_->quit_requested = false;
}

bool Aio::Poll(bool block) {
  // Execute pre-wait callbacks.
  for (const auto &fn : impl_->before_wait_functions) {
    fn();
  }

  int ret;
  if (block) {
    // Submit any pending submissions and block until at least 1 completion is
    // available.
    ret = io_uring_submit_and_wait(&impl_->ring, 1);
  } else {
    // Just submit any pending submissions without blocking.
    ret = io_uring_submit(&impl_->ring);
  }

  if (ret < 0 && ret != -EINTR && ret != -EAGAIN) {
    ABSL_LOG(ERROR) << "io_uring submit failed: " << aos_strerror(-ret);
  }

  return impl_->ReapCompletions();
}

void Aio::Quit() {
  impl_->quit_requested = true;
  impl_->run = false;
  Wakeup();
}

void Aio::Wakeup() { impl_->event_fd.Write(); }

void Aio::AsyncRead(int fd, std::span<char> buffer, AsyncRequest *request) {
  impl_->AsyncRead(fd, buffer, request);
}

void Aio::AsyncWrite(int fd, std::span<const char> buffer,
                     AsyncRequest *request) {
  impl_->AsyncWrite(fd, buffer, request);
}

void Aio::Cancel(AsyncRequest *request) { impl_->Cancel(request); }

void Aio::BeforeWait(std::function<void()> function) {
  impl_->BeforeWait(std::move(function));
}

void Aio::OnReadable(int fd, std::function<void()> callback) {
  impl_->OnReadable(fd, std::move(callback));
}

void Aio::OnError(int fd, std::function<void()> callback) {
  impl_->OnError(fd, std::move(callback));
}

void Aio::OnWritable(int fd, std::function<void()> callback) {
  impl_->OnWritable(fd, std::move(callback));
}

void Aio::OnEvents(int fd, std::function<void(uint32_t)> callback) {
  impl_->OnEvents(fd, std::move(callback));
}

void Aio::DeleteFd(int fd) { impl_->DeleteFd(fd); }

void Aio::ForgetClosedFd(int fd) { impl_->ForgetClosedFd(fd); }

void Aio::EnableWritable(int fd) { impl_->EnableWritable(fd); }

void Aio::DisableWritable(int fd) { impl_->DisableWritable(fd); }

void Aio::SetEvents(int fd, uint32_t events) { impl_->SetEvents(fd, events); }

void Aio::RegisterSignalFd(ipc_lib::SignalFd *sfd,
                           std::function<void()> callback) {
  impl_->RegisterSignalFd(sfd, std::move(callback));
}

void Aio::UnregisterSignalFd(ipc_lib::SignalFd *sfd) {
  impl_->UnregisterSignalFd(sfd);
}

Aio::Timer::Timer(Aio *aio) : state_(std::make_unique<TimerState>()) {
  state_->aio = aio;
  state_->aio->impl_->InitializeTimer(state_.get());
}

Aio::Timer::~Timer() {
  if (state_) {
    state_->aio->impl_->DestroyTimer(std::move(state_));
  }
}

void Aio::Timer::Schedule(aos::monotonic_clock::time_point deadline,
                          aos::monotonic_clock::duration interval,
                          CompletionCallback callback, void *context) {
  state_->aio->impl_->ScheduleTimer(state_.get(), deadline, interval, callback,
                                    context);
}

void Aio::Timer::Cancel() {
  state_->aio->impl_->CancelTimer(state_.get(), false);
}

bool Aio::Timer::IsPending() const {
  return state_ && state_->aio->impl_->TimerIsPending(state_.get());
}

}  // namespace aos
