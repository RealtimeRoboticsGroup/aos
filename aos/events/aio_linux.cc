#include <errno.h>
#include <fcntl.h>
#include <liburing.h>
#include <poll.h>
#include <stdio.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <sys/timerfd.h>
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
#include <memory>
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

ABSL_FLAG(bool, use_io_uring, true,
          "Whether to use the io_uring backend or fallback to epoll.");

ABSL_FLAG(size_t, aio_epoll_pool_size, 16,
          "Initial size of the pre-allocated epoll FdRegistration pool.");

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

  int fd() const { return fd_; }

  void Write() {
    uint64_t val = 1;
    ssize_t ret = write(fd_, &val, sizeof(val));
    if (ret < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      ABSL_LOG(FATAL) << "Failed to write to eventfd: " << aos_strerror(errno);
    }
  }

  uint64_t eventfd_buf = 0;
  AsyncRequest wakeup_req;

 private:
  int fd_ = -1;
};

// RAII wrapper around timerfd file descriptors.
class TimerFD {
 public:
  TimerFD() {
    fd_ = timerfd_create(CLOCK_MONOTONIC, TFD_CLOEXEC | TFD_NONBLOCK);
    ABSL_PCHECK(fd_ >= 0) << "Failed to create timerfd";
  }

  ~TimerFD() {
    if (fd_ >= 0) {
      close(fd_);
    }
  }

  // Disable copy construction and assignment to prevent duplicate ownership of
  // the descriptor.
  TimerFD(const TimerFD &) = delete;
  TimerFD &operator=(const TimerFD &) = delete;

  int fd() const { return fd_; }

 private:
  int fd_ = -1;
};

// Internal helper union to access AsyncRequest's opaque state buffer.  This
// avoids type-punning issues and pointer-to-pointer casting.
union AioState {
  struct {
    int64_t tv_sec;
    int64_t tv_nsec;
  } timespec;
  struct {
    void *ptr;
    size_t size;
  } buffer;
  struct {
    AsyncRequest *next;
    int64_t result;
  } link;
};

inline AioState &State(AsyncRequest *req) {
  static_assert(sizeof(AioState) <= sizeof(req->internal_state),
                "AioState too large");
  static_assert(alignof(AioState) <= 8, "AioState alignment mismatch");
  return *reinterpret_cast<AioState *>(req->internal_state);
}

[[maybe_unused]] inline const AioState &State(const AsyncRequest *req) {
  return *reinterpret_cast<const AioState *>(req->internal_state);
}

}  // namespace

struct Aio::TimerState {
  Aio *aio = nullptr;
  AsyncRequest request;
  aos::monotonic_clock::duration interval = aos::monotonic_clock::zero();
  aos::monotonic_clock::time_point deadline = aos::monotonic_clock::epoch();
  CompletionCallback user_callback = nullptr;
  void *user_context = nullptr;

  // Epoll-backend specific timerfd state
  std::unique_ptr<TimerFD> timer_fd;
};

struct Aio::Impl {
  virtual ~Impl() = default;

  virtual void Run() = 0;

  // Polls the event loop for completed events.  If block is true, it waits
  // for at least one event to complete.  Otherwise, it returns immediately.
  // Returns true if at least one event completed.
  virtual bool Poll(bool block) = 0;

  // Requests the event loop to stop running.
  virtual void Quit() = 0;

  // Wakes up a blocking event loop.
  virtual void Wakeup() = 0;

  // The following methods submit asynchronous I/O requests.  The callback is
  // executed when the operation completes.
  virtual void AsyncRead(int fd, std::span<char> buffer,
                         AsyncRequest *request) = 0;
  virtual void AsyncWrite(int fd, std::span<const char> buffer,
                          AsyncRequest *request) = 0;

  // Cancels a pending asynchronous request.  The callback will be executed
  // with a Canceled status.
  virtual void Cancel(AsyncRequest *request) = 0;

  // Registers a callback to be executed before waiting for events.
  virtual void BeforeWait(std::function<void()> function) = 0;

  // Legacy Readiness Hooks.
  virtual void OnReadable(int fd, std::function<void()> callback) = 0;
  virtual void OnError(int fd, std::function<void()> callback) = 0;
  virtual void OnWritable(int fd, std::function<void()> callback) = 0;
  virtual void OnEvents(int fd, std::function<void(uint32_t)> callback) = 0;
  virtual void DeleteFd(int fd) = 0;
  virtual void ForgetClosedFd(int fd) = 0;
  virtual void EnableWritable(int fd) = 0;
  virtual void DisableWritable(int fd) = 0;
  virtual void SetEvents(int fd, uint32_t events) = 0;

  // Registers a SignalFd for wakeups.
  virtual void RegisterSignalFd(ipc_lib::SignalFd *sfd,
                                std::function<void()> callback) = 0;
  virtual void UnregisterSignalFd(ipc_lib::SignalFd *sfd) = 0;

  // The following methods implement the timer backend and match the behavior of
  // the corresponding Aio::Timer methods documented in aio.h.
  virtual void InitializeTimer(Aio::TimerState *state) = 0;
  virtual void DestroyTimer(std::unique_ptr<Aio::TimerState> state) = 0;
  virtual void ScheduleTimer(Aio::TimerState *state,
                             aos::monotonic_clock::time_point deadline,
                             aos::monotonic_clock::duration interval,
                             CompletionCallback callback, void *context) = 0;
  virtual void CancelTimer(Aio::TimerState *state, bool reap) = 0;
};

class IoUringImpl : public Aio::Impl {
 public:
  IoUringImpl();
  ~IoUringImpl() override;

  void Run() override;
  bool Poll(bool block) override;
  void Quit() override;
  void Wakeup() override;

  void AsyncRead(int fd, std::span<char> buffer,
                 AsyncRequest *request) override;
  void AsyncWrite(int fd, std::span<const char> buffer,
                  AsyncRequest *request) override;
  void Cancel(AsyncRequest *request) override;
  void BeforeWait(std::function<void()> function) override;

  void OnReadable(int fd, std::function<void()> callback) override;
  void OnError(int fd, std::function<void()> callback) override;
  void OnWritable(int fd, std::function<void()> callback) override;
  void OnEvents(int fd, std::function<void(uint32_t)> callback) override;
  void DeleteFd(int fd) override;
  void ForgetClosedFd(int fd) override;
  void EnableWritable(int fd) override;
  void DisableWritable(int fd) override;
  void SetEvents(int fd, uint32_t events) override;

  void RegisterSignalFd(ipc_lib::SignalFd *sfd,
                        std::function<void()> callback) override;
  void UnregisterSignalFd(ipc_lib::SignalFd *sfd) override;

  void InitializeTimer(Aio::TimerState *state) override;
  void DestroyTimer(std::unique_ptr<Aio::TimerState> state) override;
  void ScheduleTimer(Aio::TimerState *state,
                     aos::monotonic_clock::time_point deadline,
                     aos::monotonic_clock::duration interval,
                     CompletionCallback callback, void *context) override;
  void CancelTimer(Aio::TimerState *state, bool reap) override;

 private:
  bool ReapCompletions();
  bool ReapSpecificRequest(AsyncRequest *target_req);
  void CancelRequest(AsyncRequest *request, bool reap);
  void SubmitWakeupRead();

  struct io_uring ring;
  EventFD event_fd;

  std::atomic<bool> run{true};
  std::atomic<bool> quit_requested{false};

  std::vector<std::function<void()>> before_wait_functions;

  struct LegacyState {
    IoUringImpl *impl = nullptr;
    int fd = -1;
    uint32_t events = 0;
    std::function<void()> in_fn = nullptr;
    std::function<void()> out_fn = nullptr;
    std::function<void()> err_fn = nullptr;
    std::function<void(uint32_t)> events_fn = nullptr;
    AsyncRequest request;

    void Submit();
  };
  std::unordered_map<int, std::unique_ptr<LegacyState>> legacy_states;

  struct SignalFdState {
    FileDescriptor fd = -1;
    std::function<void()> callback;
    AsyncRequest request;

    void Submit(IoUringImpl *impl);
  };
  std::unordered_map<ipc_lib::SignalFd *, std::unique_ptr<SignalFdState>>
      signalfd_states;
};

void IoUringImpl::LegacyState::Submit() {
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
  request.done = false;

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

void IoUringImpl::SignalFdState::Submit(IoUringImpl *impl) {
  struct io_uring_sqe *sqe = io_uring_get_sqe(&impl->ring);
  ABSL_CHECK(sqe != nullptr) << "Out of SQEs";

  io_uring_prep_poll_multishot(sqe, fd, POLLIN);
  io_uring_sqe_set_data(sqe, &request);

  request.callback = [](Completion completion, void *context) {
    auto state = static_cast<SignalFdState *>(context);
    if (aos::IsOk(completion.status)) {
      struct signalfd_siginfo siginfo;
      while (true) {
        ssize_t res = read(state->fd, &siginfo, sizeof(siginfo));
        if (res == sizeof(siginfo)) {
          if (state->callback) {
            state->callback();
          }
        } else if (res < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
          break;
        } else {
          ABSL_LOG(FATAL) << "Failed to read from signalfd: "
                          << aos_strerror(errno);
        }
      }
    }
  };
  request.context = this;
  request.user_data = &request;
  request.done = false;
}

IoUringImpl::IoUringImpl() {
  uint32_t depth = ::absl::GetFlag(FLAGS_aio_queue_depth);

  struct io_uring_params params;
  std::memset(&params, 0, sizeof(params));

  params.flags |= IORING_SETUP_SINGLE_ISSUER;
  params.flags |= IORING_SETUP_COOP_TASKRUN;
  params.flags |= IORING_SETUP_TASKRUN_FLAG;

  int ret = io_uring_queue_init_params(depth, &ring, &params);
  if (ret < 0) {
    std::memset(&params, 0, sizeof(params));
    ret = io_uring_queue_init_params(depth, &ring, &params);
    ABSL_PCHECK(ret == 0) << "io_uring_queue_init failed: "
                          << aos_strerror(-ret);
  }

  // When wakeup event fd read completes, re-schedule it.
  event_fd.wakeup_req.callback = [](Completion completion, void *context) {
    auto *impl = static_cast<IoUringImpl *>(context);
    if (aos::IsOk(completion.status)) {
      impl->SubmitWakeupRead();
    }
  };
  event_fd.wakeup_req.context = this;

  SubmitWakeupRead();
}

IoUringImpl::~IoUringImpl() {
  run = false;
  std::vector<AsyncRequest *> active_reqs;
  for (auto &pair : legacy_states) {
    if (!pair.second->request.done) {
      active_reqs.push_back(&pair.second->request);
    }
  }
  for (auto &pair : signalfd_states) {
    if (!pair.second->request.done) {
      active_reqs.push_back(&pair.second->request);
    }
  }
  for (auto *req : active_reqs) {
    Cancel(req);
  }
  if (!event_fd.wakeup_req.done) {
    Cancel(&event_fd.wakeup_req);
  }
  io_uring_queue_exit(&ring);
}

void IoUringImpl::Run() {
  if (quit_requested) {
    quit_requested = false;
    return;
  }
  run = true;
  while (run) {
    Poll(true);
  }
  quit_requested = false;
}

bool IoUringImpl::Poll(bool block) {
  for (const auto &fn : before_wait_functions) {
    fn();
  }

  int ret;
  if (block) {
    ret = io_uring_submit_and_wait(&ring, 1);
  } else {
    ret = io_uring_submit(&ring);
  }

  if (ret < 0 && ret != -EINTR && ret != -EAGAIN) {
    ABSL_LOG(ERROR) << "io_uring submit failed: " << aos_strerror(-ret);
  }

  return ReapCompletions();
}

void IoUringImpl::Quit() {
  quit_requested = true;
  run = false;
  Wakeup();
}

void IoUringImpl::Wakeup() { event_fd.Write(); }

void IoUringImpl::AsyncRead(int fd, std::span<char> buffer,
                            AsyncRequest *request) {
  struct io_uring_sqe *sqe = io_uring_get_sqe(&ring);
  ABSL_CHECK(sqe != nullptr) << "Out of SQEs";

  io_uring_prep_read(sqe, fd, buffer.data(), buffer.size(), -1);
  io_uring_sqe_set_data(sqe, request);
  request->done = false;
}

void IoUringImpl::AsyncWrite(int fd, std::span<const char> buffer,
                             AsyncRequest *request) {
  struct io_uring_sqe *sqe = io_uring_get_sqe(&ring);
  ABSL_CHECK(sqe != nullptr) << "Out of SQEs";

  io_uring_prep_write(sqe, fd, buffer.data(), buffer.size(), -1);
  io_uring_sqe_set_data(sqe, request);
  request->done = false;
}

void IoUringImpl::Cancel(AsyncRequest *request) {
  if (request->done) return;

  struct io_uring_sqe *sqe = io_uring_get_sqe(&ring);
  ABSL_CHECK(sqe != nullptr) << "Out of SQEs";
  io_uring_prep_cancel(sqe, request, 0);
  io_uring_sqe_set_data(sqe, nullptr);
  io_uring_submit(&ring);
}

void IoUringImpl::BeforeWait(std::function<void()> function) {
  before_wait_functions.push_back(std::move(function));
}

void IoUringImpl::OnReadable(int fd, std::function<void()> callback) {
  auto [it, inserted] = legacy_states.try_emplace(fd);
  if (inserted) {
    it->second = std::make_unique<IoUringImpl::LegacyState>();
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
    CancelRequest(&state.request, true);
    state.events = new_events;
    state.Submit();
  }
}

void IoUringImpl::OnError(int fd, std::function<void()> callback) {
  auto [it, inserted] = legacy_states.try_emplace(fd);
  if (inserted) {
    it->second = std::make_unique<IoUringImpl::LegacyState>();
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
    CancelRequest(&state.request, true);
    state.events = new_events;
    state.Submit();
  }
}

void IoUringImpl::OnWritable(int fd, std::function<void()> callback) {
  auto [it, inserted] = legacy_states.try_emplace(fd);
  if (inserted) {
    it->second = std::make_unique<IoUringImpl::LegacyState>();
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
    CancelRequest(&state.request, true);
    state.events = new_events;
    state.Submit();
  }
}

void IoUringImpl::OnEvents(int fd, std::function<void(uint32_t)> callback) {
  auto [it, inserted] = legacy_states.try_emplace(fd);
  ABSL_CHECK(inserted) << "May not replace OnEvents handlers for fd " << fd;

  it->second = std::make_unique<IoUringImpl::LegacyState>();
  it->second->impl = this;
  it->second->fd = fd;
  auto &state = *it->second;
  state.events_fn = std::move(callback);
}

void IoUringImpl::DeleteFd(int fd) {
  auto it = legacy_states.find(fd);
  ABSL_CHECK(it != legacy_states.end()) << "fd " << fd << " not found";

  auto state = std::move(it->second);
  legacy_states.erase(it);

  CancelRequest(&state->request, true);
}

void IoUringImpl::ForgetClosedFd(int fd) {
  auto it = legacy_states.find(fd);
  ABSL_CHECK(it != legacy_states.end()) << "fd " << fd << " not found";

  auto state = std::move(it->second);
  legacy_states.erase(it);

  state->request.done = true;
}

void IoUringImpl::EnableWritable(int fd) {
  auto it = legacy_states.find(fd);
  ABSL_CHECK(it != legacy_states.end()) << "fd " << fd << " not found";

  auto &state = *it->second;
  ABSL_CHECK(!state.events_fn)
      << "EnableWritable is only for fds registered using OnWritable, not "
         "OnEvents";
  uint32_t new_events = state.events | 0x04;
  if (state.events != new_events) {
    CancelRequest(&state.request, true);
    state.events = new_events;
    state.Submit();
  }
}

void IoUringImpl::DisableWritable(int fd) {
  auto it = legacy_states.find(fd);
  ABSL_CHECK(it != legacy_states.end()) << "fd " << fd << " not found";

  auto &state = *it->second;
  ABSL_CHECK(!state.events_fn)
      << "DisableWritable is only for fds registered using OnWritable, not "
         "OnEvents";
  uint32_t new_events = state.events & ~0x04;
  if (state.events != new_events) {
    CancelRequest(&state.request, true);
    state.events = new_events;
    state.Submit();
  }
}

void IoUringImpl::SetEvents(int fd, uint32_t events) {
  auto it = legacy_states.find(fd);
  ABSL_CHECK(it != legacy_states.end()) << "fd " << fd << " not found";

  auto &state = *it->second;
  ABSL_CHECK(state.events_fn)
      << "SetEvents is only for fds registered using OnEvents";
  if (state.events != events) {
    CancelRequest(&state.request, true);
    state.events = events;
    state.Submit();
  }
}

void IoUringImpl::RegisterSignalFd(ipc_lib::SignalFd *sfd,
                                   std::function<void()> callback) {
  auto [it, inserted] = signalfd_states.try_emplace(sfd);
  ABSL_CHECK(inserted) << "Duplicate signalfd registration";

  it->second = std::make_unique<IoUringImpl::SignalFdState>();
  it->second->fd = sfd->fd();
  auto &state = *it->second;
  state.callback = std::move(callback);
  state.Submit(this);
}

void IoUringImpl::UnregisterSignalFd(ipc_lib::SignalFd *sfd) {
  auto it = signalfd_states.find(sfd);
  ABSL_CHECK(it != signalfd_states.end()) << "SignalFd not found";

  auto state = std::move(it->second);
  signalfd_states.erase(it);

  state->callback = nullptr;
  CancelRequest(&state->request, true);
}

void IoUringImpl::InitializeTimer(Aio::TimerState *state) {
  state->request.done = true;
}

void IoUringImpl::DestroyTimer(std::unique_ptr<Aio::TimerState> state) {
  CancelTimer(state.get(), true);
}

void IoUringImpl::ScheduleTimer(Aio::TimerState *state,
                                aos::monotonic_clock::time_point deadline,
                                aos::monotonic_clock::duration interval,
                                CompletionCallback callback, void *context) {
  CancelTimer(state, true);

  state->interval = interval;
  state->deadline = deadline;
  state->user_callback = callback;
  state->user_context = context;
  state->request.user_data = state;
  state->request.done = false;

  state->request.callback = [](Completion completion, void *ctx) {
    auto *tstate = static_cast<Aio::TimerState *>(ctx);
    auto user_cb = tstate->user_callback;
    auto user_ctx = tstate->user_context;

    if (aos::IsOk(completion.status) &&
        tstate->interval > aos::monotonic_clock::zero()) {
      tstate->deadline += tstate->interval;
      auto *impl = tstate->aio->impl_.get();
      tstate->request.done = false;

      struct __kernel_timespec *ts =
          reinterpret_cast<struct __kernel_timespec *>(
              &State(&tstate->request).timespec);
      auto duration = tstate->deadline.time_since_epoch();
      auto secs = std::chrono::duration_cast<std::chrono::seconds>(duration);
      auto nsecs =
          std::chrono::duration_cast<std::chrono::nanoseconds>(duration - secs);
      ts->tv_sec = secs.count();
      ts->tv_nsec = nsecs.count();

      auto *ring_impl = static_cast<IoUringImpl *>(impl);
      struct io_uring_sqe *sqe = io_uring_get_sqe(&ring_impl->ring);
      ABSL_CHECK(sqe != nullptr) << "Out of SQEs";
      io_uring_prep_timeout(sqe, ts, 0, IORING_TIMEOUT_ABS);
      io_uring_sqe_set_data(sqe, &tstate->request);
      io_uring_submit(&ring_impl->ring);
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

  struct __kernel_timespec *ts = reinterpret_cast<struct __kernel_timespec *>(
      &State(&state->request).timespec);
  ts->tv_sec = secs.count();
  ts->tv_nsec = nsecs.count();

  struct io_uring_sqe *sqe = io_uring_get_sqe(&ring);
  ABSL_CHECK(sqe != nullptr) << "Out of SQEs";

  io_uring_prep_timeout(sqe, ts, 0, IORING_TIMEOUT_ABS);
  io_uring_sqe_set_data(sqe, &state->request);
}

void IoUringImpl::CancelTimer(Aio::TimerState *state, bool reap) {
  state->request.callback = nullptr;
  CancelRequest(&state->request, reap);
}

bool IoUringImpl::ReapCompletions() {
  bool processed = false;
  struct io_uring_cqe *cqe = nullptr;
  unsigned head;
  int count = 0;

  io_uring_for_each_cqe(&ring, head, cqe) {
    processed = true;
    ++count;
    auto *req = static_cast<AsyncRequest *>(io_uring_cqe_get_data(cqe));

    if (req != nullptr) {
      int32_t res = cqe->res;
      if (!(cqe->flags & IORING_CQE_F_MORE)) {
        req->done = true;
      }
      cqe->user_data = 0;

      if (req->callback) {
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

        req->callback(completion, req->context);
      }
    }
  }

  if (count > 0) {
    io_uring_cq_advance(&ring, count);
  }

  return processed;
}

bool IoUringImpl::ReapSpecificRequest(AsyncRequest *target_req) {
  unsigned head;
  struct io_uring_cqe *cqe = nullptr;
  io_uring_for_each_cqe(&ring, head, cqe) {
    if (io_uring_cqe_get_data(cqe) == target_req) {
      target_req->done = true;
      cqe->user_data = 0;
      return true;
    }
  }
  return false;
}

void IoUringImpl::CancelRequest(AsyncRequest *request, bool reap) {
  if (!request->done) {
    Cancel(request);
    if (reap) {
      while (!request->done) {
        if (!ReapSpecificRequest(request)) {
          struct io_uring_cqe *cfe = nullptr;
          int wait_ret = io_uring_wait_cqe(&ring, &cfe);
          ABSL_CHECK_EQ(wait_ret, 0)
              << "io_uring_wait_cqe failed: " << aos_strerror(-wait_ret);
        }
      }
    }
  }
}

void IoUringImpl::SubmitWakeupRead() {
  AsyncRead(event_fd.fd(),
            std::span<char>(reinterpret_cast<char *>(&event_fd.eventfd_buf),
                            sizeof(event_fd.eventfd_buf)),
            &event_fd.wakeup_req);
}

class EpollImpl : public Aio::Impl {
 public:
  EpollImpl();
  ~EpollImpl() override;

  void Run() override;
  bool Poll(bool block) override;
  void Quit() override;
  void Wakeup() override;

  void AsyncRead(int fd, std::span<char> buffer,
                 AsyncRequest *request) override;
  void AsyncWrite(int fd, std::span<const char> buffer,
                  AsyncRequest *request) override;
  void Cancel(AsyncRequest *request) override;
  void BeforeWait(std::function<void()> function) override;

  void OnReadable(int fd, std::function<void()> callback) override;
  void OnError(int fd, std::function<void()> callback) override;
  void OnWritable(int fd, std::function<void()> callback) override;
  void OnEvents(int fd, std::function<void(uint32_t)> callback) override;
  void DeleteFd(int fd) override;
  void ForgetClosedFd(int fd) override;
  void EnableWritable(int fd) override;
  void DisableWritable(int fd) override;
  void SetEvents(int fd, uint32_t events) override;

  void RegisterSignalFd(ipc_lib::SignalFd *sfd,
                        std::function<void()> callback) override;
  void UnregisterSignalFd(ipc_lib::SignalFd *sfd) override;

  void InitializeTimer(Aio::TimerState *state) override;
  void DestroyTimer(std::unique_ptr<Aio::TimerState> state) override;
  void ScheduleTimer(Aio::TimerState *state,
                     aos::monotonic_clock::time_point deadline,
                     aos::monotonic_clock::duration interval,
                     CompletionCallback callback, void *context) override;
  void CancelTimer(Aio::TimerState *state, bool reap) override;

 private:
  struct FdRegistration {
    int fd = -1;
    AsyncRequest *read_req = nullptr;
    AsyncRequest *write_req = nullptr;
    std::function<void()> in_fn = nullptr;
    std::function<void()> out_fn = nullptr;
    std::function<void()> err_fn = nullptr;
    std::function<void(uint32_t)> events_fn = nullptr;
    uint32_t events = 0;

    bool registered = false;
    uint32_t epoll_events = 0;
  };

  static constexpr uint32_t kIn = 0x01;
  static constexpr uint32_t kPri = 0x02;
  static constexpr uint32_t kOut = 0x04;
  static constexpr uint32_t kErr = 0x08;

  static constexpr uint32_t kInEvents = kIn | kPri;
  static constexpr uint32_t kOutEvents = kOut;
  static constexpr uint32_t kErrorEvents = kErr;

  FdRegistration *GetActiveRegistration(int fd) const;
  FdRegistration &GetOrCreateLegacyRegistration(int fd);
  FdRegistration &GetOrCreateAsyncRegistration(int fd);
  void ReleaseRegistration(FdRegistration *reg);
  void UpdateEpoll(int fd);
  void SubmitWakeupRead();
  void CancelRequest(AsyncRequest *request);

  bool remove_from_cancels(AsyncRequest *r) {
    AsyncRequest **curr = &pending_cancels_head;
    while (*curr != nullptr) {
      if (*curr == r) {
        *curr = State(*curr).link.next;
        return true;
      }
      curr = &State(*curr).link.next;
    }
    return false;
  }

  bool remove_from_sync(AsyncRequest *r) {
    AsyncRequest **curr = &pending_sync_completions_head;
    while (*curr != nullptr) {
      if (*curr == r) {
        *curr = State(*curr).link.next;
        return true;
      }
      curr = &State(*curr).link.next;
    }
    return false;
  }

  int epoll_fd_ = -1;
  EventFD event_fd_;
  std::vector<std::unique_ptr<FdRegistration>> registrations_;
  // If people really want to do heavy async IO, they should migrate to
  // io_uring.
  std::vector<std::unique_ptr<FdRegistration>> free_list_;
  size_t initial_pool_size_ = 16;
  std::vector<std::function<void()>> before_wait_functions_;

  bool run_ = false;
  bool quit_requested_ = false;

  AsyncRequest *pending_cancels_head = nullptr;
  AsyncRequest *pending_sync_completions_head = nullptr;
};

EpollImpl::EpollImpl() {
  epoll_fd_ = epoll_create1(EPOLL_CLOEXEC);
  ABSL_PCHECK(epoll_fd_ >= 0) << "Failed to create epoll instance";

  initial_pool_size_ = absl::GetFlag(FLAGS_aio_epoll_pool_size);
  free_list_.reserve(initial_pool_size_ * 2);
  for (size_t i = 0; i < initial_pool_size_; ++i) {
    free_list_.push_back(std::make_unique<FdRegistration>());
  }
  registrations_.reserve(initial_pool_size_ * 2);

  // When wakeup event fd read completes, re-schedule it.
  event_fd_.wakeup_req.callback = [](Completion completion, void *context) {
    auto *impl = static_cast<EpollImpl *>(context);
    if (aos::IsOk(completion.status)) {
      impl->SubmitWakeupRead();
    }
  };
  event_fd_.wakeup_req.context = this;

  SubmitWakeupRead();
}

EpollImpl::~EpollImpl() {
  run_ = false;
  if (epoll_fd_ >= 0) {
    close(epoll_fd_);
  }
}

EpollImpl::FdRegistration *EpollImpl::GetActiveRegistration(int fd) const {
  auto it = std::lower_bound(registrations_.begin(), registrations_.end(), fd,
                             [](const std::unique_ptr<FdRegistration> &reg,
                                int value) { return reg->fd < value; });
  if (it != registrations_.end() && (*it)->fd == fd) {
    return it->get();
  }
  return nullptr;
}

EpollImpl::FdRegistration &EpollImpl::GetOrCreateLegacyRegistration(int fd) {
  if (auto *reg = GetActiveRegistration(fd)) {
    return *reg;
  }
  auto new_reg = std::make_unique<FdRegistration>();
  auto *ptr = new_reg.get();
  ptr->fd = fd;
  auto it = std::lower_bound(registrations_.begin(), registrations_.end(), fd,
                             [](const std::unique_ptr<FdRegistration> &reg,
                                int value) { return reg->fd < value; });
  registrations_.insert(it, std::move(new_reg));
  return *ptr;
}

EpollImpl::FdRegistration &EpollImpl::GetOrCreateAsyncRegistration(int fd) {
  if (auto *reg = GetActiveRegistration(fd)) {
    return *reg;
  }
  ABSL_CHECK(!free_list_.empty()) << "Async registration pool exhausted";
  auto owned_reg = std::move(free_list_.back());
  free_list_.pop_back();
  auto *ptr = owned_reg.get();
  ptr->fd = fd;
  auto it = std::lower_bound(registrations_.begin(), registrations_.end(), fd,
                             [](const std::unique_ptr<FdRegistration> &reg,
                                int value) { return reg->fd < value; });
  registrations_.insert(it, std::move(owned_reg));
  return *ptr;
}

void EpollImpl::ReleaseRegistration(FdRegistration *reg) {
  auto it =
      std::lower_bound(registrations_.begin(), registrations_.end(), reg->fd,
                       [](const std::unique_ptr<FdRegistration> &r, int value) {
                         return r->fd < value;
                       });
  ABSL_CHECK(it != registrations_.end() && it->get() == reg);

  std::unique_ptr<FdRegistration> owned_reg = std::move(*it);
  registrations_.erase(it);

  owned_reg->fd = -1;
  owned_reg->read_req = nullptr;
  owned_reg->write_req = nullptr;
  owned_reg->in_fn = nullptr;
  owned_reg->out_fn = nullptr;
  owned_reg->err_fn = nullptr;
  owned_reg->events_fn = nullptr;
  owned_reg->events = 0;
  owned_reg->registered = false;
  owned_reg->epoll_events = 0;

  if (free_list_.size() < 2 * initial_pool_size_) {
    free_list_.push_back(std::move(owned_reg));
  }
}

void EpollImpl::Run() {
  if (quit_requested_) {
    quit_requested_ = false;
    return;
  }
  run_ = true;
  while (run_) {
    Poll(true);
  }
  quit_requested_ = false;
}

bool EpollImpl::Poll(bool block) {
  for (const auto &fn : before_wait_functions_) {
    fn();
  }

  bool processed = false;

  // Handle any pending cancels first to complete them.
  while (pending_cancels_head != nullptr) {
    auto *req = pending_cancels_head;
    pending_cancels_head = State(req).link.next;

    if (!req->done) {
      req->done = true;
      if (req->callback) {
        req->callback(Completion{aos::MakeError("Canceled"), 0, req->user_data},
                      req->context);
      }
      processed = true;
    }
  }

  // Process synchronous completions (e.g. invalid FDs).
  while (pending_sync_completions_head != nullptr) {
    auto *req = pending_sync_completions_head;
    pending_sync_completions_head = State(req).link.next;

    if (!req->done) {
      req->done = true;
      if (req->callback) {
        int64_t res = State(req).link.result;
        Completion completion;
        completion.user_data = req->user_data;
        if (res >= 0) {
          completion.status = aos::Ok();
          completion.result = static_cast<int32_t>(res);
        } else {
          completion.status = aos::MakeError("io_uring error");
          completion.result = static_cast<int32_t>(-res);
        }
        req->callback(completion, req->context);
      }
      processed = true;
    }
  }

  int timeout = (block && !processed) ? -1 : 0;

  struct epoll_event event;
  int num_events;
  do {
    num_events = epoll_wait(epoll_fd_, &event, 1, timeout);
  } while (num_events == -1 && errno == EINTR && block && !processed);

  if (num_events == -1) {
    if (errno == EINTR) {
      return processed;
    }
    ABSL_PCHECK(num_events != -1);
  }

  if (num_events > 0) {
    processed = true;
    auto *reg = static_cast<FdRegistration *>(event.data.ptr);
    uint32_t got_events = event.events;
    uint32_t events = 0;
    if (got_events & EPOLLIN) events |= kIn;
    if (got_events & EPOLLPRI) events |= kPri;
    if (got_events & EPOLLOUT) events |= kOut;
    if (got_events & EPOLLERR) events |= kErr;
    if (got_events & EPOLLHUP) events |= kErr;

    if (events & kInEvents) {
      if (reg->in_fn) {
        reg->in_fn();
      }
    }

    if (reg->fd != -1 && (events & kOutEvents)) {
      if (reg->out_fn) {
        reg->out_fn();
      }
    }

    if (reg->fd != -1 && (events & kErrorEvents)) {
      if (reg->err_fn) {
        reg->err_fn();
      }
    }

    if (reg->fd != -1 && reg->events_fn) {
      reg->events_fn(events);
    }
  }

  return processed;
}

void EpollImpl::Quit() {
  quit_requested_ = true;
  run_ = false;
  Wakeup();
}

void EpollImpl::Wakeup() { event_fd_.Write(); }

void EpollImpl::AsyncRead(int fd, std::span<char> buffer,
                          AsyncRequest *request) {
  request->done = false;
  if (fd < 0) {
    State(request).link.next = pending_sync_completions_head;
    State(request).link.result = -EBADF;
    pending_sync_completions_head = request;
    return;
  }
  auto &reg = GetOrCreateAsyncRegistration(fd);

  ABSL_CHECK(reg.events_fn == nullptr)
      << "Cannot mix OnEvents and AsyncRead/AsyncWrite on fd " << fd;
  ABSL_CHECK(reg.in_fn == nullptr)
      << "Cannot mix OnReadable and AsyncRead on fd " << fd;
  ABSL_CHECK(reg.read_req == nullptr) << "Duplicate AsyncRead on fd " << fd;
  reg.read_req = request;

  State(request).buffer.ptr = buffer.data();
  State(request).buffer.size = buffer.size();

  reg.in_fn = [this, fd]() {
    auto &r = GetOrCreateAsyncRegistration(fd);
    auto *req = r.read_req;
    if (!req) return;
    char *data = static_cast<char *>(State(req).buffer.ptr);
    size_t size = State(req).buffer.size;
    ssize_t res = read(fd, data, size);
    if (res >= 0 || (errno != EAGAIN && errno != EWOULDBLOCK)) {
      r.read_req = nullptr;
      r.in_fn = nullptr;
      req->done = true;
      UpdateEpoll(fd);
      if (req->callback) {
        Completion completion;
        completion.user_data = req->user_data;
        if (res >= 0) {
          completion.status = aos::Ok();
          completion.result = static_cast<int32_t>(res);
        } else {
          completion.status = aos::MakeError("io_uring error");
          completion.result = static_cast<int32_t>(errno);
        }
        req->callback(completion, req->context);
      }
    }
  };

  UpdateEpoll(fd);
}

void EpollImpl::AsyncWrite(int fd, std::span<const char> buffer,
                           AsyncRequest *request) {
  request->done = false;
  if (fd < 0) {
    State(request).link.next = pending_sync_completions_head;
    State(request).link.result = -EBADF;
    pending_sync_completions_head = request;
    return;
  }
  auto &reg = GetOrCreateAsyncRegistration(fd);

  ABSL_CHECK(reg.events_fn == nullptr)
      << "Cannot mix OnEvents and AsyncRead/AsyncWrite on fd " << fd;
  ABSL_CHECK(reg.out_fn == nullptr)
      << "Cannot mix OnWritable and AsyncWrite on fd " << fd;
  ABSL_CHECK(reg.write_req == nullptr) << "Duplicate AsyncWrite on fd " << fd;
  reg.write_req = request;

  State(request).buffer.ptr = const_cast<char *>(buffer.data());
  State(request).buffer.size = buffer.size();

  reg.out_fn = [this, fd]() {
    auto &r = GetOrCreateAsyncRegistration(fd);
    auto *req = r.write_req;
    if (!req) return;
    const char *data = static_cast<const char *>(State(req).buffer.ptr);
    size_t size = State(req).buffer.size;
    ssize_t res = write(fd, data, size);
    if (res >= 0 || (errno != EAGAIN && errno != EWOULDBLOCK)) {
      r.write_req = nullptr;
      r.out_fn = nullptr;
      req->done = true;
      UpdateEpoll(fd);
      if (req->callback) {
        Completion completion;
        completion.user_data = req->user_data;
        if (res >= 0) {
          completion.status = aos::Ok();
          completion.result = static_cast<int32_t>(res);
        } else {
          completion.status = aos::MakeError("io_uring error");
          completion.result = static_cast<int32_t>(errno);
        }
        req->callback(completion, req->context);
      }
    }
  };

  UpdateEpoll(fd);
}

void EpollImpl::Cancel(AsyncRequest *request) { CancelRequest(request); }

void EpollImpl::BeforeWait(std::function<void()> function) {
  before_wait_functions_.push_back(std::move(function));
}

void EpollImpl::OnReadable(int fd, std::function<void()> callback) {
  auto &reg = GetOrCreateLegacyRegistration(fd);
  ABSL_CHECK(!reg.events_fn)
      << "Cannot mix OnEvents and OnReadable for fd " << fd;
  ABSL_CHECK(reg.read_req == nullptr)
      << "Cannot mix OnReadable and AsyncRead on fd " << fd;
  if (reg.in_fn) {
    ABSL_CHECK(!callback) << "Duplicate in functions for " << fd;
  }
  reg.in_fn = std::move(callback);

  reg.events |= kInEvents;
  UpdateEpoll(fd);
}

void EpollImpl::OnError(int fd, std::function<void()> callback) {
  auto &reg = GetOrCreateLegacyRegistration(fd);
  ABSL_CHECK(!reg.events_fn) << "Cannot mix OnEvents and OnError for fd " << fd;
  if (reg.err_fn) {
    ABSL_CHECK(!callback) << "Duplicate error functions for " << fd;
  }
  reg.err_fn = std::move(callback);

  reg.events |= kErrorEvents;
  UpdateEpoll(fd);
}

void EpollImpl::OnWritable(int fd, std::function<void()> callback) {
  auto &reg = GetOrCreateLegacyRegistration(fd);
  ABSL_CHECK(!reg.events_fn)
      << "Cannot mix OnEvents and OnWritable for fd " << fd;
  ABSL_CHECK(reg.write_req == nullptr)
      << "Cannot mix OnWritable and AsyncWrite on fd " << fd;
  if (reg.out_fn) {
    ABSL_CHECK(!callback) << "Duplicate out functions for " << fd;
  }
  reg.out_fn = std::move(callback);

  reg.events |= kOutEvents;
  UpdateEpoll(fd);
}

void EpollImpl::OnEvents(int fd, std::function<void(uint32_t)> callback) {
  auto &reg = GetOrCreateLegacyRegistration(fd);
  ABSL_CHECK(reg.read_req == nullptr && reg.write_req == nullptr)
      << "Cannot mix OnEvents and AsyncRead/AsyncWrite on fd " << fd;
  ABSL_CHECK(!reg.in_fn && !reg.out_fn && !reg.err_fn)
      << "May not replace OnEvents handlers for fd " << fd;
  ABSL_CHECK(!reg.events_fn)
      << "May not replace OnEvents handlers for fd " << fd;
  reg.events_fn = std::move(callback);
}

void EpollImpl::DeleteFd(int fd) {
  auto *reg = GetActiveRegistration(fd);
  ABSL_CHECK(reg != nullptr) << "fd " << fd << " not found";

  if (reg->registered) {
    int ret = epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, fd, nullptr);
    ABSL_PCHECK(ret == 0 || errno == ENOENT)
        << "epoll_ctl DEL failed for fd " << fd;
  }

  ReleaseRegistration(reg);
}

void EpollImpl::ForgetClosedFd(int fd) {
  auto *reg = GetActiveRegistration(fd);
  ABSL_CHECK(reg != nullptr) << "fd " << fd << " not found";

  ReleaseRegistration(reg);
}

void EpollImpl::EnableWritable(int fd) {
  auto *reg = GetActiveRegistration(fd);
  ABSL_CHECK(reg != nullptr) << "fd " << fd << " not found";
  ABSL_CHECK(!reg->events_fn)
      << "EnableWritable is only for fds registered using OnWritable, not "
         "OnEvents";

  uint32_t new_events = reg->events | kOutEvents;
  if (reg->events != new_events) {
    reg->events = new_events;
    UpdateEpoll(fd);
  }
}

void EpollImpl::DisableWritable(int fd) {
  auto *reg = GetActiveRegistration(fd);
  ABSL_CHECK(reg != nullptr) << "fd " << fd << " not found";
  ABSL_CHECK(!reg->events_fn)
      << "DisableWritable is only for fds registered using OnWritable, not "
         "OnEvents";

  uint32_t new_events = reg->events & ~kOutEvents;
  if (reg->events != new_events) {
    reg->events = new_events;
    UpdateEpoll(fd);
  }
}

void EpollImpl::SetEvents(int fd, uint32_t events) {
  auto *reg = GetActiveRegistration(fd);
  ABSL_CHECK(reg != nullptr) << "fd " << fd << " not found";
  ABSL_CHECK(reg->events_fn)
      << "SetEvents is only for fds registered using OnEvents";

  if (reg->events != events) {
    reg->events = events;
    UpdateEpoll(fd);
  }
}

void EpollImpl::RegisterSignalFd(ipc_lib::SignalFd *sfd,
                                 std::function<void()> callback) {
  int fd = sfd->fd();
  OnReadable(fd, [fd, callback = std::move(callback)]() {
    struct signalfd_siginfo siginfo;
    while (true) {
      ssize_t res = read(fd, &siginfo, sizeof(siginfo));
      if (res == sizeof(siginfo)) {
        if (callback) callback();
      } else if (res < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        break;
      } else {
        ABSL_LOG(FATAL) << "Failed to read from signalfd: "
                        << aos_strerror(errno);
      }
    }
  });
}

void EpollImpl::UnregisterSignalFd(ipc_lib::SignalFd *sfd) {
  DeleteFd(sfd->fd());
}

void EpollImpl::InitializeTimer(Aio::TimerState *state) {
  state->timer_fd = std::make_unique<TimerFD>();
  state->request.done = true;

  OnReadable(state->timer_fd->fd(), [state]() {
    uint64_t buf;
    ssize_t result = read(state->timer_fd->fd(), &buf, sizeof(buf));
    if (result == -1 && errno == EAGAIN) {
      return;
    }
    ABSL_PCHECK(result == sizeof(buf));

    if (state->interval == aos::monotonic_clock::zero()) {
      state->request.done = true;
    }

    if (state->user_callback) {
      Completion completion;
      completion.user_data = state->request.user_data;
      completion.status = aos::Ok();
      completion.result = 0;
      state->user_callback(completion, state->user_context);
    }
  });
}

void EpollImpl::DestroyTimer(std::unique_ptr<Aio::TimerState> state) {
  CancelTimer(state.get(), true);
  DeleteFd(state->timer_fd->fd());
}

void EpollImpl::ScheduleTimer(Aio::TimerState *state,
                              aos::monotonic_clock::time_point deadline,
                              aos::monotonic_clock::duration interval,
                              CompletionCallback callback, void *context) {
  CancelTimer(state, true);

  state->interval = interval;
  state->deadline = deadline;
  state->user_callback = callback;
  state->user_context = context;
  state->request.user_data = state;
  state->request.done = false;

  struct itimerspec its;
  its.it_interval = ::aos::time::to_timespec(interval);
  its.it_value = ::aos::time::to_timespec(deadline);

  int ret =
      timerfd_settime(state->timer_fd->fd(), TFD_TIMER_ABSTIME, &its, nullptr);
  ABSL_PCHECK(ret == 0) << "timerfd_settime failed: " << aos_strerror(errno);
}

void EpollImpl::CancelTimer(Aio::TimerState *state, bool /*reap*/) {
  if (state->timer_fd) {
    struct itimerspec its;
    std::memset(&its, 0, sizeof(its));
    timerfd_settime(state->timer_fd->fd(), 0, &its, nullptr);
  }
  state->request.done = true;
  state->user_callback = nullptr;
}

void EpollImpl::UpdateEpoll(int fd) {
  auto *reg = GetActiveRegistration(fd);
  if (!reg) return;

  uint32_t desired_events = 0;
  if (reg->read_req) desired_events |= EPOLLIN;
  if (reg->write_req) desired_events |= EPOLLOUT;

  if (reg->events & kIn) desired_events |= EPOLLIN;
  if (reg->events & kPri) desired_events |= EPOLLPRI;
  if (reg->events & kOut) desired_events |= EPOLLOUT;
  if (reg->events & kErr) desired_events |= EPOLLERR;

  if (desired_events == 0) {
    if (reg->registered) {
      int ret = epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, fd, nullptr);
      ABSL_PCHECK(ret == 0 || errno == ENOENT)
          << "epoll_ctl DEL failed for fd " << fd;
      reg->registered = false;
      reg->epoll_events = 0;
    }
  } else {
    struct epoll_event ev;
    std::memset(&ev, 0, sizeof(ev));
    ev.events = desired_events;
    ev.data.ptr = reg;
    if (reg->registered) {
      if (reg->epoll_events != desired_events) {
        int ret = epoll_ctl(epoll_fd_, EPOLL_CTL_MOD, fd, &ev);
        ABSL_PCHECK(ret == 0) << "epoll_ctl MOD failed for fd " << fd;
        reg->epoll_events = desired_events;
      }
    } else {
      int ret = epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, fd, &ev);
      ABSL_PCHECK(ret == 0) << "epoll_ctl ADD failed for fd " << fd;
      reg->registered = true;
      reg->epoll_events = desired_events;
    }
  }
}

void EpollImpl::SubmitWakeupRead() {
  AsyncRead(event_fd_.fd(),
            std::span<char>(reinterpret_cast<char *>(&event_fd_.eventfd_buf),
                            sizeof(event_fd_.eventfd_buf)),
            &event_fd_.wakeup_req);
}

void EpollImpl::CancelRequest(AsyncRequest *request) {
  if (request->done) return;

  // Remove from pending lists if already there.
  remove_from_cancels(request);
  remove_from_sync(request);

  bool is_read = false;
  bool is_write = false;
  int found_fd = -1;
  for (const auto &reg : registrations_) {
    if (reg->read_req == request) {
      is_read = true;
      found_fd = reg->fd;
      break;
    }
    if (reg->write_req == request) {
      is_write = true;
      found_fd = reg->fd;
      break;
    }
  }

  if (is_read) {
    auto *reg = GetActiveRegistration(found_fd);
    if (reg) {
      reg->read_req = nullptr;
      reg->in_fn = nullptr;
      UpdateEpoll(found_fd);
    }
  } else if (is_write) {
    auto *reg = GetActiveRegistration(found_fd);
    if (reg) {
      reg->write_req = nullptr;
      reg->out_fn = nullptr;
      UpdateEpoll(found_fd);
    }
  }

  State(request).link.next = pending_cancels_head;
  pending_cancels_head = request;
}

Aio::Aio() {
  bool try_io_uring = ::absl::GetFlag(FLAGS_use_io_uring);
  if (try_io_uring) {
    struct io_uring r;
    int ret = io_uring_queue_init(8, &r, 0);
    if (ret == 0) {
      io_uring_queue_exit(&r);
      impl_ = std::make_unique<IoUringImpl>();
      return;
    } else {
      ABSL_LOG(WARNING) << "io_uring not supported by kernel (error " << ret
                        << ").  Falling back to epoll backend.";
    }
  }
  impl_ = std::make_unique<EpollImpl>();
}

Aio::~Aio() = default;

void Aio::Run() { impl_->Run(); }

bool Aio::Poll(bool block) { return impl_->Poll(block); }

void Aio::Quit() { impl_->Quit(); }

void Aio::Wakeup() { impl_->Wakeup(); }

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

Aio::Timer::Timer(Aio *aio) : state_(std::make_unique<Aio::TimerState>()) {
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

}  // namespace aos
