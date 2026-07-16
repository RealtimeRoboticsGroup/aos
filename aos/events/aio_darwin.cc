#include <errno.h>
#include <fcntl.h>
#include <mach/mach_time.h>
#include <poll.h>
#include <stdio.h>
#include <sys/event.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <set>
#include <unordered_map>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "absl/numeric/int128.h"

#include "aos/events/aio.h"
#include "aos/events/aio_internal.h"
#include "aos/ipc_lib/signalfd.h"
#include "aos/libc/aos_strerror.h"
#include "aos/time/time.h"

ABSL_FLAG(size_t, aio_epoll_pool_size, 16,
          "Initial size of the pre-allocated epoll FdRegistration pool.");

ABSL_FLAG(uint32_t, aio_queue_depth, 1024,
          "Depth of the io_uring submission and completion queues.");

ABSL_FLAG(bool, use_io_uring, false,
          "Whether to use the io_uring backend (always false on macOS).");

namespace aos {

class KqueueImpl;

namespace {

// AtForkHandler handles process forks. Open kqueue file descriptors are not
// valid inside the child process, and their associated event registrations must
// be reset to keep them operational.
//
// By registering prepare, parent, and child handlers via pthread_atfork(), we
// intercept forks and recreate the kqueue instance for the child,
// re-registering all active signals and timers.
class AtForkHandler {
 public:
  static AtForkHandler *Instance() {
    static AtForkHandler *instance = new AtForkHandler();
    return instance;
  }

  void RegisterKqueueImpl(KqueueImpl *impl) {
    std::lock_guard<std::mutex> lock(mutex_);
    impls_.insert(impl);
  }

  void UnregisterKqueueImpl(KqueueImpl *impl) {
    std::lock_guard<std::mutex> lock(mutex_);
    impls_.erase(impl);
  }

  void ResetAll();

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

  std::mutex mutex_;
  std::set<KqueueImpl *> impls_;
};

// RAII wrapper around standard pipe file descriptors to simulate Linux eventfd.
//
// macOS lacks support for eventfd(). Since Aio needs a way to asynchronously
// wake up a blocking event loop (for example, to interrupt a sleeping kevent()
// call when Aio::Quit() is called from another thread), we use a pipe.
//
// The read end of the pipe is registered with kqueue. When Quit() is invoked,
// it triggers Wakeup() which writes a single byte to the write end of the pipe,
// forcing kevent() to wake up immediately so the event loop thread can check
// should_run() and terminate clean.
class EventFD {
 public:
  EventFD() {
    int pipefd[2];
    ABSL_PCHECK(pipe(pipefd) == 0) << "Failed to create pipe";
    ABSL_PCHECK(fcntl(pipefd[0], F_SETFL, O_NONBLOCK) == 0);
    ABSL_PCHECK(fcntl(pipefd[1], F_SETFL, O_NONBLOCK) == 0);
    ABSL_PCHECK(fcntl(pipefd[0], F_SETFD, FD_CLOEXEC) == 0);
    ABSL_PCHECK(fcntl(pipefd[1], F_SETFD, FD_CLOEXEC) == 0);
    fd_ = pipefd[0];
    write_fd_ = pipefd[1];
  }

  ~EventFD() {
    if (fd_ >= 0) {
      close(fd_);
    }
    if (write_fd_ >= 0) {
      close(write_fd_);
    }
  }

  EventFD(const EventFD &) = delete;
  EventFD &operator=(const EventFD &) = delete;

  int fd() const { return fd_; }
  int write_fd() const { return write_fd_; }

  void Write() {
    char val = 1;
    ssize_t ret = write(write_fd_, &val, sizeof(val));
    if (ret < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      ABSL_LOG(FATAL) << "Failed to write to pipe: " << aos_strerror(errno);
    }
  }

  uint64_t eventfd_buf = 0;
  AsyncRequest wakeup_req;

 private:
  int fd_ = -1;
  int write_fd_ = -1;
};

// Internal helper union overlayed on top of AsyncRequest's opaque state buffer
// (`internal_state`).
//
// To prevent exporting backend-specific types in the public `aio.h` header,
// AsyncRequest provides a generic 16-byte buffer. This union maps it to:
// - `timespec`: Used for absolute/relative deadline timepoints.
// - `buffer`: Stores transient buffer pointer and size for simulated
// read/write.
// - `link`: Tracks next/result fields for completed queues or cancellations.
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

// Converts a monotonic clock timepoint to Mach absolute time ticks.
// kevent timer filters (`EVFILT_TIMER`) on macOS require deadline ticks
// when specified with `NOTE_MACHTIME` and `NOTE_ABSOLUTE`.
uint64_t ToMachTicks(aos::monotonic_clock::time_point time) {
  static mach_timebase_info_data_t timebase_info = []() {
    mach_timebase_info_data_t info;
    mach_timebase_info(&info);
    return info;
  }();

  uint64_t nanos = std::chrono::nanoseconds(time.time_since_epoch()).count();
  return static_cast<uint64_t>((absl::uint128(nanos) * timebase_info.denom) /
                               timebase_info.numer);
}

}  // namespace

class KqueueImpl;

struct KqueueTimerState : public Aio::TimerState {
  explicit KqueueTimerState(KqueueImpl *impl) : impl_(impl) {}
  ~KqueueTimerState() override;

  void Initialize() override;
  void Schedule(aos::monotonic_clock::time_point deadline,
                aos::monotonic_clock::duration interval,
                CompletionCallback callback, void *context) override;
  void Cancel(bool reap) override;

 private:
  KqueueImpl *impl_;
};

// KqueueImpl implements the Aio::Impl interface using macOS kqueue.
//
// Since kqueue is readiness-based, KqueueImpl emulates completion-based I/O
// (AsyncRead/AsyncWrite) by:
// 1. Stashing user buffers/contexts in the opaque AsyncRequest state.
// 2. Registering read/write filters on kqueue.
// 3. Performing the actual read() or write() system call inside kqueue callback
//    when the file descriptor is ready.
// 4. Executing the user completion callback.
//
// To satisfy real-time safety constraints, KqueueImpl uses a pre-allocated pool
// of FdRegistration objects, avoiding dynamic memory allocation on hot paths.
class KqueueImpl : public Aio::Impl {
  friend struct KqueueTimerState;

 public:
  KqueueImpl();
  ~KqueueImpl() override;

  std::unique_ptr<Aio::TimerState> MakeTimerState() override;

  void Run() override;
  bool Poll(bool block) override;
  void Quit() override;
  bool should_run() const override;
  void Wakeup();

  void AsyncRead(int fd, std::span<char> buffer,
                 AsyncRequest *request) override;
  void AsyncWrite(int fd, std::span<const char> buffer,
                  AsyncRequest *request) override;
  void Cancel(AsyncRequest *request) override;
  void BeforeWait(std::function<void()> function) override;

  // Legacy Readiness API implementations (used by the Epoll interface class).
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
  void ConsumeSignalFd(ipc_lib::SignalFd *sfd) override;

  void ResetOnFork();

 private:
  // Tracks active event handlers and request status for a registered file
  // descriptor.
  struct FdRegistration {
    int fd = -1;
    // Pointers to currently active completion-based asynchronous requests.
    AsyncRequest *read_req = nullptr;
    AsyncRequest *write_req = nullptr;

    // Callbacks for legacy readiness-based handlers.
    std::function<void()> in_fn = nullptr;
    std::function<void()> out_fn = nullptr;
    std::function<void()> err_fn = nullptr;
    std::function<void(uint32_t)> events_fn = nullptr;
    uint32_t events = 0;

    // kqueue registration tracking state.
    bool registered = false;
    uint32_t epoll_events = 0;
  };

  // Readiness bitfield flags mapped to mock epoll event structures.
  static constexpr uint32_t kIn = 0x01;
  static constexpr uint32_t kPri = 0x02;
  static constexpr uint32_t kOut = 0x04;
  static constexpr uint32_t kErr = 0x08;

  static constexpr uint32_t kInEvents = kIn | kPri;
  static constexpr uint32_t kOutEvents = kOut;
  static constexpr uint32_t kErrorEvents = kErr;

  // Returns the registration for fd if it is currently registered, or nullptr.
  FdRegistration *GetActiveRegistration(int fd) const;

  // Retrieves the registration for fd, creating a new one from the
  // pre-allocated pool if not present.
  FdRegistration &GetOrCreateLegacyRegistration(int fd);
  FdRegistration &GetOrCreateAsyncRegistration(int fd);

  // Returns a registration to the pool when it is fully cleaned up and no
  // longer needed.
  void ReleaseRegistration(FdRegistration *reg);

  // Syncs registration's requested events (epoll_events) with the actual kqueue
  // filters.
  void UpdateKqueue(int fd);

  // Registers the wakeup pipe read filter with kqueue.
  void SubmitWakeupRead();

  // Cancels a pending request, moving it to the pending cancels queue.
  void CancelRequest(AsyncRequest *request);

  bool RemoveFromCancels(AsyncRequest *r) {
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

  bool RemoveFromSync(AsyncRequest *r) {
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

  int kqueue_fd_ = -1;
  EventFD event_fd_;

  // Pre-allocated registration pool to guarantee no-allocation on hot paths.
  std::vector<std::unique_ptr<FdRegistration>> registrations_;
  std::vector<std::unique_ptr<FdRegistration>> free_list_;
  size_t initial_pool_size_ = 16;
  std::vector<std::function<void()>> before_wait_functions_;

  bool run_ = false;
  bool quit_requested_ = false;

  // Linked list heads for delayed cancel/sync callbacks executed outside the
  // Wait.
  AsyncRequest *pending_cancels_head = nullptr;
  AsyncRequest *pending_sync_completions_head = nullptr;

  struct SignalRegistration {
    ipc_lib::SignalFd *sfd = nullptr;
    int signal_number = -1;
    std::function<void()> callback = nullptr;
  };
  std::unordered_map<int, SignalRegistration> signals_;

  // Double-linked list of scheduled timers.
  void InsertActiveTimer(Aio::TimerState *state);
  void RemoveActiveTimer(Aio::TimerState *state);
  Aio::TimerState *active_timers_head_ = nullptr;
};

namespace {
void AtForkHandler::ResetAll() {
  for (KqueueImpl *impl : impls_) {
    impl->ResetOnFork();
  }
}
}  // namespace

std::unique_ptr<Aio::TimerState> KqueueImpl::MakeTimerState() {
  return std::make_unique<KqueueTimerState>(this);
}

KqueueImpl::KqueueImpl() {
  kqueue_fd_ = kqueue();
  ABSL_PCHECK(kqueue_fd_ >= 0) << "Failed to create kqueue instance";
  ABSL_PCHECK(fcntl(kqueue_fd_, F_SETFD, FD_CLOEXEC) == 0);

  initial_pool_size_ = absl::GetFlag(FLAGS_aio_epoll_pool_size);
  free_list_.reserve(initial_pool_size_ * 2);
  for (size_t i = 0; i < initial_pool_size_; ++i) {
    free_list_.push_back(std::make_unique<FdRegistration>());
  }
  registrations_.reserve(initial_pool_size_ * 2);

  // When wakeup event fd read completes, re-schedule it.
  event_fd_.wakeup_req.callback = [](Completion completion, void *context) {
    auto *impl = static_cast<KqueueImpl *>(context);
    if (aos::IsOk(completion.status)) {
      impl->SubmitWakeupRead();
    }
  };
  event_fd_.wakeup_req.context = this;

  AtForkHandler::Instance()->RegisterKqueueImpl(this);

  SubmitWakeupRead();
}

KqueueImpl::~KqueueImpl() {
  AtForkHandler::Instance()->UnregisterKqueueImpl(this);
  run_ = false;
  if (kqueue_fd_ >= 0) {
    close(kqueue_fd_);
  }
}

void KqueueImpl::InsertActiveTimer(Aio::TimerState *state) {
  if (state->is_active) return;
  state->is_active = true;
  state->next_active = active_timers_head_;
  state->prev_active = nullptr;
  if (active_timers_head_) {
    active_timers_head_->prev_active = state;
  }
  active_timers_head_ = state;
}

void KqueueImpl::RemoveActiveTimer(Aio::TimerState *state) {
  if (!state->is_active) return;
  state->is_active = false;
  if (state->prev_active) {
    state->prev_active->next_active = state->next_active;
  } else {
    active_timers_head_ = state->next_active;
  }
  if (state->next_active) {
    state->next_active->prev_active = state->prev_active;
  }
  state->prev_active = nullptr;
  state->next_active = nullptr;
}

void KqueueImpl::ResetOnFork() {
  close(kqueue_fd_);
  kqueue_fd_ = kqueue();
  ABSL_PCHECK(kqueue_fd_ >= 0) << "Failed to recreate kqueue on fork";
  ABSL_PCHECK(fcntl(kqueue_fd_, F_SETFD, FD_CLOEXEC) == 0);

  // Re-register all signals
  for (const auto &pair : signals_) {
    struct kevent sig_ev;
    EV_SET(&sig_ev, pair.first, EVFILT_SIGNAL, EV_ADD | EV_ENABLE, 0, 0, NULL);
    ABSL_PCHECK(kevent(kqueue_fd_, &sig_ev, 1, NULL, 0, NULL) == 0);
  }

  // Re-register all active timers
  Aio::TimerState *timer = active_timers_head_;
  while (timer != nullptr) {
    if (!timer->request.done) {
      struct kevent ev;
      uint64_t mach_ticks = ToMachTicks(timer->deadline);
      EV_SET(&ev, reinterpret_cast<uintptr_t>(timer), EVFILT_TIMER,
             EV_ADD | EV_ENABLE | EV_ONESHOT, NOTE_MACHTIME | NOTE_ABSOLUTE,
             mach_ticks, timer);
      ABSL_PCHECK(kevent(kqueue_fd_, &ev, 1, NULL, 0, NULL) == 0);
    }
    timer = timer->next_active;
  }

  // Re-register all active fds
  for (const auto &reg : registrations_) {
    if (!reg) continue;
    reg->registered = false;
    reg->epoll_events = 0;
    UpdateKqueue(reg->fd);
  }
}

KqueueImpl::FdRegistration *KqueueImpl::GetActiveRegistration(int fd) const {
  auto it = std::lower_bound(registrations_.begin(), registrations_.end(), fd,
                             [](const std::unique_ptr<FdRegistration> &reg,
                                int value) { return reg->fd < value; });
  if (it != registrations_.end() && (*it)->fd == fd) {
    return it->get();
  }
  return nullptr;
}

KqueueImpl::FdRegistration &KqueueImpl::GetOrCreateLegacyRegistration(int fd) {
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

KqueueImpl::FdRegistration &KqueueImpl::GetOrCreateAsyncRegistration(int fd) {
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

void KqueueImpl::ReleaseRegistration(FdRegistration *reg) {
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

void KqueueImpl::Run() {
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

bool KqueueImpl::Poll(bool block) {
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

  // Process synchronous completions.
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
  struct kevent event;
  struct timespec timeout;
  timeout.tv_sec = 0;
  timeout.tv_nsec = 0;
  struct timespec *timeout_ptr = (block && !processed) ? nullptr : &timeout;
  int num_events;
  do {
    num_events = kevent(kqueue_fd_, NULL, 0, &event, 1, timeout_ptr);
  } while (num_events == -1 && errno == EINTR && block && !processed);

  if (num_events == -1) {
    if (errno == EINTR) {
      return processed;
    }
    ABSL_PCHECK(num_events != -1);
  }

  if (num_events > 0) {
    processed = true;
    if (event.filter == EVFILT_TIMER) {
      auto *state = static_cast<Aio::TimerState *>(event.udata);
      if (state && !state->request.done) {
        if (state->interval == aos::monotonic_clock::zero()) {
          state->request.done = true;
          RemoveActiveTimer(state);
        } else {
          state->deadline += state->interval;
          struct kevent arm_ev;
          uint64_t mach_ticks = ToMachTicks(state->deadline);
          EV_SET(&arm_ev, reinterpret_cast<uintptr_t>(state), EVFILT_TIMER,
                 EV_ADD | EV_ENABLE | EV_ONESHOT, NOTE_MACHTIME | NOTE_ABSOLUTE,
                 mach_ticks, state);
          ABSL_PCHECK(kevent(kqueue_fd_, &arm_ev, 1, NULL, 0, NULL) == 0);
        }
        if (state->user_callback) {
          Completion completion;
          completion.user_data = state->request.user_data;
          completion.status = aos::Ok();
          completion.result = 0;
          state->user_callback(completion, state->user_context);
        }
      }
    } else if (event.filter == EVFILT_SIGNAL) {
      int sig_num = event.ident;
      auto it = signals_.find(sig_num);
      if (it != signals_.end() && it->second.callback) {
        it->second.callback();
      }
    } else {
      auto *reg = static_cast<FdRegistration *>(event.udata);
      uint32_t got_events = 0;
      if (event.filter == EVFILT_READ) {
        got_events |= kIn;
        if (event.flags & EV_EOF) {
          got_events |= kErr;
        }
      } else if (event.filter == EVFILT_WRITE) {
        got_events |= kOut;
        if (event.flags & EV_EOF) {
          got_events |= kErr;
        }
      }
      if (event.flags & EV_ERROR) {
        got_events |= kErr;
      }

      uint32_t events = 0;
      if (got_events & kIn) events |= kIn;
      if (got_events & kOut) events |= kOut;
      if (got_events & kErr) events |= kErr;

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
  }

  return processed;
}

void KqueueImpl::Quit() {
  quit_requested_ = true;
  run_ = false;
  Wakeup();
}

bool KqueueImpl::should_run() const { return run_ && !quit_requested_; }

void KqueueImpl::Wakeup() { event_fd_.Write(); }

void KqueueImpl::AsyncRead(int fd, std::span<char> buffer,
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
      UpdateKqueue(fd);
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

  UpdateKqueue(fd);
}

void KqueueImpl::AsyncWrite(int fd, std::span<const char> buffer,
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
      UpdateKqueue(fd);
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

  UpdateKqueue(fd);
}

void KqueueImpl::Cancel(AsyncRequest *request) { CancelRequest(request); }

void KqueueImpl::BeforeWait(std::function<void()> function) {
  before_wait_functions_.push_back(std::move(function));
}

void KqueueImpl::OnReadable(int fd, std::function<void()> callback) {
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
  UpdateKqueue(fd);
}

void KqueueImpl::OnError(int fd, std::function<void()> callback) {
  auto &reg = GetOrCreateLegacyRegistration(fd);
  ABSL_CHECK(!reg.events_fn) << "Cannot mix OnEvents and OnError for fd " << fd;
  if (reg.err_fn) {
    ABSL_CHECK(!callback) << "Duplicate error functions for " << fd;
  }
  reg.err_fn = std::move(callback);

  reg.events |= kErrorEvents;
  UpdateKqueue(fd);
}

void KqueueImpl::OnWritable(int fd, std::function<void()> callback) {
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
  UpdateKqueue(fd);
}

void KqueueImpl::OnEvents(int fd, std::function<void(uint32_t)> callback) {
  auto &reg = GetOrCreateLegacyRegistration(fd);
  ABSL_CHECK(reg.read_req == nullptr && reg.write_req == nullptr)
      << "Cannot mix OnEvents and AsyncRead/AsyncWrite on fd " << fd;
  ABSL_CHECK(!reg.in_fn && !reg.out_fn && !reg.err_fn)
      << "May not replace OnEvents handlers for fd " << fd;
  ABSL_CHECK(!reg.events_fn)
      << "May not replace OnEvents handlers for fd " << fd;
  reg.events_fn = std::move(callback);
}

void KqueueImpl::DeleteFd(int fd) {
  auto *reg = GetActiveRegistration(fd);
  ABSL_CHECK(reg != nullptr) << "fd " << fd << " not found";

  if (reg->registered) {
    struct kevent events[2];
    int n = 0;
    if (reg->epoll_events & (kIn | kPri | kErr)) {
      EV_SET(&events[n++], fd, EVFILT_READ, EV_DELETE, 0, 0, NULL);
    }
    if (reg->epoll_events & (kOut | kErr)) {
      EV_SET(&events[n++], fd, EVFILT_WRITE, EV_DELETE, 0, 0, NULL);
    }
    if (n > 0) {
      kevent(kqueue_fd_, events, n, NULL, 0, NULL);
    }
  }

  ReleaseRegistration(reg);
}

void KqueueImpl::ForgetClosedFd(int fd) {
  auto *reg = GetActiveRegistration(fd);
  ABSL_CHECK(reg != nullptr) << "fd " << fd << " not found";

  ReleaseRegistration(reg);
}

void KqueueImpl::EnableWritable(int fd) {
  auto *reg = GetActiveRegistration(fd);
  ABSL_CHECK(reg != nullptr) << "fd " << fd << " not found";
  ABSL_CHECK(!reg->events_fn) << "EnableWritable is only for fds registered "
                                 "using OnWritable, not OnEvents";

  uint32_t new_events = reg->events | kOutEvents;
  if (reg->events != new_events) {
    reg->events = new_events;
    UpdateKqueue(fd);
  }
}

void KqueueImpl::DisableWritable(int fd) {
  auto *reg = GetActiveRegistration(fd);
  ABSL_CHECK(reg != nullptr) << "fd " << fd << " not found";
  ABSL_CHECK(!reg->events_fn) << "DisableWritable is only for fds registered "
                                 "using OnWritable, not OnEvents";

  uint32_t new_events = reg->events & ~kOutEvents;
  if (reg->events != new_events) {
    reg->events = new_events;
    UpdateKqueue(fd);
  }
}

void KqueueImpl::SetEvents(int fd, uint32_t events) {
  auto *reg = GetActiveRegistration(fd);
  ABSL_CHECK(reg != nullptr) << "fd " << fd << " not found";
  ABSL_CHECK(reg->events_fn)
      << "SetEvents is only for fds registered using OnEvents";

  if (reg->events != events) {
    reg->events = events;
    UpdateKqueue(fd);
  }
}

void KqueueImpl::RegisterSignalFd(ipc_lib::SignalFd *sfd,
                                  std::function<void()> callback) {
  int sig_num = ipc_lib::kWakeupSignal;
  auto [it, inserted] = signals_.try_emplace(sig_num);
  ABSL_CHECK(inserted) << "Duplicate signal registration for sfd " << sfd;

  it->second.sfd = sfd;
  it->second.signal_number = sig_num;
  it->second.callback = std::move(callback);

  struct kevent ev;
  EV_SET(&ev, sig_num, EVFILT_SIGNAL, EV_ADD | EV_ENABLE, 0, 0, NULL);
  ABSL_PCHECK(kevent(kqueue_fd_, &ev, 1, NULL, 0, NULL) == 0);
}

void KqueueImpl::UnregisterSignalFd(ipc_lib::SignalFd *sfd) {
  int sig_num = ipc_lib::kWakeupSignal;
  auto it = signals_.find(sig_num);
  ABSL_CHECK(it != signals_.end() && it->second.sfd == sfd)
      << "SignalFd not found";

  struct kevent ev;
  EV_SET(&ev, sig_num, EVFILT_SIGNAL, EV_DELETE, 0, 0, NULL);
  kevent(kqueue_fd_, &ev, 1, NULL, 0, NULL);

  signals_.erase(it);
}

void KqueueImpl::ConsumeSignalFd(ipc_lib::SignalFd *sfd) {
  (void)sfd;
  // kqueue automatically consumes the signal event when returning it.
}

KqueueTimerState::~KqueueTimerState() { Cancel(true); }

void KqueueTimerState::Initialize() { request.done = true; }

void KqueueTimerState::Schedule(aos::monotonic_clock::time_point deadline,
                                aos::monotonic_clock::duration interval,
                                CompletionCallback callback, void *context) {
  ABSL_CHECK_GE(deadline, aos::monotonic_clock::epoch());
  Cancel(true);

  this->interval = interval;
  this->deadline = deadline;
  this->user_callback = callback;
  this->user_context = context;
  this->request.user_data = this;
  this->request.done = false;
  impl_->InsertActiveTimer(this);

  struct kevent ev;
  uint64_t mach_ticks = ToMachTicks(deadline);
  EV_SET(&ev, reinterpret_cast<uintptr_t>(this), EVFILT_TIMER,
         EV_ADD | EV_ENABLE | EV_ONESHOT, NOTE_MACHTIME | NOTE_ABSOLUTE,
         mach_ticks, this);

  ABSL_PCHECK(kevent(impl_->kqueue_fd_, &ev, 1, NULL, 0, NULL) == 0);
}

void KqueueTimerState::Cancel(bool /*reap*/) {
  struct kevent ev;
  EV_SET(&ev, reinterpret_cast<uintptr_t>(this), EVFILT_TIMER, EV_DELETE, 0, 0,
         NULL);
  kevent(impl_->kqueue_fd_, &ev, 1, NULL, 0, NULL);

  impl_->RemoveActiveTimer(this);
  request.done = true;
  user_callback = nullptr;
}

void KqueueImpl::UpdateKqueue(int fd) {
  auto *reg = GetActiveRegistration(fd);
  if (!reg) return;

  uint32_t desired_events = 0;
  if (reg->read_req) desired_events |= kIn;
  if (reg->write_req) desired_events |= kOut;

  if (reg->events & kIn) desired_events |= kIn;
  if (reg->events & kPri) desired_events |= kIn;
  if (reg->events & kOut) desired_events |= kOut;
  if (reg->events & kErr) desired_events |= kErr;

  bool want_read = (desired_events & kIn) || (desired_events & kErr);
  bool has_read = reg->registered &&
                  ((reg->epoll_events & kIn) || (reg->epoll_events & kErr));

  if (want_read != has_read) {
    struct kevent ev;
    if (want_read) {
      int flags = EV_ADD | EV_ENABLE;
      if (!(desired_events & kIn) && (desired_events & kErr)) {
        flags |= EV_CLEAR;
      }
      EV_SET(&ev, fd, EVFILT_READ, flags, 0, 0, reg);
    } else {
      EV_SET(&ev, fd, EVFILT_READ, EV_DELETE, 0, 0, NULL);
    }
    ABSL_PCHECK(kevent(kqueue_fd_, &ev, 1, NULL, 0, NULL) == 0);
  }

  bool want_write = (desired_events & kOut) || (desired_events & kErr);
  bool has_write = reg->registered &&
                   ((reg->epoll_events & kOut) || (reg->epoll_events & kErr));

  if (want_write != has_write) {
    struct kevent ev;
    if (want_write) {
      int flags = EV_ADD | EV_ENABLE;
      if (!(desired_events & kOut) && (desired_events & kErr)) {
        flags |= EV_CLEAR;
      }
      EV_SET(&ev, fd, EVFILT_WRITE, flags, 0, 0, reg);
    } else {
      EV_SET(&ev, fd, EVFILT_WRITE, EV_DELETE, 0, 0, NULL);
    }
    ABSL_PCHECK(kevent(kqueue_fd_, &ev, 1, NULL, 0, NULL) == 0);
  }

  reg->registered = (want_read || want_write);
  reg->epoll_events = desired_events;
}

void KqueueImpl::SubmitWakeupRead() {
  AsyncRead(event_fd_.fd(),
            std::span<char>(reinterpret_cast<char *>(&event_fd_.eventfd_buf),
                            sizeof(event_fd_.eventfd_buf)),
            &event_fd_.wakeup_req);
}

void KqueueImpl::CancelRequest(AsyncRequest *request) {
  if (request->done) return;

  RemoveFromCancels(request);
  RemoveFromSync(request);

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
      UpdateKqueue(found_fd);
    }
  } else if (is_write) {
    auto *reg = GetActiveRegistration(found_fd);
    if (reg) {
      reg->write_req = nullptr;
      reg->out_fn = nullptr;
      UpdateKqueue(found_fd);
    }
  }

  State(request).link.next = pending_cancels_head;
  pending_cancels_head = request;
}

Aio::Aio() { impl_ = std::make_unique<KqueueImpl>(); }
}  // namespace aos
