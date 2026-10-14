#ifndef AOS_EVENTS_AIO_READINESS_BACKEND_H_
#define AOS_EVENTS_AIO_READINESS_BACKEND_H_

#include <sys/types.h>

#include <functional>
#include <span>
#include <string>
#include <vector>

#include "aos/events/aio_internal.h"
#include "aos/events/aio_registrations.h"
#include "aos/events/aio_state.h"
#include "aos/events/file_descriptor.h"

namespace aos::internal {

// Everything a readiness-driven Aio backend does that is not a syscall.
//
// epoll, kqueue and IOCP all work the same way: keep a table of what each fd
// is registered for, ask the kernel which one is ready, and dispatch.  They
// differ only in how interest is expressed to the kernel and how the kernel
// answers.  That difference is the five virtuals below; everything else --
// the public entry points, the raw request machinery, the drain loop, the
// fork check, and the dispatch rules EPoll established -- lives here once.
//
// Sharing this is mostly about the subtle parts rather than the long ones.
// Independent copies of it have already drifted into three separate bugs:
// a hangup reaching the wrong handler, a use-after-free when a callback
// deletes its own fd, and a fork going unnoticed because one backend never
// registered the atfork handler.
//
// Completion delivery mirrors IoUringImpl's, so every backend resolves and
// delivers requests at the same points:
//
//   * Submit does no I/O.  AsyncRead()/AsyncWrite() queue the request, and
//     the harvest makes its first attempt in submission order -- what
//     io_uring's kernel does with its SQ: try inline, poll-arm on EAGAIN.
//     Only a request that got EAGAIN is armed on the kernel.
//   * Resolving is eager.  One Poll() harvests everything that is ready --
//     every raw AsyncRead/AsyncWrite whose fd the kernel reports (the I/O
//     itself runs right there), every submit-time result, every Cancel() --
//     sets `done`, and queues the completion on ready_.  That is io_uring's
//     CQ drain.  A raw I/O can make another raw request ready (a pipe's write
//     filling its read end), so the harvest keeps asking, non-blocking, until
//     nothing more is ready -- io_uring runs both in the same submit.
//   * Delivery is rationed: one user callback per Poll(), in FIFO order, off
//     ready_ -- io_uring's pending_dispatch_.  The next harvest only happens
//     once ready_ is empty, which is also what keeps this fair: a callback
//     that re-arms itself on a request that resolves at once (a regular
//     file) lands behind everything the previous harvest found, timers and
//     legacy fds included, instead of in front of them.
//
// Legacy handlers are the one thing the harvest cannot resolve early, since
// dispatching them *is* running user code.  The harvest stops at the first
// one and queues a marker in ready_ in its place; delivering the marker
// dispatches one legacy event -- io_uring's legacy-epoll poll CQE and
// DrainLegacyEpoll(), the same shape.
class ReadinessBackend : public Aio::Impl {
 public:
  ~ReadinessBackend() override;

  void Run() final;
  bool should_run() const final;
  void Quit() final;
  bool Poll(bool block) final;

  void AsyncRead(FileDescriptor fd, std::span<char> buffer,
                 AsyncRequest *request) final;
  void AsyncWrite(FileDescriptor fd, std::span<const char> buffer,
                  AsyncRequest *request) final;
  void Cancel(AsyncRequest *request) final;
  void BeforeWait(std::function<void()> function) final;

  void OnReadable(FileDescriptor fd, std::function<void()> callback) final;
  void OnError(FileDescriptor fd, std::function<void()> callback) final;
  void OnWritable(FileDescriptor fd, std::function<void()> callback) final;
  void OnEvents(FileDescriptor fd,
                std::function<void(uint32_t)> callback) final;
  void DeleteFd(FileDescriptor fd) final;
  void ForgetClosedFd(FileDescriptor fd) final;
  void EnableWritable(FileDescriptor fd) final;
  void DisableWritable(FileDescriptor fd) final;
  void SetEvents(FileDescriptor fd, uint32_t events) final;

  bool HasRawRequestsInFlight() const final;

 protected:
  // backend_name is what this backend calls itself in operational-failure
  // messages ("epoll", "kqueue", "iocp"); aio_test asserts on it.
  ReadinessBackend(size_t pool_size, std::string backend_name);

  // --- The seam.  Everything below is a syscall or close to one. ---

  // Arms or disarms reg's kernel registration to match what it is now
  // interested in: its legacy mask, and its Armed() raw requests.  False
  // means the kernel refused to add a raw registration, in which case the
  // caller completes the armed requests with an error rather than waiting
  // for an event that cannot come.  Every other failure dies: an fd closed
  // with a request armed on it is a contract violation (see
  // Aio::AsyncRead()), and this is where a backend can see one.
  virtual bool UpdateRegistration(FdRegistration *reg) = 0;

  // Drops reg's kernel registration entirely.  Strict, as
  // EPoll::DoEpollCtl() is: the fd having been closed already means it isn't
  // ours any more, and the number may belong to someone else --
  // ForgetClosedFd() is the API for that, and for a raw request the caller
  // must Cancel() and wait for the callback before closing.
  virtual void RemoveRegistration(FdRegistration *reg) = 0;

  enum class WaitResult {
    // Nothing was ready, or the wait was interrupted.
    kNothing,
    // The backend handled something itself -- a kqueue timer or signal
    // filter, which epoll delivers as an ordinary fd instead -- and ran a
    // callback.  That is this Poll()'s one.
    kHandled,
    // The backend handled something itself but ran no callback: a timer event
    // for a deadline that has not arrived, or a signal with no receiver
    // registered to take it.  Internal work, so the caller keeps going.
    kInternal,
    // *out is filled in.
    kFdReady,
  };
  struct FdReady {
    FdRegistration *reg = nullptr;
    // The readiness bits, already translated to kIn/kPri/kOut/kErr and
    // already masked down to what the caller subscribed to.
    uint32_t events = 0;
    // Whether the condition was terminal -- a hangup or an error.  A raw
    // request has to observe one by running, since it cannot be consumed.
    bool terminal = false;
  };
  // Waits for at most one event.  Readiness is level-triggered and a
  // reported fd goes to the back of the kernel's ready list, so asking
  // repeatedly rotates through every ready fd.
  virtual WaitResult WaitForOne(bool block, FdReady *out) = 0;

  // Rebuilds the kernel object in a forked child.  Called with the
  // registration table intact; the caller re-arms every registration
  // afterwards through UpdateRegistration().
  virtual void RecreateKernelState() = 0;

  // Runs after RecreateKernelState() and ReArmAllRegistrations(), for work a
  // backend can only do once the table is armed on the new kernel object --
  // EpollImpl replaces its inherited timerfds here, which registers new fds
  // that the re-arm above must not try to add a second time.
  virtual void AfterForkReArm() {}

  // Breaks a blocked WaitForOne() out.  Reached from Quit(), which is
  // async-signal-safe, so this must be too.
  virtual void WakeLoop() = 0;

  // The fd the loop's own wakeup read is on, so it can be told apart from a
  // caller's request.
  virtual FileDescriptor wakeup_fd() const = 0;
  // What owns fd if the loop made it for itself, or nullptr for a caller's
  // registration.  The wakeup here; a backend adds its own timers and
  // receiver, which it registers like any other fd.
  virtual const char *OwnFdName(FileDescriptor fd);
  // In a forked child, before the rebuild creates any descriptor: dies if any
  // fd the loop made for itself was closed (CheckOwnFdOpenAfterFork()).  The
  // wakeup here; a backend adds its timers and receiver, checked directly
  // rather than by asking OwnFdName() of every registration.
  virtual void CheckOwnFdsOpenAfterFork();

  // Called before a registration's first raw I/O.  The I/O runs inside
  // Poll(), and readiness does not promise room for the whole buffer, so a
  // backend whose DoRawIo() has no per-call way to avoid blocking sets
  // O_NONBLOCK here, as libuv's uv_poll_init() does.  See Aio::AsyncRead().
  virtual void PrepareRawFd(FileDescriptor /*fd*/) {}

  // One non-blocking read or write for a raw request: read(2) or write(2) by
  // default, which PrepareRawFd() made non-blocking.  EpollImpl overrides it
  // to avoid touching the fd's flags.
  virtual ssize_t DoRawIo(FdRegistration *reg,
                          FdRegistration::Direction direction, void *data,
                          size_t size);

  // --- Shared state the backends still need to reach. ---
  FdRegistrationTable registrations_;
  const std::string backend_name_;

  // Registers an fd whose readiness is the loop's own business -- the
  // wakeup.  The harvest runs `drain` as internal work, which neither spends
  // Poll()'s one-callback budget nor counts as the caller's registration,
  // and it comes from the legacy list rather than the raw pool.
  void OnInternalReadable(FileDescriptor fd, std::function<void()> drain);

  // Re-arms every live registration on a freshly rebuilt kernel object.
  void ReArmAllRegistrations();
  // The common half of the forked-child rebuild.
  void HandleForkInternal();
  // Rebuilds if the process forked since we last looked.  Every public
  // entry point calls this first, so a forked child that submits work
  // before its first Poll() operates on live kernel state.
  void CheckForFork();

  // True for all of Poll(), including the scrub of retired registrations
  // that runs after the dispatch guard drops: that scrub destroys
  // std::functions, and their captures' destructors are user code too.
  // Backs Poll()'s reentrancy CHECK, like IoUringImpl's in_poll_.
  bool in_poll_ = false;
  bool in_before_wait_ = false;
  std::vector<std::function<void()>> before_wait_functions_;

  // Quit() writes these from other threads and from a signal handler
  // (ShmEventLoop's SIGINT/SIGHUP/SIGTERM handler), and should_run() reads
  // them; plain bools would be a data race that can miss the shutdown
  // outright.  A handler may only touch lock-free atomics, so assert that;
  // a non-lock-free atomic could deadlock against the interrupted thread.
  static_assert(std::atomic<bool>::is_always_lock_free,
                "Quit() runs in a signal handler, so these have to be usable "
                "from one");
  // Starts true, so should_run() is true before the first Run().  Run()
  // clears it on exit, Quit() on shutdown.
  std::atomic<bool> run_ = true;
  std::atomic<bool> quit_requested_ = false;

  size_t last_fork_count_ = 0;

  // Both request lists are doubly linked through AioState::link.  A request
  // is on at most one, and link.queued says which and why.
  struct PendingLinkTraits {
    static AsyncRequest *&next(AsyncRequest *request) {
      return State(request).link.next;
    }
    static AsyncRequest *&prev(AsyncRequest *request) {
      return State(request).link.prev;
    }
  };
  // Which of the two lists a request is on, and why.  Kept in
  // AioState::link.queued, a raw int32_t because io_uring shares the field;
  // Kind()/SetKind() are the only places that convert.
  enum class QueueKind : int32_t {
    kNone = 0,
    // On pending_: submitted, I/O not attempted yet (done still false).  The
    // request already holds its registration's slot; link.result holds the
    // fd until the attempt replaces it with the outcome.  See SubmitRaw().
    kPendingSubmitRead,
    kPendingSubmitWrite,
    // On pending_: outcome known, not resolved yet (done still false).
    // link.result holds the result or -errno.
    kPendingResult,
    // On pending_: Cancel()ed.  AioState::raw.ptr holds the FdRegistration
    // whose claim this holds (nullptr if it was not armed here), since the
    // buffer is no longer needed.
    kPendingCancelRead,
    kPendingCancelWrite,
    // On ready_: resolved (done set), callback not run yet.
    kReadyResult,
    kReadyCanceled,
    // On ready_: legacy_marker_, standing in for one legacy event.
    kReadyLegacy,
  };
  static QueueKind Kind(const AsyncRequest *request) {
    return static_cast<QueueKind>(State(request).link.queued);
  }
  static void SetKind(AsyncRequest *request, QueueKind kind) {
    State(request).link.queued = static_cast<int32_t>(kind);
  }
  static bool IsPendingSubmit(QueueKind kind) {
    return kind == QueueKind::kPendingSubmitRead ||
           kind == QueueKind::kPendingSubmitWrite;
  }
  static bool IsPendingCancel(QueueKind kind) {
    return kind == QueueKind::kPendingCancelRead ||
           kind == QueueKind::kPendingCancelWrite;
  }
  static bool IsPending(QueueKind kind) {
    return IsPendingSubmit(kind) || kind == QueueKind::kPendingResult ||
           IsPendingCancel(kind);
  }
  static bool IsReady(QueueKind kind) {
    return kind == QueueKind::kReadyResult || kind == QueueKind::kReadyCanceled;
  }
  // Whether request is linked on list.  O(n); only reached on paths that
  // are already errors or rare.
  static bool ListContains(
      const IntrusiveDoublyLinkedList<AsyncRequest, PendingLinkTraits> &list,
      const AsyncRequest *request);
  // What submit and Cancel() queued since the last Poll(), in that order.
  // Harvest() resolves first attempts and submit-time failures first, in
  // order, then the cancels in theirs, as io_uring posts one submitted
  // batch's CQEs.
  IntrusiveDoublyLinkedList<AsyncRequest, PendingLinkTraits> pending_;
  // FIFO of resolved requests awaiting their callback -- io_uring's
  // pending_dispatch_.
  IntrusiveDoublyLinkedList<AsyncRequest, PendingLinkTraits> ready_;
  // See QueueKind::kReadyLegacy.  Never armed; only its link is used.
  AsyncRequest legacy_marker_;
  // Queues a submitted request, or one whose outcome is already known -- a
  // submit-time error, a Cancel() -- for the next harvest, which takes them
  // in this order.  `kind` is one of the QueueKind::kPending* values.
  void QueuePending(AsyncRequest *request, QueueKind kind);
  // Sets done and, if it has a callback to deliver, moves request onto
  // ready_.  `kind` is kReadyResult or kReadyCanceled.
  void QueueReady(AsyncRequest *request, QueueKind kind);

  using Direction = FdRegistration::Direction;
  // reg's request in `direction` if it is armed -- waiting on the kernel --
  // and nullptr if there is none or it has not had its first attempt yet.
  // What UpdateRegistration() arms for.
  static AsyncRequest *Armed(FdRegistration *reg, Direction direction) {
    AsyncRequest *req = reg->slot(direction);
    return req != nullptr && Kind(req) == QueueKind::kNone ? req : nullptr;
  }
  // The raw request path.  AsyncRead() and AsyncWrite() differ only in
  // direction.
  void SubmitRaw(FileDescriptor fd, void *data, size_t size,
                 AsyncRequest *request, Direction direction);
  // Performs reg's pending I/O in one direction.  Returns whether the
  // request resolved (false: EAGAIN, still waiting).
  bool TryRawIo(FdRegistration *reg, Direction direction);
  // A submitted request's first attempt, taken off pending_ in submission
  // order: the I/O now if it can complete, else armed on the kernel --
  // io_uring's own order (try inline, poll-arm on EAGAIN).  Returns whether
  // it resolved; arming is not processing anything.
  bool StartRaw(AsyncRequest *request, Direction direction);
  // Services a harvested event on a raw registration.  Returns whether any
  // request resolved.
  bool ServiceRaw(const FdReady &ready);
  // After a raw registration's requests or claims change: retires it if
  // nothing is left, otherwise updates the kernel to match what is.
  void SettleRawRegistration(FdRegistration *reg);
  // Resolves every armed request on reg with the errno UpdateRegistration()
  // left in reg->registration_errno.
  void FailRawRegistration(FdRegistration *reg);

  enum class HarvestResult {
    kNothing,
    // Something was resolved or seen.
    kProcessed,
    // The backend ran a callback of its own (WaitResult::kHandled), which is
    // this Poll()'s one.
    kCallbackRan,
  };
  // Resolves pending_, then waits (or doesn't) for the kernel and resolves
  // what it reports, stopping at the first legacy event.
  HarvestResult Harvest(bool block, FdReady *harvested);
  // Everything Poll() does inside its dispatch guard.
  bool PollOnce(bool block);
  // Delivers the legacy marker: one legacy fd's event, per
  // EPoll::DoCallbacks().  Returns false if none is ready any more.
  bool DispatchLegacyEvent(FdReady *harvested);
  // Runs the handlers for one legacy event.
  void RunLegacyHandlers(const FdReady &ready);

  // UpdateRegistration(), fatal if it refuses the fd.  For every caller with
  // no request to hand the refusal to: a registration the kernel is not
  // holding never fires again, so the alternative is an fd that silently
  // stops waking.  The raw paths are the exception -- they have a request,
  // and turn a refusal into the error completion aio.h promises
  // (FailRawRegistration()).
  void UpdateRegistrationOrDie(FdRegistration *reg);
  // Dies if request is already armed, or armed and not yet delivered, on
  // this loop.  See the definition.
  void CheckNotAlreadyInFlight(AsyncRequest *request) const;
  // Dies if the kernel reported an event for a registration that is no
  // longer live.  DeleteFd() and a raw registration's retirement remove the
  // kernel registration first, strictly, so neither leaves one behind.
  // ForgetClosedFd() cannot: its fd is already closed.  kqueue drops a
  // closed fd's registrations itself, but epoll watches the file, and only
  // forgets it when the last reference closes -- one still open through a
  // dup() or a fork() keeps the entry, and its next event lands here.
  static void CheckLive(const FdRegistration *reg);
};

}  // namespace aos::internal

#endif  // AOS_EVENTS_AIO_READINESS_BACKEND_H_
