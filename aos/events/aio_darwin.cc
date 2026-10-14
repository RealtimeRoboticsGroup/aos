#include "aos/events/aio.h"

#include <errno.h>
#include <fcntl.h>
#include <mach/mach_time.h>
#include <poll.h>
#include <pthread.h>
#include <stdio.h>
#include <sys/event.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "absl/numeric/int128.h"

#include "aos/events/aio_internal.h"
#include "aos/events/aio_readiness_backend.h"
#include "aos/events/aio_registrations.h"
#include "aos/events/aio_state.h"
#include "aos/events/aio_unix.h"
#include "aos/events/pipe.h"
#include "aos/events/timer_queue.h"
#include "aos/ipc_lib/thread_signal.h"
#include "aos/libc/aos_strerror.h"
#include "aos/realtime.h"
#include "aos/time/time.h"

ABSL_FLAG(size_t, aio_pool_size, 16,
          "Initial size of the pre-allocated internal::FdRegistration pool.");

ABSL_FLAG(uint32_t, aio_queue_depth, 1024,
          "Depth of the io_uring submission and completion queues.");

ABSL_FLAG(std::string, aio_backend, "kqueue",
          "Which Aio backend to use.  macOS has only \"kqueue\"; any other "
          "name is fatal, as an unknown one is on every platform.");

namespace aos {

// The shared readiness encoding, as aio_linux.cc pulls it in too.
using internal::kErr;
using internal::kErrorEvents;
using internal::kIn;
using internal::kInEvents;
using internal::kOut;
using internal::kOutEvents;
using internal::kPri;

class KqueueImpl;

namespace {

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
  // Rounded up, so the knote never fires before the deadline it was armed for.
  // Truncating let it come back up to one tick early, where the "is it
  // actually due" guard below rejects it and re-arms -- an extra kevent()
  // round trip per firing, for nothing.
  const absl::uint128 ticks =
      (absl::uint128(nanos) * timebase_info.denom + timebase_info.numer - 1) /
      timebase_info.numer;
  // kevent()'s data field is an intptr_t.  No timebase in use today maps a
  // representable time past it, but a deadline as far out as
  // monotonic_clock::max_time would wrap negative under one that did; clamp
  // instead, since that deadline never arrives either way.
  constexpr absl::uint128 kMaxTicks =
      static_cast<uint64_t>(std::numeric_limits<intptr_t>::max());
  return static_cast<uint64_t>(std::min(ticks, kMaxTicks));
}

}  // namespace

class KqueueImpl;

struct KqueueTimerState : public Aio::TimerState {
  explicit KqueueTimerState(KqueueImpl *impl) : impl_(impl) {}
  ~KqueueTimerState() override;

  // Owned by KqueueImpl::active_timers_ -- see TimerQueue, which is the only
  // thing allowed to touch them.  They live here rather than on
  // Aio::TimerState because prev_active/next_active there are the Linux
  // backends' intrusive list, and a timer is only ever on one backend.
  KqueueTimerState *left = nullptr;
  KqueueTimerState *right = nullptr;
  KqueueTimerState *parent = nullptr;
  bool red = false;
  uint64_t sequence = 0;
  // Whether this timer is counted in KqueueImpl::live_timers_.  Set by
  // Initialize(), cleared by the destructor, so the count cannot drift if a
  // state is ever built without being initialized.
  bool counted_live = false;

  void Initialize() override;
  void Schedule(aos::monotonic_clock::time_point deadline,
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
// of internal::FdRegistration objects, avoiding dynamic memory allocation on
// hot paths.
class KqueueImpl : public internal::ReadinessBackend {
  friend struct KqueueTimerState;

 public:
  KqueueImpl();
  ~KqueueImpl() override;

  std::unique_ptr<Aio::TimerState> MakeTimerState() override;

  // Legacy Readiness API implementations (used by the Epoll interface class).
  void RegisterThreadSignalReceiver(ipc_lib::ThreadSignalReceiver *receiver,
                                    std::function<void()> callback) override;
  void UnregisterThreadSignalReceiver(
      ipc_lib::ThreadSignalReceiver *receiver) override;
  void ConsumeThreadSignalReceiver(
      ipc_lib::ThreadSignalReceiver *receiver) override;

 protected:
  // --- internal::ReadinessBackend's seam, in terms of kevent(). ---
  bool UpdateRegistration(internal::FdRegistration *reg) override;
  void RemoveRegistration(internal::FdRegistration *reg) override;
  WaitResult WaitForOne(bool block, FdReady *out) override;
  void RecreateKernelState() override;
  void WakeLoop() override;
  FileDescriptor wakeup_fd() const override { return wakeup_pipe_->read_fd(); }
  void PrepareRawFd(FileDescriptor fd) override {
    internal::SetNonBlocking(fd);
  }

 private:
  // Tracks active event handlers and request status for a registered file
  // descriptor.

  // The readiness bits (kIn/kOut, never kErr) this registration actually
  // subscribed to, through On*()/SetEvents() or a raw request.
  //
  // UpdateRegistration() arms filters from this, and Poll() masks what the
  // kernel reports back down to it.  Both need the same answer: kqueue has no
  // error-only filter, so watching for errors on the write side means arming
  // a whole EVFILT_WRITE, and that filter also reports writability nobody
  // asked for.  epoll never hands back an unsubscribed readiness bit, so
  // Poll() has to drop those to stay a faithful mirror of it.
  static uint32_t DesiredReadiness(internal::FdRegistration *reg) {
    uint32_t desired = 0;
    if (Armed(reg, Direction::kRead) != nullptr) desired |= kIn;
    if (Armed(reg, Direction::kWrite) != nullptr) desired |= kOut;
    // kPri widens to kIn: kqueue cannot tell priority data apart, so a kPri
    // subscriber gets the read filter and has to be told when it fires.
    if (reg->events & kInEvents) desired |= kIn;
    if (reg->events & kOutEvents) desired |= kOut;
    return desired;
  }

  // Syncs registration's requested events (epoll_events) with the actual kqueue
  // filters.
  // Returns false if fd turned out to be closed (kevent reported EBADF), in
  // which case nothing ends up registered for it.
  // Reconciles reg's kqueue filters with what it is now interested in.
  // Takes the registration rather than an fd, as EPoll::DoEpollCtl() does:
  // every caller has just looked it up.

  // Registers the wakeup pipe read filter with kqueue.  Both the constructor
  // and RecreateKernelState() -- which replaces the pipe after a fork -- go
  // through here, so the handler is written once.
  void ArmWakeupRead();

  // Cancels a pending request, moving it to the pending cancels queue.
  void CancelRequest(AsyncRequest *request);

  int kqueue_fd_ = -1;
  // The wakeup pipe.  Quit()/Wakeup() write a byte to it from any thread --
  // and from ShmEventLoop's signal handler -- to break Poll() out of
  // kevent(); a legacy OnReadable() handler drains the writes.  macOS has no
  // eventfd(2), so a self-pipe it is.
  //
  // Never empty.  It is an optional so that RecreateKernelState() can destroy
  // and reconstruct it in place, which is the whole of replacing a pipe:
  // ~Pipe() closes both ends and the constructor opens a fresh pair.
  std::optional<Pipe> wakeup_pipe_{std::in_place};
  // Somewhere for the wakeup read to land.  Never examined -- the write is
  // the whole signal.
  char wakeup_buf_[8] = {};

  // Linked list heads for delayed cancel/sync callbacks executed outside the
  // Wait.

  // The one ThreadSignalReceiver this loop may have -- aio.h permits exactly
  // one at a time, and it is always watching kWakeupSignal.
  ipc_lib::ThreadSignalReceiver *signal_receiver_ = nullptr;
  std::function<void()> signal_callback_ = nullptr;
  // A wakeup that has been observed but not yet handed to a callback --
  // because none was registered when it arrived, or because a fork lost it.
  // Held until one is, which is what makes a successor receiver own the
  // wakeups its predecessor never consumed; the signalfd backends get that
  // for free, since an unregistered signalfd simply stops being polled and
  // keeps whatever is queued in it.
  //
  // The fork case is the other setter, and the harder one.  A signal that
  // arrives between the fork and the rebuild is unrecoverable on macOS:
  // the receiver leaves kWakeupSignal at SIG_IGN, so the kernel discards it
  // on delivery rather than leaving it pending, and EVFILT_SIGNAL only counts
  // what is delivered while its knote exists -- and after a fork the whole
  // kqueue, knote included, is gone.  Linux has no such window; its signalfd
  // is an ordinary fd that survives the fork with anything already queued.
  //
  // RecreateKernelState() cannot tell whether a wakeup was actually missed,
  // so it assumes one was.  These wakeups are explicitly allowed to be
  // spurious (see thread_signal.h), while a dropped one hangs the loop.
  bool signal_wakeup_pending_ = false;
  // Whether the EVFILT_SIGNAL knote is on the kqueue.  Deliberately not the
  // same question as "is a receiver registered": the knote is left armed
  // across an unregister, because deleting it destroys the pending count with
  // it and that count is what a successor receiver is owed.
  bool signal_knote_armed_ = false;
  // True while the callback is on the stack.  Unregistering from inside it
  // must not clear the std::function that is executing; Poll() finishes the
  // job once the frame has unwound.
  bool signal_in_callback_ = false;
  bool signal_clear_pending_ = false;
  // Runs the receiver's callback with the reentrancy guard that
  // UnregisterThreadSignalReceiver()'s deferred path depends on.  Both
  // delivery paths go through here: a second copy of this that forgets to set
  // signal_in_callback_ lets an unregister from inside the callback destroy
  // the std::function that is executing, which is a use-after-free on its
  // captures rather than anything the CHECKs would catch.
  void DispatchSignalCallback() {
    if (!signal_callback_) {
      return;
    }
    signal_in_callback_ = true;
    signal_callback_();
    signal_in_callback_ = false;
    if (signal_clear_pending_) {
      ClearSignalRegistration();
    }
  }
  void ClearSignalRegistration() {
    signal_receiver_ = nullptr;
    signal_callback_ = nullptr;
    // signal_wakeup_pending_ is deliberately not cleared: an undelivered
    // wakeup outlives the receiver that failed to consume it.
    signal_in_callback_ = false;
    signal_clear_pending_ = false;
  }

  void InsertActiveTimer(KqueueTimerState *state);
  void RemoveActiveTimer(KqueueTimerState *state);
  // Every armed timer, in dispatch order: front() is both the deadline to wait
  // for and the timer to hand over when it arrives.
  //
  // The order is the contract.  Both Linux backends give a timer its own
  // timerfd and let the kernel sort them, and callers rely on the result: a
  // callback for the earlier deadline may cancel, reschedule or destroy a
  // later timer whose firing is already queued.  kqueue cannot be asked for
  // the same thing -- see UpdateTimerKnote() -- so it is maintained here.
  struct ActiveTimerTraits {
    static KqueueTimerState *&left(KqueueTimerState *state) {
      return state->left;
    }
    static KqueueTimerState *&right(KqueueTimerState *state) {
      return state->right;
    }
    static KqueueTimerState *&parent(KqueueTimerState *state) {
      return state->parent;
    }
    static bool &red(KqueueTimerState *state) { return state->red; }
    static uint64_t &sequence(KqueueTimerState *state) {
      return state->sequence;
    }
    static aos::monotonic_clock::time_point deadline(
        const KqueueTimerState *state) {
      return state->deadline;
    }
  };
  TimerQueue<KqueueTimerState, ActiveTimerTraits> active_timers_;

  // Points the one EVFILT_TIMER knote at active_timers_.front(), or removes it
  // when there is nothing left to wait for.  Idempotent, so every path that
  // can change the front just calls it.
  void UpdateTimerKnote();
  // The knote's ident.  Any constant will do -- kqueue keys a knote by
  // (ident, filter), so this cannot collide with the fd-numbered
  // EVFILT_READ/EVFILT_WRITE knotes.
  static constexpr uintptr_t kTimerIdent = 1;
  // Timers that exist, armed or not -- active_timers_ holds only the armed
  // ones, and the destructor's check is about the Timer object's lifetime.
  int live_timers_ = 0;
  // What UpdateTimerKnote() last armed, so it can skip a syscall when the
  // front deadline has not moved.  EV_ONESHOT means a firing disarms it.
  bool timer_armed_ = false;
  aos::monotonic_clock::time_point armed_deadline_ =
      aos::monotonic_clock::epoch();
};

std::unique_ptr<Aio::TimerState> KqueueImpl::MakeTimerState() {
  return std::make_unique<KqueueTimerState>(this);
}

KqueueImpl::KqueueImpl()
    : internal::ReadinessBackend(absl::GetFlag(FLAGS_aio_pool_size), "kqueue") {
  kqueue_fd_ = kqueue();
  ABSL_PCHECK(kqueue_fd_ >= 0) << "Failed to create kqueue instance";
  ABSL_PCHECK(fcntl(kqueue_fd_, F_SETFD, FD_CLOEXEC) == 0);

  internal::RegisterForkCounters();
  last_fork_count_ = internal::ForkCount();

  ArmWakeupRead();
}

// Not a persistent AsyncRead: that version re-armed from its own completion,
// inside dispatch, where the slot it just retired cannot be scrubbed yet -- so
// each wakeup took a fresh one, and running the pool dry turned any wakeup
// into "Async registration pool exhausted".  Level-triggered needs no re-arm,
// it comes from the legacy list rather than the pool, and draining it is
// internal work that does not spend a Poll() (see OnInternalReadable()).
// HandleForkInternal()'s ReArmAllRegistrations() covers it across a fork like
// any other -- on the replacement pipe, which RecreateKernelState() has
// already put in place by then.
void KqueueImpl::ArmWakeupRead() {
  OnInternalReadable(wakeup_pipe_->read_fd(), [this]() {
    // Drain every queued wakeup; the write is the whole signal, the bytes
    // are never examined.  Non-blocking, so this ends on EAGAIN.
    while (read(wakeup_pipe_->read_fd(), wakeup_buf_, sizeof(wakeup_buf_)) >
           0) {
    }
  });
}

KqueueImpl::~KqueueImpl() {
  // Registrations a Poll() retired are reclaimed at the end of that Poll(),
  // so this is normally a no-op.  Done first regardless: a parked
  // registration's captures (a handler which deleted its own fd and owned a
  // Timer, say) must be destroyed before the CHECKs below count what is
  // still alive.  Same as ~EpollImpl().
  registrations_.ScrubRetired();
  // Owner-facing state must be gone first, as EPoll::~EPoll() CHECKed and
  // aio.h documents.  A live Aio::Timer's destructor dereferences this impl,
  // so one outliving its Aio is a use-after-free.  Same set of checks as
  // ~EpollImpl().
  // Every live timer, not just the armed ones.  ~Aio::Timer() reaches through
  // Aio::impl_ to destroy its state, so a Timer outliving its Aio is a
  // use-after-free whether or not it was ever scheduled -- active_timers_
  // cannot answer this, because it holds only what is armed.  aio_linux.cc's
  // active_timers_ counts every timer for the same reason, which is why this
  // is what makes TimerOutlivesAioDeathTest agree across backends.
  ABSL_CHECK_EQ(live_timers_, 0)
      << ": An Aio::Timer must be destroyed before its Aio";
  // A receiver left behind by an unregister that happened inside its own
  // callback is already retired -- Poll() just never ran again to clear it.
  ABSL_CHECK(signal_receiver_ == nullptr || signal_clear_pending_)
      << ": The ThreadSignalReceiver must be unregistered before destroying "
         "the Aio";
  // Only the internal wakeup read and async-only registrations may remain:
  // aio.h's constraint 2 permits destroying the Aio with raw requests still
  // pending.  Caller fd registrations must be gone, as EPoll always CHECKed.
  for (const auto &reg : registrations_) {
    if (!reg) continue;
    ABSL_CHECK(reg->internal || reg->async_only)
        << ": fd " << reg->fd
        << " must be removed (DeleteFd()/ForgetClosedFd()) before destroying "
           "the Aio";
  }
  // Cleared here rather than left to their destructors so that the
  // std::functions a parked registration still holds are destroyed while this
  // impl is intact -- a capture's destructor can reenter it.
  run_ = false;
  if (kqueue_fd_ >= 0) {
    close(kqueue_fd_);
  }
}

void KqueueImpl::InsertActiveTimer(KqueueTimerState *state) {
  if (state->is_active) return;
  state->is_active = true;
  active_timers_.Insert(state);
  UpdateTimerKnote();
}

void KqueueImpl::RemoveActiveTimer(KqueueTimerState *state) {
  if (!state->is_active) return;
  state->is_active = false;
  active_timers_.Remove(state);
  UpdateTimerKnote();
}

// Points the one timer knote at the earliest deadline.  (A knote -- kqueue's
// own term, from kevent(2) -- is one registered filter: the kernel-side note
// created by EV_ADD and reported back by kevent().)
//
// One knote rather than one per timer, because per-timer knotes let the
// kernel pick which of several simultaneously-due timers to report, and its
// pick is the reverse of the order they were scheduled in.  ADR 0002 has the
// full reasoning, including why it cannot be corrected after the fact.
void KqueueImpl::UpdateTimerKnote() {
  const KqueueTimerState *const front = active_timers_.front();
  if (front == nullptr) {
    if (timer_armed_) {
      struct kevent ev;
      EV_SET(&ev, kTimerIdent, EVFILT_TIMER, EV_DELETE, 0, 0, nullptr);
      // ENOENT just means the one-shot already fired and took itself off.
      kevent(kqueue_fd_, &ev, 1, nullptr, 0, nullptr);
      timer_armed_ = false;
    }
    return;
  }
  if (timer_armed_ && armed_deadline_ == front->deadline) {
    return;
  }
  struct kevent ev;
  EV_SET(&ev, kTimerIdent, EVFILT_TIMER, EV_ADD | EV_ENABLE | EV_ONESHOT,
         NOTE_MACHTIME | NOTE_ABSOLUTE, ToMachTicks(front->deadline), nullptr);
  ABSL_PCHECK(kevent(kqueue_fd_, &ev, 1, nullptr, 0, nullptr) == 0)
      << ": Failed to arm the timer knote";
  timer_armed_ = true;
  armed_deadline_ = front->deadline;
}

void KqueueImpl::WakeLoop() {
  char val = 1;
  // Quit() reaches this from ShmEventLoop's SIGINT/SIGHUP/SIGTERM handler, so
  // everything here has to be async-signal-safe.  write() is; Pipe::Write()
  // is not, because it PCHECKs, which is why this one is by hand.
  // Overwriting errno is not safe either: we interrupted a thread which may be
  // partway through checking its own errno, so put back whatever was there.
  const int saved_errno = errno;
  const ssize_t ret = write(wakeup_pipe_->write_fd(), &val, sizeof(val));
  const int write_errno = errno;
  errno = saved_errno;
  if (ret < 0 && write_errno != EAGAIN && write_errno != EWOULDBLOCK) {
    // ABSL_RAW_LOG formats into a stack buffer and writes the result out
    // directly, where ABSL_LOG would allocate and take locks.  The errno goes
    // out as a bare number for the same reason -- aos_strerror() formats
    // through thread_local storage with snprintf().
    ABSL_RAW_LOG(FATAL, "Failed to write to the wakeup pipe: errno %d",
                 write_errno);
  }
}

// A kqueue is not inherited across fork(2) at all -- unlike an epoll fd,
// which is inherited and merely shared -- so there is nothing to salvage and
// the child builds a new one.  Detection is the shared counter; see
// aos/events/aio_unix.h.
void KqueueImpl::RecreateKernelState() {
  close(kqueue_fd_);
  kqueue_fd_ = kqueue();
  ABSL_PCHECK(kqueue_fd_ >= 0) << "Failed to recreate kqueue on fork";
  ABSL_PCHECK(fcntl(kqueue_fd_, F_SETFD, FD_CLOEXEC) == 0);

  // The wakeup pipe is inherited across the fork like any other descriptor,
  // and sharing one with the parent is the same defect the timerfds have on
  // Linux: the drain in ArmWakeupRead()'s handler consumes every queued byte,
  // so a Quit() the parent wrote for itself gets eaten by the child and the
  // parent stays blocked in kevent().
  //
  // Replaced here, before the blanket re-arm, rather than after it the way
  // EpollImpl::AfterForkReArm() must: an EPOLL_CTL_ADD of an fd the re-arm
  // already added fails with EEXIST, so Linux cannot put the replacement in
  // early.  EV_ADD on an existing filter updates it in place instead, so
  // arming the new pipe now and having ReArmAllRegistrations() add it a
  // second time costs nothing -- and the fd the re-arm walks to is then the
  // replacement rather than the parent's.
  //
  // Released directly rather than through DeleteFd(), because there is
  // nothing left to detach: the old registration only ever existed on the
  // kqueue closed just above.  Not through ForgetClosedFd() either, which is
  // the caller's API and refuses the loop's own descriptors.  It has to come
  // before the pipe is replaced, since pipe(2) may hand back the very
  // descriptor numbers being freed.
  internal::FdRegistration *wakeup_reg =
      registrations_.GetActive(wakeup_pipe_->read_fd());
  ABSL_CHECK(wakeup_reg != nullptr);
  registrations_.Release(wakeup_reg);
  wakeup_pipe_.emplace();
  ArmWakeupRead();

  // Only the filters that are not fd registrations are this function's job.
  // ReadinessBackend::HandleForkInternal() calls ReArmAllRegistrations()
  // immediately after this returns, which re-arms every readable/writable fd
  // through UpdateRegistration().  Signals and timers are kqueue filters
  // rather than registrations, so nothing else would rebuild them.
  if (signal_knote_armed_) {
    struct kevent sig_ev;
    EV_SET(&sig_ev, ipc_lib::kWakeupSignal, EVFILT_SIGNAL, EV_ADD | EV_ENABLE,
           0, 0, NULL);
    ABSL_PCHECK(kevent(kqueue_fd_, &sig_ev, 1, NULL, 0, NULL) == 0);
    // Nothing tells us whether a wakeup landed in the window this rebuild
    // closes, so assume one did -- see signal_wakeup_pending_.
    signal_wakeup_pending_ = true;
  }

  // Re-arm the timers.  Only one knote to rebuild however many are armed,
  // since active_timers_ is ordered and only its front is ever waited on.
  timer_armed_ = false;
  UpdateTimerKnote();
}

void KqueueImpl::RemoveRegistration(internal::FdRegistration *reg) {
  if (!reg->registered) {
    return;
  }
  struct kevent events[2];
  int n = 0;
  if (reg->epoll_events & (kIn | kPri | kErr)) {
    EV_SET(&events[n++], reg->fd, EVFILT_READ, EV_DELETE, 0, 0, NULL);
  }
  if (reg->epoll_events & (kOut | kErr)) {
    EV_SET(&events[n++], reg->fd, EVFILT_WRITE, EV_DELETE, 0, 0, NULL);
  }
  if (n > 0) {
    // Strict, like EPoll and EpollImpl; see
    // ReadinessBackend::RemoveRegistration().  The kernel drops a closed
    // fd's filters itself, so EBADF or ENOENT here means the fd was closed
    // while registered.
    ABSL_PCHECK(kevent(kqueue_fd_, events, n, NULL, 0, NULL) == 0)
        << "kevent EV_DELETE failed for fd " << reg->fd
        << "; was it closed while registered?";
  }
  reg->registered = false;
  reg->epoll_events = 0;
}

internal::ReadinessBackend::WaitResult KqueueImpl::WaitForOne(bool block,
                                                              FdReady *out) {
  // Deliver the post-fork signal wakeups RecreateKernelState() queued.  Same
  // shape as the EVFILT_SIGNAL dispatch below, so a consumer cannot tell the
  // two apart.
  if (signal_wakeup_pending_ && signal_callback_) {
    signal_wakeup_pending_ = false;
    DispatchSignalCallback();
    return WaitResult::kHandled;
  }

  struct kevent event;
  struct timespec timeout;
  timeout.tv_sec = 0;
  timeout.tv_nsec = 0;
  struct timespec *timeout_ptr = block ? nullptr : &timeout;
  int num_events;
  do {
    num_events = kevent(kqueue_fd_, NULL, 0, &event, 1, timeout_ptr);
  } while (num_events == -1 && errno == EINTR && block);

  if (num_events == -1) {
    if (errno == EINTR) {
      return WaitResult::kNothing;
    }
    ABSL_PCHECK(num_events != -1);
  }

  if (num_events > 0) {
    // A timer or a signal.  kqueue delivers those as filters of their own,
    // where epoll delivers them as ordinary fds that fall out of the shared
    // path -- so they are dispatched here and the caller has nothing left to
    // do.
    if (event.filter == EVFILT_TIMER) {
      // EV_ONESHOT, so the firing disarmed it.
      timer_armed_ = false;

      // The knote carries no identity -- it is the wakeup for whatever
      // deadline was earliest when it was armed, and active_timers_ says who
      // that is.  Take the front, which is the earliest deadline and, among
      // timers sharing it, the one scheduled first.  Exactly one, so a
      // callback here still sees every other timer in this batch as pending
      // and may cancel, reschedule or destroy it.
      //
      // Only if it is genuinely due: the front can move to a later deadline
      // between the kernel queueing this event and the loop reading it, and
      // the re-arm below is what waits for the new one.  Not a spin -- that
      // re-arm is for a future deadline, so the next blocking kevent() sleeps.
      bool callback_ran = false;
      KqueueTimerState *const state = active_timers_.front();
      if (state != nullptr && !state->request.done &&
          state->deadline <= aos::monotonic_clock::now()) {
        // Resolve before dispatching -- the callback may destroy this timer.
        state->request.done = true;
        RemoveActiveTimer(state);
        if (state->user_callback) {
          Completion completion;
          // Timer::Schedule() takes no user_data, so a timer completion
          // carries nullptr rather than an internal pointer -- see
          // Completion::user_data in aio.h, and both Linux backends.
          completion.user_data = nullptr;
          completion.status = aos::Ok();
          completion.result = 0;
          state->user_callback(completion, state->user_context);
          callback_ran = true;
        }
      }
      // After the callback, so that whatever it did to the timers is what
      // gets armed.  A deadline already past re-arms to fire immediately,
      // which is how the rest of a batch is delivered one wait at a time.
      UpdateTimerKnote();
      return callback_ran ? WaitResult::kHandled : WaitResult::kInternal;
    }
    if (event.filter == EVFILT_SIGNAL) {
      if (signal_callback_) {
        DispatchSignalCallback();
        return WaitResult::kHandled;
      }
      // Nothing registered to take it.  Holding it rather than dropping it
      // is what keeps the knote being left armed from turning "polled while
      // unregistered" into a lost wakeup -- EVFILT_SIGNAL's count is
      // consumed by being reported, so this is the only place it can go.
      signal_wakeup_pending_ = true;
      return WaitResult::kInternal;
    }

    auto *reg = static_cast<internal::FdRegistration *>(event.udata);
    // Translate into the mask epoll would have reported for the same
    // condition, so everything below can stay a bit-for-bit mirror of
    // EpollImpl::Poll().  Two kqueue-isms have to be undone to get there.
    //
    // First, which side hung up.  EV_EOF is the only terminal indication
    // kqueue gives -- there is no separate error bit -- so it has to be
    // read against the filter that reported it.  On the write filter it
    // means the reader is gone and every future write() fails with EPIPE,
    // which is precisely what epoll reports as EPOLLERR; that is the
    // pipe-with-a-closed-read-end case, and dropping it would leave
    // OnError() silent on Darwin with nothing else to raise it.  On the
    // read filter it means the writer is gone and reads return a clean
    // EOF, which is EPOLLHUP -- not an error, and epoll pairs it with
    // EPOLLIN so the reader drains.  Folding that one into kErr would
    // instead abort on the err_fn CHECK below for the ordinary
    // OnReadable()-only registration.
    //
    // Second, readiness nobody subscribed to; see DesiredReadiness().
    uint32_t events = 0;
    if (event.filter == EVFILT_READ) {
      events |= kIn;
    } else if (event.filter == EVFILT_WRITE) {
      events |= kOut;
      if (event.flags & EV_EOF) {
        events |= kErr;
      }
    }
    if (event.flags & EV_ERROR) {
      events |= kErr;
    }
    // Masking readiness can never hide a filter we still need drained: a
    // filter is left level-triggered exactly when its readiness bit is
    // desired, and armed EV_CLEAR (one edge, then quiet) when it exists
    // only to watch for errors -- see UpdateRegistration().  So a bit dropped
    // here belongs to an edge-triggered filter that will not re-report it,
    // and Poll() cannot spin on it.
    events &= DesiredReadiness(reg) | kErrorEvents;
    const bool terminal = (event.flags & (EV_EOF | EV_ERROR)) != 0;

    out->reg = reg;
    out->events = events;
    out->terminal = terminal;
    return WaitResult::kFdReady;
  }

  // kevent() timed out with nothing ready.
  return WaitResult::kNothing;
}

void KqueueImpl::RegisterThreadSignalReceiver(
    ipc_lib::ThreadSignalReceiver *receiver, std::function<void()> callback) {
  CheckForFork();
  ABSL_CHECK(signal_receiver_ == nullptr)
      << "Duplicate ThreadSignalReceiver registration: only one receiver "
         "may be active at a time (see Aio::RegisterThreadSignalReceiver)";

  signal_receiver_ = receiver;
  signal_callback_ = std::move(callback);

  if (!signal_knote_armed_) {
    struct kevent ev;
    EV_SET(&ev, ipc_lib::kWakeupSignal, EVFILT_SIGNAL, EV_ADD | EV_ENABLE, 0, 0,
           NULL);
    ABSL_PCHECK(kevent(kqueue_fd_, &ev, 1, NULL, 0, NULL) == 0);
    signal_knote_armed_ = true;
  }
}

void KqueueImpl::UnregisterThreadSignalReceiver(
    ipc_lib::ThreadSignalReceiver *receiver) {
  CheckForFork();
  ABSL_CHECK(signal_receiver_ == receiver) << "ThreadSignalReceiver not found";

  // The knote stays armed.  EV_DELETE would take its pending count with it,
  // and aio.h promises that count to whoever registers next: a wakeup still
  // kernel-side when its receiver goes away belongs to the successor.  The
  // signalfd backends keep it by doing nothing -- an unregistered signalfd
  // simply stops being polled, holding whatever is queued in it -- and this
  // is the equivalent of doing nothing.  A firing with no receiver registered
  // parks in signal_wakeup_pending_ rather than being dropped, and the knote
  // dies with the kqueue in ~KqueueImpl().

  if (signal_in_callback_) {
    // Called from inside this receiver's own callback.  Clearing the
    // std::function that is executing would free it out from under itself,
    // so leave it to Poll().
    //
    // receiver stays set deliberately, so a Register() before Poll() gets
    // there -- i.e. from inside this same callback -- fails the duplicate
    // check rather than overwriting the std::function still running.  The
    // destructor tolerates the leftover by testing clear_pending.
    signal_clear_pending_ = true;
    return;
  }
  ClearSignalRegistration();
}

void KqueueImpl::ConsumeThreadSignalReceiver(
    ipc_lib::ThreadSignalReceiver *receiver) {
  CheckForFork();
  (void)receiver;
  // kqueue automatically consumes the signal event when returning it.
}

KqueueTimerState::~KqueueTimerState() {
  Cancel(true);
  if (counted_live) {
    counted_live = false;
    --impl_->live_timers_;
  }
}

void KqueueTimerState::Initialize() {
  request.done = true;
  // Counted from construction rather than from the first Schedule(), the same
  // point aio_linux.cc links a timer at -- the destructor CHECK is about the
  // Timer object's lifetime, not about whether it happens to be armed.
  counted_live = true;
  ++impl_->live_timers_;
}

void KqueueTimerState::Schedule(aos::monotonic_clock::time_point deadline,
                                CompletionCallback callback, void *context) {
  impl_->CheckForFork();
  ABSL_CHECK_GE(deadline, aos::monotonic_clock::epoch());
  Cancel(true);

  this->deadline = deadline;
  this->user_callback = callback;
  this->user_context = context;
  this->request.done = false;
  // Inserting is what arms the knote, if this timer is now the earliest.
  impl_->InsertActiveTimer(this);
}

void KqueueTimerState::Cancel(bool /*reap*/) {
  impl_->CheckForFork();
  // Removing re-points the knote at whatever is earliest now, which also
  // suppresses a firing this timer had already queued but not been dispatched.
  impl_->RemoveActiveTimer(this);
  request.done = true;
  user_callback = nullptr;
}

bool KqueueImpl::UpdateRegistration(internal::FdRegistration *reg) {
  const FileDescriptor fd = reg->fd;

  uint32_t desired_events = DesiredReadiness(reg);
  // Registered-vs-not is decided on the caller's untranslated mask, as epoll
  // decides it.  A mask made only of bits with no filter of their own (a bare
  // EPOLLHUP, say) still has to hear about errors and hangups, which epoll
  // reports for any registered fd, so it watches for them the way kErr does.
  if (reg->events & ~(kInEvents | kOutEvents)) desired_events |= kErr;

  bool want_read = (desired_events & kIn) || (desired_events & kErr);
  bool has_read = reg->registered &&
                  ((reg->epoll_events & kIn) || (reg->epoll_events & kErr));

  // EV_CLEAR is only set when the filter exists purely to report errors, so
  // it has to be re-evaluated whenever that changes -- not just when the
  // filter is added or removed.  Going from error-only to readable keeps
  // want_read true, and without this the filter would stay edge-triggered and
  // we'd miss readiness; going the other way would leave it level-triggered
  // and spin. EV_ADD on an already-registered filter updates its flags in
  // place.
  const bool want_read_clear = want_read && !(desired_events & kIn);
  const bool has_read_clear = has_read && !(reg->epoll_events & kIn);

  if (want_read != has_read ||
      (want_read && want_read_clear != has_read_clear)) {
    struct kevent ev;
    if (want_read) {
      int flags = EV_ADD | EV_ENABLE;
      if (want_read_clear) {
        flags |= EV_CLEAR;
      }
      EV_SET(&ev, fd, EVFILT_READ, flags, 0, 0, reg);
    } else {
      EV_SET(&ev, fd, EVFILT_READ, EV_DELETE, 0, 0, NULL);
    }
    if (kevent(kqueue_fd_, &ev, 1, NULL, 0, NULL) != 0) {
      // Adding a filter the kernel refuses is reported rather than died on:
      // a raw request gets an error completion, and a legacy registration,
      // which has no request to carry it, dies in UpdateRegistrationOrDie()
      // instead.  Removing one is strict, as RemoveRegistration() is.
      if (want_read && (errno == EBADF || errno == ENOENT)) {
        // What FailRawRegistration() delivers to the caller.  Left
        // unset it reads 0, and the "error" completion arrives as a success
        // carrying no bytes -- see aio_linux.cc, which records it here too.
        reg->registration_errno = errno;
        reg->registered = false;
        reg->epoll_events = 0;
        return false;
      }
      ABSL_PLOG(FATAL) << "kevent " << (want_read ? "EV_ADD" : "EV_DELETE")
                       << " failed for fd " << fd;
    }
  }

  bool want_write = (desired_events & kOut) || (desired_events & kErr);
  bool has_write = reg->registered &&
                   ((reg->epoll_events & kOut) || (reg->epoll_events & kErr));

  // Same EV_CLEAR re-evaluation as the read filter above.
  const bool want_write_clear = want_write && !(desired_events & kOut);
  const bool has_write_clear = has_write && !(reg->epoll_events & kOut);

  if (want_write != has_write ||
      (want_write && want_write_clear != has_write_clear)) {
    struct kevent ev;
    if (want_write) {
      int flags = EV_ADD | EV_ENABLE;
      if (want_write_clear) {
        flags |= EV_CLEAR;
      }
      EV_SET(&ev, fd, EVFILT_WRITE, flags, 0, 0, reg);
    } else {
      EV_SET(&ev, fd, EVFILT_WRITE, EV_DELETE, 0, 0, NULL);
    }
    if (kevent(kqueue_fd_, &ev, 1, NULL, 0, NULL) != 0) {
      // Adding a filter the kernel refuses is reported rather than died on:
      // a raw request gets an error completion, and a legacy registration,
      // which has no request to carry it, dies in UpdateRegistrationOrDie()
      // instead.  Removing one is strict, as RemoveRegistration() is.
      if (want_write && (errno == EBADF || errno == ENOENT)) {
        // What FailRawRegistration() delivers to the caller.  Left
        // unset it reads 0, and the "error" completion arrives as a success
        // carrying no bytes -- see aio_linux.cc, which records it here too.
        reg->registration_errno = errno;
        reg->registered = false;
        reg->epoll_events = 0;
        return false;
      }
      ABSL_PLOG(FATAL) << "kevent " << (want_write ? "EV_ADD" : "EV_DELETE")
                       << " failed for fd " << fd;
    }
  }

  reg->registered = (want_read || want_write);
  reg->epoll_events = desired_events;
  return true;
}

// No silent fallback: a name this build has no backend for is fatal, even
// though there is only the one to fall back to.
Aio::Aio() {
  const std::string backend = ::absl::GetFlag(FLAGS_aio_backend);
  if (backend != "kqueue") {
    ABSL_LOG(FATAL) << "Unknown --aio_backend \"" << backend
                    << "\"; this build supports \"kqueue\".";
  }
  impl_ = std::make_unique<KqueueImpl>();
}
}  // namespace aos
