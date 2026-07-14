#include "aos/events/aio.h"

#include <errno.h>
#include <fcntl.h>
#include <liburing.h>
#include <poll.h>
#include <stdio.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <sys/signalfd.h>
#include <sys/socket.h>
#include <sys/timerfd.h>
#include <sys/utsname.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/events/aio_internal.h"
#include "aos/ipc_lib/thread_signal.h"
#include "aos/libc/aos_strerror.h"
#include "aos/realtime.h"
#include "aos/time/time.h"

ABSL_FLAG(uint32_t, aio_queue_depth, 256,
          "Depth of the io_uring submission and completion queues.");

namespace aos {
namespace {

// The calling thread's id, cached: aos::GetThreadId() is an uncached
// syscall(SYS_gettid), and the submitter-thread check runs on every entry
// point -- Poll(), AsyncRead/AsyncWrite, Cancel, timer Schedule()/Cancel()
// -- where an extra kernel round trip per call is real cost and would
// falsify Schedule()'s documented one-syscall claim.
thread_local pid_t cached_tid = 0;

pid_t CachedThreadId() {
  if (cached_tid == 0) {
    cached_tid = aos::GetThreadId();
  }
  return cached_tid;
}

// Attempts io_uring_queue_init_params with the given flags, once -- no
// retries.  A failure always indicates a real problem (a memory-starved
// machine, or a kernel below the floor RequireMinimumKernelVersion()
// enforces) and must fail loudly rather than be absorbed.  Returns the
// result rather than CHECKing it, so each caller can die with its own
// message naming the flags that failed.
int InitOneRing(struct io_uring *ring, uint32_t depth, unsigned flags) {
  struct io_uring_params params;
  std::memset(&params, 0, sizeof(params));
  params.flags = flags;
  return io_uring_queue_init_params(depth, ring, &params);
}

// The running kernel's release string (`uname -r`), for failure messages.
std::string KernelRelease() {
  struct utsname un;
  if (uname(&un) != 0) {
    return "unknown";
  }
  return un.release;
}

// The oldest kernel this backend runs on, enforced at construction.
//
// The feature floor is 6.1: every io_uring feature this file submits
// exists by then.  IORING_SETUP_SINGLE_ISSUER (6.0) and
// IORING_SETUP_DEFER_TASKRUN (6.1) are the newest; everything else --
// IORING_SETUP_R_DISABLED and io_uring_enable_rings() (5.10), multishot
// poll (5.13), IORING_SETUP_COOP_TASKRUN | TASKRUN_FLAG (5.19), async
// cancel (5.5) -- is older.  Checked explicitly rather than left to the
// -EINVAL a missing flag would produce, so the failure names the actual
// requirement instead of a bare errno.
//
// 6.12 is the *recommended* floor: the release where PREEMPT_RT was merged
// into mainline.  A realtime robotics codebase has little reason to run
// anything older, but nothing in this file needs it, so it is a suggestion
// in the error message, not a check.
constexpr int kMinimumKernelMajor = 6;
constexpr int kMinimumKernelMinor = 1;

void RequireMinimumKernelVersion() {
  const std::string release = KernelRelease();
  int major = 0;
  int minor = 0;
  ABSL_CHECK_EQ(sscanf(release.c_str(), "%d.%d", &major, &minor), 2)
      << "Could not parse the kernel release \"" << release << "\"";
  ABSL_CHECK(major > kMinimumKernelMajor ||
             (major == kMinimumKernelMajor && minor >= kMinimumKernelMinor))
      << "Kernel " << release
      << " is too old: aos's io_uring backend requires >= "
      << kMinimumKernelMajor << "." << kMinimumKernelMinor
      << " (IORING_SETUP_SINGLE_ISSUER | IORING_SETUP_DEFER_TASKRUN); >= "
         "6.12 recommended (the release PREEMPT_RT was merged mainline).";
}

// Initializes (or reinitializes) the io_uring instance at *ring on the
// SINGLE_ISSUER tier -- the only tier this file creates directly.  Called
// from the constructor; the one exception, DowngradeFromSingleIssuer(),
// builds its unconstrained ring inline at that single call site.
//
// See documentation/adr/0001-aio-io-uring-single-issuer.md for the full
// design rationale.  Short version: IORING_SETUP_SINGLE_ISSUER |
// DEFER_TASKRUN gives deterministic completion delivery, and
// IORING_SETUP_R_DISABLED keeps ring creation itself from binding a
// submitter thread -- binding happens lazily, the first time the ring is
// driven (EnsureBound()/CheckSubmitterThread()).  SINGLE_ISSUER's binding
// is permanent (no unbind or rebind), which is why anything on this tier
// must be destroyed on the thread that called Run()/Poll() on it, and why
// EnsureBound() downgrades instances whose threading shape can't satisfy
// that.
//
// Requires the kernel floor RequireMinimumKernelVersion() enforces and
// fails loudly on anything older -- there is no old-kernel fallback to
// lose behavior the rest of this file depends on.
void InitSingleIssuerRing(struct io_uring *ring, uint32_t depth) {
  int ret =
      InitOneRing(ring, depth,
                  IORING_SETUP_SINGLE_ISSUER | IORING_SETUP_DEFER_TASKRUN |
                      IORING_SETUP_R_DISABLED);
  ABSL_PCHECK(ret == 0)
      << "io_uring_queue_init failed: " << aos_strerror(-ret) << " on kernel "
      << KernelRelease()
      << " -- aos requires a Linux kernel >= 6.1 (6.12 recommended) for "
         "IORING_SETUP_SINGLE_ISSUER | IORING_SETUP_DEFER_TASKRUN | "
         "IORING_SETUP_R_DISABLED support.";
}

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
    // Quit() reaches this from ShmEventLoop's SIGINT/SIGHUP/SIGTERM handler, so
    // everything here has to be async-signal-safe.  write() is.  Overwriting
    // errno is not: we interrupted a thread which may be partway through
    // checking its own errno, so put back whatever was there.
    const int saved_errno = errno;
    const ssize_t ret = write(fd_, &val, sizeof(val));
    const int write_errno = errno;
    errno = saved_errno;
    if (ret < 0 && write_errno != EAGAIN && write_errno != EWOULDBLOCK) {
      // ABSL_RAW_LOG formats into a stack buffer and writes the result out
      // directly, where ABSL_LOG would allocate and take locks.  The errno goes
      // out as a bare number for the same reason -- aos_strerror() formats
      // through thread_local storage with snprintf().
      ABSL_RAW_LOG(FATAL, "Failed to write to eventfd: errno %d", write_errno);
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

// Internal helper to access AsyncRequest's opaque state buffer.  This
// avoids type-punning issues and pointer-to-pointer casting.  Deliberately
// no union and no submit-path scratch space sharing storage with `link`:
// AsyncRead()/AsyncWrite() may legally be called on a request that is still
// linked on pending_dispatch_ (a queued-but-undispatched completion), so
// anything a submit path wrote into aliased storage would corrupt the
// dispatch list.
struct AioState {
  struct {
    AsyncRequest *next;
    // Doubly linked so IoUringImpl::UnlinkPendingDispatch() can splice a
    // request out of pending_dispatch_ in O(1) instead of scanning for it.
    AsyncRequest *prev;
    int32_t result;
    // Set while linked on IoUringImpl::pending_dispatch_ (see
    // QueuePendingDispatch()).
    int32_t queued;
  } link;
  // Incarnation counter for the io_uring backend: incremented every time a
  // fresh kernel op is submitted under this request's identity, and encoded
  // into that op's user_data.  See EncodeUserData() for why 16 bits
  // cannot wrap into ambiguity.
  uint16_t generation;
  // Set while this request is a caller-submitted AsyncRead/AsyncWrite in
  // flight (the internal wakeup read is excluded).  Backs
  // IoUringImpl::raw_requests_in_flight_, which is what lets a ring
  // rebuild refuse loudly instead of silently dropping raw requests it has
  // no registry to re-arm -- see DowngradeFromSingleIssuer().
  uint8_t raw_io;
  // Set at queue time when this request's callback is the caller's own
  // completion callback rather than one of this file's trampolines, so
  // ReapCompletions() can tell whether dispatching it spends the
  // one-user-completion-per-Poll() budget.  Snapshotted from raw_io, which
  // DrainCompletions() clears before the dispatch ever happens.  The
  // trampolines (timers, thread-signal receivers, the legacy-epoll poll)
  // report for themselves instead -- they routinely dispatch without
  // delivering anything to the user.
  uint8_t user_visible;
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

// The user_data encoding from the IoUringImpl design comment.  Kernel side:
// IORING_OP_ASYNC_CANCEL matches its target by exact user_data compare
// (io_cancel_req_match()), which is what makes incarnation targeting exact.
// A cancel's ack carries a low-bit tag (pointer alignment keeps those bits
// free) so it can be told apart from a target's own completion -- and,
// deliberately, the loop-owned cancel_ack_sentinel_'s identity rather than
// its target's.  An ack that named its target would require the target to
// outlive the ack's drain, which can trail the target's own callback (a CQ
// overflow can relegate the ack to the kernel's overflow list); naming the
// immortal sentinel instead is what lets a caller free an AsyncRequest as
// soon as its callback has run (aio.h's constraint 2), and lets orphaned
// internal states be recycled without waiting on ack traffic.
//
// Wrap safety: colliding with a stale in-flight op would take 65536
// incarnations of one identity queued between two io_uring_enter calls;
// arms-behind-terminals caps that at one.
constexpr uint64_t kAckTagMask = 0x3;
constexpr uint64_t kCancelAckTag = 0x1;
constexpr int kGenerationShift = 48;
constexpr uint64_t kPointerMask =
    ((uint64_t{1} << kGenerationShift) - 1) & ~kAckTagMask;
static_assert(alignof(AsyncRequest) >= 4,
              "ack tags borrow the two low pointer bits");

// Encodes user_data for an op targeting `req`'s *current* incarnation --
// used for cancel targets and their acks.
inline uint64_t EncodeUserData(AsyncRequest *req, uint64_t tag) {
  const uint64_t ptr = reinterpret_cast<uint64_t>(req);
  ABSL_CHECK_EQ(ptr >> kGenerationShift, uint64_t{0})
      << "AsyncRequest pointer does not fit the user_data encoding";
  return (uint64_t{State(req).generation} << kGenerationShift) | ptr | tag;
}

// Starts a new incarnation of `req` and returns its encoded user_data --
// call exactly once per fresh kernel-op submission under this identity.
inline uint64_t NewIncarnationUserData(AsyncRequest *req) {
  ++State(req).generation;
  return EncodeUserData(req, 0);
}

// Translates a raw io_uring CQE result code into the public Completion
// status/result pair.  Only one caller today (ReapCompletions()), but kept
// separate since the translation logic is a distinct concern from iterating
// the CQ.
inline Completion CompletionFromResult(AsyncRequest *req, int32_t res) {
  Completion completion;
  completion.user_data = req->user_data;

  if (res == -ECANCELED) {
    completion.status = aos::MakeError("Canceled");
    completion.result = 0;
  } else if (res < 0) {
    completion.status = aos::MakeError("io_uring error");
    completion.result = -res;
  } else {
    completion.status = aos::Ok();
    completion.result = res;
  }
  return completion;
}

// AOS's fd-readiness event encoding, shared by both backends and by the
// public SetEvents()/OnEvents() contract.  Numerically identical to the
// low 4 bits of the real epoll event flags (the documented "raw epoll
// events" contract in aio.h).
constexpr uint32_t kIn = 0x01;   // EPOLLIN
constexpr uint32_t kPri = 0x02;  // EPOLLPRI
constexpr uint32_t kOut = 0x04;  // EPOLLOUT
constexpr uint32_t kErr = 0x08;  // EPOLLERR

constexpr uint32_t kInEvents = kIn | kPri;
constexpr uint32_t kOutEvents = kOut;
constexpr uint32_t kErrorEvents = kErr;

// The absolute it_value to arm a timerfd with for `deadline`.
//
// timerfd_settime(2) reads an all-zero it_value as "disarm this timer", not
// as "expire at time zero" -- so aos::monotonic_clock::epoch() is the one
// deadline that cannot be handed to it verbatim.  Passed straight through, a
// timer scheduled at the epoch silently never fires, which is strictly worse
// than either firing or refusing: nothing reports it.
//
// Scheduling at the epoch is legal and has to keep working.  A deadline in
// the past means "fire as soon as the loop is driven" everywhere else in
// this API, aos::TimerHandler::Schedule() is documented to accept it, and
// the simulated event loop's timeline *starts* at the epoch -- so an
// application that works in sim has to behave the same on a ShmEventLoop.
// The other backends have no equivalent quirk (kqueue takes an absolute mach
// time of 0, IOCP an absolute deadline of 0; both are simply in the past),
// so this is the only place the divergence could come from.
//
// One nanosecond past the epoch is decades in the past on any running
// system, so it expires immediately -- the requested behavior -- while being
// non-zero, so the kernel arms rather than disarms.
inline struct timespec AbsoluteTimerfdValue(
    aos::monotonic_clock::time_point deadline) {
  struct timespec ts = ::aos::time::to_timespec(deadline);
  if (ts.tv_sec == 0 && ts.tv_nsec == 0) {
    ts.tv_nsec = 1;
  }
  return ts;
}

}  // namespace

class IoUringImpl;

// A one-shot timer, implemented as a timerfd whose readiness io_uring
// watches.
//
// The deadline lives entirely in the kernel's timerfd, not in an io_uring
// op: Schedule() and Cancel() are each a single timerfd_settime() and submit
// nothing to the ring.  io_uring's only job is telling us when timer_fd
// became readable, via one multishot poll armed once in Initialize() and
// held for the state's whole life -- so the steady state costs zero
// submissions per firing.
//
// This replaces an earlier design built on IORING_OP_TIMEOUT with a
// steady-state IORING_TIMEOUT_MULTISHOT and IORING_TIMEOUT_UPDATE reschedules.
// Two reasons, both in documentation/adr/0001-aio-io-uring-single-issuer.md:
//
//   * IORING_TIMEOUT_MULTISHOT cannot keep phase.  The kernel rejects
//     MULTISHOT|ABS outright (__io_timeout_prep() in linux/io_uring/
//     timeout.c), so a repeating op is necessarily relative, and
//     io_timeout_complete() re-arms it with hrtimer_start(period) from
//     whenever task work ran -- not hrtimer_forward() from the previous
//     deadline.  Every period absorbs the wakeup latency and it accumulates,
//     measured at 4-10us per period on an idle machine and growing with how
//     busy the loop is.  That defect is what took repeating timers out of
//     Aio's API entirely (see Aio::Timer::Schedule()); a caller-driven
//     period against an absolute deadline has nowhere to accumulate error.
//   * Retargeting a timerfd is one atomic settime() that cannot race
//     anything.  Retargeting an armed io_uring timeout meant
//     IORING_TIMEOUT_UPDATE or IORING_OP_ASYNC_CANCEL, both of which resolve
//     their target by a lookup that a firing op is briefly absent from --
//     the source of essentially every lifecycle bug in that ADR.
struct IoUringTimerState : public Aio::TimerState {
  explicit IoUringTimerState(IoUringImpl *impl) : impl_(impl) {}
  ~IoUringTimerState() override;

  void Initialize() override;
  void Schedule(aos::monotonic_clock::time_point deadline,
                CompletionCallback callback, void *context) override;
  void Cancel(bool reap) override;

  // Public so IoUringImpl can reach them for recycling and ring rebuilds,
  // like the state on the Aio::TimerState base.

  // This timer's deadline and period.  Outlives recycling: MakeTimerState()
  // hands a drained state back out with its timerfd intact rather than
  // paying timerfd_create() again (Reset()).
  std::unique_ptr<TimerFD> timer_fd;

  // While the owning Timer is destroyed with the poll still in flight, the
  // impl owns this state on its intrusive orphan list (orphaned_timers_,
  // linked through next_orphan) and recycles it once the poll's terminal
  // completion has drained.  Membership on that list is the one and only
  // record of orphan-hood -- there is deliberately no flag to fall out of
  // sync with it.  See DestroyTimerState().
  IoUringTimerState *next_orphan = nullptr;

  // (Re-)arms the multishot poll on timer_fd.  Called once from
  // Initialize(), again from the completion handler if the kernel ever
  // terminates the op, and again for every live timer whenever the ring is
  // rebuilt (ReArmPersistentRegistrations()).  That last caller passes
  // `draining_ok`, since re-arming every timer at once can outrun a shallow
  // --aio_queue_depth -- see GetSqeForRingReconstruction().
  void SubmitPoll(bool draining_ok = false);

  // Clears everything a recycled state must not inherit, keeping the pieces
  // that are safe (and expensive) to carry over: the timerfd and the
  // request's generation counter.  See MakeTimerState().
  void Reset();

 private:
  // request's completion handler, installed by SubmitPoll(): reads the
  // timerfd and delivers the user callback.
  static void OnTimerFdReadable(Completion completion, void *ctx);

  IoUringImpl *impl_;
};

// The io_uring backend.  Design and invariants; the comments at each method
// carry only the local kernel details.
//
// Threading.  One ring, one submitter.  By default the ring is created with
// IORING_SETUP_SINGLE_ISSUER | IORING_SETUP_DEFER_TASKRUN | R_DISABLED and
// binds to whichever thread first calls Poll()/Run() (EnsureBound()); every
// entry point CHECKs the binding afterward.  If that first caller is not
// the constructing thread, the ring is rebuilt on the COOP_TASKRUN tier
// with no binding instead -- see aio.h for why that case must work.
//
// Incarnations.  Callers reuse one AsyncRequest across many arm/cancel
// cycles (every wakeup-read re-arm, every poll re-arm), so a bare pointer
// in user_data is ambiguous: a slow cancel or stale CQE from a previous
// cycle could be mistaken for -- or act on -- the current one.  To make
// reuse safe, every SQE's user_data encodes
//   [63:48] generation | [47:2] AsyncRequest pointer | [1:0] ack tag.
// One *incarnation* = one armed kernel op under a request identity; the
// generation increments on every fresh arm (NewIncarnationUserData()).  The
// kernel looks up a cancel's target by exact user_data compare, so an op
// aimed at incarnation N cannot touch incarnation N+1, and every target
// CQE names the incarnation that produced it -- staleness is decided by
// comparison, never inferred from surrounding state.  (Ack CQEs name
// cancel_ack_sentinel_ instead of their target -- see EncodeUserData().)
//
// Timers.  Not io_uring ops at all: each is a timerfd, and io_uring only
// watches it for readability -- see IoUringTimerState.  That is what keeps
// scheduling drift-free and keeps reschedules out of the cancel/lookup
// races the incarnation scheme above exists to contain.
//
// Destruction.  Nothing in this backend blocks on the kernel.  Destroying
// a Timer (or unregistering a signal receiver) with traffic still in
// flight *orphans* its state onto an intrusive list, with the user
// callback stripped and a cancel in flight; destruction itself is
// CheckNotRealtime() (it can free).  Once an orphan's terminal completion
// has drained (and dispatched or been unlinked), it is *recycled* onto a
// freelist that MakeTimerState() reuses -- cancel acks never gate this,
// since they name cancel_ack_sentinel_ rather than the orphan (see the
// EncodeUserData() block).  Recycling is pointer manipulation only,
// because it runs inside Poll() -- including on RT threads; memory is
// bounded by peak usage and only actually freed in ~IoUringImpl().
//
// Dispatch.  Completion callbacks routinely reenter the ring: a timer
// callback reschedules, a cancel drains.  Dispatching straight off the CQ
// iteration would let that reentry advance the CQ head under the live
// scan (corrupting it -- a real crash in the earlier design).  So
// completion handling is two-phase: DrainCompletions() extracts CQEs and
// queues callbacks on pending_dispatch_ without running any user code,
// then ReapCompletions() dispatches.  Poll() itself is not reentrant --
// a callback calling Poll() dies on a CHECK (see there).
//
// Legacy fds.  OnReadable()/OnWritable()/OnEvents() fds live on one
// embedded epoll instance, watched by a single-shot POLL_ADD that is
// re-armed on every firing (SubmitLegacyEpollPoll()).
//
// History and rejected alternatives:
// documentation/adr/0001-aio-io-uring-single-issuer.md.
class IoUringImpl : public Aio::Impl {
  friend struct IoUringTimerState;

 public:
  IoUringImpl();
  ~IoUringImpl() override;

  std::unique_ptr<Aio::TimerState> MakeTimerState() override;
  void DestroyTimerState(std::unique_ptr<Aio::TimerState> state) override;

  void Run() override;

  bool Poll(bool block) override;
  void Quit() override;
  void Wakeup();

  void AsyncRead(FileDescriptor fd, std::span<char> buffer,
                 AsyncRequest *request) override;
  void AsyncWrite(FileDescriptor fd, std::span<const char> buffer,
                  AsyncRequest *request) override;
  void Cancel(AsyncRequest *request) override;
  void BeforeWait(std::function<void()> function) override;

  void OnReadable(FileDescriptor fd, std::function<void()> callback) override;
  void OnError(FileDescriptor fd, std::function<void()> callback) override;
  void OnWritable(FileDescriptor fd, std::function<void()> callback) override;
  void OnEvents(FileDescriptor fd,
                std::function<void(uint32_t)> callback) override;
  void DeleteFd(FileDescriptor fd) override;
  void ForgetClosedFd(FileDescriptor fd) override;
  void EnableWritable(FileDescriptor fd) override;
  void DisableWritable(FileDescriptor fd) override;
  void SetEvents(FileDescriptor fd, uint32_t events) override;

  void RegisterThreadSignalReceiver(ipc_lib::ThreadSignalReceiver *receiver,
                                    std::function<void()> callback) override;
  void UnregisterThreadSignalReceiver(
      ipc_lib::ThreadSignalReceiver *receiver) override;
  void ConsumeThreadSignalReceiver(
      ipc_lib::ThreadSignalReceiver *receiver) override;

 private:
  bool ReapCompletions();
  // Extracts every currently-available CQE in one batch: advances the CQ
  // past all of them, sets each request's `done`, and -- for any that have
  // one -- queues its callback on pending_dispatch_ rather than calling it.
  // Never dispatches anything itself, which is what makes it safe to call
  // from anywhere, at any nesting depth, including from inside another
  // callback -- see the definition for why batching the CQ advance is safe
  // here specifically (it would not be, if this function ever dispatched).
  bool DrainCompletions();
  // Appends req (already resolved -- see DrainCompletions()) to the FIFO
  // pending dispatch list, using the intrusive AsyncRequest::internal_state
  // link field (see AioState::link), doubly linked so
  // UnlinkPendingDispatch() can remove an arbitrary entry in O(1).
  // res is stashed alongside it since the CQE itself is gone by dispatch
  // time.
  void QueuePendingDispatch(AsyncRequest *req, int32_t res);
  // Removes req from pending_dispatch_ if it's there, no-op otherwise.  O(1)
  // via req's own prev/next (see AioState::link.prev), not a scan.
  // CancelRequest() calls this unconditionally, including when request->done
  // is already true: a request can be sitting resolved-but-undispatched on
  // pending_dispatch_ (its completion already arrived, queued by
  // DrainCompletions(), but ReapCompletions()'s dispatch loop hasn't reached
  // it yet) when its owning object is destroyed -- e.g. an earlier callback in
  // the very same dispatch batch destroys a different, still-pending timer.
  // Without this, the dispatch loop would walk into freed/reused memory when
  // it got to that node.  Safe to call while a dispatch loop is mid-run: it
  // pops from the live list each iteration (see ReapCompletions()), so an
  // unlink here is picked up immediately.
  void UnlinkPendingDispatch(AsyncRequest *req);
  void CancelRequest(AsyncRequest *request);
  void SubmitWakeupRead();
  // Returns a submission queue entry for use inside
  // ReArmPersistentRegistrations() only, flushing the queue to the kernel
  // (an io_uring_submit() syscall) and retrying once if it is currently
  // full.  ReArmPersistentRegistrations() re-arms every persistent request
  // (wakeup read, the legacy-fd epoll poll, receivers, and every active
  // timer) in a single pass, which can queue more entries than the ring is
  // deep -- e.g. several active timers with a shallow --aio_queue_depth.  A
  // plain io_uring_get_sqe() would return null and abort; draining the
  // queue frees the entries so reconstruction can continue.
  //
  // Deliberately NOT used by the steady-state submission paths (AsyncRead,
  // SubmitLegacyEpollPoll, ThreadSignalReceiverState::Submit): those call
  // ArmSqe() and CHECK-fail immediately on exhaustion.  Ring
  // reconstruction already has unbounded latency (it tears down and rebuilds
  // the whole ring; nothing about it is real-time), so paying for a syscall
  // there is fine.  On the steady-state path, exhaustion is a
  // real-time-affecting config error (--aio_queue_depth too small for the
  // app's actual concurrent registrations) and should crash immediately and
  // deterministically rather than silently absorb an unbounded-latency
  // syscall on an RT thread.
  struct io_uring_sqe *GetSqeForRingReconstruction();

  // Acquires an SQE for arming one kernel op -- every steady-state arm site
  // goes through here -- and does the queue-capacity accounting that makes
  // exhaustion predictable instead of a surprise:
  //   * CHECK-fails on an exhausted submission queue with a message naming
  //     --aio_queue_depth and which of the two deterministic causes hit
  //     (arming more than the depth before the first Run()/Poll(), when
  //     nothing can flush, or staging more than the depth between Poll()
  //     calls).
  //   * Counts armed ops (CountArmedOp()) and VLOG(1)s, once per ring, the
  //     first time the concurrent count exceeds the depth -- the earliest
  //     moment the configuration is known to be undersized, usually app
  //     startup, long before either crash above or CQ-overflow degradation
  //     is hit.  VLOG rather than LOG because arm sites run on RT paths --
  //     see CountArmedOp().
  struct io_uring_sqe *ArmSqe();
  // The accounting half of ArmSqe(), shared with
  // GetSqeForRingReconstruction() (which acquires its SQE by draining
  // instead).  See armed_ops_.
  void CountArmedOp();

  // Flushes queued SQEs to the kernel.  No-op if the ring hasn't been
  // enabled yet (see EnsureBound()): io_uring rejects submission on a ring
  // created with IORING_SETUP_R_DISABLED until io_uring_enable_rings()
  // succeeds.  Entries already placed via io_uring_get_sqe() are still
  // valid and wait in the local SQ ring; the first submit after
  // EnsureBound() enables it picks them up.  This is what lets scheduling
  // calls made before the owning Aio's first Run()/Poll() -- e.g. arming a
  // timer from a setup thread, before a worker thread calls Run() -- keep
  // working instead of failing outright.
  void MaybeSubmit();
  // Binds submitter_tid_ to the calling thread.  The first call only, also
  // calls io_uring_enable_rings() (unless DowngradeFromSingleIssuer() has
  // already run for this ring).  Called at the top of Poll(), so it covers
  // both Run() (whose loop calls Poll() as its first action) and tests that
  // call Poll() directly.  Idempotent no-op after the first call.
  void EnsureBound();
  // Tears down the SINGLE_ISSUER ring and rebuilds on the unconstrained
  // COOP_TASKRUN tier, for the one threading shape SINGLE_ISSUER cannot
  // serve (constructed on one thread, driven on another -- see
  // InitSingleIssuerRing()'s comment).
  //
  // Only reachable from EnsureBound()'s first call, i.e. before any
  // Poll() has ever driven this ring.  That is what makes the swap safe:
  // the ring was created IORING_SETUP_R_DISABLED and never enabled, and
  // MaybeSubmit() refuses to submit until it is, so the kernel has never
  // seen a single SQE.  Everything "armed" so far exists as userspace
  // bookkeeping (the wakeup read, the legacy-fd epoll poll, receiver
  // registrations, active timers) plus SQEs queued locally in the old SQ
  // ring.  Those queued SQEs die with the old ring's mmap;
  // ReArmPersistentRegistrations() regenerates fresh ones from the
  // bookkeeping, so nothing observable is lost.  The one thing that
  // cannot be regenerated -- a raw AsyncRead/AsyncWrite submitted before
  // the first Poll() -- is refused loudly: the CHECK below dies rather
  // than drop it silently.
  //
  // TODO(austin): restructure the remaining construct-here/Run()-there
  // callers (test helpers) to live on a single thread for their whole
  // lifetime, then delete this downgrade path and the COOP_TASKRUN tier
  // with it.
  void DowngradeFromSingleIssuer();
  // Re-submits the wakeup read, fd registrations, thread-signal receivers,
  // and active timers to the current ring.  Used by
  // DowngradeFromSingleIssuer(): it replaces the ring with a fresh one and
  // needs to restore everything that was registered on the old one.
  void ReArmPersistentRegistrations();
  // ABSL_CHECKs that the calling thread is submitter_tid_.  Two cases where
  // this is a no-op: before the first EnsureBound() call (submitter_tid_
  // unset, so pre-Run() setup calls are allowed from any thread), and after
  // DowngradeFromSingleIssuer() (single_issuer_ false, so nothing enforces
  // same-thread-ness anymore).  Without this, IORING_SETUP_SINGLE_ISSUER
  // would still catch a violation, but only once something actually
  // submits, and only with a bare -EEXIST -- this gives an immediate,
  // actionable crash instead, including from the destructor, which the
  // kernel itself has no opportunity to check.  Not applied to Quit(): it
  // never touches the ring (just writes to an eventfd) and is documented as
  // callable from any thread.
  void CheckSubmitterThread() const;

  // True once the ring can accept submissions.  Set by EnsureBound() the
  // first time io_uring_enable_rings() succeeds.  Always true when
  // !single_issuer_: that tier has no enable step and is live immediately.
  // Gates MaybeSubmit().
  bool ring_enabled_ = false;
  // Whether the current ring uses the SINGLE_ISSUER tier -- see
  // InitSingleIssuerRing() comment.  Goes false, permanently, after
  // DowngradeFromSingleIssuer() runs.  Gates the enforcement in
  // CheckSubmitterThread() and the enable step in EnsureBound().  Does not
  // gate submitter_tid_ tracking itself -- both tiers still use that for
  // EnsureBound()'s idempotency check.
  bool single_issuer_ = false;
  // The thread that constructed this IoUringImpl.  EnsureBound() compares
  // this against the first caller's thread to detect the mismatch
  // DowngradeFromSingleIssuer() exists to handle.  Unlike submitter_tid_,
  // this is set once and never cleared.
  pid_t construction_tid_ = 0;
  // The thread that first called Poll() (directly, or via Run()) on this
  // ring since it was last (re)constructed -- see EnsureBound() and
  // CheckSubmitterThread().
  std::optional<pid_t> submitter_tid_;

  struct io_uring ring;
  EventFD event_fd;

  // The immortal identity every async-cancel ack is submitted under -- see
  // the EncodeUserData() block for why acks deliberately never name their
  // target.  Never armed as an op itself; its generation stays 0, so every
  // ack drains as "current" against it.  cancel_acks_outstanding_ is a
  // pure sanity counter backing DrainCompletions()'s CHECK that acks never
  // arrive unexpectedly; nothing gates on it.  Reset (with the rest of the
  // in-flight bookkeeping) on ring rebuild, since a dead ring's acks never
  // arrive.
  AsyncRequest cancel_ack_sentinel_;
  int cancel_acks_outstanding_ = 0;

  // Capacity of the submission queue (ring.sq.ring_entries; the kernel may
  // round --aio_queue_depth up to a power of two).  Refreshed whenever the
  // ring is (re)built.
  uint32_t sq_capacity_ = 0;
  // Kernel ops currently armed: one per live timer poll, thread-signal
  // receiver poll, in-flight AsyncRead/AsyncWrite, unacknowledged Cancel(),
  // plus the wakeup read and the legacy-epoll poll.  Incremented by
  // CountArmedOp() at every arm site; decremented in DrainCompletions()
  // when a terminal CQE or a cancel ack drains; reset on ring rebuild
  // (in-flight ops die with the old ring).  This is the number that
  // predicts queue exhaustion: the SQ must hold `armed_ops_` staged
  // entries before the first Run()/Poll() (nothing can flush a disabled
  // ring) and the CQ (2x the depth) absorbs up to one pending completion
  // per armed op, so "peak armed_ops_ <= --aio_queue_depth" is the sizing
  // rule -- checked and warned about at arm time (ArmSqe()) rather than
  // discovered as a bare exhaustion crash or silent CQ-overflow
  // degradation later.
  int armed_ops_ = 0;
  // Ensures the undersized---aio_queue_depth VLOG fires once per ring,
  // not once per arm.
  bool queue_depth_warned_ = false;

  // Link accessors for requests on pending_dispatch_ -- the links live in
  // AsyncRequest::internal_state (AioState::link), which is a union, so
  // membership is tracked by AioState::link.queued rather than the links
  // themselves.
  struct DispatchLinkTraits {
    static AsyncRequest *&next(AsyncRequest *request) {
      return State(request).link.next;
    }
    static AsyncRequest *&prev(AsyncRequest *request) {
      return State(request).link.prev;
    }
  };
  // FIFO of resolved requests waiting for their callback to run -- see
  // AioState::link for why the links live there.  Populated only by
  // DrainCompletions(),
  // drained only by ReapCompletions() -- see both for why extraction and
  // dispatch are deliberately two separate steps.
  IntrusiveDoublyLinkedList<AsyncRequest, DispatchLinkTraits> pending_dispatch_;
  // True while ReapCompletions() is dispatching.  Backs Poll()'s
  // reentrancy CHECK, and tells teardown paths that a callback frame may
  // be live so they park state instead of freeing it.
  bool dispatching_ = false;
  // Set by ReapCompletions()'s dispatch loop once it has delivered a
  // user-visible completion, which is what ends that Poll()'s dispatch.
  // Internal plumbing -- the wakeup read, the legacy-epoll poll re-arm, a
  // legacy firing that turns out to have nothing left to report -- must not
  // consume the budget: those are queued at a rate that is an io_uring
  // implementation detail, and spending a whole Poll() on each would make
  // "how many Poll()s until my callback runs" backend-dependent all over
  // again, which is the entire thing the one-per-Poll() contract fixes.
  bool user_dispatch_ = false;

  // Caller-submitted AsyncRead/AsyncWrite requests currently in flight
  // (AioState::raw_io set; the internal wakeup read excluded).  There is
  // no registry that could re-arm these across a ring rebuild, so
  // DowngradeFromSingleIssuer() CHECKs this is zero rather than dropping
  // them silently.
  int raw_requests_in_flight_ = 0;

  std::atomic<bool> run{false};
  std::atomic<bool> quit_requested{false};

  std::vector<std::function<void()>> before_wait_functions;
  // True while Poll() is running the before-wait functions; BeforeWait()
  // CHECKs it -- see there.
  bool in_before_wait_ = false;

  // Readiness state for one fd registered via OnReadable/OnWritable/OnError/
  // OnEvents.  Unlike AsyncRead/AsyncWrite/timers, this has no io_uring
  // request of its own: all legacy fds share one embedded epoll instance
  // (legacy_epoll_fd_) and one repeatedly-rearmed poll on it -- see
  // UpdateLegacyEpoll()/SubmitLegacyEpollPoll()/DrainLegacyEpoll() and
  // documentation/adr/0001-aio-io-uring-single-issuer.md.
  struct LegacyState {
    int fd = -1;
    // kIn/kPri/kOut/kErr encoding (matches SetEvents()'s documented "raw
    // epoll events" contract).
    uint32_t events = 0;
    std::function<void()> in_fn = nullptr;
    std::function<void()> out_fn = nullptr;
    std::function<void()> err_fn = nullptr;
    std::function<void(uint32_t)> events_fn = nullptr;
    // Whether this fd currently has an epoll_ctl registration (ADD done, DEL
    // not yet done) on legacy_epoll_fd_.
    bool epoll_registered = false;
    // Intrusive link for retired_legacy_states_ -- see DeleteFd().
    LegacyState *next_retired = nullptr;
  };
  absl::flat_hash_map<int, std::unique_ptr<LegacyState>> legacy_states;
  struct RetiredLegacyTraits {
    static LegacyState *&next(LegacyState *state) {
      return state->next_retired;
    }
  };
  // LegacyStates removed by DeleteFd()/ForgetClosedFd() from inside a
  // dispatch are parked here rather than freed: one of the removed
  // state's own std::functions can be the code currently executing (see
  // DeleteFd()), so the state must stay allocated until no callback can
  // still be running out of it.  Swept at the end of the same outermost
  // dispatch that parked them (ReapCompletions()) -- a context that is
  // deterministically non-RT, because DeleteFd()/ForgetClosedFd() are
  // CheckNotRealtime().  There is deliberately no RT conditional anywhere
  // in this lifecycle.  Owned raw pointers (released from their
  // unique_ptrs at retirement).
  IntrusiveStack<LegacyState, RetiredLegacyTraits> retired_legacy_states_;

  // One shared epoll instance backing every OnReadable/OnWritable/OnError/
  // OnEvents registration, instead of a separate io_uring poll op per fd.
  // Mask changes (EnableWritable/DisableWritable/SetEvents/etc.) become a
  // plain epoll_ctl(MOD) call -- no io_uring interaction, no cancel-and-
  // resubmit -- since the kernel already supports updating an armed epoll
  // registration in place, unlike io_uring's POLL_ADD.  See
  // UpdateLegacyEpoll().
  int legacy_epoll_fd_ = -1;
  // The single io_uring registration for the whole legacy-fd subsystem: a
  // poll on legacy_epoll_fd_ itself (an epoll instance's fd is itself
  // pollable -- it reports readable exactly when epoll_wait() on it would
  // return at least one event).  Single-shot, explicitly re-armed on every
  // firing rather than multishot -- see SubmitLegacyEpollPoll() for why
  // multishot doesn't work here.
  AsyncRequest legacy_epoll_request_;
  // (Re-)arms legacy_epoll_request_.  Called at construction, again every
  // time it fires (from its own completion callback), and again any time
  // the ring is rebuilt (DowngradeFromSingleIssuer(), via
  // ReArmPersistentRegistrations()).
  void SubmitLegacyEpollPoll();
  // Adds/updates/removes state's epoll_ctl registration on legacy_epoll_fd_
  // to match its current `events` mask.  Called after every OnReadable/
  // OnWritable/OnError/OnEvents/EnableWritable/DisableWritable/SetEvents
  // call that actually changes the mask.
  void UpdateLegacyEpoll(LegacyState *state);
  // legacy_epoll_request_'s completion callback (via SubmitLegacyEpollPoll()):
  // dispatches the one fd epoll_wait() reports ready on legacy_epoll_fd_,
  // holding that fd's LegacyState pointer across its in/out/err callbacks --
  // a callback that deletes its own registration parks the state with fd
  // tombstoned to -1, which is what stops the remaining bits (see the
  // body).  One event per firing, deliberately: see the body for why a
  // batch would be both unfair and unterminating.  Level-triggered, so a
  // callback that doesn't fully consume a fd's readiness (e.g. a partial
  // read) sees it again on the next firing.
  // Returns whether it actually ran a legacy callback -- ReapCompletions()
  // spends its one-per-Poll() budget on user-visible work only, and this
  // firing routinely finds nothing left to report.
  bool DrainLegacyEpoll();

  struct ThreadSignalReceiverState {
    ipc_lib::ThreadSignalReceiver *receiver = nullptr;
    FileDescriptor fd = -1;
    std::function<void()> callback;
    AsyncRequest request;
    // Stored by Submit() so the completion callback can resubmit when the
    // kernel terminates the multishot op (see the callback).
    IoUringImpl *impl = nullptr;
    ThreadSignalReceiverState *next_orphan = nullptr;
    // Set by UnregisterThreadSignalReceiver() -- a plain flag rather than
    // nulling `callback`, which can be the std::function currently
    // executing (a receiver callback may unregister its own receiver);
    // destroying it there would free the running lambda's captures.  Once
    // set, the trampoline never touches the signalfd again: the fd is the
    // caller's (documented destroyable the moment unregistration returns),
    // and may already back a successor receiver whose wakeups this state
    // must not consume.
    bool unregistered = false;
    // True while the trampoline is inside `callback()`.  Lets
    // UnregisterThreadSignalReceiver() destroy the callback immediately
    // when it is not the code calling it, and defer to the end of the
    // dispatch when it is (see deferred_receiver_callback_clear_).
    bool in_callback = false;

    void Submit(IoUringImpl *impl);
  };
  // The one receiver state whose user callback unregistered its own
  // receiver from inside that very callback: the std::function is still
  // executing, so UnregisterThreadSignalReceiver() defers destroying it to
  // the end of the outermost dispatch (ReapCompletions()), once every
  // frame has unwound.  A single slot suffices -- dispatch is
  // single-threaded and never nests, so at most one receiver callback can
  // be executing.
  ThreadSignalReceiverState *deferred_receiver_callback_clear_ = nullptr;
  // The active receiver, if any.  Only one may be registered at a time:
  // every ThreadSignalSender wakeup arrives via the same signal
  // (ipc_lib::kWakeupSignal), so multiple receivers on one loop could not
  // tell their wakeups apart -- and nothing needs more than one.
  std::unique_ptr<ThreadSignalReceiverState> receiver_state_;

  struct TimerOrphanTraits {
    static IoUringTimerState *&next(IoUringTimerState *state) {
      return state->next_orphan;
    }
  };
  struct ReceiverOrphanTraits {
    static ThreadSignalReceiverState *&next(ThreadSignalReceiverState *state) {
      return state->next_orphan;
    }
  };
  // States whose owner died while a kernel op was still in flight.
  // Intrusive stacks: the recycling sweep runs inside Poll() on RT
  // threads, so moving an orphan to the freelist must not allocate.  Swept
  // by RecycleDrainedOrphans(); scrubbed wholesale on ring rebuild
  // (completions die with the old ring).  Owned raw pointers (released
  // from their unique_ptrs at orphan time).
  IntrusiveStack<IoUringTimerState, TimerOrphanTraits> orphaned_timers_;
  IntrusiveStack<ThreadSignalReceiverState, ReceiverOrphanTraits>
      orphaned_receivers_;
  // Drained orphans are recycled here (and reused by MakeTimerState() /
  // RegisterThreadSignalReceiver) rather than freed: recycling is pointer
  // manipulation, legal on any thread including RT, and it bounds memory
  // by peak usage instead of cumulative churn.  Actual deallocation only
  // happens in ~IoUringImpl().
  IntrusiveStack<IoUringTimerState, TimerOrphanTraits> free_timers_;
  IntrusiveStack<ThreadSignalReceiverState, ReceiverOrphanTraits>
      free_receivers_;
  void RecycleDrainedOrphans();

  struct ActiveTimerTraits {
    static Aio::TimerState *&next(Aio::TimerState *state) {
      return state->next_active;
    }
    static Aio::TimerState *&prev(Aio::TimerState *state) {
      return state->prev_active;
    }
  };
  // Every *live* timer -- linked by Initialize(), unlinked only when the
  // owning Timer is destroyed -- so ring rebuilds can re-arm the poll each
  // one holds on its timerfd (ReArmPersistentRegistrations()).  is_active
  // tracks membership.  Deliberately not "every scheduled timer", which is
  // what it used to mean back when a timer *was* an io_uring op: whether a
  // timer is currently scheduled now lives entirely in its timerfd, and the
  // poll needs re-arming either way.
  IntrusiveDoublyLinkedList<Aio::TimerState, ActiveTimerTraits> active_timers_;
  void LinkTimer(Aio::TimerState *state) {
    if (state->is_active) return;
    active_timers_.PushFront(state);
    state->is_active = true;
    state->canceling = false;
  }
  void UnlinkTimer(Aio::TimerState *state) {
    if (!state->is_active) return;
    active_timers_.Remove(state);
    state->is_active = false;
  }
};

void IoUringImpl::UpdateLegacyEpoll(LegacyState *state) {
  uint32_t desired_epoll = 0;
  if (state->events & kIn) desired_epoll |= EPOLLIN;
  if (state->events & kPri) desired_epoll |= EPOLLPRI;
  if (state->events & kOut) desired_epoll |= EPOLLOUT;
  if (state->events & kErr) desired_epoll |= EPOLLERR;
  // Not EPOLLHUP: never requested explicitly -- the kernel always reports
  // EPOLLERR/EPOLLHUP regardless of the registered mask (epoll_ctl(2)).
  // DrainLegacyEpoll() folds EPOLLHUP into the same kErr "error" bit as
  // EPOLLERR, and routes that bit to the read/write handler when no error
  // handler is registered -- see there.

  // Registered-vs-not is decided on the caller's untranslated mask,
  // exactly as EPoll::DoEpollCtl() always did: a mask of only bits the
  // translation above drops (e.g. a bare EPOLLHUP) must keep the fd
  // registered -- with an empty event set, which the kernel accepts and
  // still augments with EPOLLERR/EPOLLHUP -- not silently unregister it.
  if (state->events == 0) {
    if (state->epoll_registered) {
      int ret = epoll_ctl(legacy_epoll_fd_, EPOLL_CTL_DEL, state->fd, nullptr);
      ABSL_PCHECK(ret == 0 || errno == ENOENT)
          << "epoll_ctl DEL failed for fd " << state->fd;
      state->epoll_registered = false;
    }
    return;
  }

  struct epoll_event ev;
  std::memset(&ev, 0, sizeof(ev));
  ev.events = desired_epoll;
  // The fd is enough: DrainLegacyEpoll() dispatches immediately after its
  // epoll_wait() (nothing can delete a registration in between), and
  // staleness *during* dispatch is handled by the fd = -1 tombstone on the
  // parked state, not by re-consulting this field.
  ev.data.fd = state->fd;
  if (state->epoll_registered) {
    int ret = epoll_ctl(legacy_epoll_fd_, EPOLL_CTL_MOD, state->fd, &ev);
    ABSL_PCHECK(ret == 0) << "epoll_ctl MOD failed for fd " << state->fd;
  } else {
    int ret = epoll_ctl(legacy_epoll_fd_, EPOLL_CTL_ADD, state->fd, &ev);
    ABSL_PCHECK(ret == 0) << "epoll_ctl ADD failed for fd " << state->fd;
    state->epoll_registered = true;
  }
}

void IoUringImpl::SubmitLegacyEpollPoll() {
  // Single-shot, re-armed on every firing.  POLL_ADD (multishot included)
  // only completes on a fresh wait-queue edge, not on a still-true
  // condition -- a multishot registration silently stops firing for
  // level-style readiness like "still writable" (hung
  // EPollLikeBasicWritable live).  A fresh single-shot arm re-checks
  // current readiness at arm time, so it fires immediately if the
  // condition never went away.
  struct io_uring_sqe *sqe = ArmSqe();

  io_uring_prep_poll_add(sqe, legacy_epoll_fd_, POLLIN);
  io_uring_sqe_set_data64(sqe, NewIncarnationUserData(&legacy_epoll_request_));

  legacy_epoll_request_.callback = [](Completion completion, void *context) {
    auto *impl = static_cast<IoUringImpl *>(context);
    // Nothing ever cancels this poll, so any failure is unexpected -- and
    // absorbing it would silently stop every OnReadable/OnWritable/
    // OnError/OnEvents callback on this loop (this poll is the only thing
    // that ever fires them).
    ABSL_CHECK(aos::IsOk(completion.status))
        << "Poll on the embedded legacy-fd epoll instance failed: "
        << aos_strerror(completion.result);
    // Re-arm before dispatching: a single-shot poll is consumed by this
    // firing, so the next check needs to be queued now, regardless of
    // what a callback below ends up doing (including deleting fds this
    // very drain pass would otherwise still be about to look at).
    impl->SubmitLegacyEpollPoll();
    if (impl->DrainLegacyEpoll()) {
      impl->user_dispatch_ = true;
    }
  };
  legacy_epoll_request_.context = this;
  legacy_epoll_request_.user_data = &legacy_epoll_request_;
  legacy_epoll_request_.done = false;
}

bool IoUringImpl::DrainLegacyEpoll() {
  // One event per firing.  Fairness comes from level-triggered epoll
  // itself: a still-ready fd goes back on the ready list's tail, so
  // successive firings rotate through every ready fd.  The re-armed
  // POLL_ADD (queued before this by the caller) checks readiness at arm
  // time, so any remaining events fire it again on the next Poll().  This
  // is also what keeps legacy fds fair against the ring's native work: a
  // Poll() dispatches at most one legacy event alongside that cycle's
  // native completions, instead of a whole batch crowding them out.  Not
  // a drain-to-empty loop: that busy-spins forever on any fd whose
  // callback doesn't consume its own readiness (e.g. LegacyFdTest's
  // OnEvents callback, which just counts -- confirmed live).  One event
  // also means no stale-batch hazard: nothing runs between epoll_wait()
  // and dispatch, so the by-fd lookup below cannot race a deletion.
  struct epoll_event event;
  int n = epoll_wait(legacy_epoll_fd_, &event, 1, 0);
  if (n < 0) {
    if (errno == EINTR) return false;
    ABSL_PCHECK(n >= 0) << "epoll_wait on the embedded legacy-fd epoll "
                           "instance failed";
  }
  if (n == 0) {
    return false;
  }

  const int ready_fd = event.data.fd;
  LegacyState *state = [this, ready_fd]() -> LegacyState * {
    auto it = legacy_states.find(ready_fd);
    return it == legacy_states.end() ? nullptr : it->second.get();
  }();
  if (state == nullptr) return false;

  uint32_t ready_epoll = 0;
  if (event.events & EPOLLIN) ready_epoll |= kIn;
  if (event.events & EPOLLPRI) ready_epoll |= kPri;
  if (event.events & EPOLLOUT) ready_epoll |= kOut;
  if (event.events & EPOLLERR) ready_epoll |= kErr;
  if (event.events & EPOLLHUP) ready_epoll |= kErr;

  if (state->events_fn) {
    state->events_fn(ready_epoll);
    return true;
  }
  // The kernel reports EPOLLERR/EPOLLHUP whether or not they were requested
  // (epoll_ctl(2)), and a bare EPOLLHUP is level-triggered state that cannot
  // be consumed: nothing is read()able and there is nothing to reset.  If no
  // err_fn is registered, the error bit must still reach *some* handler --
  // otherwise this firing dispatches nothing while the re-armed poll (queued
  // by our caller, before this drain) finds the epoll instance still
  // readable and fires again immediately, a silent busy loop that blocking
  // Poll(true) calls spin through forever (confirmed live: a pipe reporting
  // bare EPOLLHUP after its write end closed and its data drained).  Route
  // it to in_fn -- a read() there observes the EOF/error, the conventional
  // readiness-loop treatment of HUP -- or, failing that, to out_fn, where a
  // write() observes EPIPE.  Either callback then has the information and
  // the responsibility to DeleteFd(), which is what actually ends the
  // level-triggered re-firing.
  uint32_t dispatch_epoll = ready_epoll;
  if ((dispatch_epoll & kErrorEvents) && !state->err_fn) {
    if (state->in_fn) {
      dispatch_epoll |= kIn;
    } else if (state->out_fn) {
      dispatch_epoll |= kOut;
    } else {
      // Unreachable today: a LegacyState without events_fn always has at
      // least one of in_fn/out_fn/err_fn.  Keep the loud failure (matching
      // EPoll's historical CHECK on unhandled error events) rather than
      // silently busy-looping if that ever changes.
      ABSL_LOG(FATAL) << "Error event 0x" << std::hex << event.events
                      << std::dec << " on fd " << ready_fd
                      << " with no handler registered to observe it";
    }
  }
  // The state pointer is held across all three callbacks -- the same shape
  // EPoll::Poll() always had, made safe by parking: a callback that
  // DeleteFd()s this registration (its own fd is the ordinary case) parks
  // the state, still allocated, with fd tombstoned to -1.  Checking that
  // tombstone is the whole staleness story.  In particular there is no
  // by-fd re-lookup to get confused by a callback that deletes its fd,
  // closes it, and registers a fresh fd that reuses the same number --
  // the old event's remaining bits die with the tombstone.
  bool dispatched = false;
  if (dispatch_epoll & kInEvents) {
    // CHECK rather than skip, as EPoll::InOutEventData::DoCallbacks()
    // always has: silently dropping an enabled readiness bit with no
    // handler turns the misuse into an unconsumable level-triggered event
    // -- a silent busy loop.  (Unreachable via the error-routing above,
    // which only sets bits whose handler exists.)
    ABSL_CHECK(state->in_fn)
        << ": No handler registered for input events on descriptor "
        << ready_fd;
    state->in_fn();
    dispatched = true;
    if (state->fd == -1) return dispatched;
  }
  if (dispatch_epoll & kOutEvents) {
    ABSL_CHECK(state->out_fn)
        << ": No handler registered for output events on descriptor "
        << ready_fd;
    state->out_fn();
    dispatched = true;
    if (state->fd == -1) return dispatched;
  }
  // No CHECK on err_fn, unlike in/out above: error bits arrive unsolicited
  // (the kernel reports EPOLLERR/EPOLLHUP regardless of the registered
  // mask), so a null err_fn with kErr set is the routine case -- the bit
  // was already delivered through the in_fn/out_fn routing above, or died
  // on that routing's FATAL if nothing could observe it.
  if ((dispatch_epoll & kErrorEvents) && state->err_fn) {
    state->err_fn();
    dispatched = true;
  }
  return dispatched;
}

void IoUringImpl::ThreadSignalReceiverState::Submit(IoUringImpl *impl) {
  this->impl = impl;
  struct io_uring_sqe *sqe = impl->ArmSqe();

  io_uring_prep_poll_multishot(sqe, fd, POLLIN);
  io_uring_sqe_set_data64(sqe, NewIncarnationUserData(&request));

  request.callback = [](Completion completion, void *context) {
    auto state = static_cast<ThreadSignalReceiverState *>(context);
    if (state->unregistered) {
      // A CQE that was already kernel-side (or queued) when the receiver
      // unregistered.  Never touch the signalfd: it belongs to the caller
      // again -- who may have destroyed the receiver, or registered a
      // successor on the same fd whose wakeups this orphan must not
      // consume.  No revival either.  (-ECANCELED from the unregister's
      // own cancel lands here too, silently, as it should.)
      return;
    }
    if (aos::IsOk(completion.status)) {
      // Coalesce: consume every pending signal first, then invoke the
      // callback once.  The callback checks all of its sources anyway, so
      // per-signal invocations would just be redundant scans.  The order
      // is the load-bearing part: no signal is ever consumed after the
      // last callback, so a wakeup landing during (or after) the callback
      // stays pending, re-fires the multishot poll, and produces another
      // callback on a later Poll() -- consumed-then-notified is the
      // guarantee, matching EpollImpl's ConsumeWakeup()-then-callback.
      //
      // Batched reads: signalfd dequeues as many pending siginfos as fit
      // the buffer, and a short count means the queue was empty at that
      // moment -- so the common one-signal wakeup costs exactly one
      // syscall, with no EAGAIN bounce through the kernel to terminate
      // the loop.
      struct signalfd_siginfo siginfos[16];
      bool consumed_any = false;
      while (true) {
        const ssize_t res = read(state->fd, siginfos, sizeof(siginfos));
        if (res > 0) {
          consumed_any = true;
          if (static_cast<size_t>(res) < sizeof(siginfos)) {
            break;  // Short read: nothing was left pending.
          }
        } else if (res < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
          break;
        } else {
          ABSL_LOG(FATAL) << "Failed to read from signalfd: "
                          << aos_strerror(errno);
        }
      }
      if (consumed_any) {
        state->impl->user_dispatch_ = true;
        state->in_callback = true;
        state->callback();
        state->in_callback = false;
        if (state->unregistered) {
          // The callback unregistered its own receiver.  The fd is the
          // caller's again: skip the revival check.  `state` itself is
          // still valid -- unregistration parked it.
          return;
        }
      }
    } else {
      // An error completion while the registration is still wanted (the
      // unregistered case returned above).  Dying is deliberate: re-arming
      // below would resubmit the same failing op in a silent infinite
      // loop, and swallowing it would silently disable every watcher
      // wakeup on this loop forever.  -EINVAL specifically means the
      // kernel lacks IORING_POLL_ADD_MULTI (added in 5.13, below the floor
      // RequireMinimumKernelVersion() enforces -- see
      // documentation/adr/0001-aio-io-uring-single-issuer.md).
      ABSL_LOG(FATAL) << "Thread-signal receiver multishot poll failed: "
                      << aos_strerror(completion.result);
    }
    // A multishot poll op normally stays armed forever, posting one CQE
    // per firing with IORING_CQE_F_MORE set ("more coming") -- request.done
    // stays false.  The kernel revokes that whenever it cannot post such a
    // per-firing CQE: with the CQ full, io_req_post_cqe() fails and
    // io_poll_check_events() (linux/io_uring/poll.c) terminates the whole
    // op with a normal, final completion (no F_MORE, so request.done goes
    // true) rather than dropping the event.  A final completion on a
    // still-registered receiver can only be that: nothing else ends this
    // op while anyone still wants it (the unregistered case returned
    // above, and the destructor never delivers completions at all).
    // Re-arm, or every watcher wakeup after a CQ overflow would be lost
    // forever.
    if (state->request.done) {
      state->Submit(state->impl);
      state->impl->MaybeSubmit();
    }
  };
  request.context = this;
  request.user_data = &request;
  request.done = false;
}

// Timer destruction, without blocking: disarm the timerfd, then either free
// now (if the poll on it has already resolved) or strip the user-facing
// callback, cancel the poll, and park the state on orphaned_timers_ until
// the poll's terminal completion has drained (RecycleDrainedOrphans()).
// The identity -- and the timerfd the poll names -- stay alive the whole
// time, preserving "never free an identity with CQEs in flight".  (The
// cancel's ack is not such a CQE: it names cancel_ack_sentinel_, not this
// state, so it never gates recycling.)
void IoUringImpl::DestroyTimerState(std::unique_ptr<Aio::TimerState> state) {
  // Deterministically illegal under RT: the fast path below frees, and a
  // function that only *sometimes* frees would only sometimes trip the
  // malloc hook -- a data-dependent crash.  Nothing legitimately destroys
  // a timer from an RT thread.
  aos::CheckNotRealtime();
  auto *tstate = static_cast<IoUringTimerState *>(state.get());
  CheckSubmitterThread();
  // Disarm before anything else.  A recycled state must reach the freelist
  // with a quiet timerfd -- Reset() keeps the fd rather than recreating it,
  // so a still-armed timer would otherwise make its next owner's poll fire
  // for a schedule nobody asked for.
  tstate->Cancel(true);
  UnlinkTimer(tstate);
  // Unconditionally, and before the fast path below -- for the same reason
  // CancelRequest() does it unconditionally.  This timer's poll completion
  // can be sitting resolved-but-undispatched on pending_dispatch_ right now
  // (an earlier callback in this very dispatch batch destroying a different
  // timer is an ordinary pattern), and the fast path frees the state.  The
  // dispatch loop would then walk into freed memory when it reached that
  // node -- a live SIGSEGV inside OnTimerFdReadable(), reproduced by
  // DestroyTimerWithTerminatedPollAndQueuedCompletion.  Reachable only when
  // the poll had been terminated by CQ overflow and its terminal completion
  // is drained but not yet dispatched, since a live multishot poll is never
  // `done`.
  UnlinkPendingDispatch(&tstate->request);
  if (tstate->request.done) {
    return;  // state destroys here; nothing in flight names it.
  }
  if (!tstate->request.done && !tstate->canceling) {
    tstate->canceling = true;
    CancelRequest(&tstate->request);
  }
  state.release();
  orphaned_timers_.Push(tstate);
}

// Recycles every orphan whose kernel traffic has fully drained: terminal
// completion drained (done) and dispatched or unlinked (!queued).  Cancel
// acks don't factor in -- they name cancel_ack_sentinel_, never the
// orphan.  Called after dispatch in ReapCompletions(): recycling mid-drain
// would invalidate memory a queued completion or later CQE in the same
// batch could still name.
void IoUringImpl::RecycleDrainedOrphans() {
  auto drained = [](AsyncRequest *req) {
    return req->done && !State(req).link.queued;
  };
  orphaned_timers_.MoveMatchingTo(&free_timers_, [&](IoUringTimerState *state) {
    return drained(&state->request);
  });
  orphaned_receivers_.MoveMatchingTo(&free_receivers_,
                                     [&](ThreadSignalReceiverState *state) {
                                       return drained(&state->request);
                                     });
}

std::unique_ptr<Aio::TimerState> IoUringImpl::MakeTimerState() {
  // Timer construction is a mutating entry point like every other:
  // Initialize() links the timer and arms its poll, so it gets the same
  // thread enforcement as Schedule()/Cancel()/destruction.
  CheckSubmitterThread();
  // Deterministically illegal under RT, like DestroyTimerState(): the
  // freelist miss below allocates, and a function that only sometimes
  // allocates would only sometimes trip the malloc hook.
  aos::CheckNotRealtime();
  if (IoUringTimerState *state = free_timers_.Pop(); state != nullptr) {
    // Reset() rather than whole-object assignment, so the recycled state
    // keeps its already-created (and already-disarmed) timerfd instead of
    // paying timerfd_create()/close() again.  Aio::Timer's constructor
    // calls Initialize(), which re-arms the poll on it.
    state->Reset();
    return std::unique_ptr<Aio::TimerState>(state);
  }
  return std::make_unique<IoUringTimerState>(this);
}

IoUringImpl::IoUringImpl() {
  RequireMinimumKernelVersion();
  construction_tid_ = CachedThreadId();
  uint32_t depth = ::absl::GetFlag(FLAGS_aio_queue_depth);
  InitSingleIssuerRing(&ring, depth);
  sq_capacity_ = ring.sq.ring_entries;
  single_issuer_ = true;
  // Not enabled until EnsureBound() -- see IORING_SETUP_R_DISABLED above.
  ring_enabled_ = false;

  // When wakeup event fd read completes, re-schedule it.
  event_fd.wakeup_req.callback = [](Completion completion, void *context) {
    auto *impl = static_cast<IoUringImpl *>(context);
    // Nothing ever cancels this read, so any failure is unexpected -- and
    // it cannot be absorbed: without a re-armed read, Wakeup() (and
    // therefore Quit()) would silently stop working.
    ABSL_CHECK(aos::IsOk(completion.status))
        << "Wakeup eventfd read failed: " << aos_strerror(completion.result);
    impl->SubmitWakeupRead();
  };
  event_fd.wakeup_req.context = this;

  SubmitWakeupRead();

  legacy_epoll_fd_ = epoll_create1(EPOLL_CLOEXEC);
  ABSL_PCHECK(legacy_epoll_fd_ >= 0)
      << "Failed to create embedded epoll instance";
  SubmitLegacyEpollPoll();
}

IoUringImpl::~IoUringImpl() {
  // Enforces "destroy an Aio on the same thread that called Run()/Poll() on
  // it".  Nothing else in this destructor calls CheckSubmitterThread(), so
  // without this the check would only fire indirectly -- and only if there
  // happened to be something left to Cancel() below, surfacing as a cryptic
  // kernel -EEXIST rather than this clear message.  No-op if Run()/Poll() was
  // never called (submitter_tid_ unset): nothing bound yet to violate.
  CheckSubmitterThread();

  // Owner-facing state must be gone first, exactly as EPoll::~EPoll() has
  // always CHECKed, and as aio.h documents ("All Fds must be cleaned up
  // before this class is destroyed").  A live Aio::Timer is the sharpest
  // case: its destructor dereferences this impl, so a Timer outliving its
  // Aio is a use-after-free that would otherwise go silent.
  ABSL_CHECK(active_timers_.empty())
      << ": An Aio::Timer must be destroyed before its Aio";
  ABSL_CHECK(legacy_states.empty())
      << ": All fds must be removed (DeleteFd()/ForgetClosedFd()) before "
         "destroying the Aio";
  ABSL_CHECK(receiver_state_ == nullptr)
      << ": The ThreadSignalReceiver must be unregistered before destroying "
         "the Aio";

  run = false;
  // Deliberately NO walk of pending_dispatch_ here: the queued requests it
  // names are caller-owned and may already be destroyed -- aio.h permits a
  // request whose callback never ran to die with (or before) its Aio, and
  // BasicPipeReadWrite's natural declaration order does exactly that.  The
  // stale link.queued a surviving request carries out of here is cleared
  // at its next arm instead -- see AsyncRead().
  // No per-request cancels: io_uring_queue_exit() reaps every in-flight op
  // kernel-side, and nothing drains this ring again, so no CQE can ever be
  // observed after this point.  Orphaned states (destroyed after the exit,
  // as members) are therefore safe to free unconditionally.
  io_uring_queue_exit(&ring);
  while (IoUringTimerState *state = orphaned_timers_.Pop()) {
    delete state;
  }
  while (ThreadSignalReceiverState *state = orphaned_receivers_.Pop()) {
    delete state;
  }
  while (IoUringTimerState *state = free_timers_.Pop()) {
    delete state;
  }
  while (ThreadSignalReceiverState *state = free_receivers_.Pop()) {
    delete state;
  }
  // retired_legacy_states_ is expected empty here -- parked states are
  // freed at the end of the dispatch that parked them -- but sweep it as
  // a leak-proof backstop rather than trusting that silently.
  while (LegacyState *state = retired_legacy_states_.Pop()) {
    delete state;
  }
  // Not tied to the ring at all -- legacy fds' own epoll_ctl registrations
  // don't need canceling, just closing the instance that holds them.
  if (legacy_epoll_fd_ >= 0) {
    close(legacy_epoll_fd_);
  }
}

void IoUringImpl::Run() {
  run = true;
  // The loop consults quit_requested as well as run: a Quit() racing this
  // startup can have its `run = false` store clobbered by the store above,
  // leaving quit_requested as the only record that a shutdown was asked for
  // (see AioTest.QuitRacingWithRunStartup).  This also covers a Quit() that
  // landed entirely before Run(): the loop body never executes.
  while (run && !quit_requested) {
    Poll(true);
  }
  // Post-Quit() drain, keeping EPoll::Run()'s contract: whatever is
  // already resolved -- queued-but-undispatched completions on
  // pending_dispatch_, ready CQEs -- is delivered before returning rather
  // than dropped.  ReapCompletions() depends on this when it reports
  // progress for completions drained by an earlier Poll().
  while (Poll(false)) {
  }
  run = false;
  quit_requested = false;
}

struct io_uring_sqe *IoUringImpl::GetSqeForRingReconstruction() {
  struct io_uring_sqe *sqe = io_uring_get_sqe(&ring);
  if (sqe == nullptr) {
    // The submission queue is full.  Flush what is queued to the kernel,
    // which frees the entries, then get one from the now-drained queue.
    // A plain io_uring_submit() (not MaybeSubmit()) is correct here: this is
    // reachable only from within ReArmPersistentRegistrations(), whose
    // caller (DowngradeFromSingleIssuer()) enables the fresh ring before
    // doing any of the reconstruction work that calls this -- see there.
    int ret = io_uring_submit(&ring);
    ABSL_PCHECK(ret >= 0) << "io_uring_submit failed: " << aos_strerror(-ret);
    sqe = io_uring_get_sqe(&ring);
    ABSL_CHECK(sqe != nullptr) << "Out of SQEs after draining the queue";
  }
  CountArmedOp();
  return sqe;
}

struct io_uring_sqe *IoUringImpl::ArmSqe() {
  struct io_uring_sqe *sqe = io_uring_get_sqe(&ring);
  ABSL_CHECK(sqe != nullptr)
      << "Out of io_uring SQEs: more than --aio_queue_depth (" << sq_capacity_
      << ") submissions staged since the last flush, with " << armed_ops_
      << " kernel ops already armed.  "
      << (ring_enabled_
              ? "Poll() is what flushes staged entries; submitting this many "
                "operations between Poll() calls needs a larger "
                "--aio_queue_depth."
              : "Nothing can flush before the first Run()/Poll() call enables "
                "the ring, so every operation armed before then holds a "
                "staged entry; arm fewer than --aio_queue_depth operations "
                "(timers, receivers, raw requests, plus two internal "
                "registrations) before first driving the loop, or raise the "
                "flag.");
  CountArmedOp();
  return sqe;
}

void IoUringImpl::CountArmedOp() {
  ++armed_ops_;
  if (armed_ops_ > static_cast<int>(sq_capacity_) && !queue_depth_warned_) {
    queue_depth_warned_ = true;
    // VLOG, not LOG: this can run on an RT thread (arm sites live inside
    // Poll() and on the documented RT-safe submission paths), where
    // default-on logging allocates and locks.  Disabled-verbosity VLOG is
    // one atomic load -- RT-fine -- and a user who enables verbosity on
    // an RT app has made that trade explicitly.
    ABSL_VLOG(1)
        << armed_ops_
        << " concurrently armed io_uring operations exceed --aio_queue_depth ("
        << sq_capacity_
        << ").  This works, but in a degraded mode: completions beyond the "
           "CQ take the kernel's slow overflow path and multishot polls get "
           "terminated and re-armed, and arming this many before the first "
           "Run()/Poll() -- or submitting this many between Poll() calls -- "
           "is a crash.  Raise --aio_queue_depth to at least the peak "
           "concurrent operation count: every live timer, thread-signal "
           "receiver, in-flight AsyncRead/AsyncWrite, and unacknowledged "
           "Cancel() holds one slot, plus two internal registrations.";
  }
}

void IoUringImpl::MaybeSubmit() {
  if (!ring_enabled_) return;
  int ret = io_uring_submit(&ring);
  // A streaming CHECK is fine on this RT-reachable path: the failure is
  // fatal either way.  (Under the RT malloc hooks the message build
  // trips NewHook's own FATAL first, which loses this message but still
  // dies with this frame in the stack.)
  ABSL_PCHECK(ret >= 0) << "io_uring_submit failed: " << aos_strerror(-ret);
}

void IoUringImpl::EnsureBound() {
  if (submitter_tid_) return;
  if (single_issuer_ && CachedThreadId() != construction_tid_) {
    DowngradeFromSingleIssuer();
  }
  if (single_issuer_) {
    int ret = io_uring_enable_rings(&ring);
    ABSL_PCHECK(ret == 0) << "io_uring_enable_rings failed: "
                          << aos_strerror(-ret);
    ring_enabled_ = true;
  }
  submitter_tid_ = CachedThreadId();
}

void IoUringImpl::DowngradeFromSingleIssuer() {
  // Rebuilding the ring would silently drop any caller-submitted
  // AsyncRead/AsyncWrite still in flight -- there is no registry to
  // re-arm them from (see ReArmPersistentRegistrations()).  Refuse
  // loudly instead.
  ABSL_CHECK_EQ(raw_requests_in_flight_, 0)
      << ": Cannot downgrade for the construct-here/run-there thread shape "
         "with caller-submitted AsyncRead/AsyncWrite requests in flight -- "
         "they would be dropped silently.  Submit raw requests from the "
         "thread that drives the loop, after its first Run()/Poll().";
  io_uring_queue_exit(&ring);
  // The unconstrained COOP_TASKRUN tier: no thread binding, no
  // destructor-thread requirement, ring live immediately.  Built inline
  // because this is the only place it exists -- every other ring in this
  // file is SINGLE_ISSUER (see InitSingleIssuerRing()) -- and it is
  // deleted along with this whole function by the TODO above.  Same kernel
  // floor (RequireMinimumKernelVersion()): these flags only need 5.19, but
  // the primary tier's SINGLE_ISSUER | DEFER_TASKRUN need 6.1.
  uint32_t depth = ::absl::GetFlag(FLAGS_aio_queue_depth);
  int ret = InitOneRing(&ring, depth,
                        IORING_SETUP_COOP_TASKRUN | IORING_SETUP_TASKRUN_FLAG);
  ABSL_PCHECK(ret == 0) << "io_uring_queue_init failed: " << aos_strerror(-ret)
                        << " on kernel " << KernelRelease()
                        << " -- aos requires a Linux kernel >= 6.1 "
                           "(6.12 recommended).";
  sq_capacity_ = ring.sq.ring_entries;
  single_issuer_ = false;
  ring_enabled_ = true;
  ReArmPersistentRegistrations();
}

void IoUringImpl::CheckSubmitterThread() const {
  if (single_issuer_ && submitter_tid_) {
    ABSL_CHECK_EQ(CachedThreadId(), *submitter_tid_)
        << ": Aio touched from a different thread than the one that first "
           "called Run()/Poll() on it -- see the threading constraints "
           "documented on aos::Aio.";
  }
}

// Re-submits everything that was registered on the old ring to the current
// one: the wakeup read, the legacy-fd epoll poll, the thread-signal
// receivers, and every active timer.  Called from
// DowngradeFromSingleIssuer(), which replaces the ring with a fresh one and
// needs to restore what was on the old one.
//
// Legacy fds themselves (legacy_states, and their epoll_ctl registrations on
// legacy_epoll_fd_) need no re-arming here: DowngradeFromSingleIssuer()
// doesn't touch legacy_epoll_fd_ at all (it isn't part of the ring).
//
// In-flight AsyncRead/AsyncWrite requests cannot be re-submitted: there's
// no registry tracking them, and re-issuing a write in particular could
// duplicate it.  That loss is never silent, though: the downgrade path
// refuses to run at all with raw requests in flight
// (DowngradeFromSingleIssuer()'s CHECK on raw_requests_in_flight_).
void IoUringImpl::ReArmPersistentRegistrations() {
  // Every entry on pending_dispatch_ is a completion the *old* ring
  // produced: drained by DrainCompletions() but not yet dispatched.  Drop
  // them all before touching anything else.  Two things go wrong otherwise,
  // and both are the same mistake as the one DestroyTimerState() unlinks to
  // avoid -- recycling or re-arming a request that is still linked:
  //
  //   * The wholesale orphan recycle below ignores link.queued (unlike
  //     RecycleDrainedOrphans(), which checks it).  A queued orphan would
  //     reach the freelist, and MakeTimerState()'s Reset() zeroes the
  //     request -- including the intrusive links -- while it is still in
  //     this list, truncating it or tripping its own "removing a node that
  //     is not on this list" CHECK.  Its new owner could then be handed a
  //     spurious firing belonging to the previous one.
  //   * Every request re-armed below (the wakeup read, the legacy-fd poll,
  //     receivers, every timer's poll) may be queued too.  Dispatching a
  //     dead ring's completion against a freshly re-armed request re-arms it
  //     a second time, putting two incarnations on one identity.
  //
  // Only reachable if a rebuild ever happens mid-dispatch, which nothing
  // does today: the only rebuild, DowngradeFromSingleIssuer(), runs before
  // the first Poll() has ever dispatched anything.  This is written to
  // keep that true rather than to fix an observed failure.  Safe to do
  // underneath a live
  // ReapCompletions() loop: that pops from the live list each iteration, so
  // it simply finds it empty.
  while (AsyncRequest *req = pending_dispatch_.PopFront()) {
    State(req).link.queued = 0;
  }

  // Orphans' completions died with the old ring: nothing will ever drain
  // for them again, so recycle them all now.  So did every in-flight op
  // and cancel ack -- reset the counters to match; the re-arms below
  // recount through CountArmedOp().
  orphaned_timers_.MoveMatchingTo(&free_timers_,
                                  [](IoUringTimerState *) { return true; });
  orphaned_receivers_.MoveMatchingTo(
      &free_receivers_, [](ThreadSignalReceiverState *) { return true; });
  cancel_acks_outstanding_ = 0;
  armed_ops_ = 0;

  event_fd.wakeup_req.done = true;
  SubmitWakeupRead();

  legacy_epoll_request_.done = true;
  SubmitLegacyEpollPoll();

  if (receiver_state_ != nullptr) {
    receiver_state_->request.done = true;
    receiver_state_->Submit(this);
  }

  // Only the *poll* needs rebuilding.  The timers themselves live in their
  // timerfds, which are ordinary file descriptors that a ring rebuild
  // never touched, still armed.  So there is no deadline or period to
  // reconstruct here, and no in-flight-cancel case to unwind -- the old
  // ring's -ECANCELED simply never arrives, and nothing was waiting for
  // it.
  Aio::TimerState *curr = active_timers_.front();
  while (curr != nullptr) {
    Aio::TimerState *next = decltype(active_timers_)::Next(curr);
    curr->request.done = true;
    curr->canceling = false;
    static_cast<IoUringTimerState *>(curr)->SubmitPoll(/*draining_ok=*/true);
    curr = next;
  }
}

bool IoUringImpl::Poll(bool block) {
  // Not reentrant: a nested Poll() can never dispatch (the outer loop owns
  // delivery), so reentry could only silently starve its caller.  Die at
  // the call site instead; fatal is RT-clean.
  ABSL_CHECK(!dispatching_ && !in_before_wait_)
      << "Aio::Poll() reentered from inside a completion callback or "
         "before-wait function; wait by returning to the event loop instead";

  CheckSubmitterThread();
  EnsureBound();

  // Registering a new before-wait function from inside one is disallowed
  // (BeforeWait() CHECKs in_before_wait_): the push_back could reallocate
  // the vector, moving the very std::function this loop is executing out
  // from under itself.  EPoll never supported it either -- its range-for
  // hit the same reallocation as silent UB.
  in_before_wait_ = true;
  for (const std::function<void()> &fn : before_wait_functions) {
    fn();
  }
  in_before_wait_ = false;

  int ret;
  // Already-queued completions are delivered before going back to the
  // kernel at all.  One CQ drain yields a whole batch but Poll() dispatches
  // one of them (see ReapCompletions()), so the rest are delivered by the
  // next few Poll()s -- and going through the ring for each of those would
  // turn a batch of N into N io_uring_enter(2) calls, which is the entire
  // cost of the one-completion-per-Poll() contract if it isn't skipped.
  // Nothing is lost by not looking: no new CQE can appear while we aren't
  // entering the kernel, so the queue strictly shrinks and this converges.
  //
  // Staged SQEs still get flushed, so work a callback submitted isn't stuck
  // behind the rest of the queue.  A bare submit is enough -- fetching
  // completions is exactly what we are deferring.
  if (!pending_dispatch_.empty()) {
    if (io_uring_sq_ready(&ring) > 0) {
      ret = io_uring_submit(&ring);
      if (ret < 0 && ret != -EINTR && ret != -EAGAIN) {
        // VLOG, not LOG: Poll() legitimately runs under ScopedRealtime,
        // where default-on logging either allocates (ABSL_LOG, fatal via
        // the malloc hook) or blocks in a write(2) from the RT thread
        // (RAW_LOG) -- and fatal would turn an absorbable transient
        // submit error into a crash.  The error is absorbed either way
        // (the staged SQEs stay put and the next Poll() retries); a user
        // who wants to see it enables verbosity, an explicit opt-in.
        ABSL_VLOG(1) << "io_uring submit failed: " << aos_strerror(-ret);
      }
    }
    return ReapCompletions();
  }
  if (block) {
    // Loop until a completion actually exists.  One submit_and_wait()
    // call is not a reliable wait: when io_uring_enter(2) both submits
    // and waits, it returns the submit count and discards the wait result
    // ("if (!ret) ret = ret2;"), so a wait interrupted by unrelated task
    // work (e.g. another ring's teardown via io_tctx_exit_cb) looks like
    // success with an empty CQ.  Bare -EINTR on later iterations is
    // retried the same way: signals never end a blocking Poll() (see
    // Aio::Poll()); Quit()'s eventfd write is what wakes it.
    //
    // pending_dispatch_ is necessarily empty here (the short-circuit above
    // returned otherwise), so blocking cannot stall queued work.
    do {
      ret = io_uring_submit_and_wait(&ring, 1);
    } while (ret == -EINTR || (ret >= 0 && io_uring_cq_ready(&ring) == 0));
  } else {
    // Not io_uring_submit(): DEFER_TASKRUN only delivers completions on
    // an enter with IORING_ENTER_GETEVENTS set, which a bare submit
    // doesn't do.  submit_and_get_events() sets it with min_complete=0 --
    // still non-blocking, but delivery actually happens.
    ret = io_uring_submit_and_get_events(&ring);
  }

  if (ret < 0 && ret != -EINTR && ret != -EAGAIN) {
    // VLOG for the same RT-safety reasons as the short-circuit path above.
    ABSL_VLOG(1) << "io_uring submit failed: " << aos_strerror(-ret);
  }

  return ReapCompletions();
}

void IoUringImpl::Quit() {
  // Already asked to stop.  Bail out rather than re-arming the wakeup: once
  // Run() is draining it polls with a zero timeout, so a Quit() called from a
  // BeforeWait callback (or any other per-Poll path) would refill the queue
  // every time around and the drain would never finish.  This is the 2021
  // EPoll::Quit() guard -- f74daa655, "Make EPoll actually return from Run even
  // if you call Quit repeatedly" -- which the drain has always needed.
  //
  // Suppressing the wakeup is safe: quit_requested is only cleared by Run() on
  // its way out, so while it is set the loop has either already been woken or
  // is in the non-blocking drain and cannot block again.
  if (quit_requested) {
    return;
  }

  quit_requested = true;
  run = false;
  Wakeup();
}

void IoUringImpl::Wakeup() { event_fd.Write(); }

void IoUringImpl::AsyncRead(FileDescriptor fd, std::span<char> buffer,
                            AsyncRequest *request) {
  CheckSubmitterThread();
  struct io_uring_sqe *sqe = ArmSqe();

  io_uring_prep_read(sqe, fd, buffer.data(), buffer.size(), -1);
  io_uring_sqe_set_data64(sqe, NewIncarnationUserData(request));
  // A request legally reused after a previous Aio was destroyed can still
  // carry that dead loop's dispatch-queue flag (the destructor cannot
  // clear it -- the request may equally have died first).  A legal
  // caller's request is never on THIS loop's queue at arm time (reuse is
  // only allowed once the callback has run, which unlinks), so clearing
  // is unconditionally safe, and without it QueuePendingDispatch() would
  // take its already-linked early return and silently drop the callback.
  State(request).link.queued = 0;
  request->done = false;
  // The internal wakeup read is a persistent registration with its own
  // re-arm path; everything else is a raw request a ring rebuild could not
  // reconstruct -- count it (see DowngradeFromSingleIssuer()).
  if (request != &event_fd.wakeup_req && !State(request).raw_io) {
    State(request).raw_io = 1;
    ++raw_requests_in_flight_;
  }
}

void IoUringImpl::AsyncWrite(FileDescriptor fd, std::span<const char> buffer,
                             AsyncRequest *request) {
  CheckSubmitterThread();
  struct io_uring_sqe *sqe = ArmSqe();

  io_uring_prep_write(sqe, fd, buffer.data(), buffer.size(), -1);
  io_uring_sqe_set_data64(sqe, NewIncarnationUserData(request));
  // See AsyncRead() for the stale-queued-flag clear.
  State(request).link.queued = 0;
  request->done = false;
  // See AsyncRead().
  if (!State(request).raw_io) {
    State(request).raw_io = 1;
    ++raw_requests_in_flight_;
  }
}

void IoUringImpl::Cancel(AsyncRequest *request) {
  CheckSubmitterThread();
  // Already completed (its callback may still be queued for dispatch):
  // nothing left to cancel.  Documented on Aio::Cancel() -- the callback
  // delivers the original result, and the silent no-op is deliberate.
  if (request->done) return;

  struct io_uring_sqe *sqe = ArmSqe();
  // The cancel *targets* the request's current incarnation exactly (the
  // kernel matches by full user_data compare), but its *ack* is submitted
  // under cancel_ack_sentinel_'s identity, tagged so DrainCompletions()
  // can validate the kernel's answer (design invariant 3).  Deliberately
  // not the target's identity: the ack can outlive the target's own
  // Canceled callback (a CQ overflow relegates it to the kernel's
  // overflow list, which flushes on a later enter), and an ack that named
  // the target would require the caller's AsyncRequest -- or an orphaned
  // internal state -- to stay allocated until an event the caller cannot
  // observe.  Naming the immortal sentinel is what makes aio.h's
  // constraint 2 as simple as "alive until the callback runs".
  io_uring_prep_cancel64(sqe, EncodeUserData(request, 0), 0);
  io_uring_sqe_set_data64(sqe,
                          EncodeUserData(&cancel_ack_sentinel_, kCancelAckTag));
  ++cancel_acks_outstanding_;
  MaybeSubmit();
}

void IoUringImpl::BeforeWait(std::function<void()> function) {
  // Same-thread only, like every other registration call: Poll() iterates
  // before_wait_functions, so a push_back from another thread would race
  // it.
  CheckSubmitterThread();
  // Not from inside a before-wait function: the push_back can reallocate
  // the vector while Poll()'s iteration is executing an element in the old
  // storage.  Deterministically illegal rather than sometimes-corrupting.
  ABSL_CHECK(!in_before_wait_)
      << ": BeforeWait() may not be called from a before-wait function";
  before_wait_functions.push_back(std::move(function));
}

void IoUringImpl::OnReadable(FileDescriptor fd,
                             std::function<void()> callback) {
  CheckSubmitterThread();
  auto [it, inserted] = legacy_states.try_emplace(fd);
  if (inserted) {
    it->second = std::make_unique<IoUringImpl::LegacyState>();
    it->second->fd = fd;
  } else {
    ABSL_CHECK(!it->second->in_fn) << "Duplicate in functions for " << fd;
  }
  auto &state = *it->second;
  ABSL_CHECK(!state.events_fn)
      << "Cannot mix OnEvents and OnReadable for fd " << fd;
  state.in_fn = std::move(callback);
  uint32_t new_events = state.events | kInEvents | kErrorEvents;
  if (state.events != new_events) {
    state.events = new_events;
    UpdateLegacyEpoll(&state);
  }
}

void IoUringImpl::OnError(FileDescriptor fd, std::function<void()> callback) {
  CheckSubmitterThread();
  auto [it, inserted] = legacy_states.try_emplace(fd);
  if (inserted) {
    it->second = std::make_unique<IoUringImpl::LegacyState>();
    it->second->fd = fd;
  } else {
    ABSL_CHECK(!it->second->err_fn) << "Duplicate error functions for " << fd;
  }
  auto &state = *it->second;
  ABSL_CHECK(!state.events_fn)
      << "Cannot mix OnEvents and OnError for fd " << fd;
  state.err_fn = std::move(callback);
  uint32_t new_events = state.events | kErrorEvents;
  if (state.events != new_events) {
    state.events = new_events;
    UpdateLegacyEpoll(&state);
  }
}

void IoUringImpl::OnWritable(FileDescriptor fd,
                             std::function<void()> callback) {
  CheckSubmitterThread();
  auto [it, inserted] = legacy_states.try_emplace(fd);
  if (inserted) {
    it->second = std::make_unique<IoUringImpl::LegacyState>();
    it->second->fd = fd;
  } else {
    ABSL_CHECK(!it->second->out_fn) << "Duplicate out functions for " << fd;
  }
  auto &state = *it->second;
  ABSL_CHECK(!state.events_fn)
      << "Cannot mix OnEvents and OnWritable for fd " << fd;
  state.out_fn = std::move(callback);
  uint32_t new_events = state.events | kOutEvents;
  if (state.events != new_events) {
    state.events = new_events;
    UpdateLegacyEpoll(&state);
  }
}

void IoUringImpl::OnEvents(FileDescriptor fd,
                           std::function<void(uint32_t)> callback) {
  CheckSubmitterThread();
  auto [it, inserted] = legacy_states.try_emplace(fd);
  ABSL_CHECK(inserted) << "May not replace OnEvents handlers for fd " << fd;

  it->second = std::make_unique<IoUringImpl::LegacyState>();
  it->second->fd = fd;
  auto &state = *it->second;
  state.events_fn = std::move(callback);
}

void IoUringImpl::DeleteFd(FileDescriptor fd) {
  CheckSubmitterThread();
  // Deterministically illegal under RT, like DestroyTimerState(): both
  // paths below end in a free -- inline here, or at the end of the
  // dispatch that parked the state -- and a function that only sometimes
  // freed would only sometimes trip the RT malloc hook, a data-dependent
  // crash.  Nothing legitimately removes an fd registration from an RT
  // thread (EPoll::DeleteFd() has always freed, so this was never legal;
  // now it fails with a message instead of a hook crash).
  aos::CheckNotRealtime();
  auto it = legacy_states.find(fd);
  ABSL_CHECK(it != legacy_states.end()) << "fd " << fd << " not found";

  if (it->second->epoll_registered) {
    int ret = epoll_ctl(legacy_epoll_fd_, EPOLL_CTL_DEL, fd, nullptr);
    ABSL_PCHECK(ret == 0 || errno == ENOENT)
        << "epoll_ctl DEL failed for fd " << fd;
  }
  // Mid-dispatch the state is parked, not destroyed: a callback may
  // DeleteFd() its own fd, in which case one of this state's
  // std::functions is the function currently executing -- destroying it
  // here would free the lambda's captures out from under the running
  // code.  Parked states are freed at the end of this same outermost
  // dispatch (ReapCompletions()), on this thread, in this same non-RT
  // context -- see the CheckNotRealtime() above for why that context is
  // guaranteed.  Outside dispatch nothing can be executing out of the
  // state and it is freed right here.
  if (dispatching_) {
    // Tombstone before parking: DrainLegacyEpoll() holds the state pointer
    // across its callbacks and stops dispatching when fd goes to -1.
    it->second->fd = -1;
    retired_legacy_states_.Push(it->second.release());
  }
  legacy_states.erase(it);
}

void IoUringImpl::ForgetClosedFd(FileDescriptor fd) {
  CheckSubmitterThread();
  // Deterministic, like DeleteFd() -- see there.
  aos::CheckNotRealtime();
  auto it = legacy_states.find(fd);
  ABSL_CHECK(it != legacy_states.end()) << "fd " << fd << " not found";

  // fd is already closed, which drops the kernel's epoll registration for it
  // automatically (epoll_ctl(2)) -- nothing to undo here beyond forgetting
  // our own bookkeeping.
  // Parked (with the same tombstone) or freed for the same reasons as
  // DeleteFd().
  if (dispatching_) {
    it->second->fd = -1;
    retired_legacy_states_.Push(it->second.release());
  }
  legacy_states.erase(it);
}

void IoUringImpl::EnableWritable(FileDescriptor fd) {
  CheckSubmitterThread();
  auto it = legacy_states.find(fd);
  ABSL_CHECK(it != legacy_states.end()) << "fd " << fd << " not found";

  auto &state = *it->second;
  ABSL_CHECK(!state.events_fn)
      << "EnableWritable is only for fds registered using OnWritable, not "
         "OnEvents";
  uint32_t new_events = state.events | kOutEvents;
  if (state.events != new_events) {
    state.events = new_events;
    UpdateLegacyEpoll(&state);
  }
}

void IoUringImpl::DisableWritable(FileDescriptor fd) {
  CheckSubmitterThread();
  auto it = legacy_states.find(fd);
  ABSL_CHECK(it != legacy_states.end()) << "fd " << fd << " not found";

  auto &state = *it->second;
  ABSL_CHECK(!state.events_fn)
      << "DisableWritable is only for fds registered using OnWritable, not "
         "OnEvents";
  uint32_t new_events = state.events & ~kOutEvents;
  if (state.events != new_events) {
    state.events = new_events;
    UpdateLegacyEpoll(&state);
  }
}

void IoUringImpl::SetEvents(FileDescriptor fd, uint32_t events) {
  CheckSubmitterThread();
  auto it = legacy_states.find(fd);
  ABSL_CHECK(it != legacy_states.end()) << "fd " << fd << " not found";

  auto &state = *it->second;
  ABSL_CHECK(state.events_fn)
      << "SetEvents is only for fds registered using OnEvents";
  if (state.events != events) {
    state.events = events;
    UpdateLegacyEpoll(&state);
  }
}

void IoUringImpl::RegisterThreadSignalReceiver(
    ipc_lib::ThreadSignalReceiver *receiver, std::function<void()> callback) {
  CheckSubmitterThread();
  // Deterministically illegal under RT, like MakeTimerState(): the
  // freelist miss below allocates (and the freelist hit destroys a stale
  // callback), and only-sometimes-allocating is a data-dependent crash.
  aos::CheckNotRealtime();
  ABSL_CHECK(receiver_state_ == nullptr)
      << "Duplicate ThreadSignalReceiver registration: only one receiver "
         "may be active at a time (see receiver_state_)";

  if (ThreadSignalReceiverState *state = free_receivers_.Pop();
      state != nullptr) {
    const uint16_t generation = State(&state->request).generation;
    *state = ThreadSignalReceiverState();
    State(&state->request).generation = generation;
    receiver_state_.reset(state);
  } else {
    receiver_state_ =
        std::make_unique<IoUringImpl::ThreadSignalReceiverState>();
  }
  receiver_state_->receiver = receiver;
  receiver_state_->fd = receiver->fd();
  auto &state = *receiver_state_;
  state.callback = std::move(callback);
  state.Submit(this);
}

void IoUringImpl::UnregisterThreadSignalReceiver(
    ipc_lib::ThreadSignalReceiver *receiver) {
  CheckSubmitterThread();
  ABSL_CHECK(receiver_state_ != nullptr &&
             receiver_state_->receiver == receiver)
      << "ThreadSignalReceiver not found";

  auto state = std::move(receiver_state_);

  // Same sometimes-frees shape as DestroyTimerState(): fail
  // deterministically under RT instead.
  aos::CheckNotRealtime();
  // Mark unregistered with a plain flag; the trampoline keys off it and
  // never touches the signalfd again (see ThreadSignalReceiverState).  The
  // std::function itself is only destroyed when provably not executing:
  // right here when this receiver's callback is not on the stack, or at
  // the end of the outermost dispatch when it is -- assigning nullptr to
  // the very std::function that called us would free the running lambda's
  // captures out from under it (and out from under the caller's own code
  // after this returns).
  state->unregistered = true;
  if (state->in_callback) {
    deferred_receiver_callback_clear_ = state.get();
  } else {
    state->callback = nullptr;
  }
  // Unconditionally, and before the fast path below -- the same reason
  // DestroyTimerState() unlinks unconditionally.  `done` here can mean
  // "drained but undispatched": the multishot poll was terminated (CQ
  // overflow), DrainCompletions() set request.done AND queued that same
  // terminal completion on pending_dispatch_ in one iteration, and the
  // dispatch loop hasn't reached it yet.  The fast path then frees the
  // state with the node still linked, and the next PopFront()/Remove()
  // walks freed memory.  RecycleDrainedOrphans() tests !link.queued on top
  // of done for exactly this hazard; unlinking here is what makes the
  // identical fast-path condition sufficient.
  UnlinkPendingDispatch(&state->request);
  if (state->request.done) {
    if (dispatching_) {
      // This unregister can be running from inside this very receiver's
      // dispatched trampoline: it reads the signalfd and invokes the user
      // callback, and -- unlike the timer trampoline, which touches
      // nothing after its user callback -- it keeps using `state`
      // afterward (the drain loop continues, then the revival check
      // runs).  Freeing here would free that executing lambda's state out
      // from under it.  Park it as a drained orphan instead:
      // RecycleDrainedOrphans() moves it to the freelist at the end of
      // this outermost dispatch (done is set and nothing is queued, so it
      // qualifies immediately), after every frame has unwound.  Reached
      // only via a terminal completion (a live multishot is never
      // `done`), i.e. after a CQ overflow terminated the poll.
      orphaned_receivers_.Push(state.release());
      return;
    }
    // Outside dispatch nothing can be executing out of this state (its
    // lambda only ever runs from the dispatch loop); it destroys here.
    return;
  }
  CancelRequest(&state->request);
  orphaned_receivers_.Push(state.release());
}

void IoUringImpl::ConsumeThreadSignalReceiver(
    ipc_lib::ThreadSignalReceiver *receiver) {
  // Same-thread only: this read()s the same signalfd the receiver's
  // multishot poll delivers through, and a cross-thread drain would race
  // the loop thread's own dispatch of it.
  CheckSubmitterThread();
  int fd = receiver->fd();
  struct signalfd_siginfo siginfo;
  while (true) {
    ssize_t res = read(fd, &siginfo, sizeof(siginfo));
    if (res < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      break;
    } else if (res < 0) {
      ABSL_LOG(FATAL) << "Failed to read from signalfd: "
                      << aos_strerror(errno);
    }
  }
}

// Destruction is handled by IoUringImpl::DestroyTimerState(), which strips
// the user callback and orphans this state until the poll's terminal
// completion has drained -- so by the time this runs, nothing is in flight
// and closing the timerfd cannot strand an armed op on it.
IoUringTimerState::~IoUringTimerState() = default;

void IoUringTimerState::Initialize() {
  // Recycled states arrive with their timerfd already created and already
  // disarmed by Reset() -- see MakeTimerState().
  if (timer_fd == nullptr) {
    timer_fd = std::make_unique<TimerFD>();
  }
  // The poll is armed here, once, and stays armed for this state's whole
  // life regardless of whether the timer is currently scheduled: a disarmed
  // timerfd is simply never readable.  That is what lets Schedule() and
  // Cancel() be pure timerfd_settime() calls that never touch the ring.
  //
  // Flushed rather than left staged.  Each live timer holds one armed poll
  // for its whole life, so N timers need N SQEs; leaving them queued means
  // building more than --aio_queue_depth timers before the first Poll()
  // exhausts the submission queue and CHECK-fails in SubmitPoll().
  // Submitting as we go keeps only one outstanding at a time.  This costs a
  // syscall per timer construction, which is fine -- construction is not on
  // any RT path (and MaybeSubmit() is a no-op until the ring is enabled, so
  // pre-Run() setup is still capped by the queue depth; that is the same
  // "depth too small for the app's registrations" limit the ring already
  // has in steady state, just reported earlier).
  impl_->LinkTimer(this);
  SubmitPoll();
  impl_->MaybeSubmit();
}

// Clears the schedule but deliberately keeps timer_fd (so recycling doesn't
// pay timerfd_create()) and the request's generation counter (keeping
// user_data unambiguous across recycling).  The caller is responsible for
// the timerfd itself being disarmed -- DestroyTimerState() does that before
// the state can ever reach the freelist.
void IoUringTimerState::Reset() {
  ABSL_CHECK(!is_active) << "Recycled a timer still on the active list";
  deadline = aos::monotonic_clock::epoch();
  user_callback = nullptr;
  user_context = nullptr;
  // Keeping the generation monotonic across recycling costs nothing and
  // keeps every incarnation of this identity unique -- a recycled state
  // reaches here with its terminal completion drained, so nothing is in
  // flight, but distinct generations keep any trace or CHECK message
  // unambiguous about which owner an op belonged to.
  const uint16_t generation = State(&request).generation;
  request = AsyncRequest();
  State(&request).generation = generation;
  canceling = false;
  next_orphan = nullptr;
}

void IoUringTimerState::SubmitPoll(bool draining_ok) {
  struct io_uring_sqe *sqe;
  if (draining_ok) {
    sqe = impl_->GetSqeForRingReconstruction();
  } else {
    sqe = impl_->ArmSqe();
  }

  // Multishot, unlike the legacy-fd epoll poll next door, and for the same
  // reason ThreadSignalReceiverState's poll can be: a timerfd's readiness
  // genuinely toggles on every read.  read() zeroes ctx->ticks
  // (fs/timerfd.c), and the next expiration calls wake_up_locked_poll() --
  // a real edge, which is the only thing IORING_OP_POLL_ADD ever completes
  // on.  So the steady state costs zero SQEs per firing.
  io_uring_prep_poll_multishot(sqe, timer_fd->fd(), POLLIN);
  io_uring_sqe_set_data64(sqe, NewIncarnationUserData(&request));

  request.callback = &IoUringTimerState::OnTimerFdReadable;
  request.context = this;
  request.done = false;
}

void IoUringTimerState::OnTimerFdReadable(Completion completion, void *ctx) {
  auto *tstate = static_cast<IoUringTimerState *>(ctx);

  if (!aos::IsOk(completion.status)) {
    // The only cancel anything ever submits for this poll is
    // DestroyTimerState()'s, which sets `canceling` first.  Any other error
    // is fatal rather than absorbed, for the same reason as the
    // thread-signal receiver's poll: re-arming would resubmit a failing op
    // forever, and swallowing it would silently stop this timer from ever
    // firing again.
    ABSL_CHECK(tstate->canceling) << "Poll on a timer's timerfd failed: "
                                  << aos_strerror(completion.result);
    return;
  }

  // Consume the expiration.  A single-shot timerfd banks at most one, and
  // reading it clears ctx->ticks (fs/timerfd.c) so the fd stops being
  // readable -- which is exactly the edge the multishot poll needs to fire
  // again for the next schedule.
  //
  // EAGAIN is normal: the poll fired on a readiness that Cancel()'s
  // timerfd_settime(0) has since zeroed.  Deliver nothing in that case.
  uint64_t buf;
  ssize_t result = read(tstate->timer_fd->fd(), &buf, sizeof(buf));
  const bool expired = result == static_cast<ssize_t>(sizeof(buf));
  if (!expired) {
    ABSL_PCHECK(result == -1 && errno == EAGAIN)
        << "Unexpected read from a timer's timerfd";
  }

  // A multishot poll normally stays armed forever, posting one CQE per
  // firing with IORING_CQE_F_MORE ("more coming") so request.done stays
  // false.  The kernel revokes that whenever it cannot post such a CQE:
  // with the CQ full, io_req_post_cqe() fails and io_poll_check_events()
  // (linux/io_uring/poll.c) ends the op with a normal, final completion
  // instead of dropping the event.  Re-arm, or this timer stops firing
  // forever after one CQ overflow.
  //
  // Done *before* dispatching, so that nothing here touches `tstate` after
  // the user callback runs -- a callback is allowed to destroy its own
  // timer, and this way that needs no guard.  Re-arming a poll the callback
  // then destroys is harmless: DestroyTimerState() cancels it.
  if (tstate->request.done && !tstate->canceling) {
    tstate->SubmitPoll();
    tstate->impl_->MaybeSubmit();
  }

  if (!expired || tstate->user_callback == nullptr) {
    return;
  }
  // Last use of tstate -- see above.
  Completion timer_completion;
  // nullptr, exactly as documented: Completion::user_data is "the opaque
  // pointer supplied by the caller", and Timer::Schedule() takes no
  // user_data -- leaking the internal state pointer here contradicted
  // that.
  timer_completion.user_data = nullptr;
  timer_completion.status = aos::Ok();
  timer_completion.result = 0;
  tstate->impl_->user_dispatch_ = true;
  tstate->user_callback(timer_completion, tstate->user_context);
}

void IoUringTimerState::Schedule(aos::monotonic_clock::time_point deadline,
                                 CompletionCallback callback, void *context) {
  impl_->CheckSubmitterThread();
  ABSL_CHECK_GE(deadline, aos::monotonic_clock::epoch());

  this->deadline = deadline;
  this->user_callback = callback;
  this->user_context = context;

  // The whole reschedule, armed or not: one syscall that atomically
  // replaces whatever the kernel had.  It also zeroes the expiration
  // counter (timerfd_setup() in fs/timerfd.c), so a previous schedule's
  // firing cannot leak into this one.  Nothing is submitted to the ring, so
  // there is no cancel to race, no update to lose, and no ack to reconcile.
  //
  // it_interval stays zero: this is a one-shot.  TFD_TIMER_ABSTIME is what
  // makes a caller-driven periodic timer (re-scheduling from its own
  // callback, as ShmTimerHandler does) hold its phase -- the deadline is a
  // point on the caller's grid, not an offset from whenever we got here.
  struct itimerspec its;
  std::memset(&its, 0, sizeof(its));
  its.it_value = AbsoluteTimerfdValue(deadline);
  int ret = timerfd_settime(timer_fd->fd(), TFD_TIMER_ABSTIME, &its, nullptr);
  ABSL_PCHECK(ret == 0) << "timerfd_settime failed: " << aos_strerror(errno);
}

void IoUringTimerState::Cancel(bool reap) {
  impl_->CheckSubmitterThread();
  // `reap` is meaningless here: there is nothing asynchronous to reap.  A
  // disarm takes effect the instant the syscall returns.
  (void)reap;

  // RT-safe: one syscall, no allocation, nothing submitted to the ring.
  // Silent by design, matching the documented Timer::Cancel() contract --
  // the pending callback is dropped, not delivered as Canceled.
  user_callback = nullptr;

  struct itimerspec its;
  std::memset(&its, 0, sizeof(its));
  int ret = timerfd_settime(timer_fd->fd(), 0, &its, nullptr);
  ABSL_PCHECK(ret == 0) << "timerfd_settime failed: " << aos_strerror(errno);
}

void IoUringImpl::QueuePendingDispatch(AsyncRequest *req, int32_t res) {
  if (State(req).link.queued) {
    // Already linked: a multishot request (e.g. ThreadSignalReceiverState's
    // POLL_MULTISHOT) can produce another completion before its previous one
    // has been dispatched.  Its callback drains everything available in one
    // call, so there's nothing to gain from linking it a second time -- and
    // doing so would corrupt the list, since the same node can't occupy two
    // positions in a linked list at once (this previously caused a self-loop
    // and an infinite dispatch spin).  Just refresh the stashed result;
    // leave the links alone.
    State(req).link.result = res;
    return;
  }
  State(req).link.result = res;
  State(req).link.queued = 1;
  pending_dispatch_.PushBack(req);
}

bool IoUringImpl::DrainCompletions() {
  // The extraction phase of the two-phase dispatch design: one
  // io_uring_for_each_cqe() scan plus a single bulk io_uring_cq_advance(),
  // setting `done` and queueing callbacks on pending_dispatch_.  MUST stay
  // callback-free: user code reentering the ring mid-scan could nest a
  // cq_advance under the uncommitted bulk one and corrupt the CQ head
  // (real in the pre-pending_dispatch_ design).
  bool processed = false;
  unsigned head;
  struct io_uring_cqe *cqe = nullptr;
  int count = 0;
  io_uring_for_each_cqe(&ring, head, cqe) {
    ++count;
    processed = true;
    const uint64_t data = io_uring_cqe_get_data64(cqe);
    // Nothing submits a null user_data: every op carries an encoded
    // (generation, request pointer, ack tag) triple -- see EncodeUserData().
    ABSL_CHECK_NE(data, uint64_t{0}) << "CQE with null user_data";
    const uint64_t tag = data & kAckTagMask;
    const uint16_t generation = static_cast<uint16_t>(data >> kGenerationShift);
    auto *req = reinterpret_cast<AsyncRequest *>(data & kPointerMask);

    if (tag == kCancelAckTag) {
      // Async-cancel ack, submitted under cancel_ack_sentinel_'s identity
      // -- never its target's, so `req` here is the immortal sentinel and
      // this path touches no caller or orphan memory (see Cancel()).
      // Handled before the generation check below because acks are
      // identity-independent: the sentinel's generation never increments,
      // so every ack is "current" by construction.
      // 0 / -EALREADY: the target's terminal completion is coming.
      // -ENOENT: the lookup missed, which for the ops this backend still
      // cancels (polls and raw reads/writes) means the target was already
      // resolving -- its terminal CQE is either already drained or
      // trailing this ack.  Either way there is nothing left to kill, so
      // the ack is validated and counted, never retried.
      ABSL_CHECK_EQ(req, &cancel_ack_sentinel_)
          << "Cancel ack not submitted under the sentinel identity";
      ABSL_CHECK(cqe->res == 0 || cqe->res == -EALREADY || cqe->res == -ENOENT)
          << "Unexpected async-cancel ack: " << aos_strerror(-cqe->res);
      ABSL_CHECK_GT(cancel_acks_outstanding_, 0)
          << "Cancel ack with none outstanding";
      --cancel_acks_outstanding_;
      // The cancel op itself is done -- release its armed_ops_ slot.
      ABSL_CHECK_GT(armed_ops_, 0) << "Ack drained with no ops armed";
      --armed_ops_;
      continue;
    }
    ABSL_CHECK_EQ(tag, uint64_t{0}) << "CQE with unknown user_data tag";

    // A target CQE.  `req` is alive: an identity is never freed with CQEs
    // still in flight -- its terminal completion is the last CQE that
    // names it (acks name the sentinel), states whose owner died stay
    // parked on the orphan lists until that terminal has drained
    // (RecycleDrainedOrphans()), and aio.h's constraint 2 holds callers to
    // the same rule.
    if (generation != State(req).generation) {
      // A superseded incarnation's CQE.  Nothing is ever legitimately
      // stale: acks cannot be (handled above under the sentinel), and a
      // stale target CQE means arms-behind-terminals broke -- die rather
      // than bury the evidence.
      ABSL_LOG(FATAL) << "Stale completion (generation " << generation
                      << " vs current " << State(req).generation
                      << ") for a target op: an incarnation was armed over "
                         "one whose completions were still in flight";
    }

    const bool raw = State(req).raw_io != 0;
    if (!(cqe->flags & IORING_CQE_F_MORE)) {
      req->done = true;
      // Terminal completion: this op no longer occupies an armed_ops_
      // slot.  (F_MORE completions leave the multishot armed.)
      ABSL_CHECK_GT(armed_ops_, 0) << "Terminal CQE with no ops armed";
      --armed_ops_;
      if (raw) {
        State(req).raw_io = 0;
        --raw_requests_in_flight_;
      }
    }
    if (req->callback) {
      State(req).user_visible = raw ? 1 : 0;
      QueuePendingDispatch(req, cqe->res);
    }
  }
  if (count > 0) {
    io_uring_cq_advance(&ring, count);
  }
  return processed;
}

bool IoUringImpl::ReapCompletions() {
  // Drain the ring first: every currently-available entry is resolved
  // (done set) and, if it has a callback, queued -- see DrainCompletions()
  // for why this part is unconditionally safe regardless of nesting.
  bool processed = DrainCompletions();

  // Poll()'s reentrancy CHECK means exactly one dispatch loop can be in
  // flight -- this one.
  dispatching_ = true;
  // One user-visible completion, not the whole queue -- see Aio::Poll() for
  // the contract.  Internal entries are dispatched for free (see
  // user_dispatch_), so the loop runs until one of them turns out to be
  // user-visible or the queue empties.  Anything a callback queues (a
  // reentrant CancelRequest() via DrainCompletions()) stays on the list
  // for the next Poll(), which is where the rest of this batch is waiting
  // anyway.
  user_dispatch_ = false;
  while (!user_dispatch_) {
    AsyncRequest *req = pending_dispatch_.PopFront();
    if (req == nullptr) {
      break;
    }
    State(req).link.queued = 0;
    int32_t res = State(req).link.result;
    // Read before dispatching, since the callback may destroy `req`.  Only
    // raw AsyncRead/AsyncWrite land here as user-visible; every trampoline
    // sets user_dispatch_ itself, at the point it actually calls user code.
    if (State(req).user_visible) {
      user_dispatch_ = true;
    }
    req->callback(CompletionFromResult(req, res), req->context);
    // DrainCompletions() may have found nothing new -- this completion was
    // drained by an earlier Poll() and only dispatched now.  Poll() must
    // still report progress, or Run()'s post-Quit() drain would stop with
    // callbacks still queued.
    processed = true;
  }
  dispatching_ = false;

  // Destroy the callback of a receiver that unregistered itself from
  // inside that very callback (see UnregisterThreadSignalReceiver()): the
  // dispatch loop is done, so the lambda has returned and nothing executes
  // out of it.  Deterministically non-RT when set, by the same argument as
  // the retired-legacy sweep below: the deferral only happens inside
  // UnregisterThreadSignalReceiver(), which is CheckNotRealtime() and ran
  // on this thread inside this very dispatch.
  if (deferred_receiver_callback_clear_ != nullptr) {
    aos::CheckNotRealtime();
    deferred_receiver_callback_clear_->callback = nullptr;
    deferred_receiver_callback_clear_ = nullptr;
  }

  // Free legacy registrations retired mid-dispatch (see DeleteFd()).  Why
  // deleting here is safe:
  //   * No callback can still be executing out of one.  A LegacyState's
  //     std::functions run in exactly one place -- DrainLegacyEpoll(),
  //     inside the legacy trampoline, dispatched only by the dispatch loop
  //     above (of which Poll()'s reentrancy CHECK permits exactly one) --
  //     and that loop just exited, so every such frame has unwound.
  //   * Nothing else can reach a parked state: DeleteFd() already erased
  //     it from legacy_states and from the epoll registration, and a
  //     LegacyState is not an AsyncRequest, so pending_dispatch_ cannot
  //     name it (which is why timers need UnlinkPendingDispatch() and
  //     these don't).
  //   * ~LegacyState runs capture destructors, which may reenter the Aio;
  //     dispatching_ is already false and the node is popped before the
  //     delete, so a reentrant DeleteFd() frees inline and a reentrant
  //     Poll() is just a fresh dispatch.
  //   * Non-RT is *checked*, not assumed: parking requires passing
  //     DeleteFd()/ForgetClosedFd()'s CheckNotRealtime() inside this very
  //     dispatch, and the check below catches the one path that could
  //     still go wrong -- a callback parking a state and then promoting
  //     this thread to RT before returning.  The condition is on list
  //     state, not RT state, so behavior never silently varies with
  //     RT-ness; the empty-list common case is one load.
  if (LegacyState *state = retired_legacy_states_.Pop()) {
    aos::CheckNotRealtime();
    do {
      delete state;
    } while ((state = retired_legacy_states_.Pop()) != nullptr);
  }

  // Safe point to recycle drained orphans: no CQ scan is live and nothing
  // is mid-dispatch.  Recycling never allocates or frees, so this is fine
  // on an RT thread too.
  RecycleDrainedOrphans();

  return processed;
}

void IoUringImpl::UnlinkPendingDispatch(AsyncRequest *req) {
  // O(1): splices req out through its own prev/next rather than scanning
  // the list to find it -- see AioState::link.prev.
  if (!State(req).link.queued) {
    return;
  }
  pending_dispatch_.Remove(req);
  State(req).link.queued = 0;
}

void IoUringImpl::CancelRequest(AsyncRequest *request) {
  // Must happen before the request->done check below: a request can already
  // be done (its completion reaped) but still sitting on pending_dispatch_
  // waiting for its callback to run -- see UnlinkPendingDispatch().
  UnlinkPendingDispatch(request);

  if (!request->done) {
    Cancel(request);
  }
}

void IoUringImpl::SubmitWakeupRead() {
  AsyncRead(event_fd.fd(),
            std::span<char>(reinterpret_cast<char *>(&event_fd.eventfd_buf),
                            sizeof(event_fd.eventfd_buf)),
            &event_fd.wakeup_req);
}

Aio::Aio() { impl_ = std::make_unique<IoUringImpl>(); }
}  // namespace aos
