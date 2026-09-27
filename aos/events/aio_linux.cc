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
#include <sys/stat.h>
#include <sys/syscall.h>
#include <sys/timerfd.h>
#include <sys/uio.h>
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
#include "aos/events/intrusive_rb_tree.h"
#include "aos/events/kernel_release.h"
#include "aos/events/socket_error.h"
#include "aos/ipc_lib/thread_signal.h"
#include "aos/libc/aos_strerror.h"
#include "aos/realtime.h"
#include "aos/time/time.h"

ABSL_FLAG(uint32_t, aio_queue_depth, 256,
          "Depth of the io_uring submission and completion queues.");

// Compiled in from the platform's //tools/platforms/io_uring constraint via
// //aos/events:aio's local_defines.  Deliberately no fallback default: a
// platform the select() misses should be a build error, not a silently-wrong
// default on a robot.
#ifndef AOS_AIO_DEFAULT_BACKEND
#error \
    "AOS_AIO_DEFAULT_BACKEND must be set by //aos/events:aio's local_defines."
#endif

ABSL_FLAG(std::string, aio_backend, AOS_AIO_DEFAULT_BACKEND,
          "Which Aio backend to use: \"io_uring\" (requires kernel >= 6.1, "
          "6.12 recommended; fails loudly if unavailable) or \"epoll\".  There "
          "is no automatic "
          "fallback -- set this to match the deployment.  Defaults to "
          "\"io_uring\" when the build's target platform declares io_uring "
          "support (//tools/platforms/io_uring) and \"epoll\" when it does "
          "not.  A name rather than a boolean so that backends can be added "
          "without changing the flag's meaning.");

ABSL_FLAG(size_t, aio_pool_size, 16,
          "Initial size of the pre-allocated registration pool for "
          "AsyncRead/AsyncWrite fds, which keeps arming a request "
          "allocation-free.");

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

  // Consumes every pending wakeup.  The counter is reset by a single read,
  // but the loop costs nothing and keeps this honest if the fd is ever made
  // semaphore-mode.  Non-blocking, so it ends on EAGAIN.
  void Drain() {
    uint64_t buf;
    while (read(fd_, &buf, sizeof(buf)) > 0) {
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

// Which caller-submitted raw op a request has in flight.
enum class RawIo : uint8_t {
  kNone = 0,
  kRead,
  kWrite,
};

// Internal helper to access AsyncRequest's opaque state buffer.
//
// The members are separate structs, not a union.  A request can reach
// AsyncRead() or AsyncWrite() still carrying `link` state from an Aio that
// was destroyed while the request was queued, which aio.h's constraint 2
// permits.  The submit path reads that state to tell that it is stale (see
// IoUringImpl::ClearStaleQueuedFlag()), so scratch space that shared memory
// with `link` would already have overwritten it.  Re-arming a request that
// is still queued on this loop is fatal; see AsyncRequest::done.
struct AioState {
  // List node, used by both backends: IoUringImpl::pending_dispatch_ and
  // EpollImpl's pending_/ready_ all use all four fields.
  struct {
    AsyncRequest *next;
    // Doubly linked so IoUringImpl::UnlinkPendingDispatch() can splice a
    // request out of pending_dispatch_ in O(1).
    AsyncRequest *prev;
    int32_t result;
    // Which pending list this request is linked on, or 0 for none.
    // IoUringImpl uses it for pending_dispatch_ (see QueuePendingDispatch());
    // EpollImpl uses it for pending_/ready_, and to say why a request is
    // there -- see EpollImpl::QueueKind.
    int32_t queued;
  } link;
  // EpollImpl only: read/write staging for AsyncRead/AsyncWrite (io_uring
  // hands its span straight to the SQE), and the registration a Cancel()ed
  // request holds its claim on.  Separate from `link` -- see the struct
  // comment.
  struct {
    void *ptr;
    size_t size;
  } epoll;
  // IoUringImpl only.
  struct {
    // List node and fd for raw_in_flight_, valid while raw_io is set --
    // this is what lets legacy registration CHECK the no-mixing contract
    // (see ClaimRawFd()).  Separate from `link` -- see the struct comment.
    AsyncRequest *raw_prev;
    AsyncRequest *raw_next;
    int32_t raw_fd;
    // Incarnation counter: incremented per fresh kernel op submitted under
    // this request's identity and encoded into that op's user_data.  See
    // EncodeUserData() for why 16 bits cannot wrap into ambiguity.
    uint16_t generation;
    // Set while a caller-submitted AsyncRead/AsyncWrite is in flight (the
    // internal wakeup read is excluded).  Backs raw_requests_in_flight_
    // (see DowngradeFromSingleIssuer()) and tells the terminal completion
    // which per-fd no-mixing claim to release.
    RawIo raw_io;
    // Set at queue time when this request's callback is the caller's own
    // rather than one of this file's trampolines, so ReapCompletions() can
    // tell whether dispatching it spends the one-user-completion-per-Poll()
    // budget.  The trampolines report for themselves.
    //
    // Note that this is a boolean flag, but uses uint8_t to ensure
    // struct alignment/packing without complications around the "bool"
    // type.
    uint8_t user_visible;
  } uring;
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
      << "AsyncRequest pointer " << req
      << " does not fit the user_data encoding";
  return (uint64_t{State(req).uring.generation} << kGenerationShift) | ptr |
         tag;
}

// Starts a new incarnation of `req` and returns its encoded user_data --
// call exactly once per fresh kernel-op submission under this identity.
inline uint64_t NewIncarnationUserData(AsyncRequest *req) {
  ++State(req).uring.generation;
  return EncodeUserData(req, 0);
}

// Arms `req` on `sqe` as a fresh incarnation, setting the four fields that
// have to move together.
//
// For this file's own registrations only.  A caller's AsyncRead()/AsyncWrite()
// request carries its own callback, context and user_data, and a cancel names
// the existing incarnation rather than starting one.
inline void ArmRequest(struct io_uring_sqe *sqe, AsyncRequest *req,
                       CompletionCallback callback, void *context) {
  io_uring_sqe_set_data64(sqe, NewIncarnationUserData(req));
  req->callback = callback;
  req->context = context;
  req->user_data = req;
  req->done = false;
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
// low 4 bits of the real epoll event flags (the encoding documented on
// Aio::OnEvents()).
constexpr uint32_t kIn = 0x01;   // EPOLLIN
constexpr uint32_t kPri = 0x02;  // EPOLLPRI
constexpr uint32_t kOut = 0x04;  // EPOLLOUT
constexpr uint32_t kErr = 0x08;  // EPOLLERR

constexpr uint32_t kInEvents = kIn | kPri;
constexpr uint32_t kOutEvents = kOut;
constexpr uint32_t kErrorEvents = kErr;

// Where a legacy registration's hangup or error goes when no handler asked
// for it: a hangup with no readable, writable or error bit to carry it, or an
// error on an fd with no error handler.  Both go to the handler that can act
// on them, the way libuv (and so UvAio) routes them: the error handler if
// there is one, otherwise whichever of readable/writable the fd subscribed
// to, where the read sees EOF or the error, and the write sees EPIPE or the
// error.
//
// The kernel keeps reporting either condition for as long as the fd is
// registered.  EPoll dropped a bare hangup, which left an OnReadable() pipe
// whose writer went away spinning Poll() forever with nothing ever told.  An
// error with no error handler was fatal, which killed an OnWritable() pipe
// whose reader went away: the kernel reports that as EPOLLERR alone, and the
// write that would have seen EPIPE never ran.
//
// A hangup that arrives alongside another bit is left alone; that bit's
// handler already runs.  An fd subscribed to neither direction keeps its
// error bit, and the error-handler CHECK still fires for it.
uint32_t RouteHangupOrError(uint32_t events, bool hangup, uint32_t subscribed,
                            bool has_err_fn) {
  constexpr uint32_t kData = kInEvents | kOutEvents;
  const bool bare_hangup = hangup && (events & (kData | kErrorEvents)) == 0;
  if (has_err_fn) {
    return bare_hangup ? (events | kErr) : events;
  }
  if (!bare_hangup && (events & kErrorEvents) == 0) {
    return events;
  }
  if ((subscribed & kData) == 0) {
    return events;
  }
  return (events & ~kErrorEvents) | (subscribed & kData);
}

// One read or write at the fd's current position that returns EAGAIN rather
// than sleeping, whatever the fd's flags: preadv2()/pwritev2() with
// RWF_NOWAIT.  glibc only wraps them from 2.26, and the roboRIO's is 2.24, so
// older ones make the syscall (Linux 4.6) directly.  A kernel without it
// answers ENOSYS, and one without RWF_NOWAIT (4.14) EOPNOTSUPP.
#ifndef RWF_NOWAIT
#define RWF_NOWAIT 0x00000008
#endif
inline ssize_t NowaitIo(bool read, int fd, struct iovec *iov) {
#if __GLIBC__ > 2 || (__GLIBC__ == 2 && __GLIBC_MINOR__ >= 26)
  return read ? preadv2(fd, iov, 1, -1, RWF_NOWAIT)
              : pwritev2(fd, iov, 1, -1, RWF_NOWAIT);
#else
  // The offset goes as two longs, low then high; -1 in both is -1.
  return syscall(read ? __NR_preadv2 : __NR_pwritev2, fd, iov, 1, -1L, -1L,
                 RWF_NOWAIT);
#endif
}

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
// One nanosecond past the epoch is long in the past (just after boot) on any
// running system, so it expires immediately -- the requested behavior -- while
// being non-zero, so the kernel arms rather than disarms.
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
// reentering it from anywhere inside it dies on a CHECK (see there).
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
  //   * Counts armed ops (CountArmedOp()) and ABSL_VLOG(1)s, once per ring, the
  //     first time the concurrent count exceeds the depth -- the earliest
  //     moment the configuration is known to be undersized, usually app
  //     startup, long before either crash above or CQ-overflow degradation
  //     is hit.  ABSL_VLOG rather than ABSL_LOG because arm sites run on RT
  //     paths -- see CountArmedOp().
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
  // Ensures the undersized---aio_queue_depth ABSL_VLOG fires once per ring,
  // not once per arm.
  bool queue_depth_warned_ = false;

  // Link accessors for requests on pending_dispatch_ -- the links live in
  // AsyncRequest::internal_state (AioState::link), and hold stale values
  // while unlinked, so membership is tracked by AioState::link.queued
  // rather than the links themselves.
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
  // DrainCompletions(), drained only by ReapCompletions() -- see both for why
  // extraction and dispatch are deliberately two separate steps.
  IntrusiveDoublyLinkedList<AsyncRequest, DispatchLinkTraits> pending_dispatch_;
  // True while ReapCompletions() is dispatching.  Tells teardown paths
  // that a callback frame may be live, so they park state instead of
  // freeing it.
  bool dispatching_ = false;
  // True for the whole of ReapCompletions(): the dispatch loop and the
  // bookkeeping after it.  Backs Poll()'s reentrancy CHECK.  Deliberately
  // wider than dispatching_, which has to go false before that bookkeeping
  // so a DeleteFd() from one of the destructors it runs frees inline.
  bool in_poll_ = false;
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
  // (AioState::uring.raw_io set; the internal wakeup read excluded).  There is
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
  absl::flat_hash_map<int, std::unique_ptr<LegacyState>> legacy_states_;
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
  // in this lifecycle.  Retiring release()es the state out of
  // legacy_states_' unique_ptr and parks it here; ReapCompletions()' sweep
  // deletes it.
  OwningIntrusiveStack<LegacyState, RetiredLegacyTraits> retired_legacy_states_;

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
  // The no-mixing contract: io_uring could run raw AsyncRead()/AsyncWrite()
  // alongside legacy registrations on one fd, but the epoll backend
  // structurally cannot, so the ban is enforced here too.  Every in-flight
  // raw request sits on raw_in_flight_ (linked through AioState::uring --
  // O(1) and malloc-free, since submits must stay RT-callable): ClaimRawFd()
  // links it and CHECKs against the fd's LegacyState, the legacy
  // registration calls scan the list via CheckNoRawOnFd(), and the terminal
  // completion unlinks in DrainCompletions().
  void ClearStaleRawState(AsyncRequest *request);
  // Dies if request is resolved but still waiting on pending_dispatch_ for
  // its callback; otherwise clears a queued flag left by a destroyed Aio.
  void ClearStaleQueuedFlag(AsyncRequest *request);
  void ClaimRawFd(AsyncRequest *request);
  void UnlinkRawRequest(AsyncRequest *request);
  // Dies if an in-flight raw request on fd conflicts with the named legacy
  // hook, with the same messages as EpollImpl's checks.
  enum class LegacyHook { kReadable, kWritable, kError, kEvents };
  void CheckNoRawOnFd(FileDescriptor fd, LegacyHook hook);
  struct RawLinkTraits {
    static AsyncRequest *&next(AsyncRequest *request) {
      return State(request).uring.raw_next;
    }
    static AsyncRequest *&prev(AsyncRequest *request) {
      return State(request).uring.raw_prev;
    }
  };
  IntrusiveDoublyLinkedList<AsyncRequest, RawLinkTraits> raw_in_flight_;
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
  OwningIntrusiveStack<IoUringTimerState, TimerOrphanTraits> orphaned_timers_;
  OwningIntrusiveStack<ThreadSignalReceiverState, ReceiverOrphanTraits>
      orphaned_receivers_;
  // Drained orphans are recycled here (and reused by MakeTimerState() /
  // RegisterThreadSignalReceiver) rather than freed: recycling is pointer
  // manipulation, legal on any thread including RT, and it bounds memory
  // by peak usage instead of cumulative churn.  Actual deallocation only
  // happens in ~IoUringImpl().
  OwningIntrusiveStack<IoUringTimerState, TimerOrphanTraits> free_timers_;
  OwningIntrusiveStack<ThreadSignalReceiverState, ReceiverOrphanTraits>
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
  // DrainLegacyEpoll() hands a hangup to OnEvents as kErr, and to the
  // in/out/err handlers only when no other bit carries it.  See
  // RouteHangupOrError().

  // Registered-vs-not is decided on the caller's untranslated mask,
  // exactly as EPoll::DoEpollCtl() always did: a mask of only bits the
  // translation above drops (e.g. a bare EPOLLHUP) must keep the fd
  // registered -- with an empty event set, which the kernel accepts and
  // still augments with EPOLLERR/EPOLLHUP -- not silently unregister it.
  if (state->events == 0) {
    if (state->epoll_registered) {
      int ret = epoll_ctl(legacy_epoll_fd_, EPOLL_CTL_DEL, state->fd, nullptr);
      // Strict, like EPoll::DoEpollCtl().  ENOENT means the fd isn't ours
      // any more, and the number could have been reused by now.  Abort to draw
      // attention to the use-after-free instead of allowing the process to
      // continue with file descriptors incorrectly closed. ForgetClosedFd() is
      // the API for an fd that is already closed.
      ABSL_PCHECK(ret == 0) << "epoll_ctl DEL failed for fd " << state->fd;
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
  //
  // That readiness is the whole epoll instance's, not one fd's, so a
  // callback which fully drains its own fd can still re-fire immediately
  // for a different one.  The SQE is only queued here; the next Poll()
  // submits it, so the kernel checks readiness after this firing's
  // callbacks ran and one they consumed produces nothing.
  // EpollReadinessSpansRegistrations covers both.
  struct io_uring_sqe *sqe = ArmSqe();

  io_uring_prep_poll_add(sqe, legacy_epoll_fd_, POLLIN);

  ArmRequest(
      sqe, &legacy_epoll_request_,
      [](Completion completion, void *context) {
        auto *impl = static_cast<IoUringImpl *>(context);
        // Nothing ever cancels this poll, so any failure is unexpected --
        // and absorbing it would silently stop every OnReadable/
        // OnWritable/OnError/OnEvents callback on this loop (this poll is
        // the only thing that ever fires them).
        ABSL_CHECK(aos::IsOk(completion.status))
            << "Poll on the embedded legacy-fd epoll instance failed: "
            << aos_strerror(completion.result);
        // Re-arm before dispatching: a single-shot poll is consumed by
        // this firing, so the next check needs to be queued now,
        // regardless of what a callback below ends up doing (including
        // deleting fds this very drain pass would otherwise still be
        // about to look at).
        impl->SubmitLegacyEpollPoll();
        if (impl->DrainLegacyEpoll()) {
          impl->user_dispatch_ = true;
        }
      },
      this);
}

bool IoUringImpl::DrainLegacyEpoll() {
  // One event per firing.  An event is not a callback: its mask can carry
  // readable, writable and error, which go to three separate handlers
  // below.  That is the bounded exception Aio::Poll() documents.
  //
  // Fairness comes from level-triggered epoll itself: a still-ready fd goes
  // back on the ready list's tail, so successive firings rotate through
  // every ready fd.  The re-armed POLL_ADD (queued before this by the
  // caller) checks readiness at arm time, so any remaining events fire it
  // again on the next Poll().  This is also what keeps legacy fds fair
  // against the ring's native work: a Poll() dispatches at most one legacy
  // event alongside that cycle's native completions, instead of a whole
  // batch crowding them out.  Not a drain-to-empty loop: that busy-spins
  // forever on any fd whose callback doesn't consume its own readiness
  // (e.g. LegacyFdTest's OnEvents callback, which just counts -- confirmed
  // live).  One event also means no stale-batch hazard: nothing runs
  // between epoll_wait() and dispatch, so the by-fd lookup below cannot
  // race a deletion.
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
    auto it = legacy_states_.find(ready_fd);
    return it == legacy_states_.end() ? nullptr : it->second.get();
  }();
  if (state == nullptr) return false;

  uint32_t ready_epoll = 0;
  if (event.events & EPOLLIN) ready_epoll |= kIn;
  if (event.events & EPOLLPRI) ready_epoll |= kPri;
  if (event.events & EPOLLOUT) ready_epoll |= kOut;
  if (event.events & EPOLLERR) ready_epoll |= kErr;
  // EPOLLHUP reaches OnEvents as kErr, and the in/out/err trio only when
  // nothing else carries it.  An error with no err_fn goes to the subscribed
  // in/out handlers.  See RouteHangupOrError().  This drain carries no raw
  // requests (CheckNoRawOnFd() keeps them on the ring), so those are the only
  // exceptions here.
  const bool terminal = (event.events & (EPOLLERR | EPOLLHUP)) != 0;

  if (state->events_fn) {
    state->events_fn(terminal ? (ready_epoll | kErr) : ready_epoll);
    return true;
  }
  ready_epoll =
      RouteHangupOrError(ready_epoll, (event.events & EPOLLHUP) != 0,
                         state->events, static_cast<bool>(state->err_fn));
  // EPoll::InOutEventData::DoCallbacks()'s rules, message text included --
  // callers depend on those semantics, so the bits the kernel reported are
  // dispatched, as RouteHangupOrError() routed them.  An error that reaches
  // the err_fn CHECK with no err_fn is fatal, which now only happens to an fd
  // subscribed to neither direction.  CheckNoRawOnFd() keeps raw requests
  // on the ring, so every fd reaching this drain is a purely legacy
  // registration.
  //
  // The state pointer is held across all three callbacks -- the same shape
  // EPoll::Poll() always had, made safe by parking: a callback that
  // DeleteFd()s this registration (its own fd is the ordinary case) parks
  // the state, still allocated, with fd tombstoned to -1.  Checking that
  // tombstone is the whole staleness story.  In particular there is no
  // by-fd re-lookup to get confused by a callback that deletes its fd,
  // closes it, and registers a fresh fd that reuses the same number --
  // the old event's remaining bits die with the tombstone.
  bool dispatched = false;
  if (ready_epoll & kInEvents) {
    ABSL_CHECK(state->in_fn)
        << ": No handler registered for input events on descriptor " << ready_fd
        << ". Received events = 0x" << std::hex << ready_epoll << std::dec;
    state->in_fn();
    dispatched = true;
    if (state->fd == -1) return dispatched;
  }
  if (ready_epoll & kOutEvents) {
    ABSL_CHECK(state->out_fn)
        << ": No handler registered for output events on descriptor "
        << ready_fd << ". Received events = 0x" << std::hex << ready_epoll
        << std::dec;
    state->out_fn();
    dispatched = true;
    if (state->fd == -1) return dispatched;
  }
  if (ready_epoll & kErrorEvents) {
    ABSL_CHECK(state->err_fn)
        << ": No handler registered for error events on descriptor " << ready_fd
        << ". Received events = 0x" << std::hex << ready_epoll << std::dec
        << ". " << internal::GetSocketErrorStr(ready_fd);
    state->err_fn();
    dispatched = true;
  }
  return dispatched;
}

void IoUringImpl::ThreadSignalReceiverState::Submit(IoUringImpl *impl) {
  this->impl = impl;
  struct io_uring_sqe *sqe = impl->ArmSqe();

  io_uring_prep_poll_multishot(sqe, fd, POLLIN);

  const CompletionCallback callback = [](Completion completion, void *context) {
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
      const bool consumed_any = state->receiver->ConsumeWakeup();
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
  ArmRequest(sqe, &request, callback, this);
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
  // Deterministically illegal under RT: this destructor might not free any
  // memory if there's nothing left, and we want to avoid non-deterministic
  // violations of RT rules.
  aos::CheckNotRealtime();

  // Owner-facing state must be gone first, exactly as EPoll::~EPoll() has
  // always CHECKed, and as aio.h documents ("All Fds must be cleaned up
  // before this class is destroyed").  A live Aio::Timer is the sharpest
  // case: its destructor dereferences this impl, so a Timer outliving its
  // Aio is a use-after-free that would otherwise go silent.
  ABSL_CHECK(active_timers_.empty())
      << ": An Aio::Timer must be destroyed before its Aio";
  ABSL_CHECK(legacy_states_.empty())
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
  // Explicitly, rather than leaving it to the members' own destructors: these
  // have to be freed after the queue_exit above, and ~LegacyState can run
  // capture destructors which reenter the Aio, so the timing is deliberate.
  // The destructors remain as the backstop.
  orphaned_timers_.Clear();
  orphaned_receivers_.Clear();
  free_timers_.Clear();
  free_receivers_.Clear();
  // retired_legacy_states_ is expected empty here -- parked states are
  // freed at the end of the dispatch that parked them -- but sweep it as
  // a leak-proof backstop rather than trusting that silently.
  retired_legacy_states_.Clear();
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
    // ABSL_VLOG, not ABSL_LOG: this can run on an RT thread (arm sites live
    // inside Poll() and on the documented RT-safe submission paths), where
    // default-on logging allocates and locks.  Disabled-verbosity ABSL_VLOG is
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
// Legacy fds themselves (legacy_states_, and their epoll_ctl registrations on
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
  //     is not on this list" ABSL_CHECK.  Its new owner could then be handed a
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
    // Only DestroyTimerState() sets `canceling`, and it unlinks the timer
    // at the same time, so nothing reachable from here has it set.  Check
    // rather than clear: clearing would resurrect the poll on a timer its
    // owner has already destroyed.
    ABSL_CHECK(!curr->canceling);
    static_cast<IoUringTimerState *>(curr)->SubmitPoll(/*draining_ok=*/true);
    curr = next;
  }
}

bool IoUringImpl::Poll(bool block) {
  // Not reentrant: a nested Poll() can never dispatch (the outer loop owns
  // delivery), so reentry could only silently starve its caller.  Die at
  // the call site instead; fatal is RT-clean.
  ABSL_CHECK(!in_poll_ && !in_before_wait_)
      << "Aio::Poll() reentered from inside a completion callback, a "
         "before-wait function, or a destructor the loop ran; wait by "
         "returning to the event loop instead";

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
        // ABSL_VLOG, not ABSL_LOG: Poll() legitimately runs under
        // ScopedRealtime, where default-on logging either allocates (ABSL_LOG,
        // fatal via the malloc hook) or blocks in a write(2) from the RT thread
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
    // ABSL_VLOG for the same RT-safety reasons as the short-circuit path above.
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
  // Before anything is armed: dying here must not leave an SQE behind.
  ClearStaleQueuedFlag(request);
  struct io_uring_sqe *sqe = ArmSqe();

  io_uring_prep_read(sqe, fd, buffer.data(), buffer.size(), -1);
  io_uring_sqe_set_data64(sqe, NewIncarnationUserData(request));
  request->done = false;
  // The internal wakeup read is a persistent registration with its own
  // re-arm path; everything else is a raw request a ring rebuild could not
  // reconstruct -- count it (see DowngradeFromSingleIssuer()), and claim
  // the fd against legacy registrations (see ClaimRawFd()).
  //
  // Each loop constructs its own event_fd which means event_fd.wakeup_req
  // cannot be reused from a dead loop.
  if (request != &event_fd.wakeup_req) {
    ClearStaleRawState(request);
    State(request).uring.raw_io = RawIo::kRead;
    State(request).uring.raw_fd = fd;
    ++raw_requests_in_flight_;
    ClaimRawFd(request);
  }
}

void IoUringImpl::AsyncWrite(FileDescriptor fd, std::span<const char> buffer,
                             AsyncRequest *request) {
  CheckSubmitterThread();
  // See AsyncRead().
  ClearStaleQueuedFlag(request);
  struct io_uring_sqe *sqe = ArmSqe();

  io_uring_prep_write(sqe, fd, buffer.data(), buffer.size(), -1);
  io_uring_sqe_set_data64(sqe, NewIncarnationUserData(request));
  request->done = false;
  // See AsyncRead().
  ClearStaleRawState(request);
  State(request).uring.raw_io = RawIo::kWrite;
  State(request).uring.raw_fd = fd;
  ++raw_requests_in_flight_;
  ClaimRawFd(request);
}

void IoUringImpl::ClearStaleQueuedFlag(AsyncRequest *request) {
  if (State(request).link.queued == 0) {
    return;
  }
  // Queued on THIS loop means resolved -- `done` is set -- but its callback
  // has not run, so it is still in flight (see AsyncRequest::done).  Re-arming
  // it would relink a node the dispatch queue still holds.  Asked of the
  // loop, not the request, for the reason ClearStaleRawState() gives; this
  // walk only runs for a request that arrives carrying the flag.
  for (AsyncRequest *req = pending_dispatch_.front(); req != nullptr;
       req = pending_dispatch_.Next(req)) {
    ABSL_CHECK(req != request) << internal::kRequestStillInFlight;
  }
  // Not ours: left over from a previous Aio destroyed with this request
  // queued, which constraint 2 permits and whose destructor could not clear
  // it (the request may equally have died first).  Kept, the flag would make
  // QueuePendingDispatch() take its already-linked early return and silently
  // drop the callback.
  State(request).link.queued = 0;
}

void IoUringImpl::ClearStaleRawState(AsyncRequest *request) {
  if (State(request).uring.raw_io == RawIo::kNone) {
    return;
  }
  // Ask this instance rather than the request.  raw_in_flight_ *is* the set
  // of requests this Aio has claimed, so it answers both cases without the
  // request having to carry an owner identity: found means the caller is
  // re-arming something still in flight, which constraint 2 forbids, and not
  // found means the state is left over from an Aio destroyed underneath it,
  // which constraint 2 explicitly permits.
  //
  // O(in-flight raw requests), the same walk and the same argument as
  // CheckNoRawOnFd() -- and this only runs for a request that arrives
  // already carrying raw state, which is the rare path.
  for (AsyncRequest *req = raw_in_flight_.front(); req != nullptr;
       req = raw_in_flight_.Next(req)) {
    ABSL_CHECK(req != request) << internal::kRequestStillInFlight;
  }
  // Left over from a previous Aio which was destroyed with this request
  // pending -- permitted by constraint 2, and not something that Aio's
  // destructor could have cleaned up: by then the request may already be
  // gone (see ~IoUringImpl, which does not walk pending_dispatch_ for the
  // same reason).  Resolved here instead, exactly as link.queued is.
  //
  // Its raw_prev/raw_next name the dead instance's list, so trusting raw_io
  // would skip ClaimRawFd() while still arming the SQE, and the terminal
  // completion's UnlinkRawRequest() would run against a list this request
  // was never on.
  State(request).uring.raw_io = RawIo::kNone;
  State(request).uring.raw_prev = nullptr;
  State(request).uring.raw_next = nullptr;
}

void IoUringImpl::ClaimRawFd(AsyncRequest *request) {
  const int fd = State(request).uring.raw_fd;
  // A lookup, never an insert -- this path must stay malloc-free.  Same
  // contract and messages as EpollImpl's AsyncRead()/AsyncWrite(), so an
  // OnError-only registration is a conflict too.
  auto it = legacy_states_.find(fd);
  if (it != legacy_states_.end()) {
    const LegacyState &state = *it->second;
    ABSL_CHECK(!state.events_fn)
        << "Cannot mix OnEvents and AsyncRead/AsyncWrite on fd " << fd;
    if (State(request).uring.raw_io == RawIo::kRead) {
      ABSL_CHECK(!state.in_fn)
          << "Cannot mix OnReadable and AsyncRead on fd " << fd;
    } else {
      ABSL_CHECK(!state.out_fn)
          << "Cannot mix OnWritable and AsyncWrite on fd " << fd;
    }
    // Legacy handlers and raw requests are exclusive on an fd in both
    // directions, so the opposite-direction handler collides too, and so
    // does an OnError-only registration.
    ABSL_CHECK(!state.in_fn && !state.out_fn && !state.err_fn)
        << "Cannot mix legacy handlers and AsyncRead/AsyncWrite on fd " << fd;
  }
  raw_in_flight_.PushFront(request);
}

void IoUringImpl::UnlinkRawRequest(AsyncRequest *request) {
  // Only this instance's list is ever unlinked from, and Remove() CHECKs that
  // request is on it.  A request carrying another instance's state is reset
  // at arm instead (see ClearStaleRawState()).
  raw_in_flight_.Remove(request);
}

void IoUringImpl::CheckNoRawOnFd(FileDescriptor fd, LegacyHook hook) {
  // Legacy handlers and raw requests are exclusive on an fd in both
  // directions, so *any* raw request naming it is fatal.  `hook` and the
  // request's direction only choose which message says so -- naming the
  // specific collision when there is one is friendlier than the general
  // rule, but they do not decide whether to die.
  //
  // O(in-flight raw requests), which registration paths can afford -- they
  // allocate anyway.
  for (AsyncRequest *req = raw_in_flight_.front(); req != nullptr;
       req = raw_in_flight_.Next(req)) {
    if (State(req).uring.raw_fd != fd) {
      continue;
    }
    const RawIo raw_io = State(req).uring.raw_io;
    const char *message;
    if (hook == LegacyHook::kEvents) {
      message = "Cannot mix OnEvents and AsyncRead/AsyncWrite on fd ";
    } else if (hook == LegacyHook::kReadable && raw_io == RawIo::kRead) {
      message = "Cannot mix OnReadable and AsyncRead on fd ";
    } else if (hook == LegacyHook::kWritable && raw_io == RawIo::kWrite) {
      message = "Cannot mix OnWritable and AsyncWrite on fd ";
    } else {
      // The cross-direction pairing, and OnError, which has no direction of
      // its own.
      message = "Cannot mix AsyncRead/AsyncWrite and legacy handlers on fd ";
    }
    ABSL_LOG(FATAL) << message << fd;
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
  before_wait_functions.emplace_back(std::move(function));
}

void IoUringImpl::OnReadable(FileDescriptor fd,
                             std::function<void()> callback) {
  CheckSubmitterThread();
  CheckNoRawOnFd(fd, LegacyHook::kReadable);
  auto [it, inserted] = legacy_states_.try_emplace(fd);
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
  CheckNoRawOnFd(fd, LegacyHook::kError);
  auto [it, inserted] = legacy_states_.try_emplace(fd);
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
  CheckNoRawOnFd(fd, LegacyHook::kWritable);
  auto [it, inserted] = legacy_states_.try_emplace(fd);
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
  CheckNoRawOnFd(fd, LegacyHook::kEvents);
  auto [it, inserted] = legacy_states_.try_emplace(fd);
  ABSL_CHECK(inserted) << "May not replace OnEvents handlers for fd " << fd;

  it->second = std::make_unique<IoUringImpl::LegacyState>();
  it->second->fd = fd;
  auto &state = *it->second;
  state.events_fn = std::move(callback);
}

void IoUringImpl::DeleteFd(FileDescriptor fd) {
  CheckSubmitterThread();
  auto it = legacy_states_.find(fd);
  ABSL_CHECK(it != legacy_states_.end()) << "fd " << fd << " not found";

  if (it->second->epoll_registered) {
    int ret = epoll_ctl(legacy_epoll_fd_, EPOLL_CTL_DEL, fd, nullptr);
    // Strict; see UpdateLegacyEpoll().
    ABSL_PCHECK(ret == 0) << "epoll_ctl DEL failed for fd " << fd;
  }
  // Mid-dispatch the state is parked, not destroyed: a callback may
  // DeleteFd() its own fd, in which case one of this state's
  // std::functions is the function currently executing -- destroying it
  // here would free the lambda's captures out from under the running
  // code.  Parked states are freed at the end of this same outermost
  // dispatch (ReapCompletions()), on this thread, in this same non-RT
  // context -- see Aio::DeleteFd()'s CheckNotRealtime() for why that context is
  // guaranteed.  Outside dispatch nothing can be executing out of the
  // state and it is freed right here.
  if (dispatching_) {
    // Tombstone before parking: DrainLegacyEpoll() holds the state pointer
    // across its callbacks and stops dispatching when fd goes to -1.
    it->second->fd = -1;
    retired_legacy_states_.Push(it->second.release());
  }
  legacy_states_.erase(it);
}

void IoUringImpl::ForgetClosedFd(FileDescriptor fd) {
  CheckSubmitterThread();
  auto it = legacy_states_.find(fd);
  ABSL_CHECK(it != legacy_states_.end()) << "fd " << fd << " not found";

  // fd is already closed, which drops the kernel's epoll registration for it
  // automatically (epoll_ctl(2)) -- nothing to undo here beyond forgetting
  // our own bookkeeping.
  // Parked (with the same tombstone) or freed for the same reasons as
  // DeleteFd().
  if (dispatching_) {
    it->second->fd = -1;
    retired_legacy_states_.Push(it->second.release());
  }
  legacy_states_.erase(it);
}

void IoUringImpl::EnableWritable(FileDescriptor fd) {
  CheckSubmitterThread();
  auto it = legacy_states_.find(fd);
  ABSL_CHECK(it != legacy_states_.end()) << "fd " << fd << " not found";

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
  auto it = legacy_states_.find(fd);
  ABSL_CHECK(it != legacy_states_.end()) << "fd " << fd << " not found";

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
  auto it = legacy_states_.find(fd);
  ABSL_CHECK(it != legacy_states_.end()) << "fd " << fd << " not found";

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
  ABSL_CHECK(receiver_state_ == nullptr)
      << "Duplicate ThreadSignalReceiver registration: only one receiver "
         "may be active at a time (see receiver_state_)";

  if (ThreadSignalReceiverState *state = free_receivers_.Pop();
      state != nullptr) {
    const uint16_t generation = State(&state->request).uring.generation;
    *state = ThreadSignalReceiverState();
    State(&state->request).uring.generation = generation;
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
  receiver->ConsumeWakeup();
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
  const uint16_t generation = State(&request).uring.generation;
  request = AsyncRequest();
  State(&request).uring.generation = generation;
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
  ArmRequest(sqe, &request, &IoUringTimerState::OnTimerFdReadable, this);
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

  // Last use of tstate.  A callback may destroy its own timer, and
  // DestroyTimerState() frees the state outright when the poll has already
  // resolved, so dispatch off copies rather than depending on the re-arm
  // above having cleared `done`.
  const CompletionCallback user_callback = tstate->user_callback;
  void *const user_context = tstate->user_context;
  IoUringImpl *const impl = tstate->impl_;

  if (!expired || user_callback == nullptr) {
    return;
  }
  Completion timer_completion;
  // nullptr, exactly as documented: Completion::user_data is "the opaque
  // pointer supplied by the caller", and Timer::Schedule() takes no
  // user_data -- leaking the internal state pointer here contradicted
  // that.
  timer_completion.user_data = nullptr;
  timer_completion.status = aos::Ok();
  timer_completion.result = 0;
  impl->user_dispatch_ = true;
  user_callback(timer_completion, user_context);
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
  // Canceled raw requests, queued behind everything else this drain finds.
  // A SINGLE_ISSUER ring posts a canceled request's CQE from task work, after
  // the inline completions of requests submitted behind the cancel; the
  // other setups post it from inside the cancel, before them.  Moving them
  // here gives every setup the first order, which is the one aio.h promises.
  IntrusiveDoublyLinkedList<AsyncRequest, DispatchLinkTraits> canceled;
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
    if (generation != State(req).uring.generation) {
      // A superseded incarnation's CQE.  Nothing is ever legitimately
      // stale: acks cannot be (handled above under the sentinel), and a
      // stale target CQE means arms-behind-terminals broke -- die rather
      // than bury the evidence.
      ABSL_LOG(FATAL) << "Stale completion (generation " << generation
                      << " vs current " << State(req).uring.generation
                      << ") for a target op: an incarnation was armed over "
                         "one whose completions were still in flight";
    }

    const bool raw = State(req).uring.raw_io != RawIo::kNone;
    if (!(cqe->flags & IORING_CQE_F_MORE)) {
      req->done = true;
      // Terminal completion: this op no longer occupies an armed_ops_
      // slot.  (F_MORE completions leave the multishot armed.)
      ABSL_CHECK_GT(armed_ops_, 0) << "Terminal CQE with no ops armed";
      --armed_ops_;
      if (raw) {
        UnlinkRawRequest(req);
        State(req).uring.raw_io = RawIo::kNone;
        --raw_requests_in_flight_;
      }
    }
    if (req->callback) {
      State(req).uring.user_visible = raw ? 1 : 0;
      if (raw && cqe->res == -ECANCELED && !State(req).link.queued) {
        State(req).link.result = cqe->res;
        State(req).link.queued = 1;
        canceled.PushBack(req);
      } else {
        QueuePendingDispatch(req, cqe->res);
      }
    }
  }
  while (AsyncRequest *req = canceled.PopFront()) {
    pending_dispatch_.PushBack(req);
  }
  if (count > 0) {
    io_uring_cq_advance(&ring, count);
  }
  return processed;
}

bool IoUringImpl::ReapCompletions() {
  // Backs Poll()'s reentrancy CHECK for everything below, callbacks and
  // the teardown bookkeeping alike.
  in_poll_ = true;
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
    if (State(req).uring.user_visible) {
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
  //
  // Cleared before the destructor runs rather than after, matching the
  // sweep below popping before it deletes: destroying the std::function
  // runs its captures' destructors, which are user code, and this pointer
  // must never name a function that is already being destroyed.
  if (ThreadSignalReceiverState *state = deferred_receiver_callback_clear_) {
    aos::CheckNotRealtime();
    deferred_receiver_callback_clear_ = nullptr;
    state->callback = nullptr;
  }

  // Free legacy registrations retired mid-dispatch (see DeleteFd()).  Why
  // deleting here is safe:
  //   * No callback can still be executing out of one.  A LegacyState's
  //     std::functions run in exactly one place -- DrainLegacyEpoll(),
  //     inside the legacy trampoline, dispatched only by the dispatch loop
  //     above (of which Poll()'s reentrancy CHECK permits exactly one) --
  //     and that loop just exited, so every such frame has unwound.
  //   * Nothing else can reach a parked state: DeleteFd() already erased
  //     it from legacy_states_ and from the epoll registration, and a
  //     LegacyState is not an AsyncRequest, so pending_dispatch_ cannot
  //     name it (which is why timers need UnlinkPendingDispatch() and
  //     these don't).
  //   * ~LegacyState runs capture destructors, which may reenter the Aio;
  //     dispatching_ is already false and the node is popped before the
  //     delete, so a reentrant DeleteFd() frees inline.  A reentrant
  //     Poll() dies here like anywhere else inside Poll(), which is what
  //     in_poll_ is for: this and the receiver-callback clear above are
  //     the only points where user code runs with the dispatch loop
  //     already marked done, and a Poll() from one would otherwise
  //     dispatch without its caller knowing.
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

  // Safe point to recycle drained orphans: no CQ scan is live and every
  // dispatch frame has unwound.  Recycling never allocates or frees, so
  // this is fine on an RT thread too.
  RecycleDrainedOrphans();

  in_poll_ = false;
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

class EpollImpl;

struct EpollTimerState : public Aio::TimerState {
  explicit EpollTimerState(EpollImpl *impl) : impl_(impl) {}
  ~EpollTimerState() override;

  void Initialize() override;
  void Schedule(aos::monotonic_clock::time_point deadline,
                CompletionCallback callback, void *context) override;
  void Cancel(bool reap) override;

  std::unique_ptr<TimerFD> timer_fd;

 private:
  EpollImpl *impl_;
};

// Completion delivery mirrors IoUringImpl's, so the two backends resolve and
// deliver requests at the same points:
//
//   * Submit does no I/O.  AsyncRead()/AsyncWrite() queue the request, and
//     the harvest makes its first attempt in submission order -- what
//     io_uring's kernel does with its SQ: try inline, poll-arm on EAGAIN.
//     Only a request that got EAGAIN is registered with epoll.
//   * Resolving is eager.  One Poll() harvests everything that is ready --
//     every raw AsyncRead/AsyncWrite whose fd epoll reports (the I/O itself
//     runs right there), every submit-time result, every Cancel() -- sets
//     `done`, and queues the completion on ready_.  That is io_uring's CQ
//     drain.  It reads epoll one event at a time and acts on each before
//     reading the next, so no dequeued event can go stale.  A raw I/O can
//     make another raw request ready (a pipe's write filling its read end),
//     so the harvest keeps asking, non-blocking, until nothing more is ready
//     -- io_uring runs both in the same submit.
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
class EpollImpl : public Aio::Impl {
  friend struct EpollTimerState;

 public:
  EpollImpl();
  ~EpollImpl() override;

  std::unique_ptr<Aio::TimerState> MakeTimerState() override;

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
  enum class Direction { kRead, kWrite };

  // Which of the two request lists a request is on, and why.  Kept in
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
    // On pending_: Cancel()ed.  AioState::epoll.ptr holds the FdRegistration
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

  struct FdRegistration {
    int fd = -1;
    // Raw requests in flight, from submit until they resolve -- including
    // while still on pending_ waiting for their first attempt, so the
    // per-fd CHECKs see them.  Only an *armed* one (see Armed()) is waiting
    // on epoll.  The harvest does their I/O directly; nothing is parked in
    // in_fn/out_fn for them, so those are only ever the caller's legacy
    // handlers.
    AsyncRequest *read_req = nullptr;
    AsyncRequest *write_req = nullptr;
    std::function<void()> in_fn = nullptr;
    std::function<void()> out_fn = nullptr;
    std::function<void()> err_fn = nullptr;
    std::function<void(uint32_t)> events_fn = nullptr;
    uint32_t events = 0;

    bool registered = false;
    uint32_t epoll_events = 0;
    // True while this registration exists only to carry AsyncRead/AsyncWrite
    // requests; retired back to the pool once the last one is resolved (see
    // SettleRawRegistration()).  Legacy registrations live until
    // DeleteFd()/ForgetClosedFd().  An fd is one or the other, never both.
    bool async_only = false;
    // Which free list this registration goes back to.  Raw registrations
    // come from the --aio_pool_size pool, which is what keeps arming a
    // request allocation-free.  Legacy ones -- every On*() fd, every timer,
    // the thread-signal receiver -- come from their own list, so they cannot
    // use up the slots the realtime submit path relies on.
    bool pooled = false;
    // Set by UpdateEpoll() when epoll_ctl(ADD) returned EPERM: a regular
    // file, always ready.  Its I/O is then done at once, blocking, instead
    // of waiting for an event that can never arrive -- see DoRawIo().
    bool unpollable = false;
    // Set by DoRawIo() when the fd refused RWF_NOWAIT -- see there.
    bool nowait_unsupported = false;
    // Canceled requests whose Canceled completion has not been resolved yet.
    // aio.h documents Cancel() as asynchronous, so the fd stays claimed
    // until then, which is the point io_uring releases its claim too (its
    // terminal CQE is drained by the next Poll(), and so is this).  Counts,
    // not flags: two requests can be canceled on one fd before either
    // resolves, and resolving the first must not free the fd for the second.
    int cancel_claims_read = 0;
    int cancel_claims_write = 0;
    // Positive errno from an epoll_ctl(ADD) the kernel refused.
    // UpdateEpoll() returns false and leaves it here; StartRaw() turns it
    // into an error completion rather than taking the process down.
    int registration_errno = 0;
    // Link for the free lists and retired_.  One link covers them all
    // because a registration is only ever on one: it moves free list ->
    // live_ -> retired_ -> free list.
    FdRegistration *next_free = nullptr;
    // Link for all_, which owns every registration for this impl's lifetime
    // so the tree and the free/retired stacks can all be non-owning.
    FdRegistration *next_all = nullptr;
    // Links for live_.  Owned by the tree; nothing else may touch them
    // while this registration is in it.
    FdRegistration *tree_left = nullptr;
    FdRegistration *tree_right = nullptr;
    FdRegistration *tree_parent = nullptr;
    bool tree_red = false;

    AsyncRequest *&slot(Direction direction) {
      return direction == Direction::kRead ? read_req : write_req;
    }
    int &cancel_claims(Direction direction) {
      return direction == Direction::kRead ? cancel_claims_read
                                           : cancel_claims_write;
    }
    bool idle() const {
      return read_req == nullptr && write_req == nullptr &&
             cancel_claims_read == 0 && cancel_claims_write == 0;
    }

    // Drops everything tying this registration to an fd -- but deliberately
    // not the callbacks.  ReleaseRegistration() can run underneath one of
    // them (a callback which DeleteFd()s its own fd), and destroying a
    // std::function there would free the frame that is executing.  Reset()
    // does that half, once it is safe.
    void Detach() {
      fd = -1;
      read_req = nullptr;
      write_req = nullptr;
      cancel_claims_read = 0;
      cancel_claims_write = 0;
      events = 0;
      registered = false;
      epoll_events = 0;
    }

    // Drops what Detach() left behind, returning this to pool state.  Only
    // safe with no dispatch in flight; see ScrubRetiredRegistrations().
    void Reset() {
      in_fn = nullptr;
      out_fn = nullptr;
      err_fn = nullptr;
      events_fn = nullptr;
      async_only = false;
      unpollable = false;
      nowait_unsupported = false;
    }
  };
  struct FreeLinkTraits {
    static FdRegistration *&next(FdRegistration *reg) { return reg->next_free; }
  };
  struct AllLinkTraits {
    static FdRegistration *&next(FdRegistration *reg) { return reg->next_all; }
  };
  // Orders the live registrations by fd.  Compare() lets
  // GetActiveRegistration() look one up from a bare fd without building a
  // registration to compare against.
  struct LiveTreeTraits {
    static FdRegistration *&left(FdRegistration *reg) { return reg->tree_left; }
    static FdRegistration *&right(FdRegistration *reg) {
      return reg->tree_right;
    }
    static FdRegistration *&parent(FdRegistration *reg) {
      return reg->tree_parent;
    }
    static bool &red(FdRegistration *reg) { return reg->tree_red; }
    static bool Less(FdRegistration *a, FdRegistration *b) {
      return a->fd < b->fd;
    }
    static int Compare(const FdRegistration *reg, int fd) {
      if (reg->fd < fd) return -1;
      if (fd < reg->fd) return 1;
      return 0;
    }
  };

  // A legacy event the harvest saw, handed straight to the marker's
  // dispatch when nothing ran in between -- see DispatchLegacyEvent().
  struct HarvestedLegacyEvent {
    FdRegistration *reg = nullptr;
    uint32_t events = 0;
  };

  FdRegistration *GetActiveRegistration(FileDescriptor fd) const;
  FdRegistration *GetOrCreateLegacyRegistration(FileDescriptor fd);
  void ReleaseRegistration(FdRegistration *reg);
  // Moves retired_ back to the free lists, destroying the parked callbacks.
  // Must only run with no dispatch in flight: a parked callback can be the
  // very function that triggered the release.
  void ScrubRetiredRegistrations();
  // Reconciles reg's epoll registration with what it is now interested in.
  // Takes the registration rather than an fd, as EPoll::DoEpollCtl() does:
  // every caller has just looked it up.
  //
  // Returns false when the kernel refused to add a raw registration and
  // left the errno in reg->registration_errno for the caller to deliver.
  // Legacy registrations die instead, as EPoll always did: there is no
  // request to carry the error.  Every other epoll_ctl() failure dies: an fd
  // the caller closed with a request armed on it is a contract violation
  // (see Aio::AsyncRead()), and this is where epoll can see one.
  bool UpdateEpoll(FdRegistration *reg);

  // The raw request path.  AsyncRead() and AsyncWrite() differ only in
  // direction.
  void SubmitRaw(FileDescriptor fd, void *data, size_t size,
                 AsyncRequest *request, Direction direction);
  // reg's request in `direction` if it is armed -- waiting on epoll -- and
  // nullptr if there is none or it has not had its first attempt yet.
  static AsyncRequest *Armed(FdRegistration *reg, Direction direction) {
    AsyncRequest *req = reg->slot(direction);
    return req != nullptr && Kind(req) == QueueKind::kNone ? req : nullptr;
  }
  // One non-blocking read or write for reg's request in `direction`.  See
  // the definition for how an fd that cannot do that is handled.
  ssize_t DoRawIo(FdRegistration *reg, Direction direction, void *data,
                  size_t size);
  // Performs reg's pending I/O in one direction.  Returns whether the
  // request resolved (false: EAGAIN, still waiting).
  bool TryRawIo(FdRegistration *reg, Direction direction);
  // A submitted request's first attempt, taken off pending_ in submission
  // order: the I/O now if it can complete, else armed on epoll -- io_uring's
  // own order (try inline, poll-arm on EAGAIN).  Returns whether it resolved;
  // arming is not processing anything.
  bool StartRaw(AsyncRequest *request, Direction direction);
  // Services a harvested epoll event on a raw registration.  Returns whether
  // any request resolved.
  bool ServiceRaw(FdRegistration *reg, uint32_t got_events);
  // After a raw registration's requests or claims change: retires it if
  // nothing is left, otherwise updates epoll to match what is.
  void SettleRawRegistration(FdRegistration *reg);
  // Dies if request is already armed, or armed and not yet delivered, on
  // this loop.  See the definition.
  void CheckNotAlreadyInFlight(AsyncRequest *request) const;
  // Dies if epoll reported an event for a registration that is no longer
  // live.  DeleteFd() and a raw registration's retirement DEL strictly
  // first, so neither leaves an entry behind.  ForgetClosedFd() cannot DEL:
  // its fd is already closed, and closing only removes the kernel's entry
  // when it was the last reference to the file.  One still open through a
  // dup() or a fork() keeps the entry, and its next event lands here.
  static void CheckLive(const FdRegistration *reg) {
    ABSL_CHECK(reg->registered)
        << ": epoll reported an event for a released registration.  Its fd "
           "was ForgetClosedFd()'d while its file was still open elsewhere (a "
           "dup() or a fork()), which leaves the kernel watching it; for a "
           "shared file, DeleteFd() before closing instead";
  }

  // Queues a submitted request, or one whose outcome is already known -- a
  // submit-time error, a Cancel() -- for the next harvest, which takes them
  // in this order.  `kind` is one of the QueueKind::kPending* values.
  void QueuePending(AsyncRequest *request, QueueKind kind);
  // Sets done and, if it has a callback to deliver, moves request onto
  // ready_.  `kind` is kReadyResult or kReadyCanceled.
  void QueueReady(AsyncRequest *request, QueueKind kind);
  // Resolves pending_, then waits (or doesn't) for epoll and resolves what it
  // reports.  Returns whether it found anything at all.
  bool Harvest(bool block, HarvestedLegacyEvent *harvested);
  // Everything Poll() does inside its dispatch-depth scope.
  bool PollOnce(bool block);
  // Delivers the legacy marker: one legacy fd's event, per
  // EPoll::DoCallbacks().  Returns false if none is ready any more.
  bool DispatchLegacyEvent(HarvestedLegacyEvent *harvested);
  // Runs the handlers for one legacy event.
  void RunLegacyHandlers(FdRegistration *reg, uint32_t got_events);
  // Marks reg as having no per-call way to avoid sleeping and makes its fd
  // O_NONBLOCK -- see DoRawIo().
  static void FallBackToNonBlocking(FdRegistration *reg);

  int epoll_fd_ = -1;
  // Whether this kernel's RWF_NOWAIT reads can return a spurious 0 (Linux 5.9
  // and 5.10; see KernelHasSpuriousNowaitZero()).  Read once at construction,
  // so the raw I/O path neither calls uname() nor allocates.
  bool nowait_unreliable_ = false;
  // Quit()/Wakeup() write this eventfd to knock Poll() out of epoll_wait().
  // Registered through wakeup_reg_ rather than as a legacy handler: draining
  // it is internal work, which must neither use up a pool slot nor spend
  // Poll()'s one-callback budget (see Aio::Poll()).
  EventFD event_fd_;
  // Identifies the wakeup eventfd in epoll events.  Never in live_ and never
  // on a free list; only its address matters.
  FdRegistration wakeup_reg_;
  // Owns every registration for this impl's lifetime, so the containers
  // below are all non-owning and moving a registration between them is
  // pointer assignment -- which is what makes ReleaseRegistration() safe on
  // the dispatch path, where it runs under RT.
  OwningIntrusiveStack<FdRegistration, AllLinkTraits> all_;
  // Every live registration, keyed on fd.  A tree rather than a sorted
  // vector because SubmitRaw() inserts on the submit path, which aio.h
  // requires to be allocation-free: the links live on the registration, so
  // inserting one allocates nothing at all.
  IntrusiveRbTree<FdRegistration, LiveTreeTraits> live_;
  // Pre-allocated pool (--aio_pool_size) for raw registrations only.
  // Intrusive, like the io_uring backend's free_timers_/free_receivers_:
  // parking and unparking are then pointer assignment.
  IntrusiveStack<FdRegistration, FreeLinkTraits> raw_free_list_;
  // Recycled legacy registrations.  Grown on demand: On*() is
  // CheckNotRealtime(), so allocating there is fine.
  IntrusiveStack<FdRegistration, FreeLinkTraits> legacy_free_list_;
  // Legacy registrations released mid-dispatch, still holding
  // std::functions -- one of which can be the currently-executing callback --
  // so they are destroyed at the end of the Poll(), by
  // ScrubRetiredRegistrations().  Raw registrations never park here; see
  // ReleaseRegistration().
  IntrusiveStack<FdRegistration, FreeLinkTraits> retired_;
  // Non-zero while Poll() is dispatching.  Tells ReleaseRegistration()
  // whether a callback frame may be live.
  int dispatch_depth_ = 0;
  // True for all of Poll(), including the scrub of retired registrations
  // that runs after dispatch_depth_ drops: that scrub destroys
  // std::functions, and their captures' destructors are user code too.
  // Backs Poll()'s reentrancy CHECK, like IoUringImpl's in_poll_.
  bool in_poll_ = false;
  std::vector<std::function<void()>> before_wait_functions_;
  // True while Poll() is running the before-wait functions; BeforeWait()
  // CHECKs it, matching IoUringImpl.
  bool in_before_wait_ = false;

  // Quit() writes these from other threads and from a signal handler
  // (ShmEventLoop's SIGINT/SIGHUP/SIGTERM handler); plain bools would be a
  // data race that can miss the shutdown outright.  A handler may only
  // touch lock-free atomics, so assert that; a non-lock-free atomic could
  // deadlock against the interrupted thread.
  static_assert(std::atomic<bool>::is_always_lock_free,
                "Quit() runs in a signal handler, so these have to be usable "
                "from one");
  std::atomic<bool> run_ = false;
  std::atomic<bool> quit_requested_ = false;

  // Both request lists are doubly linked through AioState::link.  A request
  // is on at most one, and link.queued says which and why, so membership is
  // O(1) to test without reordering anything.
  struct PendingLinkTraits {
    static AsyncRequest *&next(AsyncRequest *request) {
      return State(request).link.next;
    }
    static AsyncRequest *&prev(AsyncRequest *request) {
      return State(request).link.prev;
    }
  };
  // Whether request is linked on list.  O(n); only reached on paths that
  // are already errors or rare.
  static bool ListContains(
      const IntrusiveDoublyLinkedList<AsyncRequest, PendingLinkTraits> &list,
      const AsyncRequest *request) {
    for (AsyncRequest *req = list.front(); req != nullptr;
         req = list.Next(req)) {
      if (req == request) return true;
    }
    return false;
  }
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

  // The active ThreadSignalReceiver, if any -- at most one may be
  // registered at a time (see Aio::RegisterThreadSignalReceiver).
  ipc_lib::ThreadSignalReceiver *receiver_ = nullptr;

  // Live EpollTimerStates, counted so ~EpollImpl() can CHECK none outlive
  // it -- ~EpollTimerState dereferences this impl.
  int active_timer_count_ = 0;
};

std::unique_ptr<Aio::TimerState> EpollImpl::MakeTimerState() {
  ++active_timer_count_;
  return std::make_unique<EpollTimerState>(this);
}

EpollImpl::EpollImpl() {
  struct utsname un;
  nowait_unreliable_ =
      uname(&un) == 0 && internal::KernelHasSpuriousNowaitZero(un.release);

  epoll_fd_ = epoll_create1(EPOLL_CLOEXEC);
  ABSL_PCHECK(epoll_fd_ >= 0) << "Failed to create epoll instance";

  const size_t pool_size = absl::GetFlag(FLAGS_aio_pool_size);
  for (size_t i = 0; i < pool_size; ++i) {
    auto *reg = new FdRegistration();
    reg->pooled = true;
    all_.Push(reg);
    raw_free_list_.Push(reg);
  }

  wakeup_reg_.fd = event_fd_.fd();
  struct epoll_event ev;
  std::memset(&ev, 0, sizeof(ev));
  ev.events = EPOLLIN;
  ev.data.ptr = &wakeup_reg_;
  ABSL_PCHECK(epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, event_fd_.fd(), &ev) == 0)
      << "Failed to register the wakeup eventfd";
}

EpollImpl::~EpollImpl() {
  // Registrations a Poll() retired are reclaimed at the end of that Poll(),
  // so this is normally a no-op.  Done first regardless: a parked
  // registration's captures (a handler which deleted its own fd and owned a
  // Timer, say) must be destroyed before the CHECKs below count what is
  // still alive.
  ScrubRetiredRegistrations();
  // Owner-facing state must be gone first, as EPoll::~EPoll() CHECKed and
  // aio.h documents.  A live Aio::Timer's destructor dereferences this
  // impl, so one outliving its Aio is a use-after-free.
  ABSL_CHECK_EQ(active_timer_count_, 0)
      << ": An Aio::Timer must be destroyed before its Aio";
  ABSL_CHECK(receiver_ == nullptr)
      << ": The ThreadSignalReceiver must be unregistered before destroying "
         "the Aio";
  // Only async-only registrations may remain: aio.h's constraint 2 permits
  // destroying the Aio with raw requests still pending.  Caller fd
  // registrations must be gone, as EPoll always CHECKed.
  for (FdRegistration *reg : live_) {
    ABSL_CHECK(reg->async_only)
        << ": fd " << reg->fd
        << " must be removed (DeleteFd()/ForgetClosedFd()) before destroying "
           "the Aio";
  }
  // all_ owns every registration, live and parked alike.  Cleared here
  // rather than left to its destructor so that the std::functions a
  // registration still holds are destroyed while this impl is intact -- a
  // capture's destructor can reenter it.
  all_.Clear();
  run_ = false;
  if (epoll_fd_ >= 0) {
    close(epoll_fd_);
  }
}

EpollImpl::FdRegistration *EpollImpl::GetActiveRegistration(
    FileDescriptor fd) const {
  return live_.Find(fd);
}

EpollImpl::FdRegistration *EpollImpl::GetOrCreateLegacyRegistration(
    FileDescriptor fd) {
  if (auto *reg = GetActiveRegistration(fd)) {
    // Possibly a raw registration; each On*() names the collision it hits.
    return reg;
  }
  FdRegistration *reg = legacy_free_list_.Pop();
  if (reg == nullptr) {
    reg = new FdRegistration();
    all_.Push(reg);
  }
  reg->fd = fd;
  live_.Insert(reg);
  return reg;
}

void EpollImpl::ReleaseRegistration(FdRegistration *reg) {
  ABSL_CHECK(live_.Find(reg->fd) == reg);
  live_.Remove(reg);

  reg->Detach();

  // A raw registration holds no std::function, so nothing can be executing
  // out of it, and it goes straight back to the pool: a completion callback
  // that re-arms its own fd then reuses the slot the completion just gave
  // up, instead of needing a second one until the Poll() ends.  Detach()
  // already tombstoned fd, which is all ServiceRaw() re-reads, and nothing
  // can take the slot back out before then -- only a callback arms.
  if (dispatch_depth_ > 0 && !reg->async_only) {
    // A callback may be running out of reg -- one that DeleteFd()s its own
    // fd -- and Poll()'s dispatch may still re-read reg->fd to notice the
    // deletion.  Park it; ScrubRetiredRegistrations() destroys it once the
    // Poll() ends.
    retired_.Push(reg);
    return;
  }
  // Nothing can be executing out of it: destroy now, so a legacy
  // registration's captures die at DeleteFd() as EPoll's always did.
  reg->Reset();
  (reg->pooled ? raw_free_list_ : legacy_free_list_).Push(reg);
}

void EpollImpl::ScrubRetiredRegistrations() {
  ABSL_CHECK_EQ(dispatch_depth_, 0);
  while (FdRegistration *reg = retired_.Pop()) {
    reg->Reset();
    (reg->pooled ? raw_free_list_ : legacy_free_list_).Push(reg);
  }
  // No trimming and no free(): all_ owns every registration for this impl's
  // lifetime, so each free list only ever grows to its peak concurrent fd
  // count and hands the same objects back out.  Giving surplus back with
  // delete would put a free() inside the caller's realtime section.
}

void EpollImpl::Run() {
  run_ = true;
  // quit_requested_ as well as run_, same as IoUringImpl::Run(): a Quit()
  // racing this startup can have its run_ store clobbered by the store
  // above, and its wakeup is already spent (see
  // AioTest.QuitRacingWithRunStartup).  Also covers a Quit() that landed
  // entirely before Run().
  while (run_ && !quit_requested_) {
    Poll(true);
  }
  run_ = false;
  quit_requested_ = false;
}

bool EpollImpl::Poll(bool block) {
  // Not reentrant, matching IoUringImpl::Poll().  in_poll_ covers the whole
  // body below, the before-wait functions and the scrub included.
  ABSL_CHECK(!in_poll_)
      << "Aio::Poll() reentered from inside a completion callback, a "
         "before-wait function, or a destructor the loop ran; wait by "
         "returning to the event loop instead";
  struct InPoll {
    bool *in_poll;
    explicit InPoll(bool *p) : in_poll(p) { *in_poll = true; }
    ~InPoll() { *in_poll = false; }
  } in_poll_guard(&in_poll_);

  bool processed;
  {
    struct DispatchDepth {
      int *depth;
      explicit DispatchDepth(int *d) : depth(d) { ++*depth; }
      ~DispatchDepth() { --*depth; }
    } dispatch_depth_guard(&dispatch_depth_);
    processed = PollOnce(block);
  }

  // Reclaim what this Poll()'s callback retired (a callback that deleted its
  // own fd), now that its frame has unwound -- at the end of the same
  // dispatch, as IoUringImpl frees its parked legacy state.  Leaving it for
  // the top of the next Poll() moved the free into whatever context that one
  // ran in, which could be ScopedRealtime and trip the RT free check; here it
  // stays with the dispatch whose callback passed Aio::DeleteFd()'s
  // CheckNotRealtime().
  ScrubRetiredRegistrations();
  return processed;
}

bool EpollImpl::PollOnce(bool block) {
  // Registering a before-wait function from inside one is disallowed --
  // see IoUringImpl::BeforeWait().
  in_before_wait_ = true;
  for (const auto &fn : before_wait_functions_) {
    fn();
  }
  in_before_wait_ = false;

  bool processed = false;
  HarvestedLegacyEvent harvested;
  // Only go back to the kernel once everything already resolved has been
  // delivered -- see the class comment.
  if (ready_.empty()) {
    processed = Harvest(block, &harvested);
  }

  // At most one callback per Poll() -- see Aio::Poll().  Only the legacy
  // marker can pop without spending it (its fd may have gone quiet).
  while (AsyncRequest *req = ready_.PopFront()) {
    const QueueKind kind = Kind(req);
    SetKind(req, QueueKind::kNone);
    processed = true;
    if (kind == QueueKind::kReadyLegacy) {
      if (DispatchLegacyEvent(&harvested)) {
        return true;
      }
      continue;
    }
    // Only ever handed over once: the next legacy dispatch must ask epoll
    // again, since this callback may delete or replace that fd.
    harvested.reg = nullptr;
    Completion completion;
    completion.user_data = req->user_data;
    if (kind == QueueKind::kReadyCanceled) {
      completion.status = aos::MakeError("Canceled");
      completion.result = 0;
    } else if (State(req).link.result >= 0) {
      completion.status = aos::Ok();
      completion.result = State(req).link.result;
    } else {
      completion.status = aos::MakeError("epoll error");
      completion.result = -State(req).link.result;
    }
    req->callback(completion, req->context);
    return true;
  }
  return processed;
}

bool EpollImpl::Harvest(bool block, HarvestedLegacyEvent *harvested) {
  bool processed = false;

  // What submit and Cancel() queued since the last Poll(), resolved here
  // rather than when queued, so `done` flips inside Poll() on every backend.
  // The order is io_uring's for one submitted batch: first attempts and
  // submit-time errors in submission order, then the cancels in theirs.
  // io_uring completes the former inline while it submits the batch, and
  // posts a canceled request's completion once the submit is done, so a
  // cancel resolves after the batch's immediate completions, even ones
  // submitted after it.
  for (AsyncRequest *req = pending_.front(); req != nullptr;) {
    AsyncRequest *const next = pending_.Next(req);
    const QueueKind kind = Kind(req);
    if (!IsPendingCancel(kind)) {
      pending_.Remove(req);
      SetKind(req, QueueKind::kNone);
      if (IsPendingSubmit(kind)) {
        // A first attempt that only armed epoll resolved nothing, so it does
        // not count: Poll(false) right after an AsyncRead() on an empty pipe
        // returns false, as io_uring's does, and Poll(true) goes on to wait.
        processed |= StartRaw(req, kind == QueueKind::kPendingSubmitRead
                                       ? Direction::kRead
                                       : Direction::kWrite);
      } else {
        processed = true;
        QueueReady(req, QueueKind::kReadyResult);
      }
    }
    req = next;
  }
  while (AsyncRequest *req = pending_.PopFront()) {
    const QueueKind kind = Kind(req);
    SetKind(req, QueueKind::kNone);
    processed = true;
    // The claim Cancel() took is released now, when the request resolves.
    // io_uring releases its at the terminal CQE, which the next Poll()
    // drains -- the same moment.
    if (auto *reg = static_cast<FdRegistration *>(State(req).epoll.ptr)) {
      --reg->cancel_claims(kind == QueueKind::kPendingCancelRead
                               ? Direction::kRead
                               : Direction::kWrite);
      SettleRawRegistration(reg);
    }
    QueueReady(req, QueueKind::kReadyCanceled);
  }

  // Anything already resolved has to be delivered without waiting on the
  // kernel for more -- and a callbackless request resolved above counts,
  // though it never reaches ready_: resolving it was the work.  After the
  // first event, never wait: everything below is collecting what is already
  // there.
  //
  // One event per epoll_wait(), as EPoll always read them, never a batch: a
  // dequeued event is acted on before the next is read, so none can go stale
  // behind a callback that closed or replaced its fd.  Level-triggered epoll
  // rotates a reported fd to the back of its ready list, so asking again
  // walks every ready fd fairly.
  int timeout = (block && !processed) ? -1 : 0;
  while (true) {
    struct epoll_event event;
    int num_events;
    // EINTR while blocking is absorbed, per the contract on Aio::Poll();
    // Quit()'s wakeup write ends the wait instead.
    do {
      num_events = epoll_wait(epoll_fd_, &event, 1, timeout);
    } while (num_events == -1 && errno == EINTR && timeout != 0);
    timeout = 0;
    if (num_events == -1) {
      if (errno == EINTR) {
        return processed;
      }
      ABSL_PCHECK(num_events != -1);
    }
    if (num_events == 0) {
      break;
    }
    processed = true;

    auto *reg = static_cast<FdRegistration *>(event.data.ptr);
    if (reg == &wakeup_reg_) {
      // Internal work: neither delivered nor rationed.
      event_fd_.Drain();
      continue;
    }
    CheckLive(reg);
    if (reg->async_only) {
      // Do the I/O and keep collecting: that I/O can itself have readied
      // another raw request (writing a pipe makes its read end readable),
      // and io_uring completes both inside one submit.  No user code runs
      // here, so nothing can close an fd underneath the next event.
      if (!ServiceRaw(reg, event.events)) {
        // Reported but not actually ready.  Stop rather than spin on it.
        break;
      }
      continue;
    }
    // A legacy fd.  Running its handlers is the delivery itself, so it cannot
    // be resolved early; a marker takes its place in line, and this event is
    // handed to it directly if nothing runs in between.
    SetKind(&legacy_marker_, QueueKind::kReadyLegacy);
    ready_.PushBack(&legacy_marker_);
    harvested->reg = reg;
    harvested->events = event.events;
    break;
  }
  return processed;
}

bool EpollImpl::DispatchLegacyEvent(HarvestedLegacyEvent *harvested) {
  // The harvest's own event, when nothing has run since it was read: no
  // callback can have deleted or replaced the fd, so asking again would only
  // cost a syscall.
  if (harvested->reg != nullptr) {
    FdRegistration *reg = harvested->reg;
    harvested->reg = nullptr;
    RunLegacyHandlers(reg, harvested->events);
    return true;
  }
  // Otherwise ask epoll which legacy fd is ready now.  One event at a time:
  // level-triggered epoll rotates a reported fd to the back of its ready
  // list, which is what keeps legacy fds fair against each other (the same
  // argument as DrainLegacyEpoll()).  Raw and wakeup events met on the way
  // are resolved as internal work, so this ends once nothing but legacy
  // events can be left -- at most one pass per raw request, plus the wakeup.
  while (true) {
    struct epoll_event event;
    int num_events;
    do {
      num_events = epoll_wait(epoll_fd_, &event, 1, 0);
    } while (num_events == -1 && errno == EINTR);
    ABSL_PCHECK(num_events != -1);
    if (num_events == 0) {
      return false;
    }
    auto *reg = static_cast<FdRegistration *>(event.data.ptr);
    if (reg == &wakeup_reg_) {
      event_fd_.Drain();
      continue;
    }
    CheckLive(reg);
    if (reg->async_only) {
      if (!ServiceRaw(reg, event.events)) {
        // Reported but not actually ready.  Stop rather than spin on it;
        // the legacy event, if any, is found next time.
        return false;
      }
      continue;
    }
    RunLegacyHandlers(reg, event.events);
    return true;
  }
}

void EpollImpl::RunLegacyHandlers(FdRegistration *reg, uint32_t got_events) {
  uint32_t events = 0;
  if (got_events & EPOLLIN) events |= kIn;
  if (got_events & EPOLLPRI) events |= kPri;
  if (got_events & EPOLLOUT) events |= kOut;
  if (got_events & EPOLLERR) events |= kErr;

  if (reg->events_fn) {
    // OnEvents delivers everything, hangups included -- see the OnEvents()
    // contract in aio.h.  SetEvents(fd, EPOLLHUP) is a supported way to
    // watch for one (SetEventsUntranslatedMaskKeepsRegistration), which is
    // why this does not follow EPoll here.
    const bool terminal = (got_events & (EPOLLERR | EPOLLHUP)) != 0;
    reg->events_fn(terminal ? (events | kErr) : events);
    return;
  }

  // EPOLLHUP is absent from the translation above, as it was in EPoll: a
  // hangup is not an error event.  One that nothing else carries is routed
  // to a handler rather than dropped, and so is an error with no err_fn.
  // See RouteHangupOrError().  The CHECKs
  // below are EPoll::InOutEventData::DoCallbacks() verbatim, message text
  // included -- callers depend on those semantics.  A callback that deletes
  // its own fd tombstones reg->fd to -1 (Detach()), which ends the remaining
  // handlers.
  events = RouteHangupOrError(events, (got_events & EPOLLHUP) != 0, reg->events,
                              static_cast<bool>(reg->err_fn));
  if (events & kInEvents) {
    ABSL_CHECK(reg->in_fn)
        << ": No handler registered for input events on descriptor " << reg->fd
        << ". Received events = 0x" << std::hex << events << std::dec;
    reg->in_fn();
  }
  if (reg->fd != -1 && (events & kOutEvents)) {
    ABSL_CHECK(reg->out_fn)
        << ": No handler registered for output events on descriptor " << reg->fd
        << ". Received events = 0x" << std::hex << events << std::dec;
    reg->out_fn();
  }
  if (reg->fd != -1 && (events & kErrorEvents)) {
    ABSL_CHECK(reg->err_fn)
        << ": No handler registered for error events on descriptor " << reg->fd
        << ". Received events = 0x" << std::hex << events << std::dec << ". "
        << internal::GetSocketErrorStr(reg->fd);
    reg->err_fn();
  }
}

void EpollImpl::Quit() {
  quit_requested_ = true;
  run_ = false;
  Wakeup();
}

void EpollImpl::Wakeup() { event_fd_.Write(); }

void EpollImpl::QueuePending(AsyncRequest *request, QueueKind kind) {
  SetKind(request, kind);
  pending_.PushBack(request);
}

void EpollImpl::QueueReady(AsyncRequest *request, QueueKind kind) {
  request->done = true;
  // Nothing to deliver, so resolving it is the end of it, as io_uring never
  // queues a callbackless CQE for dispatch.  Once `done` is set the caller
  // may free it, so it must not be left on ready_ for a later Poll() to pop.
  if (!request->callback) {
    return;
  }
  SetKind(request, kind);
  ready_.PushBack(request);
}

// A request may only be armed once at a time, and "armed" lasts until its
// callback has run: resolved-but-undelivered still counts, since the
// callback is on its way.  aio.h's constraint 2 lets a request outlive the
// Aio it was armed on, so the request's own state cannot answer this -- a
// request left behind by a destroyed Aio can carry any `done` and any
// link.queued.  Ask this loop instead, the same way
// IoUringImpl::ClearStaleRawState() asks raw_in_flight_.
//
// The per-fd "Duplicate AsyncRead on fd" check catches re-arming on the *same*
// fd.  This is the cross-fd case, and the queued cases: a request Cancel()ed
// or failed at submit whose completion has not been delivered.  Re-arming
// one of those used to deliver two callbacks for one request -- Canceled,
// then the new result -- and if the caller freed the request in the first,
// as it may, the second read freed memory.
//
// Walks only when the request's own state says it might be ours, so a fresh
// or delivered request never walks anything.
void EpollImpl::CheckNotAlreadyInFlight(AsyncRequest *request) const {
  constexpr const char *kMessage = internal::kRequestStillInFlight;
  const QueueKind kind = Kind(request);
  if (IsPending(kind)) {
    ABSL_CHECK(!ListContains(pending_, request)) << kMessage;
  } else if (IsReady(kind)) {
    ABSL_CHECK(!ListContains(ready_, request)) << kMessage;
  }
  if (!request->done) {
    for (FdRegistration *reg : live_) {
      ABSL_CHECK(reg->read_req != request && reg->write_req != request)
          << kMessage;
    }
  }
}

// Non-blocking per call, without touching the fd's flags: RWF_NOWAIT makes
// this one read or write return EAGAIN instead of sleeping, and the file
// description -- which the caller may share with anything, a parent shell's
// stdout included -- stays as it was.
//
// Two kinds of fd cannot do that:
//   * A regular file or block device (unpollable): epoll cannot wait for
//     one, so its I/O runs to completion here, blocking, as aio.h documents.
//     A kernel without buffered-write RWF_NOWAIT (before 6.0) answers EINVAL
//     for one, which is recognized the same way.
//   * An fd whose file refuses RWF_NOWAIT with EOPNOTSUPP -- a tty, and on
//     kernels before 6.4 (pipes) and 6.5 (sockets) those too -- or a kernel
//     without the syscall at all (ENOSYS).  For those
//     alone the fd is made O_NONBLOCK, the way libuv's uv_poll_init() does,
//     since there is no other way to keep the loop thread from sleeping in
//     read(2) or write(2).  That flag stays set on the shared description.
//
// On Linux 5.9 and 5.10 RWF_NOWAIT is not used at all.  readv(2) lists it
// under BUGS: "Linux 5.9 and Linux 5.10 have a bug where preadv2() with the
// RWF_NOWAIT flag may return 0 even when not at end of file", and a spurious
// 0 reads as end of file.  Every fd there is treated like one that refused
// it: a regular file or block device is unpollable, anything else is made
// O_NONBLOCK.
ssize_t EpollImpl::DoRawIo(FdRegistration *reg, Direction direction, void *data,
                           size_t size) {
  const int fd = reg->fd;
  if (size == 0) {
    // An empty request goes where io_uring's does, as far as userspace can
    // tell.  readv()/writev() with an empty iovec check that the fd is open
    // for that direction (EBADF) and then return 0 without calling into the
    // file, so they never block: an inotify fd does not wait for an event,
    // and an eventfd does not refuse a write shorter than 8 bytes.
    //
    // io_uring does the same for a file with only the old ->read/->write
    // hooks (inotify), but hands the empty request to a file's
    // ->read_iter/->write_iter, which then applies its own rules.  Which
    // hooks a file has is not visible from userspace, so the backends can
    // differ there: io_uring's empty read of an eventfd fails with EINVAL,
    // where this one completes with 0.  See ZeroLengthReadOnEventfd.
    //
    // A socket's write reaches the protocol on io_uring: a datagram socket
    // sends an empty datagram, and a stream socket whose peer closed reports
    // EPIPE.  So a socket's empty write is a send(), with MSG_DONTWAIT
    // because an empty datagram can still wait for send-buffer room.
    struct iovec empty = {.iov_base = data, .iov_len = 0};
    if (direction == Direction::kRead) {
      return readv(fd, &empty, 1);
    }
    const ssize_t res = send(fd, data, 0, MSG_DONTWAIT);
    if (res >= 0 || errno != ENOTSOCK) {
      return res;
    }
    return writev(fd, &empty, 1);
  }
  struct iovec iov = {.iov_base = data, .iov_len = size};
  if (!reg->unpollable && !reg->nowait_unsupported && nowait_unreliable_) {
    struct stat st;
    if (fstat(fd, &st) == 0 && (S_ISREG(st.st_mode) || S_ISBLK(st.st_mode))) {
      reg->unpollable = true;
    } else {
      FallBackToNonBlocking(reg);
    }
  }
  if (!reg->unpollable && !reg->nowait_unsupported) {
    const ssize_t res = NowaitIo(direction == Direction::kRead, fd, &iov);
    if (res >= 0 ||
        (errno != EOPNOTSUPP && errno != ENOSYS && errno != EINVAL)) {
      return res;
    }
    if (errno == EINVAL) {
      struct stat st;
      if (fstat(fd, &st) != 0 ||
          !(S_ISREG(st.st_mode) || S_ISBLK(st.st_mode))) {
        errno = EINVAL;
        return -1;
      }
      reg->unpollable = true;
    } else {
      FallBackToNonBlocking(reg);
    }
  }
  return direction == Direction::kRead ? readv(fd, &iov, 1)
                                       : writev(fd, &iov, 1);
}

void EpollImpl::FallBackToNonBlocking(FdRegistration *reg) {
  const int fd = reg->fd;
  reg->nowait_unsupported = true;
  const int flags = fcntl(fd, F_GETFL);
  ABSL_PCHECK(flags != -1) << "fcntl(F_GETFL) failed for fd " << fd;
  if ((flags & O_NONBLOCK) == 0) {
    ABSL_PCHECK(fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0)
        << "fcntl(F_SETFL, O_NONBLOCK) failed for fd " << fd;
  }
}

void EpollImpl::AsyncRead(FileDescriptor fd, std::span<char> buffer,
                          AsyncRequest *request) {
  SubmitRaw(fd, buffer.data(), buffer.size(), request, Direction::kRead);
}

void EpollImpl::AsyncWrite(FileDescriptor fd, std::span<const char> buffer,
                           AsyncRequest *request) {
  SubmitRaw(fd, const_cast<char *>(buffer.data()), buffer.size(), request,
            Direction::kWrite);
}

void EpollImpl::SubmitRaw(FileDescriptor fd, void *data, size_t size,
                          AsyncRequest *request, Direction direction) {
  const bool is_read = direction == Direction::kRead;
  CheckNotAlreadyInFlight(request);
  // Cleared here rather than trusted: aio.h's constraint 2 lets a request
  // outlive the Aio it was armed on, so it can arrive still marked as queued
  // on a list belonging to an instance that is gone.  Same staleness, and the
  // same resolve-at-next-arm answer, as ClearStaleRawState().
  SetKind(request, QueueKind::kNone);
  request->done = false;
  if (fd < 0) {
    State(request).link.result = -EBADF;
    QueuePending(request, QueueKind::kPendingResult);
    return;
  }

  FdRegistration *reg = GetActiveRegistration(fd);
  if (reg != nullptr) {
    ABSL_CHECK(reg->events_fn == nullptr)
        << "Cannot mix OnEvents and AsyncRead/AsyncWrite on fd " << fd;
    if (is_read) {
      ABSL_CHECK(reg->read_req == nullptr)
          << "Duplicate AsyncRead on fd " << fd;
      ABSL_CHECK(reg->in_fn == nullptr)
          << "Cannot mix OnReadable and AsyncRead on fd " << fd;
    } else {
      ABSL_CHECK(reg->write_req == nullptr)
          << "Duplicate AsyncWrite on fd " << fd;
      ABSL_CHECK(reg->out_fn == nullptr)
          << "Cannot mix OnWritable and AsyncWrite on fd " << fd;
    }
    // Legacy handlers and raw requests are exclusive on an fd, in both
    // directions.  The specific checks above name the direction that
    // collides; this catches the cross-direction pairing.
    ABSL_CHECK(reg->async_only)
        << "Cannot mix legacy handlers and AsyncRead/AsyncWrite on fd " << fd;
  } else {
    // From the pool, or the heap if it has run dry.  The fallback keeps an
    // unusually large fd count working rather than turning it into a CHECK;
    // on a realtime thread the malloc hook is what objects, which is the
    // right place for that to surface.
    reg = raw_free_list_.Pop();
    if (reg == nullptr) {
      reg = new FdRegistration();
      reg->pooled = true;
      all_.Push(reg);
    }
    reg->fd = fd;
    reg->async_only = true;
    live_.Insert(reg);
  }
  reg->slot(direction) = request;
  State(request).epoll.ptr = data;
  State(request).epoll.size = size;
  // No I/O and no epoll_ctl() here: the first attempt is the harvest's, in
  // submission order with every other queued outcome -- see StartRaw().
  State(request).link.result = fd;
  QueuePending(request, is_read ? QueueKind::kPendingSubmitRead
                                : QueueKind::kPendingSubmitWrite);
}

bool EpollImpl::StartRaw(AsyncRequest *request, Direction direction) {
  FdRegistration *reg = GetActiveRegistration(State(request).link.result);
  ABSL_CHECK(reg != nullptr && reg->slot(direction) == request);
  if (TryRawIo(reg, direction)) {
    return true;
  }
  // EAGAIN: wait for readiness.
  if (!UpdateEpoll(reg)) {
    // The kernel refused to watch it.  Deliver the errno as the error
    // completion aio.h promises instead of aborting.
    reg->slot(direction) = nullptr;
    State(request).link.result = -reg->registration_errno;
    reg->registration_errno = 0;
    QueueReady(request, QueueKind::kReadyResult);
    SettleRawRegistration(reg);
    return true;
  }
  if (reg->unpollable) {
    // A regular file, found by the ADD above: nothing to wait for, so the
    // I/O runs now, blocking -- see DoRawIo().
    ABSL_CHECK(TryRawIo(reg, direction));
    return true;
  }
  return false;
}

bool EpollImpl::TryRawIo(FdRegistration *reg, Direction direction) {
  AsyncRequest *req = reg->slot(direction);
  const ssize_t res =
      DoRawIo(reg, direction, State(req).epoll.ptr, State(req).epoll.size);
  if (res < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
    return false;
  }
  // Before any epoll_ctl below can clobber it.
  const int io_errno = errno;
  reg->slot(direction) = nullptr;
  State(req).link.result = res >= 0 ? static_cast<int32_t>(res) : -io_errno;
  QueueReady(req, QueueKind::kReadyResult);
  SettleRawRegistration(reg);
  return true;
}

bool EpollImpl::ServiceRaw(FdRegistration *reg, uint32_t got_events) {
  // A raw request observes a terminal condition by running: a pending read
  // only sees EOF from its own read(), and closing an empty pipe's write end
  // raises EPOLLHUP with no EPOLLIN.  So a hangup or error is readiness for
  // whichever requests are waiting.
  //
  // That assumes the fd's socket error queue is not in use.  An unread
  // IP_RECVERR or SO_TIMESTAMPING entry keeps EPOLLERR set while read()
  // still returns EAGAIN, so every Poll() would wake for it and resolve
  // nothing.  Only recvmsg(MSG_ERRQUEUE) clears it, which no raw request
  // issues.
  const bool terminal = (got_events & (EPOLLERR | EPOLLHUP)) != 0;
  bool resolved = false;
  if (Armed(reg, Direction::kRead) != nullptr &&
      (terminal || (got_events & (EPOLLIN | EPOLLPRI)) != 0)) {
    resolved |= TryRawIo(reg, Direction::kRead);
  }
  // The read may have retired the registration (Detach() tombstones fd).
  if (reg->fd != -1 && Armed(reg, Direction::kWrite) != nullptr &&
      (terminal || (got_events & EPOLLOUT) != 0)) {
    resolved |= TryRawIo(reg, Direction::kWrite);
  }
  return resolved;
}

void EpollImpl::SettleRawRegistration(FdRegistration *reg) {
  if (reg->idle()) {
    if (reg->registered) {
      // Strict, as DeleteFd() is.  EBADF, ENOENT or EPERM here means the
      // caller closed the fd with a request armed on it, which the contract
      // forbids: the kernel entry can outlive the close (a dup() or fork()
      // keeps the file open) and would name this registration after it went
      // back to the pool.
      ABSL_PCHECK(epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, reg->fd, nullptr) == 0)
          << "epoll_ctl DEL failed for fd " << reg->fd
          << "; was it closed with a request armed on it?";
    }
    ReleaseRegistration(reg);
    return;
  }
  // Only ever narrows or MODs what is already registered -- a request is
  // added to epoll by StartRaw() alone -- so the refusal UpdateEpoll()
  // reports cannot happen here.
  ABSL_CHECK(UpdateEpoll(reg));
}

void EpollImpl::Cancel(AsyncRequest *request) {
  // Resolved already -- done, or its outcome decided and waiting on pending_.
  // The completion stands, like io_uring's cancel against an
  // already-posted CQE.  Tested rather than removed and re-queued, which
  // would reorder completions a caller can observe.
  if (request->done) {
    return;
  }
  // Armed on this loop means holding a registration's slot, from submit
  // until it resolves.  The loop is asked rather than the request's own
  // link.queued: a request left over from an Aio destroyed underneath it
  // (aio.h's constraint 2) can carry any value there, naming a list that is
  // gone.  Not found is not in flight here -- never armed, left over, or
  // already resolved with its outcome on pending_ -- and, as io_uring's
  // cancel of a user_data its ring does not hold, a no-op.
  FdRegistration *found = nullptr;
  Direction direction = Direction::kRead;
  for (FdRegistration *reg : live_) {
    if (reg->read_req == request) {
      found = reg;
      direction = Direction::kRead;
      break;
    }
    if (reg->write_req == request) {
      found = reg;
      direction = Direction::kWrite;
      break;
    }
  }
  if (found == nullptr) {
    return;
  }
  // Not attempted yet: it leaves pending_ and is canceled, as an io_uring
  // cancel that reaches an SQE before it runs.
  if (IsPendingSubmit(Kind(request))) {
    pending_.Remove(request);
    SetKind(request, QueueKind::kNone);
  }

  // The request must not also complete normally, so it leaves its slot now.
  // The fd stays claimed until the Canceled completion resolves, and the
  // registration stays alive to hold that claim -- see
  // FdRegistration::cancel_claims_read.  The registration itself is what the
  // resolve step releases, not the fd number, which may belong to someone
  // else by then.
  found->slot(direction) = nullptr;
  ++found->cancel_claims(direction);
  SettleRawRegistration(found);
  State(request).epoll.ptr = found;
  QueuePending(request, direction == Direction::kRead
                            ? QueueKind::kPendingCancelRead
                            : QueueKind::kPendingCancelWrite);
}

void EpollImpl::BeforeWait(std::function<void()> function) {
  ABSL_CHECK(!in_before_wait_)
      << ": BeforeWait() may not be called from a before-wait function";
  before_wait_functions_.push_back(std::move(function));
}

void EpollImpl::OnReadable(FileDescriptor fd, std::function<void()> callback) {
  FdRegistration *reg = GetOrCreateLegacyRegistration(fd);
  ABSL_CHECK(!reg->events_fn)
      << "Cannot mix OnEvents and OnReadable for fd " << fd;
  ABSL_CHECK(reg->read_req == nullptr && reg->cancel_claims_read == 0)
      << "Cannot mix OnReadable and AsyncRead on fd " << fd;
  ABSL_CHECK(reg->write_req == nullptr && reg->cancel_claims_write == 0)
      << "Cannot mix AsyncWrite and OnReadable on fd " << fd;
  // Unconditional, as EPoll and IoUringImpl have it: a null `callback` is
  // no exception, since it would silently clear the handler while leaving
  // the events subscribed -- which Poll()'s dispatch then dies on.
  ABSL_CHECK(!reg->in_fn) << "Duplicate in functions for " << fd;
  reg->in_fn = std::move(callback);

  reg->events |= kInEvents;
  UpdateEpoll(reg);
}

void EpollImpl::OnError(FileDescriptor fd, std::function<void()> callback) {
  FdRegistration *reg = GetOrCreateLegacyRegistration(fd);
  ABSL_CHECK(!reg->events_fn)
      << "Cannot mix OnEvents and OnError for fd " << fd;
  ABSL_CHECK(reg->idle())
      << "Cannot mix AsyncRead/AsyncWrite and OnError on fd " << fd;
  // Unconditional -- see OnReadable().
  ABSL_CHECK(!reg->err_fn) << "Duplicate error functions for " << fd;
  reg->err_fn = std::move(callback);

  reg->events |= kErrorEvents;
  UpdateEpoll(reg);
}

void EpollImpl::OnWritable(FileDescriptor fd, std::function<void()> callback) {
  FdRegistration *reg = GetOrCreateLegacyRegistration(fd);
  ABSL_CHECK(!reg->events_fn)
      << "Cannot mix OnEvents and OnWritable for fd " << fd;
  ABSL_CHECK(reg->write_req == nullptr && reg->cancel_claims_write == 0)
      << "Cannot mix OnWritable and AsyncWrite on fd " << fd;
  ABSL_CHECK(reg->read_req == nullptr && reg->cancel_claims_read == 0)
      << "Cannot mix AsyncRead and OnWritable on fd " << fd;
  // Unconditional -- see OnReadable().
  ABSL_CHECK(!reg->out_fn) << "Duplicate out functions for " << fd;
  reg->out_fn = std::move(callback);

  reg->events |= kOutEvents;
  UpdateEpoll(reg);
}

void EpollImpl::OnEvents(FileDescriptor fd,
                         std::function<void(uint32_t)> callback) {
  FdRegistration *reg = GetOrCreateLegacyRegistration(fd);
  ABSL_CHECK(reg->idle())
      << "Cannot mix OnEvents and AsyncRead/AsyncWrite on fd " << fd;
  ABSL_CHECK(!reg->in_fn && !reg->out_fn && !reg->err_fn)
      << "May not replace OnEvents handlers for fd " << fd;
  ABSL_CHECK(!reg->events_fn)
      << "May not replace OnEvents handlers for fd " << fd;
  reg->events_fn = std::move(callback);
}

void EpollImpl::DeleteFd(FileDescriptor fd) {
  auto *reg = GetActiveRegistration(fd);
  ABSL_CHECK(reg != nullptr) << "fd " << fd << " not found";
  // Async-only registrations are not the caller's to delete.  DeleteFd()
  // undoes an On*() registration; an fd carrying only AsyncRead/AsyncWrite
  // has none, and releasing it here would drop the requests without ever
  // completing them -- leaving req.done false forever, which aio.h's
  // constraint 2 says the caller may then never free or reuse.  A silent
  // hang, where io_uring gives a loud "fd not found" for the same sequence
  // because its legacy state lives in a separate table.
  //
  // Raw requests are retired by completing or Cancel()ing them, which is
  // what a caller wanting this fd gone should do.
  ABSL_CHECK(!reg->async_only) << "fd " << fd << " not found";

  if (reg->registered) {
    // Strict, like EPoll::DoEpollCtl() and IoUringImpl::DeleteFd().  ENOENT
    // means the fd isn't ours any more: it was closed, which silently took
    // it out of the interest list, and the number may already belong to
    // someone else.  Carrying on would hand this registration back to the
    // pool while an epoll entry for a dup'd descriptor could still point at
    // it.  ForgetClosedFd() is the API for an fd that is already closed.
    ABSL_PCHECK(epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, fd, nullptr) == 0)
        << "epoll_ctl DEL failed for fd " << fd;
  }

  ReleaseRegistration(reg);
}

void EpollImpl::ForgetClosedFd(FileDescriptor fd) {
  auto *reg = GetActiveRegistration(fd);
  ABSL_CHECK(reg != nullptr) << "fd " << fd << " not found";
  // Not the caller's to forget -- see DeleteFd().
  ABSL_CHECK(!reg->async_only) << "fd " << fd << " not found";

  ReleaseRegistration(reg);
}

void EpollImpl::EnableWritable(FileDescriptor fd) {
  auto *reg = GetActiveRegistration(fd);
  ABSL_CHECK(reg != nullptr) << "fd " << fd << " not found";
  // A raw-only registration is not an fd the caller registered a handler on,
  // so this is the same misuse io_uring reports as "fd not found" -- it looks
  // in a table that only holds legacy state, while this one holds both.  Left
  // through, it subscribes to EPOLLOUT with no out_fn to dispatch and crashes
  // somewhere else, or evaporates when the async registration auto-retires.
  ABSL_CHECK(!reg->async_only) << "fd " << fd << " not found";
  ABSL_CHECK(!reg->events_fn)
      << "EnableWritable is only for fds registered using OnWritable, not "
         "OnEvents";

  uint32_t new_events = reg->events | kOutEvents;
  if (reg->events != new_events) {
    reg->events = new_events;
    UpdateEpoll(reg);
  }
}

void EpollImpl::DisableWritable(FileDescriptor fd) {
  auto *reg = GetActiveRegistration(fd);
  ABSL_CHECK(reg != nullptr) << "fd " << fd << " not found";
  // Same as EnableWritable() -- see there.
  ABSL_CHECK(!reg->async_only) << "fd " << fd << " not found";
  ABSL_CHECK(!reg->events_fn)
      << "DisableWritable is only for fds registered using OnWritable, not "
         "OnEvents";

  uint32_t new_events = reg->events & ~kOutEvents;
  if (reg->events != new_events) {
    reg->events = new_events;
    UpdateEpoll(reg);
  }
}

void EpollImpl::SetEvents(FileDescriptor fd, uint32_t events) {
  auto *reg = GetActiveRegistration(fd);
  ABSL_CHECK(reg != nullptr) << "fd " << fd << " not found";
  ABSL_CHECK(reg->events_fn)
      << "SetEvents is only for fds registered using OnEvents";

  if (reg->events != events) {
    reg->events = events;
    UpdateEpoll(reg);
  }
}

void EpollImpl::RegisterThreadSignalReceiver(
    ipc_lib::ThreadSignalReceiver *receiver, std::function<void()> callback) {
  ABSL_CHECK(receiver_ == nullptr)
      << "Duplicate ThreadSignalReceiver registration: only one receiver "
         "may be active at a time (see Aio::RegisterThreadSignalReceiver)";
  receiver_ = receiver;
  OnReadable(receiver->fd(), [receiver, callback = std::move(callback)]() {
    receiver->ConsumeWakeup();
    if (callback) callback();
  });
}

void EpollImpl::UnregisterThreadSignalReceiver(
    ipc_lib::ThreadSignalReceiver *receiver) {
  ABSL_CHECK(receiver_ == receiver) << "ThreadSignalReceiver not found";
  receiver_ = nullptr;
  DeleteFd(receiver->fd());
}

void EpollImpl::ConsumeThreadSignalReceiver(
    ipc_lib::ThreadSignalReceiver *receiver) {
  // Nothing backend-specific here: the receiver's fd is an ordinary
  // level-triggered registration, so draining it is the receiver's own job.
  receiver->ConsumeWakeup();
}

EpollTimerState::~EpollTimerState() {
  Cancel(true);
  if (timer_fd) {
    impl_->DeleteFd(timer_fd->fd());
  }
  --impl_->active_timer_count_;
}

void EpollTimerState::Initialize() {
  timer_fd = std::make_unique<TimerFD>();
  request.done = true;

  impl_->OnReadable(timer_fd->fd(), [this]() {
    // EAGAIN is normal: readable when epoll reported it, but Cancel()'s
    // timerfd_settime(0) has since zeroed ctx->ticks (fs/timerfd.c).
    uint64_t buf;
    ssize_t result = read(timer_fd->fd(), &buf, sizeof(buf));
    if (result == -1 && errno == EAGAIN) {
      return;
    }
    ABSL_PCHECK(result == static_cast<ssize_t>(sizeof(buf)));

    // One-shot: nothing is armed anymore.  Resolve before dispatching, so
    // nothing here touches `this` after the user callback runs -- a
    // callback is allowed to destroy its own timer.
    request.done = true;
    CompletionCallback callback = user_callback;
    if (callback == nullptr) {
      return;
    }
    Completion completion;
    // nullptr, as documented on Timer::Schedule(): the caller supplies no
    // user_data, so none is delivered.
    completion.user_data = nullptr;
    completion.status = aos::Ok();
    completion.result = 0;
    void *const callback_context = user_context;
    // Last use of `this`.
    callback(completion, callback_context);
  });
}

void EpollTimerState::Schedule(aos::monotonic_clock::time_point deadline,
                               CompletionCallback callback, void *context) {
  ABSL_CHECK_GE(deadline, aos::monotonic_clock::epoch());
  // No Cancel() first: that is a timerfd_settime(2) of its own, and aio.h
  // promises Schedule() is one syscall.  The settime below replaces whatever
  // was armed, and it also zeroes an expiration that fired but has not been
  // read yet (fs/timerfd.c resets ctx->ticks), so the old deadline cannot
  // leak through -- the readable handler just sees EAGAIN.

  this->deadline = deadline;
  this->user_callback = callback;
  this->user_context = context;
  this->request.done = false;

  // it_interval stays zero -- one-shot; see Aio::Timer::Schedule().
  struct itimerspec its;
  std::memset(&its, 0, sizeof(its));
  its.it_value = AbsoluteTimerfdValue(deadline);

  int ret = timerfd_settime(timer_fd->fd(), TFD_TIMER_ABSTIME, &its, nullptr);
  ABSL_PCHECK(ret == 0) << "timerfd_settime failed: " << aos_strerror(errno);
}

void EpollTimerState::Cancel(bool /*reap*/) {
  if (timer_fd) {
    struct itimerspec its;
    std::memset(&its, 0, sizeof(its));
    timerfd_settime(timer_fd->fd(), 0, &its, nullptr);
  }
  request.done = true;
  user_callback = nullptr;
}

bool EpollImpl::UpdateEpoll(FdRegistration *reg) {
  const FileDescriptor fd = reg->fd;

  const bool read_armed = Armed(reg, Direction::kRead) != nullptr;
  const bool write_armed = Armed(reg, Direction::kWrite) != nullptr;
  uint32_t desired_events = 0;
  if (read_armed) desired_events |= EPOLLIN;
  if (write_armed) desired_events |= EPOLLOUT;

  if (reg->events & kIn) desired_events |= EPOLLIN;
  if (reg->events & kPri) desired_events |= EPOLLPRI;
  if (reg->events & kOut) desired_events |= EPOLLOUT;
  if (reg->events & kErr) desired_events |= EPOLLERR;

  // Registered-vs-not follows what the caller asked for, not the
  // translated mask, matching EPoll::DoEpollCtl(): a SetEvents() mask of
  // only untranslated bits (e.g. a bare EPOLLHUP) keeps the fd registered
  // with an empty event set, which still delivers EPOLLERR/EPOLLHUP.
  if (!read_armed && !write_armed && reg->events == 0) {
    if (reg->registered) {
      // Strict, like EPoll::DoEpollCtl(); see DeleteFd() and
      // SettleRawRegistration().
      ABSL_PCHECK(epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, fd, nullptr) == 0)
          << "epoll_ctl DEL failed for fd " << fd;
      reg->registered = false;
      reg->epoll_events = 0;
    }
    return true;
  }

  struct epoll_event ev;
  std::memset(&ev, 0, sizeof(ev));
  ev.events = desired_events;
  ev.data.ptr = reg;
  if (reg->registered) {
    if (reg->epoll_events == desired_events) {
      return true;
    }
    ABSL_PCHECK(epoll_ctl(epoll_fd_, EPOLL_CTL_MOD, fd, &ev) == 0)
        << "epoll_ctl MOD failed for fd " << fd;
    reg->epoll_events = desired_events;
    return true;
  }

  const int ret = epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, fd, &ev);
  if (ret != 0 && reg->async_only) {
    if (errno == EPERM) {
      // EPERM means no wait queue -- a regular file, always ready.  Only
      // tolerated for raw registrations, whose I/O StartRaw() then does at
      // once; a legacy registration would silently never fire, so that
      // still dies below.
      reg->unpollable = true;
      return true;
    }
    // The kernel refused to poll the fd.  aio.h documents an operational
    // failure as an error completion carrying the errno, so the request gets
    // one rather than the process dying.  (A closed fd never gets this far:
    // its first attempt already answered EBADF.)
    //
    // A legacy registration has no request to complete, so it still dies
    // below -- which is what EPoll has always done.
    reg->registration_errno = errno;
    return false;
  }
  ABSL_PCHECK(ret == 0) << "epoll_ctl ADD failed for fd " << fd;
  reg->registered = true;
  reg->epoll_events = desired_events;
  return true;
}

// No availability probe and no silent fallback: --aio_backend=io_uring on a
// kernel that can't deliver it fails loudly in IoUringImpl's constructor
// rather than quietly degrading to a backend that wasn't asked for.  An
// unrecognized name is fatal for the same reason.
Aio::Aio() {
  const std::string backend = ::absl::GetFlag(FLAGS_aio_backend);
  if (backend == "io_uring") {
    impl_ = std::make_unique<IoUringImpl>();
  } else if (backend == "epoll") {
    impl_ = std::make_unique<EpollImpl>();
  } else {
    ABSL_LOG(FATAL) << "Unknown --aio_backend \"" << backend
                    << "\"; this build supports \"io_uring\" and \"epoll\".";
  }
}
}  // namespace aos
