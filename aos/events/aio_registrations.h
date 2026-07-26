#ifndef AOS_EVENTS_AIO_REGISTRATIONS_H_
#define AOS_EVENTS_AIO_REGISTRATIONS_H_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include "absl/log/absl_check.h"

#include "aos/events/aio_internal.h"
#include "aos/events/file_descriptor.h"
#include "aos/events/intrusive_rb_tree.h"

namespace aos::internal {

// AOS's fd-readiness event encoding, shared by every backend and by the
// public SetEvents()/OnEvents() contract.  Numerically identical to the low
// 4 bits of the real epoll event flags (the encoding documented on
// Aio::OnEvents()).
inline constexpr uint32_t kIn = 0x01;   // EPOLLIN
inline constexpr uint32_t kPri = 0x02;  // EPOLLPRI
inline constexpr uint32_t kOut = 0x04;  // EPOLLOUT
inline constexpr uint32_t kErr = 0x08;  // EPOLLERR

inline constexpr uint32_t kInEvents = kIn | kPri;
inline constexpr uint32_t kOutEvents = kOut;
inline constexpr uint32_t kErrorEvents = kErr;

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
// whose reader went away: epoll reports that as EPOLLERR alone, and the
// write that would have seen EPIPE never ran.
//
// A hangup that arrives alongside another bit is left alone; that bit's
// handler already runs.  An fd subscribed to neither direction keeps its
// error bit, and the error-handler CHECK still fires for it.
inline uint32_t RouteHangupOrError(uint32_t events, bool hangup,
                                   uint32_t subscribed, bool has_err_fn) {
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

// What one fd is registered for, and the table that owns them.
//
// Every readiness-driven backend -- epoll, kqueue, IOCP -- keeps exactly
// this state and recycles it exactly this way.  Only the syscall that arms
// and disarms the kernel differs, and that stays in the backend.  Sharing
// the bookkeeping is mostly about the parts that are subtle rather than the
// parts that are long: the deferred teardown below took an ASAN run to find
// once already, and there is no reason for each backend to rediscover it.
struct FdRegistration {
  FileDescriptor fd = kInvalidFd;
  // The caller's outstanding AsyncRead()/AsyncWrite(), if any, from submit
  // until they resolve -- including while one still waits on the pending
  // queue for its first attempt, so the per-fd CHECKs see it.  Only an armed
  // one is waiting on the kernel; see ReadinessBackend::Armed().  The harvest
  // does their I/O directly, so nothing is parked in in_fn/out_fn for them:
  // those only ever hold the caller's legacy handlers.
  AsyncRequest *read_req = nullptr;
  AsyncRequest *write_req = nullptr;

  // The legacy readiness handlers.
  std::function<void()> in_fn = nullptr;
  std::function<void()> out_fn = nullptr;
  std::function<void()> err_fn = nullptr;
  std::function<void(uint32_t)> events_fn = nullptr;
  // The mask the caller subscribed to through On*()/SetEvents().
  uint32_t events = 0;

  // What the kernel currently has, so a backend can skip a no-op syscall.
  bool registered = false;
  uint32_t epoll_events = 0;

  // True while this registration exists only to carry AsyncRead/AsyncWrite
  // requests; retired back to the pool once the last one is resolved (see
  // ShouldRetire()).  Legacy registrations live until
  // DeleteFd()/ForgetClosedFd().  An fd is one or the other, never both.
  bool async_only = false;
  // Which free list this registration goes back to.  Raw registrations come
  // from the pool, which is what keeps arming a request allocation-free.
  // Legacy ones -- every On*() fd, every timer fd, the thread-signal
  // receiver, the wakeup -- come from their own list, so they cannot use up
  // the slots the realtime submit path relies on.
  bool pooled = false;
  // The loop's own wakeup: drained by the harvest as internal work, which
  // neither spends Poll()'s one-callback budget nor counts as the caller's
  // registration.  See ReadinessBackend::OnInternalReadable().
  bool internal = false;
  // Set when the kernel refused to poll this fd because it is always ready
  // (epoll_ctl(ADD) answering EPERM for a regular file).  Its I/O is then
  // done at once, blocking, instead of waiting for an event that cannot
  // arrive.
  bool unpollable = false;
  // Set once the backend's PrepareRawFd() has run for this registration,
  // before its first raw I/O.
  bool raw_prepared = false;
  // Linux: set once the fd refused RWF_NOWAIT and was made O_NONBLOCK
  // instead -- see EpollImpl::DoRawIo().
  bool nowait_unsupported = false;
  // Positive errno from a registration the kernel refused to add.
  // UpdateRegistration() returns false and leaves it here; the harvest turns
  // it into the error completion aio.h promises, rather than taking the
  // process down.
  int registration_errno = 0;
  // Canceled requests whose Canceled completion has not been resolved yet.
  // aio.h documents Cancel() as asynchronous, so the fd stays claimed until
  // then, which is the point io_uring releases its claim too (its terminal
  // CQE is drained by the next Poll(), and so is this).  Counts, not flags:
  // two requests can be canceled on one fd before either resolves, and
  // resolving the first must not free the fd for the second.
  int cancel_claims_read = 0;
  int cancel_claims_write = 0;

  // Link for the free list and the retired list.  One link covers both: a
  // registration is only ever on one of them, moving free -> live ->
  // retired -> free.
  FdRegistration *next_free = nullptr;
  // Link for the list that owns every allocation for the table's lifetime,
  // so the tree and the free/retired stacks can all be non-owning.
  FdRegistration *next_all = nullptr;

  // Links for the live tree, keyed on fd.  Owned by FdRegistrationTable;
  // nothing else may read or write them while the registration is live.
  FdRegistration *tree_left = nullptr;
  FdRegistration *tree_right = nullptr;
  FdRegistration *tree_parent = nullptr;
  bool tree_red = false;

  enum class Direction { kRead, kWrite };
  AsyncRequest *&slot(Direction direction) {
    return direction == Direction::kRead ? read_req : write_req;
  }
  int &cancel_claims(Direction direction) {
    return direction == Direction::kRead ? cancel_claims_read
                                         : cancel_claims_write;
  }
  // No request in flight and no cancel waiting to resolve.
  bool idle() const {
    return read_req == nullptr && write_req == nullptr &&
           cancel_claims_read == 0 && cancel_claims_write == 0;
  }

  // Drops everything tying this registration to an fd -- but deliberately
  // not the callbacks.  Release() can run underneath one of them (a
  // callback which DeleteFd()s its own fd), and destroying a std::function
  // there would free the frame that is executing.  Reset() does that half,
  // once it is safe.
  void Detach() {
    fd = kInvalidFd;
    read_req = nullptr;
    write_req = nullptr;
    cancel_claims_read = 0;
    cancel_claims_write = 0;
    events = 0;
    registered = false;
    epoll_events = 0;
  }

  // Drops what Detach() left behind, returning this to pool state.  Only
  // safe with no dispatch in flight; see ScrubRetired().
  void Reset() {
    in_fn = nullptr;
    out_fn = nullptr;
    err_fn = nullptr;
    events_fn = nullptr;
    async_only = false;
    unpollable = false;
    raw_prepared = false;
    nowait_unsupported = false;
    internal = false;
  }
};

struct FreeLinkTraits {
  static FdRegistration *&next(FdRegistration *reg) { return reg->next_free; }
};

struct AllLinkTraits {
  static FdRegistration *&next(FdRegistration *reg) { return reg->next_all; }
};

// Orders the live registrations by fd.  Compare() lets GetActive() look one
// up from a bare fd without building a registration to compare against.
struct LiveTreeTraits {
  static FdRegistration *&left(FdRegistration *reg) { return reg->tree_left; }
  static FdRegistration *&right(FdRegistration *reg) { return reg->tree_right; }
  static FdRegistration *&parent(FdRegistration *reg) {
    return reg->tree_parent;
  }
  static bool &red(FdRegistration *reg) { return reg->tree_red; }
  static bool Less(FdRegistration *a, FdRegistration *b) {
    return a->fd < b->fd;
  }
  static int Compare(const FdRegistration *reg, FileDescriptor fd) {
    if (reg->fd < fd) return -1;
    if (fd < reg->fd) return 1;
    return 0;
  }
};

class FdRegistrationTable {
 public:
  // pool_size registrations are allocated up front so the hot paths do not.
  // Running dry is not fatal -- Allocate() falls back to new -- but that
  // allocation is what the realtime malloc hook is there to catch, so the
  // pool is sized to make it not happen.
  explicit FdRegistrationTable(size_t pool_size) {
    for (size_t i = 0; i < pool_size; ++i) {
      auto *reg = new FdRegistration();
      reg->pooled = true;
      all_.Push(reg);
      free_list_.Push(reg);
    }
  }

  ~FdRegistrationTable() {
    // Cleared here rather than left to the members' destructors so that the
    // std::functions a parked registration still holds are destroyed while
    // the owning backend is still intact -- a capture's destructor can
    // reenter it.
    all_.Clear();
  }

  FdRegistrationTable(const FdRegistrationTable &) = delete;
  FdRegistrationTable &operator=(const FdRegistrationTable &) = delete;

  // Live registrations, in a tree keyed on fd.
  FdRegistration *GetActive(FileDescriptor fd) const { return live_.Find(fd); }

  // For a legacy On*() registration, which outlives any single request and
  // so comes from its own list rather than the pool.  May return a raw
  // registration; each On*() names the collision it then hits.
  FdRegistration *GetOrCreateLegacy(FileDescriptor fd) {
    if (auto *reg = GetActive(fd)) {
      return reg;
    }
    FdRegistration *reg = legacy_free_list_.Pop();
    if (reg == nullptr) {
      // On*() is CheckNotRealtime(), so allocating here is fine.
      reg = new FdRegistration();
      all_.Push(reg);
    }
    return Insert(reg, fd, false);
  }

  // For a raw AsyncRead()/AsyncWrite(), which is retired again as soon as
  // its last request completes.
  FdRegistration *GetOrCreateAsync(FileDescriptor fd) {
    if (auto *reg = GetActive(fd)) {
      return reg;
    }
    return Insert(Allocate(), fd, true);
  }

  // Whether reg exists only for raw requests and nothing on it is left to
  // resolve, so the caller should deregister it from the kernel and
  // Release() it.
  static bool ShouldRetire(const FdRegistration *reg) {
    return reg->async_only && reg->idle();
  }

  // Detaches reg from its fd and destroys what it holds -- now, unless it is
  // a legacy registration and a dispatch is in flight.  Then one of reg's
  // std::functions can be the very function calling this (a callback that
  // DeleteFd()s its own fd), and the dispatch loop may still re-read reg->fd
  // afterwards to notice the deletion, so it is parked and ScrubRetired()
  // finishes the job.
  //
  // A raw registration holds no std::function, so it goes straight back to
  // the pool: a completion callback that re-arms its own fd then reuses the
  // slot the completion just gave up, instead of needing a second one until
  // the Poll() ends.  Detach() already tombstoned fd, which is all
  // ServiceRaw() re-reads, and nothing can take the slot back out before
  // then -- only a callback arms.
  void Release(FdRegistration *reg) {
    ABSL_CHECK(live_.Find(reg->fd) == reg);
    live_.Remove(reg);
    reg->Detach();
    if (dispatch_depth_ > 0 && !reg->async_only) {
      retired_.Push(reg);
      return;
    }
    // Nothing can be executing out of it: destroy now, so a legacy
    // registration's captures die at DeleteFd() as EPoll's always did.
    reg->Reset();
    (reg->pooled ? free_list_ : legacy_free_list_).Push(reg);
  }

  // Moves everything parked by Release() back to the pool, destroying the
  // callbacks it was still holding.  Only legal with no dispatch in flight.
  //
  // No trimming and no free(): all_ owns every registration for the table's
  // lifetime, so the free list only ever grows to the peak concurrent fd
  // count and hands the same objects back out.  The sorted-vector version
  // had to give surplus back with delete, which put a free() inside the
  // caller's realtime section.
  //
  // Reset() destroys the registration's std::functions, which are only ever
  // the caller's own handlers -- raw registrations never park here.  Those
  // are only ever released by DeleteFd() or ForgetClosedFd(), both
  // aos::CheckNotRealtime(), and ReadinessBackend::Poll() scrubs at the end
  // of the dispatch that released them.
  void ScrubRetired() {
    ABSL_CHECK_EQ(dispatch_depth_, 0);
    while (FdRegistration *reg = retired_.Pop()) {
      reg->Reset();
      (reg->pooled ? free_list_ : legacy_free_list_).Push(reg);
    }
  }

  // Non-zero while the owner is running callbacks.  Release() consults it
  // to decide whether a callback frame may be live.
  int dispatch_depth() const { return dispatch_depth_; }

  // Raises dispatch_depth() for as long as it is alive.
  class DispatchGuard {
   public:
    explicit DispatchGuard(FdRegistrationTable *table) : table_(table) {
      ++table_->dispatch_depth_;
    }
    ~DispatchGuard() { --table_->dispatch_depth_; }
    DispatchGuard(const DispatchGuard &) = delete;
    DispatchGuard &operator=(const DispatchGuard &) = delete;

   private:
    FdRegistrationTable *table_;
  };

  // Iteration over the live registrations, for a backend's destructor
  // checks and for rebuilding them all after a fork.
  auto begin() const { return live_.begin(); }
  auto end() const { return live_.end(); }

 private:
  // A registration from the pool, or a fresh one if it has run dry.  The
  // fallback keeps an unusually large fd count working rather than turning
  // it into a CHECK; on a realtime thread the malloc hook is what objects,
  // which is the right place for that to surface.
  FdRegistration *Allocate() {
    if (FdRegistration *reg = free_list_.Pop()) {
      return reg;
    }
    auto *reg = new FdRegistration();
    reg->pooled = true;
    all_.Push(reg);
    return reg;
  }

  FdRegistration *Insert(FdRegistration *reg, FileDescriptor fd,
                         bool async_only) {
    reg->fd = fd;
    reg->async_only = async_only;
    live_.Insert(reg);
    return reg;
  }

  // Owns every registration for this table's lifetime, so the tree and the
  // two stacks below can all be non-owning -- moving a registration between
  // them is then pointer assignment, which is what makes Release() safe on
  // the dispatch path under realtime.
  OwningIntrusiveStack<FdRegistration, AllLinkTraits> all_;
  // Live registrations, keyed on fd.  Intrusive, so inserting one allocates
  // nothing -- which the sorted vector this replaced could not promise, since
  // it held the wakeup, every legacy fd and every timer fd and so outgrew its
  // reserve on the submit path.
  IntrusiveRbTree<FdRegistration, LiveTreeTraits> live_;
  // Raw registrations only; see FdRegistration::pooled.
  IntrusiveStack<FdRegistration, FreeLinkTraits> free_list_;
  // Recycled legacy registrations, grown on demand.
  IntrusiveStack<FdRegistration, FreeLinkTraits> legacy_free_list_;
  IntrusiveStack<FdRegistration, FreeLinkTraits> retired_;
  int dispatch_depth_ = 0;
};

}  // namespace aos::internal

#endif  // AOS_EVENTS_AIO_REGISTRATIONS_H_
