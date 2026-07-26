#include "aos/events/aio_readiness_backend.h"

#include <sys/socket.h>
#include <sys/uio.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <type_traits>
#include <utility>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/events/aio_state.h"
#include "aos/events/aio_unix.h"
#include "aos/realtime.h"

namespace aos::internal {

ReadinessBackend::ReadinessBackend(size_t pool_size, std::string backend_name)
    : registrations_(pool_size), backend_name_(std::move(backend_name)) {
  RegisterForkCounters();
  last_fork_count_ = ForkCount();
}

ReadinessBackend::~ReadinessBackend() = default;

void ReadinessBackend::ReArmAllRegistrations() {
  for (const auto &reg : registrations_) {
    if (reg->registered) {
      reg->registered = false;
      UpdateRegistrationOrDie(reg);
    }
  }
}

void ReadinessBackend::Run() {
  run_ = true;
  // Blocking polls while running, non-blocking once Quit() lands -- see
  // IoUringImpl::Run(), which this mirrors, and Aio::Run() for the contract.
  while (true) {
    if (!Poll(should_run())) {
      // Nothing was processed.  Once a shutdown has been requested, that means
      // the queue is drained and we are done.  While still running, it was a
      // blocking Poll() that only did internal work, such as the wakeup read,
      // so we go back to waiting.  (EINTR is absorbed inside Poll().)
      if (!should_run()) {
        break;
      }
    }
  }
  // run_ first: should_run() must not still report "running" once Run() has
  // returned.  A Quit() landing between these two stores is cleared along with
  // them, which cannot hang anything -- the loop has already stopped -- it only
  // means the next Run() won't start out draining.
  run_ = false;
  quit_requested_ = false;
}

bool ReadinessBackend::should_run() const { return run_ && !quit_requested_; }

// The common half of rebuilding in a forked child.  The kernel object
// itself is per-backend (RecreateKernelState()); re-arming what was
// registered on the old one is not.
void ReadinessBackend::HandleForkInternal() {
  // Shared rule; see Aio::Impl::CheckNoRawRequestsInFlightOnFork().  The
  // failure it prevents is a duplicated write: a readiness registration
  // survives a fork, so the re-arm below would otherwise hand the child work
  // the parent is still doing.
  CheckNoRawRequestsInFlightOnFork();
  // Before RecreateKernelState() creates a descriptor; see
  // CheckRegisteredFdOpenAfterFork().  The loop's own descriptors first, so a
  // child that closed everything is told about the ones it cannot forget.
  // registrations_ holds some of them too, but once they are known open the
  // pass over it finds them open as well, and only the caller's can fail.
  CheckOwnFdsOpenAfterFork();
  for (const auto &reg : registrations_) {
    CheckRegisteredFdOpenAfterFork(reg->fd);
  }

  RecreateKernelState();
  ReArmAllRegistrations();
  AfterForkReArm();
}

void ReadinessBackend::Quit() {
  // Already asked to stop.  Bail out rather than re-arming the wakeup: once
  // Run() is draining it polls with a zero timeout, so a Quit() called from a
  // BeforeWait callback (or any other per-Poll path) would refill the queue
  // every time around and the drain would never finish.  This is the 2021
  // EPoll::Quit() guard -- f74daa655, "Make EPoll actually return from Run even
  // if you call Quit repeatedly" -- which the drain has always needed.
  //
  // Suppressing the wakeup is safe: quit_requested_ is only cleared by Run() on
  // its way out, so while it is set the loop has either already been woken or
  // is in the non-blocking drain and cannot block again.
  if (quit_requested_) {
    return;
  }

  quit_requested_ = true;
  run_ = false;
  WakeLoop();
}

void ReadinessBackend::CheckForFork() {
  size_t current_fork_count = internal::ForkCount();
  if (last_fork_count_ != current_fork_count) {
    // Record the new count *before* HandleFork(): it re-submits work
    // through these same entry points (e.g. the wakeup read via
    // AsyncRead()), so a count left stale would recurse forever.
    last_fork_count_ = current_fork_count;
    HandleForkInternal();
  }
}

void ReadinessBackend::OnInternalReadable(FileDescriptor fd,
                                          std::function<void()> drain) {
  OnReadable(fd, std::move(drain));
  registrations_.GetActive(fd)->internal = true;
}

void ReadinessBackend::QueuePending(AsyncRequest *request, QueueKind kind) {
  SetKind(request, kind);
  pending_.PushBack(request);
}

void ReadinessBackend::QueueReady(AsyncRequest *request, QueueKind kind) {
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

bool ReadinessBackend::ListContains(
    const IntrusiveDoublyLinkedList<AsyncRequest, PendingLinkTraits> &list,
    const AsyncRequest *request) {
  bool found = false;
  list.ForEach([&found, request](const AsyncRequest *queued) {
    if (queued == request) found = true;
  });
  return found;
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
// as it may, the second read freed memory.  Both lists are also intrusive
// through AsyncRequest::internal_state, so pushing a request that is
// already on one would cut everything behind it out of that list.
//
// Walks only when the request's own state says it might be ours, so a fresh
// or delivered request never walks anything.
void ReadinessBackend::CheckNotAlreadyInFlight(AsyncRequest *request) const {
  constexpr const char *kMessage = kRequestStillInFlight;
  const QueueKind kind = Kind(request);
  if (IsPending(kind)) {
    ABSL_CHECK(!ListContains(pending_, request)) << kMessage;
  } else if (IsReady(kind)) {
    ABSL_CHECK(!ListContains(ready_, request)) << kMessage;
  }
  if (!request->done) {
    for (const auto &reg : registrations_) {
      ABSL_CHECK(reg->read_req != request && reg->write_req != request)
          << kMessage;
    }
  }
}

void ReadinessBackend::CheckLive(const FdRegistration *reg) {
  ABSL_CHECK(reg->registered)
      << ": the kernel reported an event for a released registration.  Its "
         "fd was ForgetClosedFd()'d while its file was still open elsewhere "
         "(a dup() or a fork()), which leaves epoll watching it; for a shared "
         "file, DeleteFd() before closing instead";
}

ssize_t ReadinessBackend::DoRawIo(FdRegistration *reg, Direction direction,
                                  void *data, size_t size) {
  if (size == 0) {
    // The same rule as EpollImpl::DoRawIo(), the way io_uring treats an empty
    // request: readv()/writev() with an empty iovec check the fd and return 0
    // without calling into the file, and a socket's empty write is a send(),
    // which reaches the protocol.  Unverified on macOS.
    struct iovec empty = {.iov_base = data, .iov_len = 0};
    if (direction == Direction::kRead) {
      return readv(reg->fd, &empty, 1);
    }
    const ssize_t res = send(reg->fd, data, 0, MSG_DONTWAIT);
    if (res >= 0 || errno != ENOTSOCK) {
      return res;
    }
    return writev(reg->fd, &empty, 1);
  }
  return direction == Direction::kRead ? read(reg->fd, data, size)
                                       : write(reg->fd, data, size);
}

void ReadinessBackend::AsyncRead(FileDescriptor fd, std::span<char> buffer,
                                 AsyncRequest *request) {
  CheckForFork();
  SubmitRaw(fd, buffer.data(), buffer.size(), request, Direction::kRead);
}

void ReadinessBackend::AsyncWrite(FileDescriptor fd,
                                  std::span<const char> buffer,
                                  AsyncRequest *request) {
  CheckForFork();
  SubmitRaw(fd, const_cast<char *>(buffer.data()), buffer.size(), request,
            Direction::kWrite);
}

void ReadinessBackend::SubmitRaw(FileDescriptor fd, void *data, size_t size,
                                 AsyncRequest *request, Direction direction) {
  const bool is_read = direction == Direction::kRead;
  CheckNotAlreadyInFlight(request);
  // Cleared rather than trusted: constraint 2 lets a request outlive the Aio
  // it was armed on, so it can arrive still marked queued on a list belonging
  // to an instance that is gone.  See IoUringImpl::ClearStaleRawState().
  SetKind(request, QueueKind::kNone);
  request->done = false;
  if (fd < 0) {
    State(request).link.result = -EBADF;
    QueuePending(request, QueueKind::kPendingResult);
    return;
  }

  FdRegistration *reg = registrations_.GetActive(fd);
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
    reg = registrations_.GetOrCreateAsync(fd);
  }
  reg->slot(direction) = request;
  State(request).raw.ptr = data;
  State(request).raw.size = size;
  // No I/O and no kernel registration here: the first attempt is the
  // harvest's, in submission order with every other queued outcome -- see
  // StartRaw().
  State(request).link.result = static_cast<int32_t>(fd);
  QueuePending(request, is_read ? QueueKind::kPendingSubmitRead
                                : QueueKind::kPendingSubmitWrite);
}

bool ReadinessBackend::StartRaw(AsyncRequest *request, Direction direction) {
  FdRegistration *reg = registrations_.GetActive(
      static_cast<FileDescriptor>(State(request).link.result));
  ABSL_CHECK(reg != nullptr && reg->slot(direction) == request);
  if (!reg->raw_prepared) {
    // Once per registration, not per request.  See PrepareRawFd().
    PrepareRawFd(reg->fd);
    reg->raw_prepared = true;
  }
  if (TryRawIo(reg, direction)) {
    return true;
  }
  // EAGAIN: wait for readiness.
  if (!UpdateRegistration(reg)) {
    // The kernel refused to watch it.  Deliver the errno as the error
    // completion aio.h promises instead of aborting.
    FailRawRegistration(reg);
    return true;
  }
  if (reg->unpollable) {
    // A regular file, found by the registration above: nothing to wait for,
    // so the I/O runs now, blocking.
    ABSL_CHECK(TryRawIo(reg, direction));
    return true;
  }
  return false;
}

bool ReadinessBackend::TryRawIo(FdRegistration *reg, Direction direction) {
  AsyncRequest *req = reg->slot(direction);
  const ssize_t res =
      DoRawIo(reg, direction, State(req).raw.ptr, State(req).raw.size);
  if (res < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
    return false;
  }
  // Before any kernel call below can clobber it.
  const int io_errno = errno;
  reg->slot(direction) = nullptr;
  State(req).link.result = res >= 0 ? static_cast<int32_t>(res) : -io_errno;
  QueueReady(req, QueueKind::kReadyResult);
  SettleRawRegistration(reg);
  return true;
}

bool ReadinessBackend::ServiceRaw(const FdReady &ready) {
  FdRegistration *const reg = ready.reg;
  // A raw request observes a terminal condition by running: a pending read
  // only sees EOF from its own read(), and closing an empty pipe's write end
  // raises a hangup with no readable bit.  So a hangup or error is readiness
  // for whichever requests are waiting.
  //
  // That assumes the fd's socket error queue is not in use.  On Linux an
  // unread IP_RECVERR or SO_TIMESTAMPING entry keeps EPOLLERR set while
  // read() still returns EAGAIN, so every Poll() would wake for it and
  // resolve nothing.  Only recvmsg(MSG_ERRQUEUE) clears it, which no raw
  // request issues.
  bool resolved = false;
  if (Armed(reg, Direction::kRead) != nullptr &&
      (ready.terminal || (ready.events & kInEvents) != 0)) {
    resolved |= TryRawIo(reg, Direction::kRead);
  }
  // The read may have retired the registration (Detach() tombstones fd).
  if (reg->fd != kInvalidFd && Armed(reg, Direction::kWrite) != nullptr &&
      (ready.terminal || (ready.events & kOutEvents) != 0)) {
    resolved |= TryRawIo(reg, Direction::kWrite);
  }
  return resolved;
}

void ReadinessBackend::SettleRawRegistration(FdRegistration *reg) {
  if (FdRegistrationTable::ShouldRetire(reg)) {
    RemoveRegistration(reg);
    registrations_.Release(reg);
    return;
  }
  if (!UpdateRegistration(reg)) {
    FailRawRegistration(reg);
  }
}

void ReadinessBackend::FailRawRegistration(FdRegistration *reg) {
  const int err = reg->registration_errno;
  reg->registration_errno = 0;
  // Only the armed ones: a request still waiting on pending_ for its first
  // attempt gets that attempt, which reports whatever is wrong with the fd.
  for (Direction direction : {Direction::kRead, Direction::kWrite}) {
    if (AsyncRequest *req = Armed(reg, direction)) {
      reg->slot(direction) = nullptr;
      State(req).link.result = -err;
      QueueReady(req, QueueKind::kReadyResult);
    }
  }
  // Retires it unless a cancel claim or an unattempted request keeps it.
  SettleRawRegistration(reg);
}

void ReadinessBackend::Cancel(AsyncRequest *request) {
  CheckForFork();
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
  for (const auto &reg : registrations_) {
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
  State(request).raw.ptr = found;
  QueuePending(request, direction == Direction::kRead
                            ? QueueKind::kPendingCancelRead
                            : QueueKind::kPendingCancelWrite);
}

void ReadinessBackend::BeforeWait(std::function<void()> function) {
  ABSL_CHECK(!in_before_wait_)
      << ": BeforeWait() may not be called from a before-wait function";
  before_wait_functions_.push_back(std::move(function));
}

void ReadinessBackend::OnReadable(FileDescriptor fd,
                                  std::function<void()> callback) {
  CheckForFork();
  internal::FdRegistration *reg = registrations_.GetOrCreateLegacy(fd);
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
  UpdateRegistrationOrDie(reg);
}

void ReadinessBackend::OnError(FileDescriptor fd,
                               std::function<void()> callback) {
  CheckForFork();
  internal::FdRegistration *reg = registrations_.GetOrCreateLegacy(fd);
  ABSL_CHECK(!reg->events_fn)
      << "Cannot mix OnEvents and OnError for fd " << fd;
  ABSL_CHECK(reg->idle())
      << "Cannot mix AsyncRead/AsyncWrite and OnError on fd " << fd;
  // Unconditional -- see OnReadable().
  ABSL_CHECK(!reg->err_fn) << "Duplicate error functions for " << fd;
  reg->err_fn = std::move(callback);

  reg->events |= kErrorEvents;
  UpdateRegistrationOrDie(reg);
}

void ReadinessBackend::OnWritable(FileDescriptor fd,
                                  std::function<void()> callback) {
  CheckForFork();
  internal::FdRegistration *reg = registrations_.GetOrCreateLegacy(fd);
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
  UpdateRegistrationOrDie(reg);
}

void ReadinessBackend::OnEvents(FileDescriptor fd,
                                std::function<void(uint32_t)> callback) {
  CheckForFork();
  internal::FdRegistration *reg = registrations_.GetOrCreateLegacy(fd);
  ABSL_CHECK(reg->idle())
      << "Cannot mix OnEvents and AsyncRead/AsyncWrite on fd " << fd;
  ABSL_CHECK(!reg->in_fn && !reg->out_fn && !reg->err_fn)
      << "May not replace OnEvents handlers for fd " << fd;
  ABSL_CHECK(!reg->events_fn)
      << "May not replace OnEvents handlers for fd " << fd;
  reg->events_fn = std::move(callback);
}

void ReadinessBackend::DeleteFd(FileDescriptor fd) {
  CheckForFork();
  auto *reg = registrations_.GetActive(fd);
  ABSL_CHECK(reg != nullptr) << "fd " << fd << " not found";
  // Async-only registrations are not the caller's to delete.  DeleteFd()
  // undoes an On*() registration; an fd carrying only AsyncRead/AsyncWrite
  // has none, and Release()ing it here would drop the requests without ever
  // completing them -- leaving req.done false forever, which aio.h's
  // constraint 2 says the caller may then never free or reuse.  A silent
  // hang, where io_uring gives a loud "fd not found" for the same sequence
  // because its legacy state lives in a separate table.
  //
  // Raw requests are retired by completing or Cancel()ing them, which is
  // what a caller wanting this fd gone should do.
  ABSL_CHECK(!reg->async_only) << "fd " << fd << " not found";

  // Strict for a legacy fd -- see RemoveRegistration().
  RemoveRegistration(reg);

  registrations_.Release(reg);
}

const char *ReadinessBackend::OwnFdName(FileDescriptor fd) {
  return fd == wakeup_fd() ? "wakeup" : nullptr;
}

void ReadinessBackend::CheckOwnFdsOpenAfterFork() {
  CheckOwnFdOpenAfterFork(wakeup_fd(), "wakeup");
}

void ReadinessBackend::ForgetClosedFd(FileDescriptor fd) {
  // registrations_ also holds the timers' and the receiver's own
  // registrations, so finding fd there does not make it the caller's.  A
  // number cannot be both: taking one the caller still has registered for an
  // own descriptor dies at once (DieOwnFdStillRegistered()), so asking which
  // it is settles it.
  if (const char *owner = OwnFdName(fd)) {
    DieForgettingOwnFd(fd, owner);
  }
  auto *reg = registrations_.GetActive(fd);
  ABSL_CHECK(reg != nullptr) << "fd " << fd << " not found";
  // Not the caller's to forget -- see DeleteFd().
  ABSL_CHECK(!reg->async_only) << "fd " << fd << " not found";

  registrations_.Release(reg);
}

void ReadinessBackend::EnableWritable(FileDescriptor fd) {
  CheckForFork();
  auto *reg = registrations_.GetActive(fd);
  ABSL_CHECK(reg != nullptr) << "fd " << fd << " not found";
  // A raw-only registration is not an fd the caller registered a handler on,
  // so this is the same misuse io_uring reports as "fd not found" -- it looks
  // in a table that only holds legacy state, while this one holds both.
  ABSL_CHECK(!reg->async_only) << "fd " << fd << " not found";
  ABSL_CHECK(!reg->events_fn)
      << "EnableWritable is only for fds registered using OnWritable, not "
         "OnEvents";

  uint32_t new_events = reg->events | kOutEvents;
  if (reg->events != new_events) {
    reg->events = new_events;
    UpdateRegistrationOrDie(reg);
  }
}

void ReadinessBackend::DisableWritable(FileDescriptor fd) {
  CheckForFork();
  auto *reg = registrations_.GetActive(fd);
  ABSL_CHECK(reg != nullptr) << "fd " << fd << " not found";
  // Same as EnableWritable() -- see there.
  ABSL_CHECK(!reg->async_only) << "fd " << fd << " not found";
  ABSL_CHECK(!reg->events_fn)
      << "DisableWritable is only for fds registered using OnWritable, not "
         "OnEvents";

  uint32_t new_events = reg->events & ~kOutEvents;
  if (reg->events != new_events) {
    reg->events = new_events;
    UpdateRegistrationOrDie(reg);
  }
}

void ReadinessBackend::SetEvents(FileDescriptor fd, uint32_t events) {
  CheckForFork();
  auto *reg = registrations_.GetActive(fd);
  ABSL_CHECK(reg != nullptr) << "fd " << fd << " not found";
  ABSL_CHECK(reg->events_fn)
      << "SetEvents is only for fds registered using OnEvents";

  if (reg->events != events) {
    reg->events = events;
    UpdateRegistrationOrDie(reg);
  }
}

void ReadinessBackend::UpdateRegistrationOrDie(internal::FdRegistration *reg) {
  if (UpdateRegistration(reg)) {
    return;
  }
  const int err = reg->registration_errno;
  reg->registration_errno = 0;
  ABSL_LOG(FATAL) << "The kernel refused the registration for fd " << reg->fd
                  << ": " << std::strerror(err)
                  << ".  A descriptor closed behind this loop's back has to be "
                     "removed with ForgetClosedFd() before it is closed; there "
                     "is no request here to report this on.";
}

bool ReadinessBackend::HasRawRequestsInFlight() const {
  for (const auto &reg : registrations_) {
    // No carve-out for the loop's own wakeup: it is an internal legacy
    // registration, not a raw request, so it cannot land here at all.
    if (reg->read_req != nullptr) {
      return true;
    }
    if (reg->write_req != nullptr) {
      return true;
    }
  }
  // A raw request stays in flight until its callback has run, so one that
  // has resolved but is still queued counts too: a submit-time result or a
  // cancel on pending_, or a result on ready_.  The legacy marker is not a
  // request.  Only reached on the fork path.
  for (const auto *queue : {&pending_, &ready_}) {
    for (AsyncRequest *req = queue->front(); req != nullptr;
         req = std::remove_pointer_t<decltype(queue)>::Next(req)) {
      if (Kind(req) != QueueKind::kReadyLegacy) {
        return true;
      }
    }
  }
  return false;
}

bool ReadinessBackend::Poll(bool block) {
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

  CheckForFork();

  bool processed;
  {
    FdRegistrationTable::DispatchGuard dispatch_guard(&registrations_);
    processed = PollOnce(block);
  }

  // Reclaim what this Poll()'s callback retired (a callback that deleted its
  // own fd), now that its frame has unwound -- at the end of the same
  // dispatch, as IoUringImpl frees its parked legacy state.  Leaving it for
  // the top of the next Poll() moved the free into whatever context that one
  // ran in, which could be ScopedRealtime and trip the RT free check; here it
  // stays with the dispatch whose callback passed DeleteFd()'s
  // CheckNotRealtime().
  registrations_.ScrubRetired();
  return processed;
}

bool ReadinessBackend::PollOnce(bool block) {
  // Registering a before-wait function from inside one is disallowed --
  // see BeforeWait().
  in_before_wait_ = true;
  for (const auto &fn : before_wait_functions_) {
    fn();
  }
  in_before_wait_ = false;

  bool processed = false;
  FdReady harvested;
  // Only go back to the kernel once everything already resolved has been
  // delivered -- see the class comment.
  if (ready_.empty()) {
    switch (Harvest(block, &harvested)) {
      case HarvestResult::kNothing:
        break;
      case HarvestResult::kProcessed:
        processed = true;
        break;
      case HarvestResult::kCallbackRan:
        return true;
    }
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
    // Only ever handed over once: the next legacy dispatch must ask the
    // kernel again, since this callback may delete or replace that fd.
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
      completion.status = aos::MakeError(backend_name_ + " error");
      completion.result = -State(req).link.result;
    }
    req->callback(completion, req->context);
    return true;
  }
  return processed;
}

ReadinessBackend::HarvestResult ReadinessBackend::Harvest(bool block,
                                                          FdReady *harvested) {
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
        // A first attempt that only armed the kernel resolved nothing, so it
        // does not count: Poll(false) right after an AsyncRead() on an empty
        // pipe returns false, as io_uring's does, and Poll(true) goes on to
        // wait.
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
    if (auto *reg = static_cast<FdRegistration *>(State(req).raw.ptr)) {
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
  bool wait = block && !processed;
  while (true) {
    FdReady ready;
    const WaitResult result = WaitForOne(wait, &ready);
    wait = false;
    if (result == WaitResult::kNothing) {
      break;
    }
    processed = true;
    if (result == WaitResult::kInternal) {
      continue;
    }
    if (result == WaitResult::kHandled) {
      // The backend dispatched a filter of its own (a kqueue timer or
      // signal), which may have run a callback: that is this Poll()'s one.
      return HarvestResult::kCallbackRan;
    }
    FdRegistration *const reg = ready.reg;
    CheckLive(reg);
    if (reg->internal) {
      // The wakeup.  Internal work: neither delivered nor rationed.
      reg->in_fn();
      continue;
    }
    if (reg->async_only) {
      // Do the I/O and keep collecting: that I/O can itself have readied
      // another raw request (writing a pipe makes its read end readable),
      // and io_uring completes both inside one submit.
      if (!ServiceRaw(ready)) {
        // Reported but not actually ready.  Stop rather than spin on it.
        break;
      }
      continue;
    }
    // A legacy fd.  Running its handlers is the delivery itself, so it cannot
    // be resolved early; a marker takes its place in line, and this event is
    // handed to it directly if nothing runs in between.
    *harvested = ready;
    SetKind(&legacy_marker_, QueueKind::kReadyLegacy);
    ready_.PushBack(&legacy_marker_);
    break;
  }
  return processed ? HarvestResult::kProcessed : HarvestResult::kNothing;
}

bool ReadinessBackend::DispatchLegacyEvent(FdReady *harvested) {
  // The harvest's own event, when nothing has run since it was read: no
  // callback can have deleted or replaced the fd, so asking again would only
  // cost a syscall.
  if (harvested->reg != nullptr) {
    const FdReady ready = *harvested;
    harvested->reg = nullptr;
    RunLegacyHandlers(ready);
    return true;
  }
  // Otherwise ask the kernel which legacy fd is ready now.  Raw and wakeup
  // events met on the way are resolved as internal work, so this ends once
  // nothing but legacy events can be left.
  while (true) {
    FdReady ready;
    switch (WaitForOne(false, &ready)) {
      case WaitResult::kNothing:
        return false;
      case WaitResult::kHandled:
        return true;
      case WaitResult::kInternal:
        continue;
      case WaitResult::kFdReady:
        break;
    }
    FdRegistration *const reg = ready.reg;
    CheckLive(reg);
    if (reg->internal) {
      reg->in_fn();
      continue;
    }
    if (reg->async_only) {
      if (!ServiceRaw(ready)) {
        return false;
      }
      continue;
    }
    RunLegacyHandlers(ready);
    return true;
  }
}

void ReadinessBackend::RunLegacyHandlers(const FdReady &ready) {
  FdRegistration *const reg = ready.reg;
  if (reg->events_fn) {
    // OnEvents delivers everything, hangups included -- see the OnEvents()
    // contract in aio.h.  SetEvents(fd, <hangup bit>) is a supported way to
    // watch for one, which is why this does not follow EPoll here.
    reg->events_fn(ready.terminal ? (ready.events | kErr) : ready.events);
    return;
  }

  // A hangup is not an error event here, as it was not in EPoll.  One that
  // nothing else carries is routed to a handler rather than dropped, and so is
  // an error with no err_fn.  See RouteHangupOrError().  The CHECKs below are
  // EPoll::InOutEventData::DoCallbacks() verbatim, message text included --
  // callers depend on those semantics.  A callback that deletes its own fd
  // tombstones reg->fd (Detach()), which ends the remaining handlers.
  const uint32_t events = RouteHangupOrError(
      ready.events, ready.terminal && (ready.events & kErr) == 0, reg->events,
      static_cast<bool>(reg->err_fn));
  if (events & kInEvents) {
    ABSL_CHECK(reg->in_fn)
        << ": No handler registered for input events on descriptor " << reg->fd
        << ". Received events = 0x" << std::hex << events << std::dec;
    reg->in_fn();
  }
  if (reg->fd != kInvalidFd && (events & kOutEvents)) {
    ABSL_CHECK(reg->out_fn)
        << ": No handler registered for output events on descriptor " << reg->fd
        << ". Received events = 0x" << std::hex << events << std::dec;
    reg->out_fn();
  }
  if (reg->fd != kInvalidFd && (events & kErrorEvents)) {
    ABSL_CHECK(reg->err_fn)
        << ": No handler registered for error events on descriptor " << reg->fd
        << ". Received events = 0x" << std::hex << events << std::dec << ". "
        << GetSocketErrorStr(reg->fd);
    reg->err_fn();
  }
}

}  // namespace aos::internal
