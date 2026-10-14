#ifndef AOS_EVENTS_AIO_UNIX_H_
#define AOS_EVENTS_AIO_UNIX_H_

#include <fcntl.h>

#include <cstddef>

#include "absl/log/absl_check.h"

#include "aos/events/socket_error.h"

namespace aos::internal {

// Pieces every POSIX Aio backend needs, kept in one place so they cannot
// drift apart between aio_linux.cc and aio_darwin.cc.  IsSocket() and
// GetSocketErrorStr() come from socket_error.h.

// Puts fd in non-blocking mode if it is not already.  The readiness backends
// run a raw request's read(2)/write(2) inside Poll() once the fd reports
// ready, and readiness does not promise room for the whole buffer, so a
// blocking fd could put the loop thread to sleep.  See Aio::AsyncRead().
//
// An fd fcntl() cannot even read the flags of (closed, most likely) is left
// alone rather than aborting: the read(2)/write(2) that follows fails the same
// way, and that errno is the error completion aio.h promises the caller.
inline void SetNonBlocking(int fd) {
  const int flags = fcntl(fd, F_GETFL);
  if (flags == -1) {
    return;
  }
  if ((flags & O_NONBLOCK) == 0) {
    ABSL_PCHECK(fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0)
        << "fcntl(F_SETFL, O_NONBLOCK) failed for fd " << fd;
  }
}

// How a backend notices it is running in a forked child.
//
// A kernel event object does not survive fork(): an io_uring's queues are
// mapped from the parent, a kqueue is not inherited at all, and an epoll
// instance is inherited but shared, so parent and child would steal each
// other's events.  Every backend therefore has to rebuild before a forked
// child uses it.
//
// The detection is a counter bumped by a pthread_atfork handler, which each
// loop compares against its own copy at the top of every public entry point
// (CheckForFork()).  Deliberately lazy, rather than rebuilding every live
// loop from inside the handler:
//
//   * A child that never touches the loop -- fork()+exec(), which is most of
//     what forking is for around here -- pays nothing, and is unaffected by a
//     rebuild it does not need.
//   * It is the only way the "a child may not inherit raw I/O" rule can hold
//     without making fork()+exec() fatal; see
//     Aio::Impl::CheckNoRawRequestsInFlightOnFork().
//   * The rebuild runs on the thread that goes on to drive the loop, rather
//     than inside an atfork handler, where async-signal-safety rules make
//     almost everything it needs to do questionable.
//
// A relaxed atomic load never syscalls, which is what makes the
// per-entry-point check cheap enough to be unconditional.  size_t because
// this only ever answers "did a fork happen since I last looked", so any
// width works -- and size_t stays single-instruction lock-free everywhere (a
// fixed 64-bit type would drag 32-bit ARM through libatomic locks; unsigned
// int wouldn't grow on 64-bit targets).

// How many times this process has been forked *into* -- bumped in the child.
size_t ForkCount();
// How many times this process has *called* fork(), which io_uring needs on
// top of the child counter: on DEFER_TASKRUN rings a multishot op
// outstanding in the parent becomes uncancelable after any fork() until a
// resync.
size_t ParentForkCount();

// Registers the handlers that bump both counters.  Idempotent, and every
// backend calls it from its constructor: an epoll-only build that left this
// to the io_uring backend never noticed a fork at all, and the child hung.
// A backend needing more than the counters registers its own handler on top;
// pthread_atfork stacks them.
void RegisterForkCounters();

// The other half of a forked child's contract: every fd it closes that is still
// registered must be ForgetClosedFd()'d before its next use of the loop.  A
// backend's rebuild calls this for each fd it is about to re-register, before
// creating any descriptor of its own.  After that, the lowest free number is
// the one the child just closed, and the re-register could succeed on the
// backend's own new fd instead of failing.  Shared so the rule and its message
// cannot drift between backends.
void CheckRegisteredFdOpenAfterFork(int fd);

// The same check for a descriptor the loop made for itself: a timer's timerfd,
// a registered thread-signal receiver's fd, or the wakeup.  The caller never
// registered it, so ForgetClosedFd() is no way out; the child has to leave it
// open.  `owner` names what it belongs to.
void CheckOwnFdOpenAfterFork(int fd, const char *owner);

// ForgetClosedFd() is for fds the caller registered.  One the loop made for
// itself was never the caller's to forget.
void DieForgettingOwnFd(int fd, const char *owner);

// A descriptor the loop just took for its own use (a timer's new timerfd, a
// thread-signal receiver's fd) has the number of one the caller still has
// registered.  The caller closed that fd without ForgetClosedFd(), and the
// kernel handed its number out again.  Caught where the reuse happens, rather
// than as a confusing collision later.
void DieOwnFdStillRegistered(int fd, const char *owner);

}  // namespace aos::internal

#endif  // AOS_EVENTS_AIO_UNIX_H_
