#ifndef AOS_EVENTS_AIO_H_
#define AOS_EVENTS_AIO_H_

#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <vector>

#include "aos/time/time.h"
#include "aos/util/status.h"

namespace aos {

namespace ipc_lib {
class ThreadSignalReceiver;
}  // namespace ipc_lib

#if defined(_WIN32)
using FileDescriptor = void *;
#else
using FileDescriptor = int;
#endif

// Represents the payload returned upon completion of an async operation.
//
// Expected statuses and result details:
// 1. Successful completion:
//    - `status` is `Ok()`.
//    - `result` is set to the non-negative count of bytes transferred for
//      I/O operations, or 0 for timers.
// 2. Explicit cancellation:
//    - `status` is an error with message "Canceled".
//    - `result` is 0.
// 3. Operational failure:
//    - `status` is an error whose static-literal message names the failing
//      backend (e.g. "io_uring error"), so a log line points at the right
//      implementation immediately.  Match failures with !IsOk(), never the
//      message text -- the text is diagnostic and differs per backend.
//    - `result` is set to the positive errno value representing the OS-level
//      error (e.g., EPIPE, EBADF, EINVAL).
//
// Note: The failure status messages are static literals to ensure that
// error propagation inside real-time threads does not trigger dynamic
// memory allocation (which is prohibited under ScopedRealtime).
struct Completion {
  // Completion status.
  aos::Status status;
  // Success/error details (e.g., bytes read/written, or positive errno).
  int32_t result;
  // Opaque pointer supplied by the caller when scheduling the request.
  void *user_data;
};

// Callback function type invoked when an asynchronous operation finishes.
using CompletionCallback = void (*)(Completion completion, void *context);

struct AsyncRequest {
  // Callback to invoke on completion.
  CompletionCallback callback = nullptr;
  // Opaque context pointer passed to the callback.
  void *context = nullptr;
  // Caller-supplied data in the Completion.
  void *user_data = nullptr;

  // Tracks if request has completed or is not currently pending.
  bool done = true;

  // Internal state managed entirely by the Aio implementation.  Callers must
  // not read or modify this field.
  alignas(8) uint8_t internal_state[32] = {0};
};

// Aio is a cross-platform asynchronous I/O multiplexer and event loop engine.
// It supports asynchronous non-blocking reads, writes, absolute timers, and
// signal event notifications.
//
// Why Aio over epoll:
// Unlike epoll, which is a readiness-based multiplexer (notifying the caller
// when a file descriptor is ready to be read or written, requiring subsequent
// synchronous system calls), Aio is completion-based.  On Linux, it leverages
// io_uring, allowing callers to submit actual I/O operations (like reads and
// writes) directly to kernel queues.  The kernel performs these operations
// asynchronously and notifies the event loop upon complete execution.  This
// eliminates user-to-kernel context switch overhead for read/write calls,
// simplifies memory ownership, and enables highly efficient multishot signal
// polling.  Additionally, a completion-based model is natively compatible
// with Windows' I/O Completion Ports (IOCP) and Overlapped I/O, allowing for
// a cleaner and more performant cross-platform abstraction than simulating
// readiness-based models.
//
// Aio also supports the primitives needed to implement the Epoll class using
// it.
//
// Constraints:
// 1. Thread Safety: Aio is designed to be driven by a single-threaded event
//    loop.  All scheduling (e.g., AsyncRead, AsyncWrite, AsyncTimer) and
//    cancellation (Cancel) operations -- including destroying the Aio
//    instance itself -- should be invoked from the thread that drives the
//    loop via Poll() or Run().  Two exceptions:
//      * Quit() is thread-safe and may be called from any thread, at any
//        time.
//      * Before the first Poll()/Run() call, no thread has bound the loop
//        yet, so there is no binding to violate: every entry point may be
//        used from any thread (though never from two threads concurrently
//        -- nothing here is synchronized).  This is what makes pre-Run()
//        setup work: constructing and configuring an Aio (arming timers,
//        registering fds) on one thread, then handing it to the thread
//        that will drive it.
//
//    The io_uring backend enforces this: IORING_SETUP_SINGLE_ISSUER binds
//    the ring to whichever thread first calls Run()/Poll() and ABSL_CHECKs
//    any later violation, destructor included.  One exception: if the
//    constructing thread differs from that first caller, the ring transparently
//    rebuilds on the unconstrained COOP_TASKRUN tier without the binding
//    (aio_linux.cc's EnsureBound()). That mismatch means senders/watchers were
//    registered on the constructing thread, and AOS shared-memory queues
//    require those to be destroyed there (a PI-futex property, see
//    lockless_queue.cc's RobustOwnershipTracker) -- which SINGLE_ISSUER's
//    binding would forbid. See
//    documentation/adr/0001-aio-io-uring-single-issuer.md.
// 2. Request Lifetime: The caller-supplied AsyncRequest object must remain
//    valid and allocated in memory from the time it is submitted until its
//    corresponding CompletionCallback is executed -- including for
//    canceled requests, whose Canceled completion arrives through Poll()
//    like any other.  Once the callback has run, the request may be freed
//    or reused; nothing in the loop or the kernel names it afterward.
//    (Backends earn that simplicity internally where the kernel would
//    otherwise complicate it -- e.g. io_uring's cancellation
//    acknowledgments deliberately carry a loop-owned identity rather than
//    the canceled request's, see aio_linux.cc's cancel_ack_sentinel_ --
//    so the caller's rule never depends on unobservable internal state.)
//    Destroying the Aio instance terminates any pending requests inside
//    the kernel, but does NOT finalize them: their callbacks are never
//    invoked, not even
//    with a Canceled status -- completions are only ever delivered inside
//    Poll()/Run(), and destruction does not poll.  A caller that needs to
//    observe a pending request finish must Cancel() it and keep Poll()ing
//    until its callback runs, before destroying the Aio.
// 3. Callback Reentrancy: Completion callbacks run synchronously inside
//    Poll() and should not block the event loop thread.  Poll() is not
//    reentrant: calling it from a completion callback or before-wait
//    function is a fatal error.  To wait for another completion, return
//    and let the event loop deliver it.
class Aio {
 public:
  struct TimerState;

  Aio();
  ~Aio();

  Aio(const Aio &) = delete;
  Aio &operator=(const Aio &) = delete;
  Aio(Aio &&) = delete;
  Aio &operator=(Aio &&) = delete;

  // Drives the loop continuously until Quit() is called.
  void Run();

  // Polls for and processes active completions.
  // Returns true if any event (including timers or wakeup signals) was
  // processed.
  //
  // Dispatches at most ONE user-visible completion per call, on every
  // backend: one raw AsyncRead/AsyncWrite completion, one timer firing, or
  // one legacy fd's readiness events.  Internal completions (the wakeup
  // read, poll re-arms, a legacy firing with nothing left to report) are
  // not rationed: a single Poll() dispatches as many of those as it takes
  // to reach the first user-visible one (or empty its queue).  One bounded
  // exception to "one": a legacy fd can deliver readable, writable, and
  // error together in one Poll(), as EPoll always has.  (A thread-signal
  // receiver is NOT an exception: pending wakeups coalesce into a single
  // callback invocation -- see RegisterThreadSignalReceiver().)
  // A backend that learns about several ready user-visible completions at
  // once (io_uring drains the whole completion queue; IOCP can have a
  // queue of synchronous failures) delivers one and leaves the rest queued
  // for the next call, so draining N ready completions takes N calls.
  // That costs a loop iteration each -- not a syscall each -- and Run() is
  // an unconditional drain loop, so it is not observable as latency.
  //
  // This is a contract, not an implementation accident: the alternative is
  // that "how many callbacks does one Poll() run" depends on which backend
  // you built against, which makes the backends distinguishable to any
  // consumer that polls by hand.
  //
  // A blocking Poll() is not interrupted by signals: every backend
  // absorbs EINTR and resumes waiting.  EPoll::Poll() returned false
  // instead, but the `while (!signal_flag && Poll(true))` idiom that
  // enabled was racy anyway (the pselect(2) race).  Shut down from a
  // signal handler with Quit(): async-signal-safe, and its wakeup is
  // queued, so it is never lost to that race.
  bool Poll(bool block);

  // Signals the loop to terminate execution.  Async-safe.
  //
  // The request is sticky and consumed only by Run(): the first Quit()
  // wakes a blocked Poll()/Run() and latches the request; every further
  // Quit() before a Run() consumes it is a silent no-op (deliberate --
  // re-arming the wakeup would keep Run()'s post-Quit() drain from ever
  // finishing).  A loop driven purely by Poll() therefore sees exactly one
  // wakeup out of any number of Quit()s, and nothing ever clears the
  // latch; such a loop must watch its own stop condition rather than
  // expect repeated Quit()s to keep waking it.  (EPoll::Quit() was
  // stricter still: outside Run() it did nothing at all.)
  void Quit();

  // Schedules an asynchronous read on a file descriptor.
  void AsyncRead(FileDescriptor fd, std::span<char> buffer,
                 AsyncRequest *request);

  // Schedules an asynchronous write to a file descriptor.
  void AsyncWrite(FileDescriptor fd, std::span<const char> buffer,
                  AsyncRequest *request);

  class Timer {
   public:
    Timer(Aio *aio);
    ~Timer();

    Timer(const Timer &) = delete;
    Timer &operator=(const Timer &) = delete;
    Timer(Timer &&) = delete;
    Timer &operator=(Timer &&) = delete;

    // Schedules a single timer execution.  The callback fires once, at or
    // after the given deadline, and nothing is armed afterward.  If a timer
    // is already scheduled, it is canceled first.  RT-safe on every backend:
    // one syscall, no allocation.
    //
    // The callback is only ever invoked with an Ok() status, on every
    // backend.  There is no error to deliver: Cancel() is silent (the
    // callback is dropped, never invoked with a Canceled status), and a
    // failure of the backend's underlying timer machinery is fatal inside
    // the backend rather than surfaced here.  Callers should CHECK the
    // status rather than handle it.
    // The delivered Completion's user_data is nullptr: Schedule() takes
    // no user_data, and Completion::user_data is documented as the pointer
    // the caller supplied.
    //
    // `deadline` must be at or after aos::monotonic_clock::epoch(); earlier
    // is a caller bug and CHECK-fails.  The epoch itself is allowed and is
    // not special: like any deadline already past, it fires as soon as the
    // loop is next driven.  aos::TimerHandler::Schedule() has the same
    // boundary, and the simulated event loop's timeline starts at the epoch,
    // so application code that schedules there has to behave the same in sim
    // and on a ShmEventLoop.  (The Linux backends need a nudge to honor
    // that -- see AbsoluteTimerfdValue() in aio_linux.cc.)
    //
    // There is deliberately no repeating form.  A caller that wants a
    // periodic timer re-schedules from its own callback against an absolute
    // deadline it computes itself -- which is what every consumer in this
    // tree already does, because a periodic timer that is merely "armed
    // again every period" is not enough on its own: the owner still has to
    // decide what happens to periods that elapsed while it was busy, and
    // that policy differs per consumer (see ShmTimerHandler, which skips to
    // the next future deadline, versus PhasedLoop, which counts them).
    // Pushing the repeat down here would mean owning that policy for
    // everyone and getting it wrong for someone.
    void Schedule(aos::monotonic_clock::time_point deadline,
                  CompletionCallback callback, void *context = nullptr);

    // Cancels the active timer.  Cancellation is silent: the pending
    // callback is dropped and will not be invoked, not even with a Canceled
    // status.  (Unlike Aio::Cancel() on a raw request, which delivers a
    // Canceled completion through Poll().)
    void Cancel();

   private:
    std::unique_ptr<TimerState> state_;
  };

  // Cancels a pending request.  This submits an asynchronous cancellation
  // request to the multiplexer.  The original request will then complete
  // asynchronously through Poll, at which point its callback is invoked with
  // a status of Canceled.
  //
  // Best effort, inherently: a request that has already completed -- even if
  // its completion is still queued and its callback has not run yet -- is
  // not cancellable, and Cancel() is a silent no-op there.  The callback
  // then runs with the original result (e.g. Ok() and the bytes read), not
  // Canceled.  There is no "did it take effect" signal because none could
  // be race-free: the kernel can complete the request between any such
  // answer and the caller acting on it.  The callback's status is the one
  // authoritative answer.
  void Cancel(AsyncRequest *request);

  // Registers a function to be executed just before blocking on events.
  void BeforeWait(std::function<void()> function);

  // Registers a function to be called when the fd is readable.
  // Only one function may be registered for readability on each fd.
  //
  // A fd may be registered exclusively with OnReadable/OnWritable/OnError OR
  // OnEvents.
  void OnReadable(FileDescriptor fd, std::function<void()> callback);

  // Registers a function to be called when the fd has an error.
  // Only one function may be registered for errors on each fd.
  //
  // A fd may be registered exclusively with OnReadable/OnWritable/OnError OR
  // OnEvents.
  void OnError(FileDescriptor fd, std::function<void()> callback);

  // Registers a function to be called when the fd is writable.
  // Only one function may be registered for writability on each fd.
  //
  // A fd may be registered exclusively with OnReadable/OnWritable/OnError OR
  // OnEvents.
  void OnWritable(FileDescriptor fd, std::function<void()> callback);

  // Registers a function to be called when the configured events occur on fd.
  // The function is passed an argument containing the events which occurred.
  // Configure events to call this function for using SetEvents.
  //
  // A fd may be registered exclusively with OnReadable/OnWritable/OnError OR
  // OnEvents.
  void OnEvents(FileDescriptor fd, std::function<void(uint32_t)> callback);

  // Removes fd from the event loop.
  // All Fds must be cleaned up before this class is destroyed.
  //
  // This applies to fds registered with any functions.
  void DeleteFd(FileDescriptor fd);

  // Removes a closed fd.  When fds are closed, they are automatically
  // unregistered by the kernel.  But we need to clean up any state here.
  // All Fds must be cleaned up before this class is destroyed.
  void ForgetClosedFd(FileDescriptor fd);

  // Enables calling the existing function registered for fd when it becomes
  // writable.
  //
  // This is only for fds registered using OnWritable, not OnEvents.
  void EnableWritable(FileDescriptor fd);

  // Disables calling the existing function registered for fd when it becomes
  // writable.
  //
  // This is only for fds registered using OnWritable, not OnEvents.
  void DisableWritable(FileDescriptor fd);

  // Sets the epoll events for the given fd.  Be careful using this with
  // OnReadable/OnWritable/OnError: enabled events which fire with no handler
  // registered will result in a crash.
  //
  // This is only for fds registered using OnEvents.
  void SetEvents(FileDescriptor fd, uint32_t events);

  // Registers a ThreadSignalReceiver for wakeups.  At most one receiver may
  // be registered at a time (enforced with a CHECK): every
  // ThreadSignalSender wakeup arrives via the same signal, so multiple
  // receivers on one loop could not tell their wakeups apart anyway.
  //
  // Wakeups coalesce: every pending signal is consumed first, then the
  // callback is invoked exactly once.  The callback has to check all of
  // its sources anyway, so per-signal invocations would only be redundant
  // scans.  The ordering is the guarantee to rely on -- no signal is ever
  // consumed after the last callback invocation, so a wakeup that lands
  // during (or after) a callback always produces another callback on a
  // later Poll(); a consumed wakeup is never the one that got away.
  //
  // Lifetime: the receiver must stay valid while registered.  Once
  // UnregisterThreadSignalReceiver() returns, the Aio never touches the
  // receiver again -- the caller may destroy it immediately, and a new
  // receiver registered afterward owns every subsequent wakeup (a
  // predecessor's leftover kernel traffic is discarded without consuming
  // anything).
  void RegisterThreadSignalReceiver(ipc_lib::ThreadSignalReceiver *receiver,
                                    std::function<void()> callback);
  void UnregisterThreadSignalReceiver(ipc_lib::ThreadSignalReceiver *receiver);

  // Consumes/drains all pending signals for the registered
  // ThreadSignalReceiver.
  void ConsumeThreadSignalReceiver(ipc_lib::ThreadSignalReceiver *receiver);

 private:
  struct Impl;
  friend class IoUringImpl;

  std::unique_ptr<Impl> impl_;
};

}  // namespace aos

#endif  // AOS_EVENTS_AIO_H_
