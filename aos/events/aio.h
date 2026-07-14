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
class SignalFd;
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
//    - `status` is an error with message "io_uring error".
//    - `result` is set to the positive errno value representing the OS-level
//      error (e.g., EPIPE, EBADF, EINVAL).
//
// Note: The failure status message is a static literal ("io_uring error")
// to ensure that error propagation inside real-time threads does not trigger
// dynamic memory allocation (which is prohibited under ScopedRealtime).
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

  // The following fields are internal state managed entirely by the Aio
  // implementation to track absolute timeout deadlines for OS-level timers.
  // Callers must not read or modify these fields.
  int64_t tv_sec = 0;
  int64_t tv_nsec = 0;
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
// Aio also supports the primatives needed to implement the Epoll class using
// it.
//
// Constraints:
// 1. Thread Safety: Aio is designed to be driven by a single-threaded event
//    loop.  All scheduling (e.g., AsyncRead, AsyncWrite, AsyncTimer) and
//    cancellation (Cancel) operations must be invoked from the thread that
//    drives the loop via Poll() or Run().  The only exceptions are Wakeup()
//    and Quit(), which are thread-safe and may be called from any thread.
// 2. Request Lifetime: The caller-supplied AsyncRequest object must remain
//    valid and allocated in memory from the time it is submitted until its
//    corresponding CompletionCallback is executed.  Destroying the Aio
//    instance will automatically cancel all active requests.
// 3. Callback Reentrancy: Completion callbacks are executed synchronously
//    during Poll() and should avoid blocking the event loop thread.
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
  bool Poll(bool block);

  // Signals the loop to terminate execution. Async-safe.
  void Quit();

  // Interrupts a blocking Poll() call, forcing it to wake up immediately.
  void Wakeup();

  // Schedules an asynchronous read on a file descriptor.
  void AsyncRead(int fd, std::span<char> buffer, AsyncRequest *request);

  // Schedules an asynchronous write to a file descriptor.
  void AsyncWrite(int fd, std::span<const char> buffer, AsyncRequest *request);

  class Timer {
   public:
    Timer(Aio *aio);
    ~Timer();

    Timer(const Timer &) = delete;
    Timer &operator=(const Timer &) = delete;
    Timer(Timer &&) = delete;
    Timer &operator=(Timer &&) = delete;

    // Schedules a timer execution.  The callback fires at or after the given
    // deadline. If a timer is already scheduled, it is canceled first.
    void Schedule(aos::monotonic_clock::time_point deadline,
                  CompletionCallback callback, void *context = nullptr) {
      Schedule(deadline, aos::monotonic_clock::zero(), callback, context);
    }
    void Schedule(aos::monotonic_clock::time_point deadline,
                  aos::monotonic_clock::duration interval,
                  CompletionCallback callback, void *context = nullptr);

    // Cancels the active active timer.  The callback will be called with a
    // status of Canceled.
    void Cancel();

    // Returns true if the timer has a pending scheduling request.
    bool IsPending() const;

   private:
    std::unique_ptr<TimerState> state_;
  };

  // Cancels a pending request.  This submits an asynchronous cancellation
  // request to the multiplexer.  The original request will then complete
  // asynchronously through Poll, at which point its callback is invoked with a
  // status of Canceled.
  void Cancel(AsyncRequest *request);

  // Registers a function to be executed just before blocking on events.
  void BeforeWait(std::function<void()> function);

  // Registers a function to be called when the fd is readable.
  // Only one function may be registered for readability on each fd.
  //
  // A fd may be registered exclusively with OnReadable/OnWritable/OnError OR
  // OnEvents.
  void OnReadable(int fd, std::function<void()> callback);

  // Registers a function to be called when the fd has an error.
  // Only one function may be registered for errors on each fd.
  //
  // A fd may be registered exclusively with OnReadable/OnWritable/OnError OR
  // OnEvents.
  void OnError(int fd, std::function<void()> callback);

  // Registers a function to be called when the fd is writable.
  // Only one function may be registered for writability on each fd.
  //
  // A fd may be registered exclusively with OnReadable/OnWritable/OnError OR
  // OnEvents.
  void OnWritable(int fd, std::function<void()> callback);

  // Registers a function to be called when the configured events occur on fd.
  // The function is passed an argument containing the events which occurred.
  // Configure events to call this function for using SetEvents.
  //
  // A fd may be registered exclusively with OnReadable/OnWritable/OnError OR
  // OnEvents.
  void OnEvents(int fd, std::function<void(uint32_t)> callback);

  // Removes fd from the event loop.
  // All Fds must be cleaned up before this class is destroyed.
  //
  // This applies to fds registered with any functions.
  void DeleteFd(int fd);

  // Removes a closed fd.  When fds are closed, they are automatically
  // unregistered by the kernel.  But we need to clean up any state here.
  // All Fds must be cleaned up before this class is destroyed.
  void ForgetClosedFd(int fd);

  // Enables calling the existing function registered for fd when it becomes
  // writable.
  //
  // This is only for fds registered using OnWritable, not OnEvents.
  void EnableWritable(int fd);

  // Disables calling the existing function registered for fd when it becomes
  // writable.
  //
  // This is only for fds registered using OnWritable, not OnEvents.
  void DisableWritable(int fd);

  // Sets the epoll events for the given fd.  Be careful using this with
  // OnReadable/OnWritable/OnError: enabled events which fire with no handler
  // registered will result in a crash.
  //
  // This is only for fds registered using OnEvents.
  void SetEvents(int fd, uint32_t events);

  // Registers a SignalFd for wakeups.
  void RegisterSignalFd(ipc_lib::SignalFd *sfd, std::function<void()> callback);
  void UnregisterSignalFd(ipc_lib::SignalFd *sfd);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace aos

#endif  // AOS_EVENTS_AIO_H_
