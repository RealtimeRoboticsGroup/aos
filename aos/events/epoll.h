#ifndef AOS_EVENTS_EPOLL_H_
#define AOS_EVENTS_EPOLL_H_

#include <stdint.h>

#include <functional>
#include <memory>
#include <vector>

#include "aos/time/time.h"

namespace aos {

class Aio;

// Class to wrap epoll and call a callback when an event happens.
class EPoll {
 public:
  // Owns its Aio.
  EPoll();
  // Borrows aio, which must outlive this EPoll.  Every call forwards to it:
  // the On*() handlers are registered on aio itself, and Run(), Poll(),
  // Quit() and should_run() drive and report on the whole shared loop, not
  // just this EPoll's registrations.  Destroying this EPoll neither quits aio
  // nor removes what was registered through it; DeleteFd() those first.  A
  // BeforeWait() hook cannot be removed, so one registered through a borrowed
  // EPoll lives as long as aio does.
  explicit EPoll(Aio *aio);
  ~EPoll();
  EPoll(const EPoll &) = delete;
  EPoll &operator=(const EPoll &) = delete;
  EPoll(EPoll &&) = delete;
  EPoll &operator=(EPoll &&) = delete;

  // Runs until Quit() is called.
  void Run();

  // Consumes a single event. Blocks indefinitely if block is true, or
  // does not block at all. Returns true if an event was consumed, and false on
  // any retryable error or if no events are available. Dies fatally on
  // non-retryable errors.
  bool Poll(bool block);

  // Quits.  Async safe.
  void Quit();

  // Adds a function which will be called before waiting.
  void BeforeWait(std::function<void()> function);

  // Registers a function to be called when the fd is readable.
  // Only one function may be registered for readability on each fd.
  //
  // A fd may be registered exclusively with OnReadable/OnWritable/OnError OR
  // OnEvents.
  void OnReadable(int fd, ::std::function<void()> function);

  // Registers a function to be called when the fd has an error.
  // Only one function may be registered for errors on each fd.
  //
  // A fd may be registered exclusively with OnReadable/OnWritable/OnError OR
  // OnEvents.
  void OnError(int fd, ::std::function<void()> function);

  // Registers a function to be called when the fd is writable.
  // Only one function may be registered for writability on each fd.
  //
  // A fd may be registered exclusively with OnReadable/OnWritable/OnError OR
  // OnEvents.
  void OnWritable(int fd, ::std::function<void()> function);

  // Registers a function to be called when the configured events occur on fd.
  // The function is passed an argument containing the events which occurred.
  // Configure events to call this function for using SetEvents.
  //
  // The event encoding is documented on Aio::OnEvents().  One delta from
  // the pre-Aio EPoll: a hangup (EPOLLHUP) is delivered as the error bit
  // (0x08), where it used to be dropped and the callback invoked with 0.
  //
  // A fd may be registered exclusively with OnReadable/OnWritable/OnError OR
  // OnEvents.
  void OnEvents(int fd, ::std::function<void(uint32_t)> function);

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

  // Sets the events to deliver to fd's OnEvents function -- see
  // Aio::SetEvents().
  //
  // This is only for fds registered using OnEvents (enforced with a
  // CHECK, where the pre-Aio EPoll accepted it on any fd and crashed at
  // dispatch instead).
  void SetEvents(int fd, uint32_t events);

  // Returns whether we're currently running. This changes to false when we
  // start draining events to finish.
  bool should_run() const;

  // The Aio every call forwards to: this EPoll's own, or the one it borrows.
  Aio *aio() { return aio_; }

 private:
  std::unique_ptr<Aio> owned_aio_;
  Aio *aio_ = nullptr;
};

}  // namespace aos

#endif  // AOS_EVENTS_EPOLL_H_
