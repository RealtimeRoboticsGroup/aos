#ifndef AOS_IPC_LIB_THREAD_SIGNAL_H_
#define AOS_IPC_LIB_THREAD_SIGNAL_H_

#include <functional>
#include <memory>

#include "aos/events/file_descriptor.h"
#include "aos/realtime.h"

#if !defined(_WIN32)
#include <signal.h>
#endif

namespace aos::ipc_lib {

#ifdef _WIN32
const static unsigned int kWakeupSignal = 0;
#elif defined(__APPLE__)
const static unsigned int kWakeupSignal = SIGUSR1;
#else
const static unsigned int kWakeupSignal = SIGRTMIN + 2;
#endif

// Sends a wakeup to a specific thread in a (possibly different) process.
class ThreadSignalSender {
 public:
  ThreadSignalSender();
  ~ThreadSignalSender();

  ThreadSignalSender(const ThreadSignalSender &) = delete;
  ThreadSignalSender &operator=(const ThreadSignalSender &) = delete;
  ThreadSignalSender(ThreadSignalSender &&other);
  ThreadSignalSender &operator=(ThreadSignalSender &&other);

  // Wakes up the target thread.
  // On Linux, this uses rt_tgsigqueueinfo.
  // On Windows, this opens and signals the thread's named event.
  // On macOS, since cross-process thread-directed signals are not supported,
  // this sends a process-directed signal (kill) using kWakeupSignal, which
  // may trigger spurious wakeups on other threads waiting in the same process.
  void Signal(pid_t pid, pid_t tid);

 private:
#ifndef _WIN32
  const pid_t pid_;
  const uid_t uid_;
#endif
};

// The receiving half of the thread wakeup mechanism.
//
// This wraps the platform primitive used to receive kWakeupSignal: a signalfd
// on Linux, an ignored signal plus kqueue EVFILT_SIGNAL on macOS, and a named
// Event on Windows.  Callers watch it with their event loop, e.g. via
// Aio::RegisterThreadSignalReceiver.  It replaces the old SignalFd class.  If
// a signalfd of arbitrary signals is ever needed again, add a separate
// Linux-only class for it rather than generalizing this one.
class ThreadSignalReceiver {
 public:
  ThreadSignalReceiver();
  ~ThreadSignalReceiver();

  ThreadSignalReceiver(const ThreadSignalReceiver &) = delete;
  ThreadSignalReceiver &operator=(const ThreadSignalReceiver &) = delete;

  // What an event loop waits on for this receiver's wakeups: the signalfd on
  // Linux, and on Windows the Event ThreadSignalSender::Signal() signals.
  // kInvalidFd on macOS, where kqueue watches the signal itself.
  FileDescriptor fd() const { return fd_; }

  // Points this receiver at the calling thread, the thread whose wakeups it
  // delivers.  Aio::RegisterThreadSignalReceiver() calls this, because
  // registration is the first moment the receiver learns which thread it
  // serves: it runs on the polling thread, while construction may not.
  //
  // Only Windows has anything to do.  A sender finds the receiver's Event by
  // a name built from the thread's id, so the Event has to carry the polling
  // thread's name, and a second receiver on that thread would share it.  That
  // CHECKs.  On Linux and macOS the signal is sent to the thread itself, so
  // there is nothing to point.
  void BindToCurrentThread();

  // Gives up the thread BindToCurrentThread() bound this receiver to, so
  // another can be bound there, whether or not this one is destroyed.
  // Aio::UnregisterThreadSignalReceiver() calls this.  What this receiver
  // waits on stays, so a wakeup sent before it is registered again is still
  // delivered.  Only Windows has anything to do, for the same reason as
  // BindToCurrentThread().  May be called from any thread.
  void ReleaseThread();

  // Drains any pending wakeups so we don't immediately wake again.  Returns
  // whether it consumed anything; a platform that cannot tell returns true.
  bool ConsumeWakeup();

  // Leaves kWakeupSignal blocked (Linux) / ignored (macOS) when this receiver
  // is destroyed, rather than restoring the previous disposition.  This closes
  // a shutdown race: a sender can signal us between when we stop being watched
  // and when we're destroyed, and without this the stray signal would hit the
  // default action and kill the process.  There is no signal disposition to
  // restore on Windows, so it does nothing there.
  void LeaveSignalBlocked();

 private:
  // See fd().  On Windows the Event the sender opens by name and signals.  It
  // is manual-reset, so a successful wait leaves it set and ConsumeWakeup()
  // clears it.  The constructor explains why we, not the wait, have to clear
  // it.
  FileDescriptor fd_ = kInvalidFd;

#if defined(_WIN32)
  // The thread fd_ is named for, so BindToCurrentThread() can tell a real
  // thread change from a re-registration on the same thread.
  pid_t bound_tid_ = 0;
  // A HANDLE (kept as void * so this header needs no windows.h): the named
  // object that says this receiver is bound_tid_'s one receiver, held from
  // BindToCurrentThread() until ReleaseThread().  Only receivers ever create
  // it, so its existing means another receiver holds the thread.  fd_ cannot
  // say that, because a sender opens it by name while it signals.
  void *owner_ = nullptr;
#elif !defined(__APPLE__)
  // Whether the destructor unblocks kWakeupSignal.  False if it was already
  // blocked before we got here, or if LeaveSignalBlocked() was called.
  bool should_unblock_ = true;
#endif
};

}  // namespace aos::ipc_lib

#endif  // AOS_IPC_LIB_THREAD_SIGNAL_H_
