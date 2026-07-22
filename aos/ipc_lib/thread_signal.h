#ifndef AOS_IPC_LIB_THREAD_SIGNAL_H_
#define AOS_IPC_LIB_THREAD_SIGNAL_H_

#include <functional>
#include <memory>

#include "aos/realtime.h"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#elif defined(__APPLE__)
#include <signal.h>
#else
#include <signal.h>
#include <sys/signalfd.h>
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

#ifdef _WIN32
  HANDLE event_handle() const { return event_handle_; }
#endif

 private:
#ifdef _WIN32
  HANDLE event_handle_ = NULL;
#else
  const pid_t pid_;
  const uid_t uid_;
#endif
};

// The receiving half of the thread wakeup mechanism.
//
// This wraps the platform primitive used to receive kWakeupSignal (a signalfd
// on Linux, an ignored signal + kqueue EVFILT_SIGNAL on macOS) and exposes it
// as an fd, so callers can watch it with their event loop (e.g. via
// Aio::RegisterThreadSignalReceiver).  It replaces the old separate SignalFd
// class -- if raw signalfd-of-arbitrary-signals behavior is ever needed again,
// add a dedicated (Linux-only) class for it rather than generalizing this one.
//
// Windows support is added later, once the Aio loop it plugs into exists.
#ifndef _WIN32
class ThreadSignalReceiver {
 public:
  ThreadSignalReceiver();
  ~ThreadSignalReceiver();

  ThreadSignalReceiver(const ThreadSignalReceiver &) = delete;
  ThreadSignalReceiver &operator=(const ThreadSignalReceiver &) = delete;

  // The signalfd's file descriptor (Linux).  -1 on platforms that don't back
  // the receiver with an fd (macOS).
#if defined(__APPLE__)
  int fd() const { return -1; }
#else
  int fd() const { return fd_; }
#endif

  // Drains any pending wakeups so we don't immediately wake again.
  void ConsumeWakeup();

  // Leaves kWakeupSignal blocked (Linux) / ignored (macOS) when this receiver
  // is destroyed, rather than restoring the previous disposition.  This closes
  // a shutdown race: a sender can signal us between when we stop being watched
  // and when we're destroyed, and without this the stray signal would hit the
  // default action and kill the process.
  void LeaveSignalBlocked();

 private:
  // Nothing backs the receiver on macOS, so it has no state at all.
#if !defined(__APPLE__)
  // Reads a single signalfd_siginfo.  On error/EAGAIN the resulting ssi_signo
  // is 0.
  signalfd_siginfo Read();

  int fd_ = -1;
  // Whether the destructor unblocks kWakeupSignal.  False if it was already
  // blocked before we got here, or if LeaveSignalBlocked() was called.
  bool should_unblock_ = true;
#endif
};
#endif  // !_WIN32

}  // namespace aos::ipc_lib

#endif  // AOS_IPC_LIB_THREAD_SIGNAL_H_
