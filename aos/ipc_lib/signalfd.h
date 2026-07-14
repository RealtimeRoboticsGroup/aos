#ifndef AOS_IPC_LIB_SIGNALFD_H_
#define AOS_IPC_LIB_SIGNALFD_H_

#if defined(__linux__)
#include <signal.h>
#include <sys/signalfd.h>
#elif defined(_WIN32)
#include <windows.h>

#include "aos/ipc_lib/thread_signal.h"
#else
#include "aos/ipc_lib/thread_signal.h"
#endif

#include <initializer_list>

namespace aos::ipc_lib {

// Class to manage a signalfd.
class SignalFd {
 public:
  // Constructs a SignalFd for the provided list of signals.
  // Blocks the signals at the same time in this thread.
  SignalFd(::std::initializer_list<unsigned int> signal_list);

  SignalFd(const SignalFd &) = delete;
  SignalFd &operator=(const SignalFd &) = delete;
  ~SignalFd();

#if defined(__linux__)
  // Returns the file descriptor for the signalfd.
  int fd() { return fd_; }

  // Reads a signalfd_siginfo.  If there was an error, the resulting ssi_signo
  // will be 0.
  signalfd_siginfo Read();

  // Ensures the destructor will leave the specific signal blocked. This can be
  // helpful if the signal is sent asynchronously, such that it may arrive after
  // this object is destroyed, to ensure that doesn't kill the process.
  void LeaveSignalBlocked(unsigned int signal);
#elif defined(_WIN32)
  HANDLE event_handle() const { return signal_.event_handle(); }
  HANDLE fd() const { return signal_.event_handle(); }
#else
  // Fallback for non-Linux, non-Windows systems (e.g. Darwin stub)
  int fd() const { return -1; }
#endif

 private:
#if defined(__linux__)
  int fd_ = -1;

  // The signals we blocked in the constructor.
  sigset_t blocked_mask_;
#elif defined(_WIN32)
  ThreadSignal signal_;
#else
  ThreadSignal signal_;
#endif
};

}  // namespace aos::ipc_lib

#endif  // AOS_IPC_LIB_SIGNALFD_H_
