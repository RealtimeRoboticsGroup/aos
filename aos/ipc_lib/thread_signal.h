#ifndef AOS_IPC_LIB_THREAD_SIGNAL_H_
#define AOS_IPC_LIB_THREAD_SIGNAL_H_

#include "aos/realtime.h"

#ifdef _WIN32
#include <windows.h>
#else
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

class ThreadSignal {
 public:
  ThreadSignal();
  ~ThreadSignal();

  ThreadSignal(const ThreadSignal &) = delete;
  ThreadSignal &operator=(const ThreadSignal &) = delete;
  ThreadSignal(ThreadSignal &&other);
  ThreadSignal &operator=(ThreadSignal &&other);

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

}  // namespace aos::ipc_lib

#endif  // AOS_IPC_LIB_THREAD_SIGNAL_H_
