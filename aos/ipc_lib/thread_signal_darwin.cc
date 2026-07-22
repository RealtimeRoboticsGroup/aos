#include <errno.h>
#include <signal.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstring>
#include <mutex>
#include <ostream>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/ipc_lib/thread_signal.h"

namespace aos::ipc_lib {

ThreadSignalSender::ThreadSignalSender() : pid_(getpid()), uid_(getuid()) {}
ThreadSignalSender::~ThreadSignalSender() {}
ThreadSignalSender::ThreadSignalSender(ThreadSignalSender && /*other*/)
    : pid_(getpid()), uid_(getuid()) {}
ThreadSignalSender &ThreadSignalSender::operator=(ThreadSignalSender &&) {
  return *this;
}

void ThreadSignalSender::Signal(pid_t pid, pid_t tid) {
  (void)tid;
  // macOS can't target a specific thread, so we signal the whole process.
  // ESRCH: the target process already exited -- a benign race.  Anything else
  // is a real problem and should be loud rather than silently dropping the
  // wakeup.
  if (kill(pid, kWakeupSignal) < 0 && errno != ESRCH) {
    ABSL_PLOG(FATAL) << "kill(" << pid << ") failed";
  }
}

namespace {

// Manages the process-wide SIG_IGN disposition of kWakeupSignal, which is
// shared by every ThreadSignalReceiver (the disposition is process-wide, not
// per-thread).  Installs SIG_IGN when the first receiver registers and restores
// the original disposition only when the last one unregisters.
//
// Restoring per-receiver would be a bug: with receivers A then B alive,
// destroying A first would restore SIG_DFL while B is still using the signal,
// so B's next wakeup would hit the default action and terminate the process.
//
// A single process-wide instance is reached via Get().
class WakeupSignalDisposition {
 public:
  static WakeupSignalDisposition &Get() {
    static WakeupSignalDisposition instance;
    return instance;
  }

  // Adds a receiver, ignoring kWakeupSignal on the first one.
  void Register() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (receiver_count_ == 0) {
      struct sigaction sa;
      std::memset(&sa, 0, sizeof(sa));
      sa.sa_handler = SIG_IGN;
      sigemptyset(&sa.sa_mask);
      sa.sa_flags = 0;
      ABSL_PCHECK(sigaction(kWakeupSignal, &sa, &original_sa_) == 0);
      // A fresh generation of receivers defaults to restoring again, even if a
      // previous generation left the signal ignored via LeaveBlocked().
      restore_on_last_ = true;
    }
    ++receiver_count_;
  }

  // Removes a receiver, restoring the original disposition after the last one
  // (unless LeaveBlocked() was called).
  void Unregister() {
    std::lock_guard<std::mutex> lock(mutex_);
    --receiver_count_;
    if (receiver_count_ == 0 && restore_on_last_) {
      sigaction(kWakeupSignal, &original_sa_, nullptr);
    }
  }

  // Leaves kWakeupSignal ignored once the last receiver exits, rather than
  // restoring, so a late signal can't hit the default action and kill the
  // process.
  void LeaveBlocked() {
    std::lock_guard<std::mutex> lock(mutex_);
    restore_on_last_ = false;
  }

 private:
  WakeupSignalDisposition() = default;

  std::mutex mutex_;
  int receiver_count_ = 0;        // Number of live receivers.
  bool restore_on_last_ = true;   // Whether the last receiver restores.
  struct sigaction original_sa_;  // Disposition before the first receiver.
};

}  // namespace

// macOS doesn't have signalfd; the receiver just ignores kWakeupSignal
// process-wide (the Aio kqueue backend watches it via EVFILT_SIGNAL).
ThreadSignalReceiver::ThreadSignalReceiver() {
  WakeupSignalDisposition::Get().Register();
}

ThreadSignalReceiver::~ThreadSignalReceiver() {
  WakeupSignalDisposition::Get().Unregister();
}

void ThreadSignalReceiver::ConsumeWakeup() {
  // kqueue automatically consumes the signal event when returning it.
}

void ThreadSignalReceiver::LeaveSignalBlocked() {
  WakeupSignalDisposition::Get().LeaveBlocked();
}

}  // namespace aos::ipc_lib
