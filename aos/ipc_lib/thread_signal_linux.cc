#include "aos/ipc_lib/thread_signal.h"

#include <errno.h>
#include <string.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <unistd.h>

#include <csignal>
#include <ostream>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

namespace aos::ipc_lib {

ThreadSignalSender::ThreadSignalSender() : pid_(getpid()), uid_(getuid()) {}
ThreadSignalSender::~ThreadSignalSender() {}
ThreadSignalSender::ThreadSignalSender(ThreadSignalSender && /*other*/)
    : pid_(getpid()), uid_(getuid()) {}
ThreadSignalSender &ThreadSignalSender::operator=(ThreadSignalSender &&) {
  return *this;
}

void ThreadSignalSender::Signal(pid_t pid, pid_t tid) {
  siginfo_t uinfo;
  memset(&uinfo, 0, sizeof(uinfo));
  uinfo.si_code = SI_QUEUE;
  uinfo.si_pid = pid_;
  uinfo.si_uid = uid_;
  uinfo.si_value.sival_int = 0;
  if (syscall(SYS_rt_tgsigqueueinfo, pid, tid, kWakeupSignal, &uinfo) < 0 &&
      errno != ESRCH) {
    // ESRCH: the target thread already exited -- a benign race we can't avoid.
    //
    // Everything else, including EAGAIN, is fatal.  EAGAIN means
    // RLIMIT_SIGPENDING (the cap on outstanding queued realtime signals) was
    // hit for this user.  That cap is a per-user aggregate across every
    // pending realtime signal, not scoped to kWakeupSignal or to this target
    // thread, so it does NOT mean the target already has a wakeup pending --
    // some unrelated thread saturating the limit can cause this send to be
    // dropped outright, leaving the target waiting with no record that
    // anything went wrong.  See
    // https://github.com/RealtimeRoboticsGroup/aos/issues/14 for a real
    // instance of this happening.  A loud crash here is preferable to a
    // silently dropped wakeup that hangs a thread indefinitely.
    ABSL_PLOG(FATAL) << "rt_tgsigqueueinfo(" << pid << ", " << tid
                     << ") failed";
  }
}

namespace {

// Builds a signal mask containing just kWakeupSignal.
sigset_t WakeupSignalMask() {
  sigset_t mask;
  ABSL_CHECK_EQ(0, sigemptyset(&mask));
  ABSL_CHECK_EQ(0, sigaddset(&mask, kWakeupSignal));
  return mask;
}

}  // namespace

ThreadSignalReceiver::ThreadSignalReceiver() {
  const sigset_t mask = WakeupSignalMask();
  // Block kWakeupSignal, then create the signalfd.  A signalfd is inert while
  // its signal is unblocked -- the signal still takes its default action (fatal
  // for a realtime signal), fd or not -- so it's the block, not the fd, that
  // protects us.  Blocking first only matters for the narrow gap between the
  // two calls: a wakeup landing there pends and is later drained by the fd
  // (created below) rather than being fatal, which is the sub-window the
  // reverse order left open.
  //
  // Neither order closes the window *before* this block; a wakeup that early is
  // fatal regardless.  That's benign because nothing directs a wakeup at this
  // thread until it's registered with an event loop, which is after
  // construction.
  sigset_t old_mask;
  ABSL_CHECK_EQ(0, pthread_sigmask(SIG_BLOCK, &mask, &old_mask));
  // Now build the signalfd.  Make it nonblocking so it works well with an event
  // loop, and have it close on exec.  signalfd() returns any non-negative fd on
  // success (including 0, e.g. if stdin is closed) and -1 on error.
  ABSL_PCHECK((fd_ = signalfd(-1, &mask, SFD_NONBLOCK | SFD_CLOEXEC)) >= 0);
  // If kWakeupSignal was already blocked before we blocked it, leave it blocked
  // on destruction.
  if (sigismember(&old_mask, kWakeupSignal)) {
    should_unblock_ = false;
  }
}

ThreadSignalReceiver::~ThreadSignalReceiver() {
  // Unwind the constructor.  Unblock kWakeupSignal and close the fd.
  if (should_unblock_) {
    const sigset_t mask = WakeupSignalMask();
    sigset_t old_mask;
    ABSL_CHECK_EQ(0, pthread_sigmask(SIG_UNBLOCK, &mask, &old_mask));
    // Verify nobody else unblocked it in the meantime.
    if (!sigismember(&old_mask, kWakeupSignal)) {
      ABSL_LOG(FATAL) << "Some other code unblocked our signal";
    }
  }
  ABSL_PCHECK(close(fd_) == 0);
}

signalfd_siginfo ThreadSignalReceiver::Read() {
  signalfd_siginfo result;
  const int ret =
      read(fd_, static_cast<void *>(&result), sizeof(signalfd_siginfo));
  // If we didn't get the right amount of data, signal that there was a problem
  // by setting the signal number to 0.
  if (ret != static_cast<int>(sizeof(signalfd_siginfo))) {
    result.ssi_signo = 0;
  } else {
    ABSL_CHECK_NE(0u, result.ssi_signo);
  }
  return result;
}

void ThreadSignalReceiver::ConsumeWakeup() {
  while (true) {
    signalfd_siginfo result = Read();
    if (result.ssi_signo == 0) {
      break;
    }
    ABSL_CHECK_EQ(result.ssi_signo, kWakeupSignal);
  }
}

void ThreadSignalReceiver::LeaveSignalBlocked() {
  // Skip the unblock on destruction, so kWakeupSignal stays blocked and a late
  // signal can't kill the process.
  should_unblock_ = false;
}

}  // namespace aos::ipc_lib
