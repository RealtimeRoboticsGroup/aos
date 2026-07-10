#include <errno.h>
#include <string.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <unistd.h>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/ipc_lib/thread_signal.h"

namespace aos::ipc_lib {

ThreadSignal::ThreadSignal() : pid_(getpid()), uid_(getuid()) {}
ThreadSignal::~ThreadSignal() {}
ThreadSignal::ThreadSignal(ThreadSignal && /*other*/)
    : pid_(getpid()), uid_(getuid()) {}
ThreadSignal &ThreadSignal::operator=(ThreadSignal &&) { return *this; }

void ThreadSignal::Signal(pid_t pid, pid_t tid) {
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

}  // namespace aos::ipc_lib
