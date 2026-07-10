#include <string.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <unistd.h>

#include "absl/log/absl_check.h"

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
  syscall(SYS_rt_tgsigqueueinfo, pid, tid, kWakeupSignal, &uinfo);
}

}  // namespace aos::ipc_lib
