#include <windows.h>

#include "absl/log/absl_check.h"
#include "absl/strings/str_cat.h"

#include "aos/ipc_lib/thread_signal.h"

namespace aos::ipc_lib {

ThreadSignal::ThreadSignal() {
  const pid_t pid = GetCurrentProcessId();
  const pid_t tid = GetCurrentThreadId();
  std::string name = absl::StrCat("Local\\aos-wakeup-", pid, "-", tid);
  event_handle_ = CreateEventA(NULL, FALSE, FALSE, name.c_str());
  ABSL_PCHECK(event_handle_ != NULL) << "CreateEventA failed";
}

ThreadSignal::~ThreadSignal() {
  if (event_handle_ != NULL) {
    CloseHandle(event_handle_);
  }
}

ThreadSignal::ThreadSignal(ThreadSignal &&other) {
  event_handle_ = other.event_handle_;
  other.event_handle_ = NULL;
}

ThreadSignal &ThreadSignal::operator=(ThreadSignal &&other) {
  std::swap(event_handle_, other.event_handle_);
  return *this;
}

void ThreadSignal::Signal(pid_t pid, pid_t tid) {
  std::string name = absl::StrCat("Local\\aos-wakeup-", pid, "-", tid);
  HANDLE hEvent = OpenEventA(EVENT_MODIFY_STATE, FALSE, name.c_str());
  if (hEvent == NULL) {
    const DWORD error = GetLastError();
    // The target thread may have already exited and closed its event before we
    // got here.  That's a benign race we can't avoid (the analog of ESRCH on
    // Linux, which we likewise ignore).  Anything else -- notably a permissions
    // error -- is a real problem and should be loud rather than silently
    // dropping the wakeup.
    ABSL_PCHECK(error == ERROR_FILE_NOT_FOUND)
        << "OpenEventA(" << name << ") failed with " << error;
    return;
  }
  ABSL_PCHECK(SetEvent(hEvent)) << "SetEvent(" << name << ") failed";
  ABSL_PCHECK(CloseHandle(hEvent)) << "CloseHandle failed";
}

}  // namespace aos::ipc_lib
