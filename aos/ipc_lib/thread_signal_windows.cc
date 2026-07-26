#ifndef NOMINMAX
#define NOMINMAX
#endif
#include "aos/ipc_lib/thread_signal.h"

#include <windows.h>

#include <cstdio>

#include "absl/log/absl_check.h"

namespace aos::ipc_lib {

ThreadSignalSender::ThreadSignalSender() {
  const pid_t pid = GetCurrentProcessId();
  const pid_t tid = GetCurrentThreadId();
  char name[64];
  int len = snprintf(name, sizeof(name), "Local\\aos-wakeup-%d-%d",
                     static_cast<int>(pid), static_cast<int>(tid));
  ABSL_CHECK(len > 0 && len < static_cast<int>(sizeof(name)));
  event_handle_ = CreateEventA(NULL, FALSE, FALSE, name);
  ABSL_PCHECK(event_handle_ != NULL) << "CreateEventA failed";
}

ThreadSignalSender::~ThreadSignalSender() {
  if (event_handle_ != NULL) {
    CloseHandle(event_handle_);
  }
}

ThreadSignalSender::ThreadSignalSender(ThreadSignalSender &&other) {
  event_handle_ = other.event_handle_;
  other.event_handle_ = NULL;
}

ThreadSignalSender &ThreadSignalSender::operator=(ThreadSignalSender &&other) {
  std::swap(event_handle_, other.event_handle_);
  return *this;
}

void ThreadSignalSender::Signal(pid_t pid, pid_t tid) {
  char name[64];
  int len = snprintf(name, sizeof(name), "Local\\aos-wakeup-%d-%d",
                     static_cast<int>(pid), static_cast<int>(tid));
  ABSL_CHECK(len > 0 && len < static_cast<int>(sizeof(name)));
  HANDLE hEvent = OpenEventA(EVENT_MODIFY_STATE, FALSE, name);
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

// ThreadSignalReceiver on Windows is added later, alongside the Aio (IOCP) loop
// it registers its Event with.

}  // namespace aos::ipc_lib
