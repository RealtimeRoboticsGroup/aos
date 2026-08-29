#ifndef NOMINMAX
#define NOMINMAX
#endif
#include "aos/ipc_lib/thread_signal.h"

#include <windows.h>

#include <cstdio>

#include "absl/log/absl_check.h"

namespace aos::ipc_lib {
namespace {

// Both halves derive the Event name from the target (pid, tid) pair, so a
// sender in another process can find the receiver's Event without sharing a
// handle.
void WakeupEventName(pid_t pid, pid_t tid, char *name, size_t size) {
  int len = snprintf(name, size, "Local\\aos-wakeup-%d-%d",
                     static_cast<int>(pid), static_cast<int>(tid));
  ABSL_CHECK(len > 0 && len < static_cast<int>(size));
}

}  // namespace

// The sender holds nothing.  It opens the receiver's Event by name on each
// Signal(), and only the receiver creates the Event.
ThreadSignalSender::ThreadSignalSender() = default;
ThreadSignalSender::~ThreadSignalSender() = default;
ThreadSignalSender::ThreadSignalSender(ThreadSignalSender &&) = default;
ThreadSignalSender &ThreadSignalSender::operator=(ThreadSignalSender &&) =
    default;

// TODO(austin): Keep the handles this opens, keyed by (pid, tid) in a table
// sized to the queue's watchers, so a repeat Signal() is one SetEvent() rather
// than a lookup by name every time.
void ThreadSignalSender::Signal(pid_t pid, pid_t tid) {
  char name[64];
  WakeupEventName(pid, tid, name, sizeof(name));
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

ThreadSignalReceiver::ThreadSignalReceiver() {
  char name[64];
  WakeupEventName(GetCurrentProcessId(), GetCurrentThreadId(), name,
                  sizeof(name));
  // Manual-reset, and initially unsignaled.  An auto-reset Event is consumed
  // by whoever waits on it, and the waiter here is the kernel:
  // RegisterWaitForSingleObject clears the bit before any of our code runs.
  // The wakeup would then exist only as a completion in flight inside this
  // process, and nothing in the kernel could recover it if that completion
  // were dropped.  A signalfd works the other way: the signal stays in the fd
  // until we read it, which is what makes the consume-then-notify guarantee
  // in aio.h possible.  Manual-reset gives us that same control.
  // ConsumeWakeup() clears the bit, in the place read() has on Linux, and a
  // lost completion leaves the bit set for the next wait to deliver again.
  event_handle_ = CreateEventA(NULL, TRUE, FALSE, name);
  ABSL_PCHECK(event_handle_ != NULL) << "CreateEventA(" << name << ") failed";
}

ThreadSignalReceiver::~ThreadSignalReceiver() {
  if (event_handle_ != NULL) {
    ABSL_PCHECK(CloseHandle(event_handle_)) << "CloseHandle failed";
  }
}

bool ThreadSignalReceiver::ConsumeWakeup() {
  // The Event is manual-reset, so the wait which delivered the wakeup left
  // the bit set: this is the drain, not a belt-and-braces second one.  It is
  // the analogue of read()ing the signalfd until it is empty -- every wakeup
  // pending at this instant is consumed, and one arriving after it stays set
  // and produces a later notification rather than being lost.
  ABSL_PCHECK(ResetEvent(event_handle_)) << "ResetEvent failed";
  // Whether the Event was set is not something ResetEvent() reports.
  return true;
}

void ThreadSignalReceiver::LeaveSignalBlocked() {
  // Nothing to do: wakeups arrive as an Event rather than a signal, so there is
  // no process-wide disposition that could kill us after destruction.
}

}  // namespace aos::ipc_lib
