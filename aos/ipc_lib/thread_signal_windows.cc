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

// Claims the calling thread for one receiver, and returns the handle that
// holds the claim, or NULL if another receiver already holds it.
//
// The claim is a named object of its own rather than the wakeup Event,
// because the Event's existence says nothing about receivers:
// ThreadSignalSender::Signal() opens it by name for as long as it takes to
// set it, so it can outlive the receiver that made it by that long, and a
// new receiver on the same thread in that window found it and was refused
// (NewReceiverWhileASenderHoldsTheOldEvent).  Nothing but a receiver ever
// creates or opens this one, so it exists exactly while a receiver holds it.
HANDLE ClaimThread() {
  char name[64];
  const int len = snprintf(name, sizeof(name), "Local\\aos-wakeup-owner-%d-%d",
                           static_cast<int>(GetCurrentProcessId()),
                           static_cast<int>(GetCurrentThreadId()));
  ABSL_CHECK(len > 0 && len < static_cast<int>(sizeof(name)));
  const HANDLE claim = CreateEventA(NULL, TRUE, FALSE, name);
  ABSL_CHECK(claim != NULL)
      << "CreateEventA(" << name << ") failed: error " << GetLastError();
  if (GetLastError() == ERROR_ALREADY_EXISTS) {
    ABSL_CHECK(CloseHandle(claim))
        << "CloseHandle failed: error " << GetLastError();
    return NULL;
  }
  return claim;
}

// The calling thread's wakeup Event.  It is created, or opened if a sender
// still has the last one open.  Manual-reset, and initially unsignaled when
// new.  An old one may still be set, which is only a spurious wakeup.
HANDLE OpenWakeupEvent() {
  char name[64];
  WakeupEventName(GetCurrentProcessId(), GetCurrentThreadId(), name,
                  sizeof(name));
  const HANDLE event = CreateEventA(NULL, TRUE, FALSE, name);
  ABSL_CHECK(event != NULL)
      << "CreateEventA(" << name << ") failed: error " << GetLastError();
  return event;
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
    ABSL_CHECK(error == ERROR_FILE_NOT_FOUND)
        << "OpenEventA(" << name << ") failed with " << error;
    return;
  }
  ABSL_CHECK(SetEvent(hEvent))
      << "SetEvent(" << name << ") failed: error " << GetLastError();
  ABSL_CHECK(CloseHandle(hEvent))
      << "CloseHandle failed: error " << GetLastError();
}

ThreadSignalReceiver::ThreadSignalReceiver() {
  // Manual-reset; see OpenWakeupEvent().  An auto-reset Event is consumed
  // by whoever waits on it, and the waiter here is the kernel: the wait
  // behind the Aio's wait completion packet clears the bit before any of our
  // code runs.
  // The wakeup would then exist only as a completion in flight inside this
  // process, and nothing in the kernel could recover it if that completion
  // were dropped.  A signalfd works the other way: the signal stays in the fd
  // until we read it, which is what makes the consume-then-notify guarantee
  // in aio.h possible.  Manual-reset gives us that same control.
  // ConsumeWakeup() clears the bit, in the place read() has on Linux, and a
  // lost completion leaves the bit set for the next wait to deliver again.
  //
  // Named for the thread building it, which is a guess; BindToCurrentThread()
  // knows.  The thread is not claimed here, only by BindToCurrentThread(): a
  // receiver that is only built, say as a fixture's member or for a loop on
  // another thread, must not keep the next one off this thread
  // (IdleReceiverDoesNotHoldItsThread).  Two receivers built on one thread
  // share this Event until one of them is bound somewhere, which is harmless,
  // since the claim keeps them from both being bound here at once.
  fd_ = OpenWakeupEvent();
  bound_tid_ = static_cast<pid_t>(GetCurrentThreadId());
}

ThreadSignalReceiver::~ThreadSignalReceiver() {
  ABSL_CHECK(CloseHandle(fd_))
      << "CloseHandle failed: error " << GetLastError();
  ReleaseThread();
}

void ThreadSignalReceiver::BindToCurrentThread() {
  // Why this platform binds at all: ThreadSignalSender addresses a (pid, tid)
  // pair, and the only rendezvous between the two halves here is the Event's
  // name, which embeds the tid.  A receiver constructed on one thread and
  // polled on another would listen on a name nobody signals, and Signal()
  // would read the resulting ERROR_FILE_NOT_FOUND as the benign
  // thread-already-exited race and drop the wakeup silently.  The POSIX
  // backends resolve the target when the signal is sent and do not care.
  //
  // Re-create the Event under the *calling* thread's name, but only when the
  // thread actually changed.  Not to save the handle, which costs nothing,
  // but because a wakeup already on its way carries the old one: the backend
  // waits on the Event through a packet keyed by its handle, and re-creating
  // on a re-registration from the same thread returns a fresh value for the
  // same Event.  SuccessorReceiverOwnsPendingWakeups is built around exactly
  // such an in-flight wakeup and catches it.
  //
  // One receiver per thread is kept by a claim on the thread, not by the
  // Event; see ClaimThread().  The claim lasts until ReleaseThread().
  const pid_t tid = static_cast<pid_t>(GetCurrentThreadId());
  if (owner_ != nullptr && bound_tid_ == tid) {
    return;
  }
  const HANDLE claim = ClaimThread();
  ABSL_CHECK(claim != NULL)
      << "Another ThreadSignalReceiver is already bound to this thread; only "
         "one may be bound at a time (see BindToCurrentThread)";
  ReleaseThread();
  owner_ = claim;
  if (bound_tid_ != tid) {
    const HANDLE handle = OpenWakeupEvent();
    ABSL_CHECK(CloseHandle(fd_))
        << "CloseHandle failed: error " << GetLastError();
    fd_ = handle;
    bound_tid_ = tid;
  }
}

void ThreadSignalReceiver::ReleaseThread() {
  // The claim is a handle, so any thread can close it, and the Event stays
  // open: a successor registered on this thread opens the same one by name,
  // and finds any wakeup a sender set on it in the meantime.
  if (owner_ != nullptr) {
    ABSL_CHECK(CloseHandle(owner_))
        << "CloseHandle failed: error " << GetLastError();
    owner_ = nullptr;
  }
}

bool ThreadSignalReceiver::ConsumeWakeup() {
  // The Event is manual-reset, so the wait which delivered the wakeup left
  // the bit set: this is the drain, not a belt-and-braces second one.  It is
  // the analogue of read()ing the signalfd until it is empty -- every wakeup
  // pending at this instant is consumed, and one arriving after it stays set
  // and produces a later notification rather than being lost.
  ABSL_CHECK(ResetEvent(fd_)) << "ResetEvent failed: error " << GetLastError();
  // Whether the Event was set is not something ResetEvent() reports.
  return true;
}

void ThreadSignalReceiver::LeaveSignalBlocked() {
  // Nothing to do: wakeups arrive as an Event rather than a signal, so there is
  // no process-wide disposition that could kill us after destruction.
}

}  // namespace aos::ipc_lib
