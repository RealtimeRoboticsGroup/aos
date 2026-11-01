#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include "aos/events/queue_event_loop.h"

#include <windows.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <vector>

#include "absl/log/absl_check.h"

namespace aos {
namespace {

// The console control handler, standing in for the SIGINT/SIGHUP/SIGTERM
// handlers on POSIX.  Windows runs it on a thread of its own, so a plain mutex
// is enough to keep it from racing registration.
//
// Installed by the first registration and never removed: with no event loop
// registered it passes every event on.  FreeConsole() and AllocConsole()
// drop the handlers a process had, so one that changes consoles after its
// first event loop registered loses this one.  Windows dispatches control
// events with its handler-list lock held, which SetConsoleCtrlHandler() takes
// too, so removing it while a dispatch waited for mutex_ deadlocked both.
//
// Installed once, it only runs ahead of the handlers that were there before
// it.  One added later runs first, and may keep the event from reaching it:
// the CRT's, for a program that first calls signal(SIGINT) after its first
// event loop, takes Ctrl-C for that program's handler.
class ConsoleCtrlHandler {
 public:
  // Gets the singleton.  It is never destroyed, because an event loop on
  // another thread can still be registered, or a control event can still
  // arrive, while the process exits.
  static ConsoleCtrlHandler *global() {
    static ConsoleCtrlHandler *const handler = new ConsoleCtrlHandler();
    return handler;
  }

  // Registers an event loop to receive Exit() calls.
  void Register(QueueEventLoop *event_loop) {
    // Outside mutex_, for the reason above.  A handler added later runs
    // before the ones already there, so this preempts the CRT's (and so
    // Python's) SIGINT emulation the way sigaction() replaces a handler on
    // POSIX.
    std::call_once(installed_, []() {
      // Pins the module this is linked into, since the handler stays after
      // a DLL could have been unloaded, such as event_loop_c after
      // ffi.dlclose(), and would then call into unmapped code.
      HMODULE module;
      ABSL_CHECK(GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                                        GET_MODULE_HANDLE_EX_FLAG_PIN,
                                    reinterpret_cast<LPCWSTR>(&HandleCtrl),
                                    &module))
          << GetLastError();
      ABSL_CHECK(SetConsoleCtrlHandler(&HandleCtrl, TRUE)) << GetLastError();
    });
    std::unique_lock<std::mutex> locker(mutex_);
    // A loop started after an event gets its own first event, as on POSIX.
    handled_ = false;
    event_loops_.push_back(event_loop);
  }

  // Unregisters an event loop to receive Exit() calls.
  void Unregister(QueueEventLoop *event_loop) {
    std::unique_lock<std::mutex> locker(mutex_);
    event_loops_.erase(
        std::find(event_loops_.begin(), event_loops_.end(), event_loop));
    if (event_loops_.empty()) {
      all_unregistered_.notify_all();
    }
  }

 private:
  static BOOL WINAPI HandleCtrl(DWORD ctrl_type) {
    return global()->DoHandleCtrl(ctrl_type);
  }

  BOOL DoHandleCtrl(DWORD ctrl_type) {
    switch (ctrl_type) {
      case CTRL_C_EVENT:
      case CTRL_BREAK_EVENT:
      case CTRL_CLOSE_EVENT:
        break;
      default:
        return FALSE;
    }
    std::unique_lock<std::mutex> locker(mutex_);
    if (event_loops_.empty()) {
      // No event loop to exit.  Pass it to the next handler.
      return FALSE;
    }
    if (handled_) {
      // The first event didn't get the loops out, so end the process, as a
      // second SIGINT does on POSIX.  Passing the event on instead could reach
      // the CRT's handler, which only sets a flag for Python to read.
      TerminateProcess(GetCurrentProcess(), STATUS_CONTROL_C_EXIT);
    }
    handled_ = true;
    for (QueueEventLoop *event_loop : event_loops_) {
      event_loop->Exit();
    }
    if (ctrl_type == CTRL_CLOSE_EVENT) {
      // Windows ends the process as soon as a handler returns from a close,
      // so Run() would never get to return.  Wait for the loops to finish,
      // inside the roughly 5 s Windows allows before it ends the process
      // anyway.
      all_unregistered_.wait_for(locker, std::chrono::milliseconds(4500),
                                 [this]() { return event_loops_.empty(); });
    }
    return TRUE;
  }

  std::once_flag installed_;
  // Mutex to protect all state.
  std::mutex mutex_;
  // Notified when the last event loop unregisters.
  std::condition_variable all_unregistered_;
  std::vector<QueueEventLoop *> event_loops_;
  // Whether an event has asked the registered loops to exit.
  bool handled_ = false;
};

}  // namespace

// Wakeups arrive as a named Event on Windows rather than as a signal (see
// ThreadSignalSender/ThreadSignalReceiver), so there is no signal disposition
// to suppress here.
void QueueEventLoop::IgnoreWakeupSignal() {}

void QueueEventLoop::RegisterSignalHandler() {
  ConsoleCtrlHandler::global()->Register(this);
}

void QueueEventLoop::UnregisterSignalHandler() {
  ConsoleCtrlHandler::global()->Unregister(this);
}

}  // namespace aos
