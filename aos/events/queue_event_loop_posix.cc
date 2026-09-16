#include <signal.h>

#include <algorithm>
#include <initializer_list>
#include <mutex>
#include <vector>

#include "absl/log/absl_check.h"

#include "aos/events/queue_event_loop.h"
#include "aos/ipc_lib/thread_signal.h"

namespace aos {
namespace {

// RAII class to mask signals.
class ScopedSignalMask {
 public:
  ScopedSignalMask(std::initializer_list<int> signals) {
    sigset_t sigset;
    ABSL_PCHECK(sigemptyset(&sigset) == 0);
    for (int signal : signals) {
      ABSL_PCHECK(sigaddset(&sigset, signal) == 0);
    }

    ABSL_PCHECK(sigprocmask(SIG_BLOCK, &sigset, &old_) == 0);
  }

  ~ScopedSignalMask() {
    ABSL_PCHECK(sigprocmask(SIG_SETMASK, &old_, nullptr) == 0);
  }

 private:
  sigset_t old_;
};

// Class to manage the static state associated with killing multiple event
// loops.
class SignalHandler {
 public:
  // Gets the singleton.  It is never destroyed, because an event loop on
  // another thread can still be registered, or a signal can still arrive, while
  // the process exits.
  static SignalHandler *global() {
    static SignalHandler *const loop = new SignalHandler();
    return loop;
  }

  // Handles the signal with the singleton.
  static void HandleSignal(int) { global()->DoHandleSignal(); }

  // Registers an event loop to receive Exit() calls.
  void Register(QueueEventLoop *event_loop) {
    // Block signals while we have the mutex so we never race with the signal
    // handler.
    ScopedSignalMask mask({SIGINT, SIGHUP, SIGTERM});
    std::unique_lock<stl_mutex> locker(mutex_);
    if (event_loops_.size() == 0) {
      // The first caller registers the signal handler.
      struct sigaction new_action;
      sigemptyset(&new_action.sa_mask);
      // This makes it so that 2 control c's to a stuck process will kill it by
      // restoring the original signal handler.
      new_action.sa_flags = SA_RESETHAND;
      new_action.sa_handler = &HandleSignal;

      ABSL_PCHECK(sigaction(SIGINT, &new_action, &old_action_int_) == 0);
      ABSL_PCHECK(sigaction(SIGHUP, &new_action, &old_action_hup_) == 0);
      ABSL_PCHECK(sigaction(SIGTERM, &new_action, &old_action_term_) == 0);
    }

    event_loops_.push_back(event_loop);
  }

  // Unregisters an event loop to receive Exit() calls.
  void Unregister(QueueEventLoop *event_loop) {
    // Block signals while we have the mutex so we never race with the signal
    // handler.
    ScopedSignalMask mask({SIGINT, SIGHUP, SIGTERM});
    std::unique_lock<stl_mutex> locker(mutex_);

    event_loops_.erase(
        std::find(event_loops_.begin(), event_loops_.end(), event_loop));

    if (event_loops_.size() == 0u) {
      // The last caller restores the original signal handlers.
      ABSL_PCHECK(sigaction(SIGINT, &old_action_int_, nullptr) == 0);
      ABSL_PCHECK(sigaction(SIGHUP, &old_action_hup_, nullptr) == 0);
      ABSL_PCHECK(sigaction(SIGTERM, &old_action_term_, nullptr) == 0);
    }
  }

 private:
  void DoHandleSignal() {
    // We block signals while grabbing the lock, so there should never be a
    // race.  Confirm that this is true using trylock.
    ABSL_CHECK(mutex_.try_lock())
        << ": sigprocmask failed to block signals while "
           "modifing the event loop list.";
    for (QueueEventLoop *event_loop : event_loops_) {
      event_loop->Exit();
    }
    mutex_.unlock();
  }

  // Mutex to protect all state.
  stl_mutex mutex_;
  std::vector<QueueEventLoop *> event_loops_;
  struct sigaction old_action_int_;
  struct sigaction old_action_hup_;
  struct sigaction old_action_term_;
};

}  // namespace

void QueueEventLoop::IgnoreWakeupSignal() {
  struct sigaction action;
  action.sa_handler = SIG_IGN;
  ABSL_PCHECK(sigemptyset(&action.sa_mask) == 0);
  action.sa_flags = 0;
  ABSL_PCHECK(sigaction(ipc_lib::kWakeupSignal, &action, nullptr) == 0);
}

void QueueEventLoop::RegisterSignalHandler() {
  SignalHandler::global()->Register(this);
}

void QueueEventLoop::UnregisterSignalHandler() {
  SignalHandler::global()->Unregister(this);
}

}  // namespace aos
