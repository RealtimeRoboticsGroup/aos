#include <pthread.h>
#include <signal.h>

#include <algorithm>
#include <atomic>
#include <initializer_list>
#include <mutex>
#include <vector>

#include "absl/log/absl_check.h"

#include "aos/events/queue_event_loop.h"
#include "aos/ipc_lib/thread_signal.h"

namespace aos {
namespace {

// RAII class to mask signals on the calling thread.  pthread_sigmask() rather
// than sigprocmask(), which on XNU changes the mask of every thread in the
// process, so loops starting and stopping on different threads could leave
// SIGINT/SIGHUP/SIGTERM blocked for all of them.
class ScopedSignalMask {
 public:
  ScopedSignalMask(std::initializer_list<int> signals) {
    sigset_t sigset;
    ABSL_PCHECK(sigemptyset(&sigset) == 0);
    for (int signal : signals) {
      ABSL_PCHECK(sigaddset(&sigset, signal) == 0);
    }

    ABSL_CHECK_EQ(pthread_sigmask(SIG_BLOCK, &sigset, &old_), 0);
  }

  ~ScopedSignalMask() {
    ABSL_CHECK_EQ(pthread_sigmask(SIG_SETMASK, &old_, nullptr), 0);
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

  // Handles the signal with the singleton.  The first one exits the loops,
  // and a second ends the process, so two Ctrl-Cs kill a stuck process.
  // Counted here, not with SA_RESETHAND, whose reset XNU can apply after
  // Unregister() has put the old action back.
  static void HandleSignal(int signal) {
    if (global()->handled_.exchange(true)) {
      Die(signal);
    }
    global()->DoHandleSignal();
  }

  // Registers an event loop to receive Exit() calls.
  void Register(QueueEventLoop *event_loop) {
    // The mask keeps the handler off this thread while it holds the mutex.
    // It may still run on another thread; see DoHandleSignal().
    ScopedSignalMask mask({SIGINT, SIGHUP, SIGTERM});
    {
      std::unique_lock<stl_mutex> locker(mutex_);
      if (event_loops_.size() == 0) {
        // The first caller registers the signal handler.
        struct sigaction new_action;
        sigemptyset(&new_action.sa_mask);
        new_action.sa_flags = 0;
        new_action.sa_handler = &HandleSignal;

        ABSL_PCHECK(sigaction(SIGINT, &new_action, &old_action_int_) == 0);
        ABSL_PCHECK(sigaction(SIGHUP, &new_action, &old_action_hup_) == 0);
        ABSL_PCHECK(sigaction(SIGTERM, &new_action, &old_action_term_) == 0);
      }
      // A loop started after a signal gets its own first signal.  So while
      // loops keep starting, a second signal only exits them, even with
      // another loop stuck.
      handled_ = false;

      event_loops_.push_back(event_loop);
    }
    DrainPending();
  }

  // Unregisters an event loop to receive Exit() calls.
  void Unregister(QueueEventLoop *event_loop) {
    ScopedSignalMask mask({SIGINT, SIGHUP, SIGTERM});
    {
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
    DrainPending();
  }

 private:
  // Ends the process with signal's default action.  Unblocked, so raise()
  // delivers it before returning.  It only returns if Register() or
  // Unregister() replaced SIG_DFL in between, so try again.
  [[noreturn]] static void Die(int signal) {
    struct sigaction default_action;
    sigemptyset(&default_action.sa_mask);
    default_action.sa_flags = 0;
    default_action.sa_handler = SIG_DFL;
    sigset_t sigset;
    sigemptyset(&sigset);
    sigaddset(&sigset, signal);
    pthread_sigmask(SIG_UNBLOCK, &sigset, nullptr);
    while (true) {
      sigaction(signal, &default_action, nullptr);
      raise(signal);
    }
  }

  // The masks only cover the thread that holds the mutex, so a
  // process-directed signal can arrive on another thread while one is held.
  // The handler cannot wait for it, so it leaves the signal in pending_, and
  // whoever held the mutex exits the loops once it has let go.  pending_ is
  // set before the try_lock(), so a try_lock() that fails because the mutex
  // is held is always seen by the holder's check after its unlock().
  void DoHandleSignal() {
    pending_ = true;
    if (!mutex_.try_lock()) {
      return;
    }
    if (pending_.exchange(false)) {
      ExitAll();
    }
    mutex_.unlock();
  }

  void DrainPending() {
    while (pending_) {
      std::unique_lock<stl_mutex> locker(mutex_);
      if (pending_.exchange(false)) {
        ExitAll();
      }
    }
  }

  void ExitAll() {
    for (QueueEventLoop *event_loop : event_loops_) {
      event_loop->Exit();
    }
  }

  // Mutex to protect all state.
  stl_mutex mutex_;
  // A signal that arrived while the mutex was held; see DoHandleSignal().
  std::atomic<bool> pending_{false};
  // Whether a signal has asked the registered loops to exit.
  std::atomic<bool> handled_{false};
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
