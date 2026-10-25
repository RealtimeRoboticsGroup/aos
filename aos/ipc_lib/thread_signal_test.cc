#include "aos/ipc_lib/thread_signal.h"

#ifdef __APPLE__
#define FUTEX_TID_MASK 0x3fffffff
#endif

#include <future>
#include <thread>

#include "absl/log/absl_check.h"
#include "gtest/gtest.h"

#ifdef _WIN32
#include <windows.h>
#else
#include <poll.h>
#include <signal.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

namespace aos::ipc_lib::testing {

// Test that we can instantiate ThreadSignalSender, send a signal, and
// successfully wake up the thread.
TEST(ThreadSignalTest, BasicSignalWakeup) {
#ifdef _WIN32
  ThreadSignalReceiver receiver;

  const pid_t pid = GetCurrentProcessId();
  const pid_t tid = GetCurrentThreadId();

  // No synchronization with the waiter below is needed: the event/signal stays
  // pending until it's waited on, so the ordering of the two doesn't matter.
  std::thread signaler([pid, tid]() {
    ThreadSignalSender signaler_signal;
    signaler_signal.Signal(pid, tid);
  });

  DWORD wait_result = WaitForSingleObject(receiver.event_handle(), 2000);
  EXPECT_EQ(wait_result, WAIT_OBJECT_0);

  signaler.join();
#else
  // On Linux, block kWakeupSignal so we can wait for it using sigwait.
  sigset_t sigset;
  sigemptyset(&sigset);
  sigaddset(&sigset, kWakeupSignal);
  ABSL_CHECK_EQ(pthread_sigmask(SIG_BLOCK, &sigset, nullptr), 0);

  ThreadSignalSender signal;

  const pid_t pid = getpid();
#ifdef __APPLE__
  uint64_t mac_tid;
  pthread_threadid_np(NULL, &mac_tid);
  const pid_t tid = static_cast<pid_t>(mac_tid) & FUTEX_TID_MASK;
#else
  const pid_t tid = syscall(SYS_gettid);
#endif

  // No synchronization with the waiter below is needed: the event/signal stays
  // pending until it's waited on, so the ordering of the two doesn't matter.
  std::thread signaler([pid, tid]() {
    ThreadSignalSender signaler_signal;
    signaler_signal.Signal(pid, tid);
  });

  int sig = 0;
  EXPECT_EQ(sigwait(&sigset, &sig), 0);
  EXPECT_EQ(sig, kWakeupSignal);

  signaler.join();
#endif
}

// This exercises the receiver by polling its signalfd, so it's Linux-only.  On
// macOS the receiver has no pollable fd: it uses an ignored signal plus kqueue
// EVFILT_SIGNAL, driven by the Aio backend, so fd() is -1.  Windows has its own
// test below.
#ifdef __linux__
TEST(ThreadSignalReceiverTest, BasicSignalWakeup) {
  ThreadSignalReceiver receiver;

  const pid_t pid = getpid();
  const pid_t tid = syscall(SYS_gettid);

  // No synchronization with the waiter below is needed: the event/signal stays
  // pending until it's waited on, so the ordering of the two doesn't matter.
  std::thread signaler([pid, tid]() {
    ThreadSignalSender signaler_signal;
    signaler_signal.Signal(pid, tid);
  });

  struct pollfd pfd;
  pfd.fd = receiver.fd();
  pfd.events = POLLIN;
  int poll_result = poll(&pfd, 1, 2000);
  EXPECT_EQ(poll_result, 1);
  EXPECT_TRUE(pfd.revents & POLLIN);

  receiver.ConsumeWakeup();

  signaler.join();
}
#endif  // __linux__

#ifdef _WIN32
// A wakeup leaves the receiver's Event set until ConsumeWakeup() clears it.
TEST(ThreadSignalReceiverTest, BasicSignalWakeup) {
  ThreadSignalReceiver receiver;
  ThreadSignalSender sender;
  sender.Signal(GetCurrentProcessId(), GetCurrentThreadId());

  // Manual-reset: waiting does not consume the wakeup.
  EXPECT_EQ(WaitForSingleObject(receiver.event_handle(), 0), WAIT_OBJECT_0);
  EXPECT_EQ(WaitForSingleObject(receiver.event_handle(), 0), WAIT_OBJECT_0);

  receiver.ConsumeWakeup();
  EXPECT_EQ(WaitForSingleObject(receiver.event_handle(), 0), WAIT_TIMEOUT);
}

// The receiver names its Event after the thread that constructs it, so a
// sender on another thread reaches it through that thread's (pid, tid).
TEST(ThreadSignalReceiverTest, SignalFromAnotherThread) {
  std::promise<pid_t> receiver_tid;
  std::promise<void> signaled;
  DWORD wait_result = WAIT_FAILED;
  std::thread receiver_thread([&]() {
    ThreadSignalReceiver receiver;
    receiver_tid.set_value(GetCurrentThreadId());
    signaled.get_future().wait();
    wait_result = WaitForSingleObject(receiver.event_handle(), 0);
  });

  ThreadSignalSender sender;
  sender.Signal(GetCurrentProcessId(), receiver_tid.get_future().get());
  signaled.set_value();
  receiver_thread.join();
  EXPECT_EQ(wait_result, WAIT_OBJECT_0);
}
#endif  // _WIN32

}  // namespace aos::ipc_lib::testing
