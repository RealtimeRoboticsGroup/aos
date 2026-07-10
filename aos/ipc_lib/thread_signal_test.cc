#include "aos/ipc_lib/thread_signal.h"

#include <chrono>
#include <thread>

#include "absl/log/absl_check.h"
#include "gtest/gtest.h"

#ifdef _WIN32
#include <windows.h>
#else
#include <signal.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

namespace aos::ipc_lib::testing {

// Test that we can instantiate ThreadSignal, send a signal, and successfully
// wake up the thread.
TEST(ThreadSignalTest, BasicSignalWakeup) {
#ifdef _WIN32
  ThreadSignal signal;
  EXPECT_NE(signal.event_handle(), nullptr);

  const pid_t pid = GetCurrentProcessId();
  const pid_t tid = GetCurrentThreadId();

  std::thread signaler([pid, tid]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    ThreadSignal signaler_signal;
    signaler_signal.Signal(pid, tid);
  });

  DWORD wait_result = WaitForSingleObject(signal.event_handle(), 2000);
  EXPECT_EQ(wait_result, WAIT_OBJECT_0);

  signaler.join();
#else
  // On Linux, block kWakeupSignal so we can wait for it using sigwait.
  sigset_t sigset;
  sigemptyset(&sigset);
  sigaddset(&sigset, kWakeupSignal);
  ABSL_CHECK_EQ(pthread_sigmask(SIG_BLOCK, &sigset, nullptr), 0);

  ThreadSignal signal;

  const pid_t pid = getpid();
  const pid_t tid = syscall(SYS_gettid);

  std::thread signaler([pid, tid]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    ThreadSignal signaler_signal;
    signaler_signal.Signal(pid, tid);
  });

  int sig = 0;
  EXPECT_EQ(sigwait(&sigset, &sig), 0);
  EXPECT_EQ(sig, kWakeupSignal);

  signaler.join();
#endif
}

}  // namespace aos::ipc_lib::testing
