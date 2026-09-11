// Tests InitEmbedded() in a process that has not initialized AOS.

#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <thread>
#include <vector>

#include "absl/log/absl_check.h"

#include "aos/init.h"

namespace {

// The signals absl's failure signal handler takes over.
constexpr int kCrashSignals[] = {
    SIGSEGV, SIGILL,  SIGFPE, SIGABRT, SIGTERM,
#ifndef _WIN32
    SIGBUS,  SIGTRAP,
#endif
};

// The handler each of kCrashSignals has now.
std::vector<uintptr_t> CrashHandlers() {
  std::vector<uintptr_t> result;
  for (int signal_number : kCrashSignals) {
#ifdef _WIN32
    // signal() is the only way to read a handler here, and it sets one too,
    // so put back what it returns.
    const _crt_signal_t handler = signal(signal_number, SIG_DFL);
    ABSL_CHECK(handler != SIG_ERR);
    ABSL_CHECK(signal(signal_number, handler) != SIG_ERR);
    result.push_back(reinterpret_cast<uintptr_t>(handler));
#else
    struct sigaction action{};
    ABSL_CHECK_EQ(sigaction(signal_number, nullptr, &action), 0);
    result.push_back((action.sa_flags & SA_SIGINFO)
                         ? reinterpret_cast<uintptr_t>(action.sa_sigaction)
                         : reinterpret_cast<uintptr_t>(action.sa_handler));
#endif
  }
  return result;
}

}  // namespace

int main() {
  ABSL_CHECK(!aos::IsInitialized());
  const std::vector<uintptr_t> before = CrashHandlers();

  // Two libraries in one process can both get there first.
  std::thread other([]() { aos::InitEmbedded(); });
  aos::InitEmbedded();
  other.join();
  ABSL_CHECK(aos::IsInitialized());
  // A later call does nothing.
  aos::InitEmbedded();

  ABSL_CHECK(CrashHandlers() == before)
      << ": InitEmbedded() installed a crash signal handler";
  return EXIT_SUCCESS;
}
