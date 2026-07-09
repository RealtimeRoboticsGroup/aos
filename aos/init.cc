#include "aos/init.h"

#ifndef _WIN32
#include <sched.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/types.h>
#include <unistd.h>
#else
#include <crtdbg.h>
#include <stdlib.h>
#endif

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "absl/debugging/failure_signal_handler.h"
#include "absl/debugging/symbolize.h"
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "absl/log/flags.h"
#include "absl/log/globals.h"
#include "absl/log/initialize.h"

#include "aos/realtime.h"
#include "aos/uuid.h"

ABSL_FLAG(bool, coredump, false, "If true, write core dumps on failure.");
ABSL_FLAG(bool, backtrace, true, "If true, print backtraces out on crashes.");

namespace aos {
namespace {
std::atomic<bool> initialized{false};
}  // namespace

bool IsInitialized() { return initialized; }

#ifdef _WIN32
namespace {
// By default on Windows, passing invalid parameters to C Runtime (CRT)
// functions (such as closing an invalid/already-closed file descriptor) causes
// the CRT to terminate the process or display a crash dialog. Registering a
// no-op invalid parameter handler overrides this, causing the CRT functions to
// return failure and set errno (e.g., EBADF), mirroring standard POSIX
// behavior.
void InvalidParameterHandler(const wchar_t * /*expression*/,
                             const wchar_t * /*function*/,
                             const wchar_t * /*file*/, unsigned int /*line*/,
                             uintptr_t /*pReserved*/) {
  // Do nothing. The CRT function will fail and set errno.
}
}  // namespace
#endif

void InitGoogle(int *argc, char ***argv) {
#ifdef _WIN32
  // Configure Windows CRT parameter validation and assert behavior:
  // 1. Force invalid CRT parameters to fail gracefully and set errno instead of
  // crashing.
  _set_invalid_parameter_handler(InvalidParameterHandler);
  // 2. Disable interactive CRT assertion popup dialogs to allow automated
  // headless testing.
  _CrtSetReportMode(_CRT_ASSERT, 0);
#endif
  ABSL_CHECK(!IsInitialized()) << "Only initialize once.";
  absl::SetStderrThreshold(absl::LogSeverityAtLeast::kInfo);
  std::vector<char *> positional_arguments =
      absl::ParseCommandLine(*argc, *argv);

  ABSL_CHECK_LE(positional_arguments.size(), static_cast<size_t>(*argc));
  for (size_t i = 0; i < positional_arguments.size(); ++i) {
    (*argv)[i] = positional_arguments[i];
  }
  *argc = positional_arguments.size();

  absl::InitializeLog();

  if (absl::GetFlag(FLAGS_backtrace)) {
    absl::InitializeSymbolizer((*argv)[0]);
    absl::FailureSignalHandlerOptions options;
    absl::InstallFailureSignalHandler(options);
  }

#ifndef _WIN32
  if (absl::GetFlag(FLAGS_coredump)) {
    WriteCoreDumps();
  }
#endif

  RegisterMallocHook();
  // Ensure that the random number generator for the UUID code is initialized
  // (it does some potentially expensive random number generation).
  UUID::Random();

  initialized = true;
}

void MarkInitialized() { initialized = true; }

}  // namespace aos
