#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "gtest/gtest.h"

#include "aos/init.h"
#include "aos/testing/tmpdir.h"

ABSL_FLAG(bool, print_logs, false,
          "Print the log messages as they are being generated.");
ABSL_FLAG(std::string, log_file, "",
          "Print all log messages to FILE instead of standard output.");

// Actually declared/defined in //aos/testing:test_logging.
//
// Linux is happy to pick the strong symbol over this one.  OSX doesn't like
// undefined symbols.  So define a weak one with no body, to make both OSes
// happy.  This lets us not add a dependency on these two libraries to every
// test, and to only use them when linked in.
#ifdef _WIN32
extern "C" {
void aos_SetLogFileName_default(const char *) {}
void aos_ForcePrintLogsDuringTests_default() {}
#pragma comment( \
    linker, "/alternatename:aos_SetLogFileName=aos_SetLogFileName_default")
#pragma comment( \
    linker,      \
    "/alternatename:aos_ForcePrintLogsDuringTests=aos_ForcePrintLogsDuringTests_default")
void aos_SetLogFileName(const char *);
void aos_ForcePrintLogsDuringTests();
}
#else
extern "C" {
__attribute__((weak)) void aos_SetLogFileName(const char *) {}
__attribute__((weak)) void aos_ForcePrintLogsDuringTests() {}
}
#endif

GTEST_API_ int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  aos::InitGoogle(&argc, &argv);

  if (absl::GetFlag(FLAGS_print_logs)) {
    aos_ForcePrintLogsDuringTests();
  }

  if (!absl::GetFlag(FLAGS_log_file).empty()) {
    aos_ForcePrintLogsDuringTests();
    aos_SetLogFileName(absl::GetFlag(FLAGS_log_file).c_str());
  }

#ifdef _WIN32
  // Start from an empty temporary directory.  Doing it here runs it exactly
  // once, before any test, rather than making TestTmpDir() work out whether
  // each of its callers is the first one.
  aos::testing::internal::ClearTestTmpDirWindows();
#endif

  // Point shared memory away from /dev/shm if we are testing.  We don't care
  // about RT in this case, so if it is backed by disk, we are fine.
  aos::testing::SetTestShmBase();

  return RUN_ALL_TESTS();
}
