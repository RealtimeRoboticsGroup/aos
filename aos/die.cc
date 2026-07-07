#include "aos/die.h"

#include <sys/types.h>

#include "aos/realtime.h"
#ifdef _WIN32
#include <process.h>
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

namespace aos {

void Die(const char *format, ...) {
  va_list args;
  va_start(args, format);
  VDie(format, args);
  // va_end(args)  // not because VDie never returns
}

namespace {

// Calculates the filename to dump the message into.
const std::string GetFilename() {
  intmax_t pid = static_cast<intmax_t>(aos::GetProcessId());
  char buffer[256];
#ifdef _WIN32
  const char *temp_dir = getenv("TEMP");
  if (!temp_dir) temp_dir = ".";
  snprintf(buffer, sizeof(buffer), "%s\\aos_fatal_error.%jd", temp_dir, pid);
#else
  snprintf(buffer, sizeof(buffer), "/tmp/aos_fatal_error.%jd", pid);
#endif
  return std::string(buffer);
}

std::atomic_bool test_mode(false);

}  // namespace

void VDie(const char *format, va_list args_in) {
  // We don't bother va_ending either of these because we're going nowhere and
  // vxworks has some weird bugs that sometimes show up...
  va_list args1, args2;

  fputs("aos fatal: ERROR!! details following\n", stderr);
  va_copy(args1, args_in);
  vfprintf(stderr, format, args1);
  if (!test_mode.load()) {
    fputs("aos fatal: ERROR!! see stderr for details\n", stdout);

    const std::string filename = GetFilename();
    if (!filename.empty()) {
      FILE *error_file = fopen(filename.c_str(), "w");
      if (error_file != NULL) {
        va_copy(args2, args_in);
        vfprintf(error_file, format, args2);
        fclose(error_file);
      } else {
        fprintf(stderr, "aos fatal: fopen('%s', \"w\") failed with %d\n",
                filename.c_str(), errno);
      }
    }
  }

  abort();
}

void SetDieTestMode(bool new_test_mode) { test_mode.store(new_test_mode); }

}  // namespace aos
