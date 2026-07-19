#include "aos/ipc_lib/shm_base.h"

#include <cstdlib>
#include <filesystem>
#include <string>

#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"

namespace {

// Where queues go by default.  On Windows a path needs a drive to mean the
// same directory to every process, whatever drive it started from, so the
// default is under %ProgramData% there.
std::string DefaultShmBase() {
#ifdef _WIN32
  const char *const program_data = std::getenv("ProgramData");
  return std::string(program_data != nullptr ? program_data
                                             : "C:\\ProgramData") +
         "\\aos";
#else
  return "/dev/shm/aos";
#endif
}

}  // namespace

ABSL_FLAG(std::string, shm_base, DefaultShmBase(),
          "Directory to place queue backing mmaped files in.  /dev/shm/aos by "
          "default, and %ProgramData%\\aos on Windows.");

namespace aos {

std::string ShmBase() {
  std::string shm_base = absl::GetFlag(FLAGS_shm_base);
#ifdef _WIN32
  // A relative path, or one missing its drive (/dev/shm/aos) or its root
  // directory (D:aos), resolves against each process's current drive or
  // directory, so two processes started from different places would never
  // see each other's messages, with no error on either side.
  ABSL_CHECK(std::filesystem::path(shm_base).is_absolute())
      << ": --shm_base " << shm_base
      << " is not absolute on Windows, so it means a different directory to "
         "a process started from another drive or directory.  Give it a "
         "drive and a root, such as C:/ProgramData/aos.";
#endif
  return shm_base;
}

}  // namespace aos

namespace aos::testing {

void SetShmBase(const std::string_view base) {
  absl::SetFlag(&FLAGS_shm_base, std::string(base) + "/aos");
}

}  // namespace aos::testing
