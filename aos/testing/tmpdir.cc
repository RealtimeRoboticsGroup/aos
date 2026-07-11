#include "aos/testing/tmpdir.h"

#include <cstdlib>
#include <filesystem>
#include <string>
#ifdef _WIN32
#include <process.h>
#endif

#include "absl/flags/flag.h"

#include "aos/ipc_lib/shm_base.h"

namespace aos::testing {

namespace {
std::string TestTmpDirOr(std::string fallback) {
  const char *tmp_dir = std::getenv("TEST_TMPDIR");
  if (tmp_dir != nullptr) {
    return tmp_dir;
  }
  return fallback;
}
}  // namespace

std::string TestTmpDir() {
#ifdef _WIN32
  // On Windows, Bazel does not provide the same mount/namespace isolation for
  // temporary directories as Linux does. As a result, parallel test shards or
  // concurrent test processes might share the same temp directory.
  //
  // Furthermore, Windows enforces strict file locking (sharing violations)
  // which prevents deleting or overwriting files that are currently open.
  // During Google Test death tests, the test binary is re-executed in a
  // subprocess. If both parent and child processes write to the same temporary
  // directory, they will collide and fail due to file locks. Appending the PID
  // ensures each process (and child subprocess) gets a unique, isolated temp
  // folder.
  std::string path = TestTmpDirOr("/tmp") + "/" + std::to_string(_getpid());
  std::filesystem::create_directories(path);
  return path;
#else
  return TestTmpDirOr("/tmp");
#endif
}

void SetTestShmBase() {
  SetShmBase(TestTmpDirOr(absl::GetFlag(FLAGS_shm_base)));
}

}  // namespace aos::testing

extern "C" void aos_testing_set_test_shm_base() {
  aos::testing::SetTestShmBase();
}
