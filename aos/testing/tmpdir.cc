#include "aos/testing/tmpdir.h"

#include <cstdlib>
#include <filesystem>
#include <optional>
#include <string>

#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"

#include "aos/ipc_lib/shm_base.h"
#include "aos/realtime.h"

namespace aos::testing {

namespace {
// Returns $TEST_TMPDIR, if it is set.
std::optional<std::string> TestTmpDirEnv() {
  const char *tmp_dir = std::getenv("TEST_TMPDIR");
  if (tmp_dir == nullptr) {
    return std::nullopt;
  }
  return std::string(tmp_dir);
}

std::string TestTmpDirOr(std::string fallback) {
  return TestTmpDirEnv().value_or(std::move(fallback));
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
  // directory, they will collide and fail due to file locks. Giving the folder
  // a distinctive, application-specific, PID-qualified name ensures each
  // process (and child subprocess) gets a unique, isolated temp folder that's
  // also unlikely to collide with unrelated files in the shared fallback
  // location.
  // Only ask the system where its temp directory is when TEST_TMPDIR hasn't
  // already told us.  temp_directory_path() fails if TEMP/TMP point somewhere
  // that doesn't exist, and that shouldn't take down a test which named a
  // perfectly good directory to work in.
  std::filesystem::path tmp_root;
  if (const std::optional<std::string> tmp_dir_env = TestTmpDirEnv();
      tmp_dir_env.has_value()) {
    tmp_root = std::filesystem::path(*tmp_dir_env);
  } else {
    std::error_code ec;
    tmp_root = std::filesystem::temp_directory_path(ec);
    ABSL_CHECK(!ec) << "Failed to find a temporary directory: " << ec.message();
  }

  // Join with path::operator/ rather than gluing a '/' on ourselves.  Windows
  // turns off the usual forward-slash-to-backslash normalization for paths
  // which start with \\?\, so a hand-written '/' in one of those never gets
  // resolved and every file we open under it fails.
  const std::filesystem::path path =
      tmp_root / ("aos_testing_tmpdir_" + std::to_string(aos::GetProcessId()));
  std::error_code ec;
  std::filesystem::create_directories(path, ec);
  ABSL_CHECK(!ec) << "Failed to create " << path.string() << ": "
                  << ec.message();
  return path.string();
#else
  return TestTmpDirOr("/tmp");
#endif
}

#ifdef _WIN32
namespace internal {
void ClearTestTmpDirWindows() {
  // Only safe because TestTmpDir() creates this directory per process and
  // nothing else writes to it.  Off Windows it returns TEST_TMPDIR or /tmp,
  // which emphatically are not ours to empty -- hence no non-Windows version.
  const std::string path = TestTmpDir();
  std::error_code ec;

  // Windows refuses to delete a read-only file, and a previous process with
  // our PID may well have left one behind.  Take write permission back on
  // everything first so remove_all() can actually empty the directory.  Errors
  // here aren't worth failing over; remove_all() below reports what it can't
  // delete.
  std::filesystem::recursive_directory_iterator it(path, ec);
  for (const std::filesystem::recursive_directory_iterator end;
       !ec && it != end; it.increment(ec)) {
    std::error_code permission_ec;
    std::filesystem::permissions(
        it->path(), std::filesystem::perms::owner_write,
        std::filesystem::perm_options::add, permission_ec);
  }
  ec.clear();

  std::filesystem::remove_all(path, ec);
  ABSL_CHECK(!ec) << "Failed to clear " << path << ": " << ec.message();
  std::filesystem::create_directories(path, ec);
  ABSL_CHECK(!ec) << "Failed to create " << path << ": " << ec.message();
}
}  // namespace internal
#endif

void SetTestShmBase() {
  SetShmBase(TestTmpDirOr(absl::GetFlag(FLAGS_shm_base)));
}

}  // namespace aos::testing
