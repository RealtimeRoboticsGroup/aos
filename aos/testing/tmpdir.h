#ifndef AOS_TESTING_TMPDIR_H_
#define AOS_TESTING_TMPDIR_H_

#include <string>

namespace aos::testing {

// Returns a usable temporary directory.
std::string TestTmpDir();

#ifdef _WIN32
namespace internal {
// Empties the directory TestTmpDir() returns, so tests start with nothing left
// in it.  On Windows that is a directory named after our process ID which
// TestTmpDir() creates and nothing else writes to, and since Windows recycles
// process IDs it can still hold files from a process that has since died.
//
// //aos/testing:googletest's main calls this once before running any test.
// Nothing else should need to.
//
// This deliberately only exists on Windows.  Everywhere else TestTmpDir() hands
// back TEST_TMPDIR or /tmp, which we share with the rest of the machine and
// must not delete.
void ClearTestTmpDirWindows();
}  // namespace internal
#endif

// Sets shm_base to a folder inside of TEST_TMPDIR if set, or --shm_base
// otherwise.
void SetTestShmBase();

}  // namespace aos::testing

#endif  // AOS_TESTING_TMPDIR_H_
