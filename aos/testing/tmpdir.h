#ifndef AOS_TESTING_TMPDIR_H_
#define AOS_TESTING_TMPDIR_H_

#include <string>

namespace aos::testing {

// Returns a usable temporary directory.
std::string TestTmpDir();

// Sets shm_base to a folder inside of TEST_TMPDIR if set, or --shm_base
// otherwise.
void SetTestShmBase();

#ifdef __cplusplus
extern "C" {
#endif
void aos_testing_set_test_shm_base();
#ifdef __cplusplus
}
#endif

}  // namespace aos::testing

#endif  // AOS_TESTING_TMPDIR_H_
