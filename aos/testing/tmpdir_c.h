#ifndef AOS_TESTING_TMPDIR_C_H_
#define AOS_TESTING_TMPDIR_C_H_

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

// Sets shm_base to a folder inside of TEST_TMPDIR if set, or --shm_base
// otherwise.  This is the C entry point for aos::testing::SetTestShmBase; it
// lives in its own header so bindgen can parse it (tmpdir.h is C++).
void aos_testing_set_test_shm_base(void);

#ifdef __cplusplus
}  // extern "C"
#endif  // __cplusplus

#endif  // AOS_TESTING_TMPDIR_C_H_
