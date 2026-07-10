use std::sync::Once;

use aos_testing_tmpdir_c::aos_testing_set_test_shm_base;

// TODO(Brian): Should we provide a proc macro attribute that handles calling this?
/// Initializes things for a test.
///
/// # Panics
///
/// Panics if non-test initialization has already been performed.
pub fn test_init() {
    static ONCE: Once = Once::new();
    ONCE.call_once(|| {
        aos_init::internal::init();
        unsafe {
            aos_testing_set_test_shm_base();
        }
        env_logger::builder().is_test(true).init();
        // TODO(Brian): Do we want any of the other stuff that `:gtest_main` has?
    });
}
