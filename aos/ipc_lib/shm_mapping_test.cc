#include "aos/ipc_lib/shm_mapping.h"

#include <filesystem>
#include <string>

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "gtest/gtest.h"

#include "aos/testing/tmpdir.h"

namespace aos::ipc_lib::testing {

class ShmMappingTest : public ::testing::Test {
 protected:
  void SetUp() override {
    test_dir_ = aos::testing::TestTmpDir() + "/shm_mapping_test";
    std::filesystem::remove_all(test_dir_);
  }

  void TearDown() override { std::filesystem::remove_all(test_dir_); }

  std::string test_dir_;
};

TEST_F(ShmMappingTest, BasicReadWrite) {
  const std::string shm_path = test_dir_ + "/test_queue";
  const size_t size = 4096;

  // 1. Create a writable mapping.
  WritableShmMapping write_mapping(shm_path, size,
                                   static_cast<std::filesystem::perms>(0777));
  ASSERT_NE(write_mapping.data(), nullptr);
  EXPECT_EQ(write_mapping.size(), size);

  // Write some data.
  char *write_ptr = static_cast<char *>(write_mapping.data());
  write_ptr[0] = 'H';
  write_ptr[1] = 'e';
  write_ptr[2] = 'l';
  write_ptr[3] = 'l';
  write_ptr[4] = 'o';

  // 2. Open a read-only mapping on the same path.
  ReadOnlyShmMapping read_mapping(shm_path, size,
                                  static_cast<std::filesystem::perms>(0777));
  ASSERT_NE(read_mapping.data(), nullptr);
  EXPECT_EQ(read_mapping.size(), size);

  // Read the data and verify.
  const char *read_ptr = static_cast<const char *>(read_mapping.data());
  EXPECT_EQ(read_ptr[0], 'H');
  EXPECT_EQ(read_ptr[1], 'e');
  EXPECT_EQ(read_ptr[2], 'l');
  EXPECT_EQ(read_ptr[3], 'l');
  EXPECT_EQ(read_ptr[4], 'o');
}

TEST_F(ShmMappingTest, ReadOnlyIsActuallyReadOnly) {
  const std::string shm_path = test_dir_ + "/test_queue_ro";
  const size_t size = 4096;

  // Create file and writable mapping first.
  WritableShmMapping write_mapping(shm_path, size,
                                   static_cast<std::filesystem::perms>(0777));
  ASSERT_NE(write_mapping.data(), nullptr);

  // Open the read-only mapping.
  ReadOnlyShmMapping read_mapping(shm_path, size,
                                  static_cast<std::filesystem::perms>(0777));
  ASSERT_NE(read_mapping.data(), nullptr);

  // Attempting to write to read-only pointer should crash.
  EXPECT_DEATH(
      {
        char *ro_ptr =
            const_cast<char *>(static_cast<const char *>(read_mapping.data()));
        ro_ptr[0] = 'X';
      },
      "");
}

TEST_F(ShmMappingTest, PageFaultDataWriteDoesNotCorrupt) {
  const std::string shm_path = test_dir_ + "/test_corruption";
  std::filesystem::remove(shm_path);

  size_t page_size = 4096;
#ifdef _WIN32
  SYSTEM_INFO sys_info;
  GetSystemInfo(&sys_info);
  page_size = sys_info.dwPageSize;
#else
  page_size = sysconf(_SC_PAGESIZE);
#endif

  // Make the file size at least 2 pages.
  const size_t size = page_size * 2;

  // 1. Create a writable mapping and write some non-zero data at page
  // boundaries.
  {
    WritableShmMapping write_mapping(shm_path, size,
                                     static_cast<std::filesystem::perms>(0777));
    ASSERT_NE(write_mapping.data(), nullptr);

    char *write_ptr = static_cast<char *>(write_mapping.data());
    // Write the same non-zero byte to the start of multiple pages.
    // If the PageFaultDataWrite bug were present, loading page 0's value
    // would set 'zero' to 'A'. Then, evaluating page 1 would see 'A' as
    // expected, matching the page 1 value, and write 0 back to it.
    write_ptr[0] = 'A';
    write_ptr[page_size] = 'A';
  }

  // 2. Open the mapping again. This runs PageFaultDataWrite on the existing
  // data.
  {
    WritableShmMapping write_mapping(shm_path, size,
                                     static_cast<std::filesystem::perms>(0777));
    ASSERT_NE(write_mapping.data(), nullptr);

    char *write_ptr = static_cast<char *>(write_mapping.data());
    EXPECT_EQ(write_ptr[0], 'A');
    EXPECT_EQ(write_ptr[page_size], 'A');
  }
}

}  // namespace aos::ipc_lib::testing
