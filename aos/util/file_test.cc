#include "aos/util/file.h"

#include <fcntl.h>
#include <sys/stat.h>

#ifdef _WIN32
#include <io.h>
inline int CloseFd(int fd) { return _close(fd); }
#else
#include <unistd.h>
inline int CloseFd(int fd) { return ::close(fd); }
#endif

#include <cstdlib>
#include <filesystem>
#include <optional>
#include <string>
#include <thread>

#include "absl/log/absl_check.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "aos/realtime.h"
#include "aos/testing/tmpdir.h"

namespace aos::util::testing {

using ::testing::ElementsAre;

// Basic test of reading a normal file.
TEST(FileTest, ReadNormalFile) {
  const std::filesystem::path tmpdir(aos::testing::TestTmpDir());
  const std::filesystem::path test_file = tmpdir / "test_file";
  WriteStringToFileOrDie(test_file.string(), "contents\n");
  EXPECT_EQ("contents\n", ReadFileToStringOrDie(test_file.string()));
}

// Basic test of reading a normal file.
TEST(FileTest, ReadNormalFileToBytes) {
  const std::filesystem::path tmpdir(aos::testing::TestTmpDir());
  const std::filesystem::path test_file = tmpdir / "test_file";
  WriteStringToFileOrDie(test_file.string(), "contents\n");
  EXPECT_THAT(ReadFileToVecOrDie(test_file.string()),
              ElementsAre('c', 'o', 'n', 't', 'e', 'n', 't', 's', '\n'));
}

#ifndef _WIN32
// Tests reading a file with 0 size that has content (like /proc files or
// pipes).
TEST(FileTest, ReadZeroSizeFileWithContent) {
  const std::filesystem::path tmpdir(aos::testing::TestTmpDir());
  const std::filesystem::path test_file = tmpdir / "test_pipe";

  // Create a named pipe.
  ASSERT_EQ(0, mkfifo(test_file.string().c_str(), 0666));

  std::thread writer([test_file]() {
    // Open the pipe for writing.
    int fd = open(test_file.string().c_str(), O_WRONLY);
    ABSL_PCHECK(fd != -1);

    // Write some data.
    const std::string data = "some data";
    const ssize_t result = ::write(fd, data.data(), data.size());
    ABSL_PCHECK(result == static_cast<ssize_t>(data.size()));

    // Close to signal EOF.
    CloseFd(fd);
  });

  // Read from the pipe.
  std::string contents = ReadFileToStringOrDie(test_file.string());
  EXPECT_EQ("some data", contents);

  writer.join();
}
#endif

#ifdef __linux__
// These rely on /proc, which is a linux specific invention

// Tests reading a file with 0 size, among other weird things.
TEST(FileTest, ReadSpecialFile) {
  const std::string stat = ReadFileToStringOrDie("/proc/self/stat");
  EXPECT_EQ('\n', stat[stat.size() - 1]);
  const std::string my_pid = ::std::to_string(getpid());
  EXPECT_EQ(my_pid, stat.substr(0, my_pid.size()));
}

// Tests maybe reading a file with 0 size, among other weird things.
TEST(FileTest, MaybeReadSpecialFile) {
  const std::optional<std::string> stat =
      MaybeReadFileToString("/proc/self/stat");
  ASSERT_TRUE(stat.has_value());
  EXPECT_EQ('\n', (*stat)[stat->size() - 1]);
  const std::string my_pid = std::to_string(getpid());
  EXPECT_EQ(my_pid, stat->substr(0, my_pid.size()));
}
#endif

// Basic test of maybe reading a normal file.
TEST(FileTest, MaybeReadNormalFile) {
  const std::filesystem::path tmpdir(aos::testing::TestTmpDir());
  const std::filesystem::path test_file = tmpdir / "test_file";
  WriteStringToFileOrDie(test_file.string(), "contents\n");
  EXPECT_EQ("contents\n", MaybeReadFileToString(test_file.string()).value());
}

// Tests maybe reading a non-existent file, and not fatally erroring.
TEST(FileTest, MaybeReadNonexistentFile) {
  const std::optional<std::string> contents = MaybeReadFileToString("/dne");
  ASSERT_FALSE(contents.has_value());
}

// Tests that the PathExists function works under normal conditions.
TEST(FileTest, PathExistsTest) {
  const std::filesystem::path tmpdir(aos::testing::TestTmpDir());
  const std::filesystem::path test_file = tmpdir / "test_file";
  // Make sure the test_file doesn't exist.
  std::filesystem::remove(test_file);
  EXPECT_FALSE(PathExists(test_file.string()));

  WriteStringToFileOrDie(test_file.string(), "abc");

  EXPECT_TRUE(PathExists(test_file.string()));
}

// Basic test of reading a normal file.
TEST(FileTest, ReadNormalFileNoMalloc) {
  const std::filesystem::path tmpdir(aos::testing::TestTmpDir());
  const std::filesystem::path test_file = tmpdir / "test_file";
  // Make sure to include a string long enough to avoid small string
  // optimization.
  WriteStringToFileOrDie(test_file.string(), "123456789\n");

  FileReader reader(test_file.string());
  EXPECT_TRUE(reader.is_open());

  aos::ScopedRealtime realtime;
  {
    std::array<char, 20> contents;
    std::optional<absl::Span<char>> read_result =
        reader.ReadContents({contents.data(), contents.size()});
    EXPECT_EQ("123456789\n",
              std::string_view(read_result->data(), read_result->size()));
  }
  {
    std::optional<std::array<char, 10>> read_result = reader.ReadString<10>();
    ASSERT_TRUE(read_result.has_value());
    EXPECT_EQ("123456789\n",
              std::string_view(read_result->data(), read_result->size()));
  }
  EXPECT_EQ(123456789, reader.ReadInt32());
}

// Test reading a non-existent file.
TEST(FileDeathTest, ReadNonExistentFile) {
  const ::std::string test_file = "/dne";

  // If error_type flag is not set or set to kFatal, this should fail.
  EXPECT_DEATH(FileReader reader(test_file),
               "opening " + test_file + ": No such file or directory");

  FileReaderErrorType error_type = FileReaderErrorType::kFatal;
  EXPECT_DEATH(FileReader reader(test_file, error_type),
               "opening " + test_file + ": No such file or directory");

  // If warning flag is set to true, read should not fail, is_open() should
  // return false, ReadContents() and ReadInt32() should fail.
  error_type = FileReaderErrorType::kNonFatal;
  FileReader reader(test_file, error_type);
  EXPECT_FALSE(reader.is_open());
  std::array<char, 16> contents;
  EXPECT_DEATH(
      reader.ReadContents(absl::Span<char>(contents.data(), contents.size())),
      "Bad file descriptor");
  EXPECT_DEATH(reader.ReadInt32(), "Bad file descriptor");
}

// Tests that we can write to a file without malloc'ing.
TEST(FileTest, WriteNormalFileNoMalloc) {
  const std::filesystem::path tmpdir(aos::testing::TestTmpDir());
  const std::filesystem::path test_file = tmpdir / "test_file";

  FileWriter writer(test_file.string());

  FileWriter::WriteResult result;
  {
    aos::ScopedRealtime realtime;
    result = writer.WriteBytes("123456789");
  }
  EXPECT_EQ(9, result.bytes_written);
  EXPECT_EQ(9, result.return_code);
  EXPECT_EQ("123456789", ReadFileToStringOrDie(test_file.string()));
}

// Tests that if we fail to write a file that the error code propagates
// correctly.
TEST(FileTest, WriteFileError) {
  const std::filesystem::path tmpdir(aos::testing::TestTmpDir());
  const std::filesystem::path test_file = tmpdir / "test_file";

  // Open with only read permissions; this should cause things to fail.
  FileWriter writer(test_file.string(), std::filesystem::perms::owner_read);

  // Mess up the file management by closing the file descriptor.
  ABSL_PCHECK(0 == CloseFd(writer.fd()));

  FileWriter::WriteResult result;
  {
    aos::ScopedRealtime realtime;
    result = writer.WriteBytes("123456789");
  }
  EXPECT_EQ(0, result.bytes_written);
  EXPECT_EQ(-1, result.return_code);
  EXPECT_EQ("", ReadFileToStringOrDie(test_file.string()));
}

// Tests that SyncDirectory opens, fsyncs, and closes a directory.
TEST(FileTest, SyncDirectory) {
  // Create a temporary directory.
  const std::filesystem::path tmp_dir = aos::testing::TestTmpDir();
  const std::filesystem::path new_dir = tmp_dir / "sync_dir_test" / "";

  ASSERT_FALSE(PathExists(new_dir.string()));
  MkdirP(new_dir.string(), std::filesystem::perms::all);
  ASSERT_TRUE(PathExists(new_dir.string()));

  // Call SyncDirectory and check that no errors occur.
  EXPECT_NO_FATAL_FAILURE(SyncDirectory(new_dir));

  // Clean up the directory.
  UnlinkRecursive(new_dir.string());
}

// Tests that MkdirPIfSpace creates the directory with and without syncing.
TEST(FileTest, MkdirPIfSpace) {
  const std::filesystem::path tmp_dir = aos::testing::TestTmpDir();
  const std::filesystem::path base_dir = tmp_dir / "mkdir_p_if_space" / "";
  const std::filesystem::path new_dir_sync =
      base_dir / "sync" / "a" / "b" / "c" / "";
  const std::filesystem::path new_dir_nosync =
      base_dir / "nosync" / "a" / "b" / "c" / "";

  // Clean-up from any previous failures.
  UnlinkRecursive(base_dir.string());

  // Test with syncing enabled.
  ASSERT_FALSE(PathExists(new_dir_sync.string()));
  ASSERT_TRUE(
      MkdirPIfSpace(new_dir_sync.string(), std::filesystem::perms::all, true));
  ASSERT_TRUE(PathExists(new_dir_sync.string()));
  EXPECT_TRUE(std::filesystem::is_directory(new_dir_sync));
  // When sync is true, both the created directory and its parent directory
  // should be synced.
  // TODO(austin): Confirm that fsync was called on both directories. This is
  // hard.

  // Test without syncing.
  ASSERT_FALSE(PathExists(new_dir_nosync.string()));
  ASSERT_TRUE(MkdirPIfSpace(new_dir_nosync.string(),
                            std::filesystem::perms::all, false));
  ASSERT_TRUE(PathExists(new_dir_nosync.string()));
  EXPECT_TRUE(std::filesystem::is_directory(new_dir_nosync));

  // Test permissions.
  const std::filesystem::path new_dir_perms =
      base_dir / "perms" / "a" / "b" / "c" / "";
  ASSERT_FALSE(PathExists(new_dir_perms.string()));
  ASSERT_TRUE(MkdirPIfSpace(new_dir_perms.string(),
                            std::filesystem::perms::owner_all, false));
  ASSERT_TRUE(PathExists(new_dir_perms.string()));
  EXPECT_TRUE(std::filesystem::is_directory(new_dir_perms));
#ifndef _WIN32
  EXPECT_EQ(std::filesystem::status(new_dir_perms).permissions(),
            std::filesystem::perms::owner_all);
  // Also check that parent directories have the permission applied.
  EXPECT_EQ(
      std::filesystem::status(base_dir / "perms" / "a" / "b").permissions(),
      std::filesystem::perms::owner_all);
  EXPECT_EQ(std::filesystem::status(base_dir / "perms" / "a").permissions(),
            std::filesystem::perms::owner_all);
#endif

  UnlinkRecursive(base_dir.string());
}

TEST(FileTest, MkdirPIfSpaceEdgeCases) {
  const std::filesystem::path tmp_dir = aos::testing::TestTmpDir();
  const std::filesystem::path base_dir = tmp_dir / "mkdir_p_edge_cases" / "";

  // Clean-up from any previous failures.
  UnlinkRecursive(base_dir.string());

  // 1. Path with no parent directory component (parent_path() is empty)
  EXPECT_TRUE(
      MkdirPIfSpace("just_file_name.txt", std::filesystem::perms::all, false));
  EXPECT_TRUE(MkdirPIfSpace("", std::filesystem::perms::all, false));

  // 2. Folder already exists
  const std::filesystem::path existing_dir_file =
      base_dir / "existing_dir" / "file.txt";
  ASSERT_TRUE(MkdirPIfSpace(existing_dir_file.string(),
                            std::filesystem::perms::all, false));
  EXPECT_TRUE(PathExists((base_dir / "existing_dir").string()));
  // Calling it again on the same path where directory already exists
  EXPECT_TRUE(MkdirPIfSpace(existing_dir_file.string(),
                            std::filesystem::perms::all, false));

  // 3. Trailing slashes
  const std::filesystem::path trailing_slash_path =
      base_dir / "trailing_slash" / "";
  ASSERT_TRUE(MkdirPIfSpace(trailing_slash_path.string(),
                            std::filesystem::perms::all, false));
  EXPECT_TRUE(PathExists((base_dir / "trailing_slash").string()));

  const std::filesystem::path multi_trailing_slashes =
      base_dir / "multi_trailing" / "" / "" / "";
  ASSERT_TRUE(MkdirPIfSpace(multi_trailing_slashes.string(),
                            std::filesystem::perms::all, false));
  EXPECT_TRUE(PathExists((base_dir / "multi_trailing").string()));

  UnlinkRecursive(base_dir.string());
}

TEST(FileTest, MaybeWriteStringToExistingFile) {
  const std::filesystem::path tmpdir(aos::testing::TestTmpDir());
  const std::filesystem::path test_file = tmpdir / "maybe_write_test";
  WriteStringToFileOrDie(test_file.string(), "original");

  EXPECT_TRUE(MaybeWriteStringToFile(test_file.string(), "replaced"));
  EXPECT_EQ("replaced", ReadFileToStringOrDie(test_file.string()));
}

TEST(FileTest, MaybeWriteStringToNonexistentFile) {
  const std::filesystem::path tmpdir(aos::testing::TestTmpDir());
  const std::filesystem::path test_file = tmpdir / "dne_maybe_write_test";
  EXPECT_TRUE(MaybeWriteStringToFile(test_file.string(), "data"));
}

TEST(FileTest, MaybeWriteStringTruncates) {
  const std::filesystem::path tmpdir(aos::testing::TestTmpDir());
  const std::filesystem::path test_file = tmpdir / "maybe_write_trunc";
  WriteStringToFileOrDie(test_file.string(), "abcdef");

  EXPECT_TRUE(MaybeWriteStringToFile(test_file.string(), "xy"));
  EXPECT_EQ("xy", ReadFileToStringOrDie(test_file.string()));
}

TEST(FileTest, MaybeWriteStringEmptyContents) {
  const std::filesystem::path tmpdir(aos::testing::TestTmpDir());
  const std::filesystem::path test_file = tmpdir / "maybe_write_empty";
  WriteStringToFileOrDie(test_file.string(), "original");

  EXPECT_TRUE(MaybeWriteStringToFile(test_file.string(), ""));
  EXPECT_EQ("", ReadFileToStringOrDie(test_file.string()));
}

TEST(FileTest, GetExecutablePath) {
  std::optional<std::filesystem::path> exec_path = GetExecutablePath();
  ASSERT_TRUE(exec_path.has_value());
  EXPECT_TRUE(std::filesystem::exists(*exec_path));
  EXPECT_TRUE(exec_path->is_absolute());
}

}  // namespace aos::util::testing
