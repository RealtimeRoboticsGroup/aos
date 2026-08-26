#include "aos/events/logging/log_backend.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <random>

#include "absl/flags/declare.h"
#include "absl/flags/flag.h"
#include "absl/flags/reflection.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "aos/containers/resizeable_buffer.h"
#include "aos/sanitizers.h"
#include "aos/testing/tmpdir.h"
#include "aos/util/file.h"

ABSL_DECLARE_FLAG(bool, sync);

namespace aos::logger::testing {

namespace {
// Helper to write simple string to the log sink
WriteResult Write(LogSink *log_sink, std::string_view content) {
  auto span = absl::Span<const uint8_t>(
      reinterpret_cast<const unsigned char *>(content.data()), content.size());
  auto queue = absl::Span<const absl::Span<const uint8_t>>(&span, 1);
  return log_sink->Write(queue);
}
std::string TestTmpDir() {
  return aos::testing::TestTmpDir() + "/" +
         ::testing::UnitTest::GetInstance()->current_test_info()->name();
}
}  // namespace

MATCHER_P(FileEq, o, "") { return arg.name == o.name && arg.size == o.size; }

TEST(LogBackendTest, CreateSimpleFile) {
  const std::string logevent = TestTmpDir() + "/logevent/";
  const std::string filename = "test.bfbs";
  LogFolder backend(logevent, false);
  auto file = backend.RequestFile(filename);
  ASSERT_EQ(file->OpenForWrite(), WriteCode::kOk);
  auto result = Write(file.get(), "test");
  EXPECT_EQ(result.code, WriteCode::kOk);
  EXPECT_EQ(result.messages_written, 1);
  EXPECT_EQ(file->Close(), WriteCode::kOk);
  EXPECT_TRUE(std::filesystem::exists(logevent + filename));

  EXPECT_THAT(backend.ListFiles(),
              ::testing::ElementsAre(FileEq(LogSource::File{
                  .name = filename,
                  .size = 4,
              })));

  auto decoder = backend.GetDecoder(filename);
  std::vector<uint8_t> buffer;
  buffer.resize(10);
  const auto count = decoder->Read(buffer.data(), buffer.data() + 10);
  ASSERT_EQ(count, 4);
  buffer.resize(4);
  std::string view(buffer.begin(), buffer.end());
  EXPECT_EQ(view, "test");
}

// ListFiles() matches the paths FileOperations reports against the backend's
// own base name.  The two are produced differently -- FindLogs() reports
// generic_string()s, while the base name is whatever the caller handed us -- so
// any disagreement in how the same directory is spelled trips the CHECK in
// ListFiles() for every file found.
//
// Redundant separators are a disagreement we can construct on Linux:
// generic_string() collapses "logevent//" to "logevent/", the raw base name
// keeps both slashes, and matching one against the other fails.  On Windows the
// same mismatch shows up for the far more common reason that the base name is a
// native path spelled with backslashes.
TEST(LogBackendTest, ListFilesWithRedundantSeparators) {
  const std::string logevent = TestTmpDir() + "/logevent//";
  const std::string filename = "test.bfbs";
  std::filesystem::remove_all(logevent);

  LogFolder backend(logevent, false);
  auto file = backend.RequestFile(filename);
  ASSERT_EQ(file->OpenForWrite(), WriteCode::kOk);
  ASSERT_EQ(Write(file.get(), "test").code, WriteCode::kOk);
  ASSERT_EQ(file->Close(), WriteCode::kOk);

  EXPECT_THAT(backend.ListFiles(),
              ::testing::ElementsAre(FileEq(LogSource::File{
                  .name = filename,
                  .size = 4,
              })));
}

TEST(FileHandlerTest, CloseSyncsDirectory) {
  absl::FlagSaver flag_saver;
  absl::SetFlag(&FLAGS_sync, true);

  const std::string logevent = TestTmpDir() + "/";
  const std::string filename = "test.log";
  const std::string filepath = logevent + filename;

  // Create the directory.
  ASSERT_TRUE(
      aos::util::MkdirPIfSpace(logevent, std::filesystem::perms::all, true));

  // Create a FileHandler.
  FileHandler file_handler(filepath, false);

  // Open, write, and close the file.
  ASSERT_EQ(file_handler.OpenForWrite(), WriteCode::kOk);
  auto result = Write(&file_handler, "test");
  EXPECT_EQ(result.code, WriteCode::kOk);
  EXPECT_EQ(result.messages_written, 1);
  EXPECT_EQ(file_handler.Close(), WriteCode::kOk);

  // Check that the file exists (created by MkdirPIfSpace with sync = true).
  EXPECT_TRUE(std::filesystem::exists(filepath));

  // Cleanup
  std::filesystem::remove(filepath);
}

class FileHandlerSyncTest : public ::testing::TestWithParam<bool> {};

// --sync promises to "sync the file after each written block", so the data has
// to be pushed at the disk during Write() and not only by Close().  Otherwise a
// crash in between silently loses writes that were already reported as durable.
//
// Every backend syncs by a different mechanism (sync_file_range() on Linux,
// F_FULLFSYNC on Darwin, _commit() on Windows) and none of them are observable
// from the filesystem afterwards, so this checks that the syncs were issued.
// The OSX backend regressed exactly here once, by dropping its F_FULLFSYNC.
TEST_P(FileHandlerSyncTest, SyncsDataDuringWrites) {
  const bool sync = GetParam();

  absl::FlagSaver flag_saver;
  absl::SetFlag(&FLAGS_sync, sync);

  const std::string logevent = TestTmpDir() + "/";
  const std::string filepath = logevent + "test.log";
  ASSERT_TRUE(
      aos::util::MkdirPIfSpace(logevent, std::filesystem::perms::all, sync));

  FileHandler file_handler(filepath, false);
  ASSERT_EQ(file_handler.OpenForWrite(), WriteCode::kOk);
  ASSERT_EQ(file_handler.data_sync_count(), 0u);

  ASSERT_EQ(Write(&file_handler, "test").code, WriteCode::kOk);
  const size_t after_first_write = file_handler.data_sync_count();

  ASSERT_EQ(Write(&file_handler, "more").code, WriteCode::kOk);
  const size_t after_second_write = file_handler.data_sync_count();

  if (sync) {
    EXPECT_GT(after_first_write, 0u)
        << "--sync is set, but Write() never synced.  A crash before Close() "
           "would lose data we already acknowledged as written.";
    EXPECT_GT(after_second_write, after_first_write)
        << "--sync syncs after *each* written block, but the second Write() "
           "didn't sync.";
  } else {
    EXPECT_EQ(after_second_write, 0u)
        << "--sync is off, so writes shouldn't be paying for syncs.";
  }

  EXPECT_EQ(file_handler.Close(), WriteCode::kOk);
  std::filesystem::remove(filepath);
}

INSTANTIATE_TEST_SUITE_P(FileHandlerSync, FileHandlerSyncTest,
                         ::testing::Bool(),
                         [](const ::testing::TestParamInfo<bool> &info) {
                           return info.param ? "SyncEnabled" : "SyncDisabled";
                         });

namespace {
// Forces every platform sync to report a full disk.  Reaching a real ENOSPC
// out of fdatasync()/F_FULLFSYNC/_commit() would take a genuinely full
// filesystem, so the failure is injected at the one seam all three share.
class OutOfSpaceSyncFileHandler : public FileHandler {
 public:
  using FileHandler::FileHandler;

  // How many times a sync actually reached this override.  Which paths sync
  // through here is backend-specific, so the test keys off this rather than
  // assuming.
  size_t sync_calls() const { return sync_calls_; }

 private:
  WriteCode PlatformSyncImpl() override {
    ++sync_calls_;
    return WriteCode::kOutOfSpace;
  }

  size_t sync_calls_ = 0;
};
}  // namespace

// A sync that fails with ENOSPC leaves the data non-durable, so it has to come
// back as kOutOfSpace instead of a clean result.  Answering kOk tells the
// logger the batch is safely on disk when it isn't: DetachedBufferWriter::
// Flush() then Clear()s it out of the encoder as durably written and keeps
// logging, rather than winding the log down.
//
// The Windows backend regressed exactly here once, by routing its per-write
// _commit() through PlatformSync() and dropping the return value.
//
// Which calls reach PlatformSyncImpl() differs per backend, so the per-write
// half is asserted per platform rather than only where a sync happens to have
// landed -- a runtime "if it synced" check would silently assert nothing on
// Linux, which is the only platform CI builds this on.  Close() syncs through
// the seam on all three.
TEST(LogBackendTest, SyncOutOfSpaceIsReported) {
  absl::FlagSaver flag_saver;
  absl::SetFlag(&FLAGS_sync, true);

  const std::string logevent = TestTmpDir() + "/";
  const std::string filepath = logevent + "test.log";
  ASSERT_TRUE(
      aos::util::MkdirPIfSpace(logevent, std::filesystem::perms::all, true));

  OutOfSpaceSyncFileHandler file_handler(filepath, false);
  ASSERT_EQ(file_handler.OpenForWrite(), WriteCode::kOk);

  const WriteResult result = Write(&file_handler, "test");
#if defined(__linux__)
  // Linux pushes each write out with sync_file_range() inside WriteV() and
  // never routes it through PlatformSyncImpl(), so a failing platform sync
  // can't reach this write.  Pinned rather than left unasserted: if Linux ever
  // starts syncing writes through the seam, this fires and the kOutOfSpace
  // expectation below it becomes the one that applies.
  EXPECT_EQ(file_handler.sync_calls(), 0u)
      << "Linux started syncing writes through PlatformSyncImpl(); it now has "
         "to propagate the failure the way Darwin and Windows do.";
  EXPECT_EQ(result.code, WriteCode::kOk);
#else
  // Darwin (F_FULLFSYNC) and Windows (_commit()) sync every write through the
  // seam, so the failure has to come back to the caller.
  EXPECT_GT(file_handler.sync_calls(), 0u)
      << "--sync is set, so each write has to flush through "
         "PlatformSyncImpl().";
  EXPECT_EQ(result.code, WriteCode::kOutOfSpace)
      << "The per-write sync reported running out of space, but Write() came "
         "back clean.  The logger would drop this batch from memory as durably "
         "written and keep going.";
#endif

  const size_t after_write = file_handler.sync_calls();
  EXPECT_EQ(file_handler.Close(), WriteCode::kOutOfSpace)
      << "Close() ran out of space while flushing, but reported a clean close.";
  EXPECT_GT(file_handler.sync_calls(), after_write)
      << "--sync is set, so Close() has to flush through PlatformSyncImpl().";

  std::filesystem::remove(filepath);
}

TEST(LogBackendTest, CreateRenamableFile) {
  const std::string logevent = TestTmpDir() + "/logevent/";
  auto backend = MakeRenamableFileBackend(logevent, false);
  auto file = backend->RequestFile("test.log");
  ASSERT_EQ(file->OpenForWrite(), WriteCode::kOk);
  auto result = Write(file.get(), "test");
  EXPECT_EQ(result.code, WriteCode::kOk);
  EXPECT_EQ(result.messages_written, 1);
  EXPECT_EQ(file->Close(), WriteCode::kOk);
  EXPECT_TRUE(std::filesystem::exists(logevent + "test.log"));
  EXPECT_EQ(4, std::filesystem::file_size(logevent + "test.log"));
}

TEST(LogBackendTest, CreateAsyncFile) {
  const std::string logevent = TestTmpDir() + "/logevent/";
  // Test this a bunch of times to try to exercise any potential deadlocks/races
  // in the threading code.
  for (int i = 0; i < 1000; ++i) {
    SCOPED_TRACE(i);
    auto backend = MakeRenamableFileBackend(logevent, false);
    auto file = backend->RequestFile("test.log", 1024);
    ASSERT_EQ(file->OpenForWrite(), WriteCode::kOk);
    for (int i = 0; i < 1000; ++i) {
      auto result = Write(file.get(), "testing");
      EXPECT_EQ(result.code, WriteCode::kOk);
      EXPECT_EQ(result.messages_written, 1);
    }
    EXPECT_EQ(file->Close(), WriteCode::kOk);
    EXPECT_TRUE(std::filesystem::exists(logevent + "test.log"));
    EXPECT_EQ(7000, std::filesystem::file_size(logevent + "test.log"));
    std::filesystem::remove(logevent + "test.log");
  }
}

TEST(LogBackendTest, OutOfSpaceTest) {
  const std::string logevent = TestTmpDir() + "/logevent/";
  // Test this a bunch of times to try to exercise any potential deadlocks/races
  // in the threading code.
  for (int i = 0; i < 1000; ++i) {
    SCOPED_TRACE(i);
    auto backend = MakeRenamableFileBackend(logevent, false);
    auto file = backend->RequestFile("test.log", 1024);
    BufferedFileHandler *file_handler =
        dynamic_cast<BufferedFileHandler *>(file.get());
    ASSERT_EQ(file->OpenForWrite(), WriteCode::kOk);
    std::array<char, 700> buffer;
    for (size_t j = 0; j < buffer.size(); ++j) {
      buffer[j] = j;
    }
    for (int i = 0; i < 100; ++i) {
      auto result = Write(file.get(), {buffer.data(), buffer.size()});
      EXPECT_EQ(result.code, WriteCode::kOk);
      EXPECT_EQ(result.messages_written, 1);
    }
    // Force the LogBackend to pretend that it ran out of space (note that this
    // does not directly stop it from writing to disk, and so is not a perfect
    // simulation of actually running out of space).
    // Trigger this in a separate thread so that we exercise multiple
    // code-paths.
    std::thread out_of_space_trigger([&file_handler]() {
      std::unique_lock lock(file_handler->buffer_index_mutex_);
      file_handler->ran_out_of_space_in_thread_ = true;
    });
    // Write a bunch of times, and force the signalling thread to join partway
    // through so that we guarantee a trigger.
    bool got_out_of_space = false;
    for (int i = 0; i < 100; ++i) {
      if (i == 50) {
        out_of_space_trigger.join();
      }
      if (Write(file.get(), {buffer.data(), buffer.size()}).code ==
          WriteCode::kOutOfSpace) {
        got_out_of_space = true;
      }
    }
    EXPECT_TRUE(got_out_of_space);
    file.reset();
    std::filesystem::remove(logevent + "test.log");
  }
}

// Once a handler has lost data to a full disk it stays stopped: writes report
// kOutOfSpace from then on instead of appending to a log that already has a
// hole in it.
//
// Critically, that has to hold for a handler which is also *closed*.  The
// Windows rename path closes handlers behind the logger's back and, when the
// flush runs out of space, leaves them closed rather than reopening a log it
// can no longer trust.  Without the guard, the next write would go to fd -1 and
// PLOG(FATAL) on EBADF instead of winding down gracefully.
TEST(LogBackendTest, StopsWritingAfterOutOfSpace) {
  // Exposes the protected hook the backends use to retire a handler.
  class TestFileHandler : public BufferedFileHandler {
   public:
    using BufferedFileHandler::BufferedFileHandler;
    using BufferedFileHandler::MarkRanOutOfSpace;
  };

  const std::string logevent = TestTmpDir() + "/logevent/";

  // Both a buffered and an unbuffered handler; the writing thread only latches
  // this for the buffered one, so the unbuffered case is the one that would
  // otherwise reach a closed descriptor.
  for (const size_t memory_buffer_size : {size_t{0}, size_t{1024}}) {
    SCOPED_TRACE(memory_buffer_size);
    std::filesystem::remove_all(logevent);
    std::filesystem::create_directories(logevent);

    TestFileHandler handler(logevent + "test.log", false, memory_buffer_size);
    ASSERT_EQ(handler.OpenForWrite(), WriteCode::kOk);
    ASSERT_EQ(Write(&handler, "good").code, WriteCode::kOk);

    // Retire it the way the rename path does: close first (which flushes and
    // is where the real caller sees kOutOfSpace come back), then mark it and
    // leave it closed rather than reopening.
    ASSERT_EQ(handler.Close(), WriteCode::kOk);
    ASSERT_FALSE(handler.is_open());
    handler.MarkRanOutOfSpace();

    // Writing to the closed, retired handler reports out of space instead of
    // touching the descriptor.
    for (int i = 0; i < 3; ++i) {
      const WriteResult result = Write(&handler, "dropped");
      EXPECT_EQ(result.code, WriteCode::kOutOfSpace);
      EXPECT_EQ(result.messages_written, 0);
      EXPECT_EQ(result.bytes_written, 0);
    }

    // Only what made it to disk before we stopped is there.
    EXPECT_EQ(std::filesystem::file_size(logevent + "test.log"), 4u);
  }
}

// Closing has to report that the writing thread ran out of space.  That thread
// is the only thing that knows the buffered data never reached disk, and the
// descriptor closing cleanly says nothing about it -- so a Close() that only
// forwards FileHandler::Close() hands back kOk after silently dropping data.
//
// Callers rely on this: DetachedBufferWriter latches ran_out_of_space_ from
// Close()'s return, and the Windows rename path decides from it whether the log
// is still worth renaming.
TEST(LogBackendTest, CloseReportsBufferedOutOfSpace) {
  class TestFileHandler : public BufferedFileHandler {
   public:
    using BufferedFileHandler::BufferedFileHandler;
    using BufferedFileHandler::MarkRanOutOfSpace;
  };

  const std::string logevent = TestTmpDir() + "/logevent/";

  for (const size_t memory_buffer_size : {size_t{0}, size_t{1024}}) {
    SCOPED_TRACE(memory_buffer_size);
    std::filesystem::remove_all(logevent);
    std::filesystem::create_directories(logevent);

    TestFileHandler handler(logevent + "test.log", false, memory_buffer_size);
    ASSERT_EQ(handler.OpenForWrite(), WriteCode::kOk);
    ASSERT_EQ(Write(&handler, "good").code, WriteCode::kOk);

    // Stands in for the writing thread hitting ENOSPC mid-flush.
    handler.MarkRanOutOfSpace();

    EXPECT_EQ(handler.Close(), WriteCode::kOutOfSpace);
  }
}

TEST(LogBackendTest, CreateFileMassiveWrite) {
#ifdef AOS_SANITIZE_MEMORY
  GTEST_SKIP() << "Skipping CreateFileMassiveWrite under MSan due to high "
                  "memory usage (>3GB + MSan overhead)";
#endif
  const std::string logevent = TestTmpDir() + "/logevent/";
  auto backend = MakeRenamableFileBackend(logevent, false);
  auto file = backend->RequestFile("test.log");
  ASSERT_EQ(file->OpenForWrite(), WriteCode::kOk);
  std::vector<uint8_t> buffer1(3e9, 0);
  std::vector<uint8_t> buffer2(3, 0);
  std::array<absl::Span<const uint8_t>, 2> spans{
      {{buffer1.data(), buffer1.size()}, {buffer2.data(), buffer2.size()}}};
  auto queue =
      absl::Span<const absl::Span<const uint8_t>>(spans.data(), spans.size());
  auto result = file->Write(queue);
  EXPECT_EQ(result.code, WriteCode::kOk);
  EXPECT_EQ(result.messages_written, 2);
  EXPECT_EQ(file->Close(), WriteCode::kOk);
  EXPECT_TRUE(std::filesystem::exists(logevent + "test.log"));
  EXPECT_EQ(buffer1.size() + buffer2.size(),
            std::filesystem::file_size(logevent + "test.log"));
#if !defined(_WIN32) && !defined(__APPLE__)
  ASSERT_TRUE(
      dynamic_cast<FileHandler *>(file.get())->encountered_incomplete_write());
#endif
  std::filesystem::remove(logevent + "test.log");
}

TEST(LogBackendTest, CreateAsyncFileLargeWrites) {
  const std::string logevent = TestTmpDir() + "/logevent/";
  auto backend = MakeRenamableFileBackend(logevent, false);
  auto file = backend->RequestFile("test.log", 1024);
  ASSERT_EQ(file->OpenForWrite(), WriteCode::kOk);
  std::array<char, 700> buffer;
  for (size_t i = 0; i < buffer.size(); ++i) {
    buffer[i] = i;
  }
  for (int i = 0; i < 1000; ++i) {
    auto result = Write(file.get(), {buffer.data(), buffer.size()});
    EXPECT_EQ(result.code, WriteCode::kOk);
    EXPECT_EQ(result.messages_written, 1);
  }
  EXPECT_EQ(file->Close(), WriteCode::kOk);
  EXPECT_TRUE(std::filesystem::exists(logevent + "test.log"));
  EXPECT_EQ(700'000, std::filesystem::file_size(logevent + "test.log"));
  std::string file_contents =
      aos::util::ReadFileToStringOrDie(logevent + "test.log");
  while (!file_contents.empty()) {
    for (size_t i = 0; i < buffer.size(); ++i) {
      SCOPED_TRACE(i);
      ASSERT_EQ(file_contents[i], buffer[i]);
    }
    file_contents = file_contents.substr(buffer.size());
  }
}

TEST(LogBackendTest, UseTempRenamableFile) {
  const std::string logevent = TestTmpDir() + "/logevent/";
  auto backend = MakeRenamableFileBackend(logevent, false);
  backend->EnableTempFiles();
  auto file = backend->RequestFile("test.log");
  ASSERT_EQ(file->OpenForWrite(), WriteCode::kOk);
  auto result = Write(file.get(), "test");
  EXPECT_EQ(result.code, WriteCode::kOk);
  EXPECT_EQ(result.messages_written, 1);
  EXPECT_TRUE(std::filesystem::exists(logevent + "test.log.tmp"));

  EXPECT_EQ(file->Close(), WriteCode::kOk);
  // Check that file is renamed.
  EXPECT_TRUE(std::filesystem::exists(logevent + "test.log"));
}

// Active directory renaming tests are supported on Windows by temporarily
// closing active file handlers inside RenameLogBase before renaming, and then
// reopening them at the new path.
TEST(LogBackendTest, RenameBaseAfterWrite) {
  const std::string logevent = TestTmpDir() + "/logevent/";
  auto backend = MakeRenamableFileBackend(logevent, false);
  auto file = backend->RequestFile("test.log");
  ASSERT_EQ(file->OpenForWrite(), WriteCode::kOk);
  auto result = Write(file.get(), "test");
  EXPECT_EQ(result.code, WriteCode::kOk);
  EXPECT_EQ(result.messages_written, 1);
  EXPECT_TRUE(std::filesystem::exists(logevent + "test.log"));

  std::string renamed = TestTmpDir() + "/renamed/";
  backend->RenameLogBase(renamed);

  EXPECT_FALSE(std::filesystem::exists(logevent + "test.log"));
  EXPECT_TRUE(std::filesystem::exists(renamed + "test.log"));

  EXPECT_EQ(file->Close(), WriteCode::kOk);
  // Check that file is renamed.
  EXPECT_TRUE(std::filesystem::exists(renamed + "test.log"));
}

// UpdateFilenameBase() has to rewrite the base even when the new base name is a
// prefix of the old one.  Deciding "already updated" by testing whether the
// filename starts with the *new* base gets this backwards: ".../run" is a
// prefix of ".../run-old/data.bfbs", so the rewrite gets skipped and the
// handler keeps the path it had before the rename.
TEST(LogBackendTest, UpdateFilenameBaseToPrefixOfItself) {
  class TestFileHandler : public FileHandler {
   public:
    using FileHandler::FileHandler;
    using FileHandler::UpdateFilenameBase;
  };

  // Never opened, so this touches no filesystem.
  TestFileHandler handler("/logs/run-old/data.bfbs", false);
  handler.UpdateFilenameBase("/logs/run-old", "/logs/run");
  EXPECT_EQ(handler.name(), "/logs/run/data.bfbs");

  // Calling it again is a no-op rather than a second rewrite.
  handler.UpdateFilenameBase("/logs/run-old", "/logs/run");
  EXPECT_EQ(handler.name(), "/logs/run/data.bfbs");
}

// Renames the log base out from under a *buffered* handler and keeps writing
// through it afterwards.
//
// The other rename tests all use unbuffered handlers (memory_buffer_size of 0),
// so they never exercise the disk-writing thread.  That matters on backends
// which have to close and reopen their files to rename the directory (Windows
// can't rename a directory containing open files): closing stops the writing
// thread, and if reopening doesn't restart it, writes afterwards land in a
// buffer nothing ever drains -- the data silently never reaches the file, and
// once the buffer fills the writer blocks forever.
TEST(LogBackendTest, RenameBaseAfterBufferedWrite) {
  const std::string logevent = TestTmpDir() + "/logevent/";
  const std::string renamed = TestTmpDir() + "/renamed/";

  auto backend = MakeRenamableFileBackend(logevent, false);
  auto file = backend->RequestFile("test.log", 1024);
  ASSERT_EQ(file->OpenForWrite(), WriteCode::kOk);

  for (int i = 0; i < 100; ++i) {
    EXPECT_EQ(Write(file.get(), "before").code, WriteCode::kOk);
  }

  EXPECT_TRUE(backend->RenameLogBase(renamed));

  // Keep writing after the rename; these have to make it to disk too.
  for (int i = 0; i < 100; ++i) {
    EXPECT_EQ(Write(file.get(), "after").code, WriteCode::kOk);
  }

  EXPECT_EQ(file->Close(), WriteCode::kOk);

  EXPECT_FALSE(std::filesystem::exists(logevent + "test.log"));
  ASSERT_TRUE(std::filesystem::exists(renamed + "test.log"));
  // 100 * strlen("before") + 100 * strlen("after")
  EXPECT_EQ(std::filesystem::file_size(renamed + "test.log"), 1100u);
}

TEST(LogBackendTest, RenameBaseWithSync) {
  // Save the original flag value and restore it at the end of the test.
  absl::FlagSaver flag_saver;
  // Set the sync flag to true for this test.
  absl::SetFlag(&FLAGS_sync, true);

  // Use unique directory names for this test.
  const std::string logevent = TestTmpDir() + "/logevent_with_sync/";
  const std::string renamed = TestTmpDir() + "/renamed_with_sync/";

  auto backend = MakeRenamableFileBackend(logevent, false);
  auto file = backend->RequestFile("test.log");
  ASSERT_EQ(file->OpenForWrite(), WriteCode::kOk);
  auto result = Write(file.get(), "test");
  EXPECT_EQ(result.code, WriteCode::kOk);
  EXPECT_EQ(result.messages_written, 1);
  EXPECT_TRUE(std::filesystem::exists(logevent + "test.log"));

  // This will now sync the parent directories due to FLAGS_sync being true.
  EXPECT_TRUE(backend->RenameLogBase(renamed));

  EXPECT_FALSE(std::filesystem::exists(logevent + "test.log"));
  EXPECT_TRUE(std::filesystem::exists(renamed + "test.log"));

  EXPECT_EQ(file->Close(), WriteCode::kOk);
  // Check that file is renamed.
  EXPECT_TRUE(std::filesystem::exists(renamed + "test.log"));
}

TEST(LogBackendTest, UseTestAndRenameBaseAfterWrite) {
  const std::string logevent = TestTmpDir() + "/logevent/";
  auto backend = MakeRenamableFileBackend(logevent, false);
  backend->EnableTempFiles();
  auto file = backend->RequestFile("test.log");
  ASSERT_EQ(file->OpenForWrite(), WriteCode::kOk);
  auto result = Write(file.get(), "test");
  EXPECT_EQ(result.code, WriteCode::kOk);
  EXPECT_EQ(result.messages_written, 1);
  EXPECT_TRUE(std::filesystem::exists(logevent + "test.log.tmp"));

  std::string renamed = TestTmpDir() + "/renamed/";
  backend->RenameLogBase(renamed);

  EXPECT_FALSE(std::filesystem::exists(logevent + "test.log.tmp"));
  EXPECT_TRUE(std::filesystem::exists(renamed + "test.log.tmp"));

  EXPECT_EQ(file->Close(), WriteCode::kOk);
  // Check that file is renamed.
  EXPECT_TRUE(std::filesystem::exists(renamed + "test.log"));
}

TEST(QueueAlignmentDeathTest, Cases) {
  QueueAligner aligner;

  // Get a 512-byte-aligned pointer to a buffer. That buffer needs to be at
  // least 3 sectors big for the purposes of this test.
  uint8_t buffer[FileHandler::kSector * 4];
  void *aligned_start = buffer;
  size_t size = sizeof(buffer);
  ASSERT_TRUE(std::align(FileHandler::kSector, FileHandler::kSector * 3,
                         aligned_start, size) != nullptr);
  ASSERT_GE(size, FileHandler::kSector * 3);

  uint8_t *start = static_cast<uint8_t *>(aligned_start);
  {
    // Only prefix
    std::vector<absl::Span<const uint8_t>> queue;
    const uint8_t *current = start + 1;
    queue.emplace_back(current, FileHandler::kSector - 2);
    aligner.FillAlignedQueue(queue);
    ASSERT_EQ(aligner.aligned_queue().size(), 1);
    const auto &prefix = aligner.aligned_queue()[0];
    EXPECT_FALSE(prefix.aligned);
    EXPECT_EQ(prefix.size, FileHandler::kSector - 2);
  }
  {
    // Only main
    std::vector<absl::Span<const uint8_t>> queue;
    const uint8_t *current = start;
    queue.emplace_back(current, FileHandler::kSector);
    aligner.FillAlignedQueue(queue);
    ASSERT_EQ(aligner.aligned_queue().size(), 1);
    const auto &main = aligner.aligned_queue()[0];
    EXPECT_TRUE(main.aligned);
    EXPECT_EQ(main.size, FileHandler::kSector);
  }
  {
    // Empty
    std::vector<absl::Span<const uint8_t>> queue;
    const uint8_t *current = start;
    queue.emplace_back(current, 0);
    EXPECT_DEATH(aligner.FillAlignedQueue(queue),
                 "Nobody should be sending empty messages");
  }
  {
    // Main and suffix
    std::vector<absl::Span<const uint8_t>> queue;
    const uint8_t *current = start;
    queue.emplace_back(current, FileHandler::kSector + 1);
    aligner.FillAlignedQueue(queue);
    ASSERT_EQ(aligner.aligned_queue().size(), 2);

    const auto &main = aligner.aligned_queue()[0];
    EXPECT_TRUE(main.aligned);
    EXPECT_EQ(main.size, FileHandler::kSector);

    const auto &suffix = aligner.aligned_queue()[1];
    EXPECT_FALSE(suffix.aligned);
    EXPECT_EQ(suffix.size, 1);
  }
  {
    // Prefix, main
    std::vector<absl::Span<const uint8_t>> queue;
    const uint8_t *current = start + 1;
    queue.emplace_back(current, 2 * FileHandler::kSector - 1);
    aligner.FillAlignedQueue(queue);
    ASSERT_EQ(aligner.aligned_queue().size(), 2);

    const auto &prefix = aligner.aligned_queue()[0];
    EXPECT_FALSE(prefix.aligned);
    EXPECT_EQ(prefix.size, FileHandler::kSector - 1);

    const auto &main = aligner.aligned_queue()[1];
    EXPECT_TRUE(main.aligned);
    EXPECT_EQ(main.size, FileHandler::kSector);
  }
  {
    // Prefix and suffix
    std::vector<absl::Span<const uint8_t>> queue;
    const uint8_t *current = start + 1;
    queue.emplace_back(current, 2 * FileHandler::kSector - 2);
    aligner.FillAlignedQueue(queue);
    ASSERT_EQ(aligner.aligned_queue().size(), 2);

    const auto &prefix = aligner.aligned_queue()[0];
    EXPECT_FALSE(prefix.aligned);
    EXPECT_EQ(prefix.size, FileHandler::kSector - 1);

    const auto &suffix = aligner.aligned_queue()[1];
    EXPECT_FALSE(suffix.aligned);
    EXPECT_EQ(suffix.size, FileHandler::kSector - 1);
  }
  {
    // Prefix, main and suffix
    std::vector<absl::Span<const uint8_t>> queue;
    const uint8_t *current = start + 1;
    queue.emplace_back(current, 3 * FileHandler::kSector - 2);
    aligner.FillAlignedQueue(queue);
    ASSERT_EQ(aligner.aligned_queue().size(), 3);

    const auto &prefix = aligner.aligned_queue()[0];
    EXPECT_FALSE(prefix.aligned);
    EXPECT_EQ(prefix.size, FileHandler::kSector - 1);

    const auto &main = aligner.aligned_queue()[1];
    EXPECT_TRUE(main.aligned);
    EXPECT_EQ(main.size, FileHandler::kSector);

    const auto &suffix = aligner.aligned_queue()[2];
    EXPECT_FALSE(suffix.aligned);
    EXPECT_EQ(suffix.size, FileHandler::kSector - 1);
  }
}

// It represents calls to Write function (batching of calls and messages) where
// int values are sizes of each message in the queue.
using WriteRecipe = std::vector<std::vector<int>>;

struct FileWriteTestBase : public ::testing::Test {
  // Draw a random byte straight from the engine.  std::uniform_int_distribution
  // isn't defined for uint8_t (MSVC rejects it), and the low bits of mt19937
  // are uniform, so this is both portable and equivalent for random test data.
  uint8_t NextRandom() { return static_cast<uint8_t>(engine()); }

  AllocatorResizeableBuffer<AlignedReallocator<aos::logger::FileHandler::kSector

                                               >>
      buffer;

  void TestRecipe(const WriteRecipe &recipe) {
    VLOG(1) << "Starting";
    for (const std::vector<int> &r : recipe) {
      VLOG(1) << "  chunk " << absl::StrJoin(r, ", ");
    }
    size_t requested_size = 0;
    for (const auto &call : recipe) {
      for (const auto &message_size : call) {
        requested_size += message_size;
      }
    }

    // Grow the cached buffer if it needs to grow.  Building a random buffer is
    // the most expensive part of the test.
    if (buffer.size() < requested_size) {
      // Make sure it is 512 aligned...  That makes sure we have the best chance
      // of transitioning to and from being aligned.
      buffer.resize((requested_size + FileHandler::kSector - 1) &
                    (~(FileHandler::kSector - 1)));
      std::generate(std::begin(buffer), std::end(buffer),
                    [this]() { return NextRandom(); });
    }

    // Back align the data to the buffer so we make sure the contents of the
    // file keep changing in case a file somehow doesn't get deleted, or
    // collides with something else.
    const uint8_t *adjusted_start =
        buffer.data() + buffer.size() - requested_size;

    // logevent has to end with '/' to be recognized as a folder.
    const std::string logevent = TestTmpDir();
    const auto file = std::filesystem::path(logevent) / "test.log";
    std::filesystem::remove_all(file);
    VLOG(1) << "Writing to " << file.c_str();

    LogFolder backend(logevent, false);
    auto handler = backend.RequestFile("test.log");
    ASSERT_EQ(handler->OpenForWrite(), WriteCode::kOk);

    // Build arguments for Write.
    size_t position = 0;
    for (const auto &call : recipe) {
      std::vector<absl::Span<const uint8_t>> queue;
      for (const auto &message_size : call) {
        const uint8_t *current = adjusted_start + position;
        queue.emplace_back(current, message_size);
        position += message_size;
      }
      auto result = handler->Write(queue);
      EXPECT_EQ(result.code, WriteCode::kOk);
      EXPECT_EQ(result.messages_written, call.size());
    }

    ASSERT_EQ(handler->Close(), WriteCode::kOk);
    EXPECT_TRUE(std::filesystem::exists(file));
    EXPECT_EQ(std::filesystem::file_size(file), requested_size);

    // Confirm that the contents then match the original buffer.
    std::ifstream file_stream(file, std::ios::in | std::ios::binary);
    std::vector<uint8_t> content((std::istreambuf_iterator<char>(file_stream)),
                                 std::istreambuf_iterator<char>());
    ASSERT_EQ(content.size(), requested_size);
    bool matches = true;
    for (size_t i = 0; i < content.size(); ++i) {
      if (content[i] != adjusted_start[i]) {
        matches = false;
        break;
      }
    }
    if (!matches) {
      ASSERT_TRUE(false);
    }
  }

  std::random_device random;
  std::mt19937 engine{random()};
};

// Tests that random sets of reads and writes always result in all the data
// being written.
TEST_F(FileWriteTestBase, RandomTest) {
  std::mt19937 engine2{random()};
  std::uniform_int_distribution<int> count_distribution{1, 5};

  // Pick a bunch of lengths that will result in things that add up to multiples
  // of 512 and end up transitioning across the aligned and unaligned boundary.
  const std::vector<int> lengths = {
      0x100b5,  0xff4b,   0x10000,  1024 - 7, 1024 - 6, 1024 - 5, 1024 - 4,
      1024 - 3, 1024 - 2, 1024 - 1, 1024,     1024 + 1, 1024 + 2, 1024 + 3,
      1024 + 4, 1024 + 5, 1024 + 6, 1024 + 7, 512 - 7,  512 - 6,  512 - 5,
      512 - 4,  512 - 3,  512 - 2,  512 - 1,  512,      512 + 1,  512 + 2,
      512 + 3,  512 + 4,  512 + 5,  512 + 6,  512 + 7};
  std::uniform_int_distribution<int> lengths_distribution{
      0, static_cast<int>(lengths.size() - 1)};

  for (int i = 0; i < 1000; ++i) {
    WriteRecipe recipe;
    int number_of_writes = count_distribution(engine2);
    for (int j = 0; j < number_of_writes; ++j) {
      int number_of_chunks = count_distribution(engine2);
      std::vector<int> r;
      for (int k = 0; k < number_of_chunks; ++k) {
        r.emplace_back(lengths[lengths_distribution(engine2)]);
      }
      recipe.emplace_back(std::move(r));
    }

    TestRecipe(recipe);
  }
}

// Test an aligned to unaligned transition to make sure everything works.
//
// Linux-only: this asserts on written_aligned(), which only the O_DIRECT
// sector-aligned WriteV() path increments.  Windows uses plain sequential
// writes with no alignment tracking, so the aligned/unaligned distinction this
// exercises doesn't exist there.
#ifndef _WIN32
TEST_F(FileWriteTestBase, AlignedToUnaligned) {
  AllocatorResizeableBuffer<AlignedReallocator<512>> aligned_buffer;
  AllocatorResizeableBuffer<AlignedReallocator<512>> unaligned_buffer;

  aligned_buffer.resize(FileHandler::kSector * 4);
  std::generate(std::begin(aligned_buffer), std::end(aligned_buffer),
                [this]() { return NextRandom(); });

  unaligned_buffer.resize(FileHandler::kSector * 4);
  std::generate(std::begin(unaligned_buffer), std::end(unaligned_buffer),
                [this]() { return NextRandom(); });

  const size_t kOffset = 53;
  absl::Span<const uint8_t> unaligned_span(unaligned_buffer.data() + kOffset,
                                           aligned_buffer.size() - kOffset);

  std::vector<absl::Span<const uint8_t>> queue;

  queue.emplace_back(aligned_buffer.data(), aligned_buffer.size());
  queue.emplace_back(unaligned_span);
  LOG(INFO) << "Queue 0 " << queue[0].size();
  LOG(INFO) << "Queue 1 " << queue[1].size();

  const std::string logevent = TestTmpDir();
  const auto file = std::filesystem::path(logevent) / "test.log";
  std::filesystem::remove_all(file);
  VLOG(1) << "Writing to " << file.c_str();

  LogFolder backend(logevent, false);
  auto handler = backend.RequestFile("test.log");
  ASSERT_EQ(handler->OpenForWrite(), WriteCode::kOk);

  auto result = handler->Write(queue);
  EXPECT_EQ(result.code, WriteCode::kOk);
  EXPECT_EQ(result.messages_written, queue.size());
#if !defined(__APPLE__) && !defined(_WIN32)
  FileHandler *file_handler = reinterpret_cast<FileHandler *>(handler.get());
  EXPECT_GT(file_handler->written_aligned(), 0);
#endif

  ASSERT_EQ(handler->Close(), WriteCode::kOk);
  EXPECT_TRUE(std::filesystem::exists(file));
  const size_t requested_size = queue[0].size() + queue[1].size();
  EXPECT_EQ(std::filesystem::file_size(file), requested_size);

  // Confirm that the contents then match the original buffer.
  std::ifstream file_stream(file, std::ios::in | std::ios::binary);
  std::vector<uint8_t> content((std::istreambuf_iterator<char>(file_stream)),
                               std::istreambuf_iterator<char>());
  ASSERT_EQ(content.size(), requested_size);
  bool matches = true;
  for (size_t i = 0; i < queue[0].size(); ++i) {
    if (content[i] != aligned_buffer.data()[i]) {
      matches = false;
      break;
    }
  }
  for (size_t i = 0; i < queue[1].size(); ++i) {
    if (content[i + queue[0].size()] != unaligned_span.data()[i]) {
      matches = false;
      break;
    }
  }
  if (!matches) {
    ASSERT_TRUE(false);
  }
}
#endif  // !_WIN32

struct FileWriteTestFixture : public ::testing::WithParamInterface<WriteRecipe>,
                              public FileWriteTestBase {};

TEST_P(FileWriteTestFixture, CheckSizeOfWrittenFile) {
  auto recipe = GetParam();
  TestRecipe(recipe);
}

// Try out some well known failure cases transitioning across the alignment
// boundary.
INSTANTIATE_TEST_SUITE_P(
    FileWriteTest, FileWriteTestFixture,
    ::testing::Values(WriteRecipe{{0x10000}}, WriteRecipe{{0x10000, 0x1000b5}},
                      WriteRecipe{{0x10000, 0x1000b5}, {0xfff4b, 0x10000}},
                      WriteRecipe{{0x1000b5, 0xfff4b}, {0x10000}},
                      WriteRecipe{{65536, 517, 65717}},
                      WriteRecipe{{65536, 517, 518, 511},
                                  {514},
                                  {505, 514},
                                  {505, 514, 65355, 519}},
                      WriteRecipe{{65536, 518, 511, 511},
                                  {65717},
                                  {65717, 65717, 518},
                                  {65536, 65536, 508, 65355},
                                  {515, 519}},
                      WriteRecipe{{0x1000b5, 0xfff4b, 0x100000}, {0x10000}}));

}  // namespace aos::logger::testing
