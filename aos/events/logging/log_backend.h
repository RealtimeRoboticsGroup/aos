#ifndef AOS_EVENTS_LOGGING_LOG_BACKEND_H_
#define AOS_EVENTS_LOGGING_LOG_BACKEND_H_

#include <fcntl.h>
#include <sys/types.h>
#ifndef _WIN32
#include <sys/uio.h>
#endif

#include <climits>
#include <condition_variable>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/types/span.h"

#include "aos/events/logging/buffer_encoder.h"
#include "aos/events/logging/logger_statistics.h"
#include "aos/gtest_prod.h"
#include "aos/time/time.h"

namespace aos::logger {

// Windows has no writev()/IOV_MAX, so cap batched writes at a reasonable
// constant there instead.
#ifdef _WIN32
inline constexpr int kIovMax = 1024;
#else
inline constexpr int kIovMax = IOV_MAX;
#endif

// Currently, all write operations only cares about out-of-space error. This is
// a simple representation of write result.
enum class WriteCode { kOk, kOutOfSpace };

struct WriteResult {
  WriteCode code = WriteCode::kOk;
  size_t messages_written = 0;
  size_t bytes_written = 0;
};

// Interface that abstract writing to log from media.
class LogSink {
 public:
  LogSink() = default;
  virtual ~LogSink() = default;

  LogSink(const LogSink &) = delete;
  LogSink &operator=(const LogSink &) = delete;

  // Try to open file. App will crash if there are other than out-of-space
  // problems with backend media.
  virtual WriteCode OpenForWrite() = 0;

  // Close the file handler.
  virtual WriteCode Close() = 0;

  // Returns true if sink is open and need to be closed.
  virtual bool is_open() const = 0;

  // Peeks messages from queue and writes it to file. Returns code when
  // out-of-space problem occurred along with number of messages from queue that
  // was written.
  virtual WriteResult Write(
      const absl::Span<const absl::Span<const uint8_t>> &queue) = 0;

  // Get access to statistics related to the write operations.
  LoggerStatistics *WriteStatistics() { return &write_stats_; }

  // Name of the log sink.
  virtual std::string_view name() const = 0;

 private:
  LoggerStatistics write_stats_;
};

// Source for iovec with an additional flag that pointer and size of data is
// aligned and be ready for O_DIRECT operation.
struct AlignedIovec {
  const uint8_t *data;
  size_t size;
  bool aligned;

  AlignedIovec(const uint8_t *data, size_t size, bool aligned)
      : data(data), size(size), aligned(aligned) {}
};

// Converts queue of pieces to write to the disk to the queue where every
// element is either aligned for O_DIRECT operation or marked as not aligned.
class QueueAligner {
 public:
  QueueAligner();

  // Reads input queue and fills with aligned and unaligned pieces. It is easy
  // to deal with smaller pieces and batch it during the write operation.
  void FillAlignedQueue(
      const absl::Span<const absl::Span<const uint8_t>> &queue);

  const std::vector<AlignedIovec> &aligned_queue() const {
    return aligned_queue_;
  }

 private:
  std::vector<AlignedIovec> aligned_queue_;
};

// FileHandler is a replacement for bare filename in log writing and reading
// operations.
//
// There are a couple over-arching constraints on writing to keep track of.
//  1) The kernel is both faster and more efficient at writing large, aligned
//     chunks with O_DIRECT set on the file.  The alignment needed is specified
//     by kSector and is file system dependent.
//  2) Not all encoders support generating round multiples of kSector of data.
//     Rather than burden the API for detecting when that is the case, we want
//     DetachedBufferWriter to be as efficient as it can at writing what given.
//  3) Some files are small and not updated frequently.  They need to be
//     flushed or we will lose data on power off.  It is most efficient to write
//     as much as we can aligned by kSector and then fall back to the non direct
//     method when it has been flushed.
//  4) Not all filesystems support O_DIRECT, and different sizes may be optimal
//     for different machines.  The defaults should work decently anywhere and
//     be tunable for faster systems.
class FileHandler : public LogSink {
 public:
  // Size of an aligned sector used to detect when the data is aligned enough to
  // use O_DIRECT instead.
  static constexpr size_t kSector = 512u;

  explicit FileHandler(std::string filename, bool supports_odirect);
  virtual ~FileHandler() override;

  FileHandler(const FileHandler &) = delete;
  FileHandler &operator=(const FileHandler &) = delete;

  // Try to open file. App will crash if there are other than out-of-space
  // problems with backend media.
  WriteCode OpenForWrite() override;

  // Close the file handler.
  WriteCode Close() override;

  // This will be true until Close() is called, unless the file couldn't be
  // created due to running out of space.
  bool is_open() const override { return fd_ != -1; }

  // Peeks messages from queue and writes it to file. Returns code when
  // out-of-space problem occurred along with number of messages from queue that
  // was written.
  //
  // The spans can be aligned or not, and can have any lengths.  This code will
  // write faster if the spans passed in start at aligned addresses, and are
  // multiples of kSector long (and the data written so far is also a multiple
  // of kSector length).
  WriteResult Write(
      const absl::Span<const absl::Span<const uint8_t>> &queue) final override;

  // Name of the log sink mostly for informational purposes.
  std::string_view name() const override { return filename_; }

  // Number of bytes written in aligned mode. Mostly for testing.
  size_t written_aligned() const { return written_aligned_; }

  // For testing; indicates whether we actually encountered a situation where
  // not all the data was able to be flushed to the buffer at once, to validate
  // that we handle massive data writes correctly.
  bool encountered_incomplete_write() const {
    return encountered_incomplete_write_;
  }

  // For testing; counts how many times we have pushed file *data* towards the
  // disk, by any platform-specific mechanism.  --sync promises to "sync the
  // file after each written block", so with it set this has to become non-zero
  // during Write(), not only at Close().  Each backend syncs differently
  // (sync_file_range() on Linux, F_FULLFSYNC on Darwin, _commit() on Windows),
  // so counting the operations is the one durability check that is meaningful
  // on all of them.  Does not count directory syncs.
  size_t data_sync_count() const { return data_sync_count_; }

 protected:
  // This is used by subclasses who need to access filename.
  std::string_view filename() const { return filename_; }

  // This is used by subclasses to update the filename when the base directory
  // is renamed. It replaces old_base_name with new_base_name in the filename.
  // Assumes that old_base_name is at the start of the filename.
  void UpdateFilenameBase(const std::string_view old_base_name,
                          const std::string_view new_base_name);

  // Actually implements the bulk of the Write() call; to be overridden by
  // subclasses.
  virtual WriteResult DoWrite(
      const absl::Span<const absl::Span<const uint8_t>> &queue);

 private:
  // Enables O_DIRECT on the open file if it is supported.  Cheap to call if it
  // is already enabled.
  void EnableDirect();
  // Disables O_DIRECT on the open file if it is supported.  Cheap to call if it
  // is already disabld.
  void DisableDirect();

  bool ODirectEnabled() const;

  // Flushes the file descriptor to disk, recording that we did so.  Callers
  // should use this rather than PlatformSyncImpl() so that every data sync is
  // counted in one place.
  //
  // Returns kOutOfSpace when the flush failed because the disk filled up.
  // Callers have to report that the same way they report a failing write: a
  // failed sync means the data isn't durable, and answering kOk would tell the
  // logger the batch is safely on disk when it isn't.  DetachedBufferWriter::
  // Flush() would then Clear() it out of the encoder as durably written and
  // keep logging, instead of winding down through kOutOfSpace.
  [[nodiscard]] WriteCode PlatformSync() {
    ++data_sync_count_;
    return PlatformSyncImpl();
  }

  // Flushes the file descriptor to disk in a platform-specific way.  Returns
  // kOutOfSpace on ENOSPC; any other failure is logged and reported as kOk,
  // since running out of space is the only one the logger can wind down for.
  //
  // Virtual only so tests can force the out-of-space case: a real ENOSPC from
  // fdatasync()/F_FULLFSYNC/_commit() needs a genuinely full filesystem, and
  // the propagation this feeds is exactly what has regressed here before.
  [[nodiscard]] virtual WriteCode PlatformSyncImpl();

  // Writes a chunk of iovecs from iovec_. aligned is true if all the data is
  // kSector byte aligned and multiples of it in length.
  // May modify iovec_.
  // Returns a write code and a number of bytes written.
  std::pair<WriteCode, size_t> WriteV(bool aligned);

 protected:
  std::string filename_;
  int fd_ = -1;

#ifndef _WIN32
  // List of iovecs to use with writev.  This is a member variable to avoid
  // churn.
  std::vector<struct iovec> iovec_;
#endif

  QueueAligner queue_aligner_;

  int total_write_bytes_ = 0;
  int last_synced_bytes_ = 0;

  size_t written_aligned_ = 0;

  bool supports_odirect_ = true;
  bool odirect_enabled_ = false;
  int flags_ = 0;

  // Used for unit tests to ensure that we have coverage of handling certain
  // corner-cases.
  bool encountered_incomplete_write_ = false;

  // See data_sync_count().
  size_t data_sync_count_ = 0;
};

namespace testing {
FORWARD_DECLARE_TEST_CASE(LogBackendTest, OutOfSpaceTest);
}

// Provides an implementation of the FileHandler which maintains an in-memory
// buffer of memory_buffer_size.
// Writes will cause the internal memory buffer to be filled, while a separate
// thread removes data from the buffer and writes it to disk. If the thread
// writing to disk cannot keep up with incoming data, then eventually the
// internal memory buffer will fill up. When this happens, calls to Write() will
// block in the same way that they would on the FileHandler.
class BufferedFileHandler : public FileHandler {
 public:
  // If memory_buffer_size is set to zero, will not perform any threading or
  // buffering.
  BufferedFileHandler(std::string filename, bool supports_odirect,
                      size_t memory_buffer_size);
  virtual ~BufferedFileHandler() { Close(); }

  WriteCode OpenForWrite() override;
  WriteCode Close() override;

 protected:
  // Latches the same out-of-space state the writing thread sets when one of its
  // writes comes back kOutOfSpace, so writes from here on report kOutOfSpace
  // instead of touching the file.
  //
  // For backends that discover the condition outside of a write: the Windows
  // rename path closes handlers itself and only sees it in Close()'s return.
  void MarkRanOutOfSpace() {
    std::unique_lock lock(buffer_index_mutex_);
    ran_out_of_space_in_thread_ = true;
  }

  // Starts the disk-writing thread, if this handler is buffered.  Does nothing
  // when memory_buffer_size_ is zero.
  //
  // Split out of OpenForWrite() for backends which have to reopen a file they
  // already wrote to: Close() stops the writing thread, and going back through
  // OpenForWrite() to restart it would start a fresh file and discard
  // everything logged so far.  Reopen the descriptor first, then call this;
  // the thread starts writing as soon as it is spawned.
  void StartWriterThread();

 private:
  // In order to exercise the out-of-space logic, we manually alter the internal
  // state of this class rather than trying to actually run out of space on a
  // filesystem.
  FRIEND_TEST_NAMESPACE(LogBackendTest, OutOfSpaceTest, testing);

  // Writes the provided messages. Will return once the messages are in the
  // internal memory buffer. This does mean that messages provided here may not
  // make it to disk if we terminate or run out of disk space before the
  // messages are written. Blocks if there is no space remaining in the internal
  // memory buffer, and waits for the disk writing thread to free up space.
  WriteResult DoWrite(
      const absl::Span<const absl::Span<const uint8_t>> &queue) override;
  WriteResult DoWriteAsync(
      const absl::Span<const absl::Span<const uint8_t>> &queue);

  // Returns the total buffer space available.
  // buffer_index_mutex_ must be acquired before calling this.
  size_t buffer_space_available() const {
    if (buffer_start_ == buffer_end_) {
      return buffer_empty_ ? buffer_.size() : 0;
    }
    return (buffer_start_ > buffer_end_)
               ? (buffer_start_ - buffer_end_)
               : ((buffer_start_ + buffer_.size()) - buffer_end_);
  }

  // Should provide a lock or buffer_index_update_ that is already held.
  // Attempts to write out the current contents of the buffer.
  void WriteCurrentBufferContents(
      std::unique_lock<std::mutex> &buffer_index_lock);

  const size_t memory_buffer_size_;
  // Indicates that Close() has been called and that the writing thread should
  // exit.
  std::atomic<bool> closing_ = false;
  // Indicates that the thread writing to disk encountered ENOSPC and that we
  // should propagate said fault to the user, on the basis that there is no
  // point in continuing to write data.
  // TODO(james): Future optimizations could choose not to propagate the ENOSPC
  // until the internal memory buffer fills up, such that if space *does* free
  // up on the filesystem then we can continue to write.
  bool ran_out_of_space_in_thread_ = false;
  // Guards access to buffer_start_ , buffer_end_, ran_out_of_space_in_thread_,
  // and buffer_empty_.
  std::mutex buffer_index_mutex_;
  // Index into buffer_ for where the current start of the buffer is. The
  // disk-writing thread will start reading data from here. Only the
  // disk-writing thread should be modifying this value.
  size_t buffer_start_ = 0;
  // Index into buffer_ for where one-past the current end of the buffer is. The
  // data insertion thread will insert data starting at buffer_end_. Only the
  // data-insertion thread should be modifying this value.
  size_t buffer_end_ = 0;
  // Indicates whether the buffer is empty. When buffer_start_ == buffer_end_ it
  // is ambiguous whether the buffer is empty or full, necessitating this.
  bool buffer_empty_ = true;
  // A notify will be sent out on buffer_index_update_ whenever buffer_start_,
  // buffer_end_, or buffer_empty_ are updated.
  std::condition_variable buffer_index_update_;
  // The actual buffer to use for storing the data to be written to file.
  // Aligned to the same alignment that we expect to use when actually doing our
  // write() calls.
  // Note that access to buffer_ is not currently guarded by the
  // buffer_index_mutex_ directly. Because only one thread is writing to the
  // buffer, only one is reading, and only one thread is responsible for each of
  // buffer_start_ and buffer_end_, we can safely have the write thread by
  // copying data out of this buffer while the insertion thread copies data into
  // another part of the buffer.
  aos::AllocatorResizeableBuffer<aos::AlignedReallocator<kSector>> buffer_;
  // Thread for file writing. Only set while the file is open and if
  // memory_buffer_size_ is greater than zero.
  std::optional<std::thread> file_writer_;
};

// Interface to decouple reading of logs and media (file system, memory or S3).
class LogSource {
 public:
  struct File {
    std::string name;
    size_t size;
  };

  virtual ~LogSource() = default;

  // Provides a list of readable sources for log reading.
  virtual std::vector<File> ListFiles() const = 0;

  // Entry point for reading the content of log file.
  virtual std::unique_ptr<DataDecoder> GetDecoder(
      const std::string_view id) const = 0;
  std::unique_ptr<DataDecoder> GetDecoder(const LogSource::File &id) const {
    return GetDecoder(id.name);
  }
};

// Interface to decouple log writing and media (file system or memory). It is
// handy to use for tests.
class LogBackend {
 public:
  virtual ~LogBackend() = default;

  // Request file-like object from the log backend. It maybe a file on a disk or
  // in memory. id is usually generated by log namer and looks like name of the
  // file within a log folder.
  virtual std::unique_ptr<LogSink> RequestFile(
      const std::string_view id, const size_t memory_buffer_size = 0) = 0;
};

// Returns what goes between a base name and a file id.  A base name naming a
// directory ("logs/") needs nothing; one that is a filename stem
// ("logs/prefix") needs an underscore, so files come out as "logs/prefix_id".
//
// Asks std::filesystem::path instead of testing for a trailing '/' so a native
// Windows base name ("C:\logs\") is recognized as a directory too.  On POSIX
// '\' is an ordinary filename character, and this agrees with the trailing-'/'
// test for every base name there.
inline std::string_view BaseNameSeparator(std::string_view base_name) {
  return std::filesystem::path(base_name).has_filename() ? "_" : "";
}

// Implements requests log files from file system.
class FileBackend : public LogBackend, public LogSource {
 public:
  // base_name is the path to the folder where log files are.
  explicit FileBackend(std::string_view base_name, bool supports_direct);

  ~FileBackend() override = default;

  // Request file from a file system. It is not open yet.
  std::unique_ptr<LogSink> RequestFile(
      const std::string_view id, const size_t memory_buffer_size = 0) override;

  // List all files that looks like log files under base_name.
  std::vector<File> ListFiles() const override;

  // Open decoder to read the content of the file.
  std::unique_ptr<DataDecoder> GetDecoder(
      const std::string_view id) const override;

 private:
  const bool supports_odirect_;
  const std::string base_name_;
  const std::string_view separator_;
};

// Provides a file backend that supports log folders.
class LogFolder : public FileBackend {
 public:
  // Opens logs in the specified folder.
  explicit LogFolder(const std::filesystem::path &log_directory_path,
                     bool supports_direct)
      : FileBackend(MakeBaseName(log_directory_path), supports_direct) {}

 private:
  // Ensures the base name names a directory, so BaseNameSeparator() picks the
  // empty separator for it.  Appending through std::filesystem::path uses the
  // platform's separator, rather than tacking a '/' onto a native Windows path
  // and leaving "C:\logs/".
  static std::string MakeBaseName(const std::filesystem::path &path) {
    return path.has_filename() ? (path / "").string() : path.string();
  }
};

// Provides a file backend that supports renaming of the base log folder and
// temporary files.
class RenamableFileBackend : public LogBackend {
 public:
  // Adds call to rename, when closed.
  class RenamableFileHandler : public BufferedFileHandler {
   public:
    RenamableFileHandler(RenamableFileBackend *owner, std::string filename,
                         bool supports_odirect, size_t memory_buffer_size);
    ~RenamableFileHandler() override = default;

    // Closes and if needed renames file.
    WriteCode Close() override;

   private:
    RenamableFileBackend *owner_;
  };

  explicit RenamableFileBackend(std::string_view base_name,
                                bool supports_odirect);
  ~RenamableFileBackend() override;

  // Request file from a file system. It is not open yet.
  std::unique_ptr<LogSink> RequestFile(
      const std::string_view id, const size_t memory_buffer_size = 0) override;

  std::string_view base_name() const { return base_name_; }

  // If temp files are enabled, then this will write files with the .tmp
  // suffix, and then rename them to the desired name after they are fully
  // written.
  //
  // This is useful to enable incremental copying of the log files.
  //
  // Defaults to writing directly to the final filename.
  void EnableTempFiles();

  // Moves the current log location to the new name. Returns true if a change
  // was made, false otherwise.
  // Only renaming the folder is supported, not the file base name.
  virtual bool RenameLogBase(std::string_view new_base_name);

 protected:
  // This function called after file closed, to adjust file names in case of
  // base name was changed or temp files enabled.
  WriteCode RenameFileAfterClose(std::string_view filename);

  // Returns true if the base directory has been renamed.
  bool was_renamed() const { return !old_base_name_.empty(); }

  std::optional<std::pair<std::string, std::string>>
  ValidateAndSplitRenamePaths(std::string_view new_base_name) const;

  void SyncParentDirectories(const std::string &current_directory,
                             const std::string &new_directory);

  // Returns true if directory names something that exists, and dies if that
  // can't be determined.
  //
  // std::filesystem::exists() throws on a failed status query, which would take
  // the process out through an exception raised in the middle of log rotation.
  // This dies on the same conditions, but as a CHECK naming the directory and
  // the OS error.
  static bool DirectoryExists(const std::string &directory);

  const bool supports_odirect_;
  std::string base_name_;
  std::string_view separator_;

  bool use_temp_files_ = false;
  std::string_view temp_suffix_;

  std::string old_base_name_;
};

#ifdef _WIN32
// Windows-specific RenamableFileBackend that tracks active file handlers to
// work around Windows file locking, which prohibits renaming a directory
// containing open files.
class WindowsRenamableFileBackend final : public RenamableFileBackend {
 public:
  class WindowsRenamableFileHandler final : public BufferedFileHandler {
   public:
    WindowsRenamableFileHandler(WindowsRenamableFileBackend *owner,
                                std::string filename, bool supports_odirect,
                                size_t memory_buffer_size);
    ~WindowsRenamableFileHandler() override;

    WriteCode Close() override;

    void ReopenAndSeek();

    // Retires this handler after the disk filled up while closing it for a
    // rename.  It stays closed and reports kOutOfSpace from then on.
    void StopForOutOfSpace() { MarkRanOutOfSpace(); }

    void UpdateFilename(const std::string_view old_base,
                        const std::string_view new_base) {
      UpdateFilenameBase(old_base, new_base);
    }

   private:
    WindowsRenamableFileBackend *owner_;
  };

  WindowsRenamableFileBackend(std::string_view base_name,
                              bool supports_odirect);
  ~WindowsRenamableFileBackend() override;

  std::unique_ptr<LogSink> RequestFile(
      const std::string_view id, const size_t memory_buffer_size = 0) override;

  bool RenameLogBase(std::string_view new_base_name) override;

 private:
  friend class WindowsRenamableFileHandler;

  // Adds handler to active_handlers_.  handlers_mutex_ must already be held --
  // RequestFile() builds the handler's filename from base_name_ and registers
  // it in one critical section, so that a rename can't land in between and
  // leave the new handler pointing at the old directory.
  void RegisterHandlerLocked(WindowsRenamableFileHandler *handler);

  // Closes handler and drops it from active_handlers_, both under
  // handlers_mutex_ so it can't interleave with a rename closing or reopening
  // the same pointer.
  void CloseAndUnregisterHandler(WindowsRenamableFileHandler *handler);

  std::mutex handlers_mutex_;
  std::vector<WindowsRenamableFileHandler *> active_handlers_;
};
#endif

// Factory function to instantiate the correct platform-specific
// RenamableFileBackend.
std::unique_ptr<RenamableFileBackend> MakeRenamableFileBackend(
    std::string_view base_name, bool supports_odirect);

}  // namespace aos::logger

#endif  // AOS_EVENTS_LOGGING_LOG_BACKEND_H_
