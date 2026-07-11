#include "aos/events/logging/log_backend.h"

#include <numeric>
#include <system_error>
#include <thread>

#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/log/vlog_is_on.h"
#include "absl/strings/match.h"
#include "absl/strings/str_cat.h"

#include "aos/events/logging/file_operations.h"
#include "aos/events/logging/logfile_decoder_options.h"
#include "aos/realtime.h"
#include "aos/util/file.h"

ABSL_FLAG(
    bool, sync, false,
    "If true, sync data to disk as we go so we don't get too far ahead.  Also "
    "fadvise that we are done with the memory once it hits disk.");

ABSL_FLAG(uint32_t, queue_reserve, 32, "Pre-reserved size of write queue.");

namespace aos::logger {
namespace {
constexpr const char *kTempExtension = ".tmp";

// Assuming that kSector is power of 2, it aligns address to the left size.
inline size_t AlignToLeft(size_t value) {
  return value & (~(FileHandler::kSector - 1));
}

inline bool IsAligned(size_t value) {
  return value % FileHandler::kSector == 0;
}

}  // namespace

QueueAligner::QueueAligner() {
  aligned_queue_.reserve(absl::GetFlag(FLAGS_queue_reserve));
}

void QueueAligner::FillAlignedQueue(
    const absl::Span<const absl::Span<const uint8_t>> &queue) {
  aligned_queue_.clear();

  size_t queue_index = 0;
  for (const auto &span : queue) {
    ++queue_index;
    // Generally, every span might have 3 optional parts (i.e. 2^3 cases):
    // 1. unaligned prefix -  from start till first aligned block.
    // 2. aligned main - block with aligned start and size
    // 3. unaligned suffix - block with aligned start, and size less than one
    // sector. If size of the span is less than 1 sector, let's call it prefix.

    auto *data = span.data();
    size_t size = span.size();
    const auto start = reinterpret_cast<size_t>(data);
    VLOG(2) << "Consider span starting at " << std::hex << start
            << " with size " << size;

    CHECK_GT(size, 0u)
        << ": Nobody should be sending empty messages.  Queue index "
        << (queue_index - 1) << " out of " << queue.size();

    const auto next_aligned =
        IsAligned(start) ? start : AlignToLeft(start) + FileHandler::kSector;
    const auto prefix_size = next_aligned - start;
    VLOG(2) << "Calculated prefix size " << std::hex << prefix_size;

    if (prefix_size >= size) {
      // size of prefix >= size of span - alignment is not possible, accept the
      // whole span
      VLOG(2) << "Only prefix found";
      CHECK_GT(size, 0u);
      aligned_queue_.emplace_back(data, size, false);
      continue;
    }
    CHECK_LT(prefix_size, FileHandler::kSector)
        << ": Wrong calculation of 'next' aligned position";
    if (prefix_size > 0) {
      // Cut the prefix and move to the main part.
      VLOG(2) << "Cutting prefix at " << std::hex << start << " of size "
              << prefix_size;
      aligned_queue_.emplace_back(data, prefix_size, false);
      data += prefix_size;
      size -= prefix_size;
      CHECK(data <= span.data() + span.size()) << " :Boundaries after prefix";
    }

    if (IsAligned(size)) {
      // the rest is aligned.
      VLOG(2) << "Returning aligned main part";
      CHECK_GT(size, 0u);
      aligned_queue_.emplace_back(data, size, true);
      continue;
    }

    const auto aligned_size = AlignToLeft(size);
    CHECK(aligned_size < size) << ": Wrong calculation of 'main' size";
    if (aligned_size > 0) {
      VLOG(2) << "Cutting main part starting " << std::hex
              << reinterpret_cast<size_t>(data) << " of size " << aligned_size;
      aligned_queue_.emplace_back(data, aligned_size, true);

      data += aligned_size;
      size -= aligned_size;
      CHECK(data <= span.data() + span.size()) << " :Boundaries after main";
    }

    VLOG(2) << "Cutting suffix part starting " << std::hex
            << reinterpret_cast<size_t>(data) << " of size " << size;
    CHECK_GT(size, 0u);
    aligned_queue_.emplace_back(data, size, false);
  }
}

FileHandler::FileHandler(std::string filename, bool supports_odirect)
    : filename_(std::move(filename)), supports_odirect_(supports_odirect) {}

BufferedFileHandler::BufferedFileHandler(std::string filename,
                                         bool supports_odirect,
                                         size_t memory_buffer_size)
    : FileHandler(filename, supports_odirect),
      memory_buffer_size_(memory_buffer_size) {
  VLOG(1) << "Allocating a memory buffer of " << memory_buffer_size << " for "
          << filename;
  buffer_.resize(AlignToLeft(memory_buffer_size_) + kSector);
}

FileHandler::~FileHandler() { Close(); }

void FileHandler::UpdateFilenameBase(const std::string_view old_base_name,
                                     const std::string_view new_base_name) {
  // Check for the old base name first.  Testing the new one first looks like it
  // would detect "already updated", but it also matches a filename that still
  // needs updating whenever the new base name is a prefix of the old one --
  // renaming ".../run-old" to ".../run" leaves ".../run-old/foo" starting with
  // ".../run", and we'd skip it and keep writing to the old directory.
  //
  // A filename that has been updated no longer starts with the old base name
  // (the rename replaced exactly that prefix), so this order is unambiguous.
  if (filename_.starts_with(old_base_name)) {
    filename_.replace(0, old_base_name.length(), new_base_name);
    return;
  }

  CHECK(filename_.starts_with(new_base_name))
      << "Expected '" << filename_ << "' to start with either old_base_name '"
      << old_base_name << "' or new_base_name '" << new_base_name << "'";
}

void BufferedFileHandler::StartWriterThread() {
  if (memory_buffer_size_ == 0) {
    return;
  }
  closing_ = false;
  CHECK(!file_writer_.has_value());
  file_writer_.emplace([this]() {
    SetCurrentThreadName(
        ("writer_" + std::filesystem::path(filename()).filename().string())
            .substr(0, 16));
    std::unique_lock lock(buffer_index_mutex_);
    while (!closing_ || (!buffer_empty_ && !ran_out_of_space_in_thread_)) {
      if (buffer_empty_ || ran_out_of_space_in_thread_) {
        buffer_index_update_.wait(lock);
      }
      WriteCurrentBufferContents(lock);
      buffer_index_update_.notify_all();
    }
  });
}

WriteCode BufferedFileHandler::OpenForWrite() {
  StartWriterThread();
  return FileHandler::OpenForWrite();
}

void BufferedFileHandler::WriteCurrentBufferContents(
    std::unique_lock<std::mutex> &buffer_index_lock) {
  CHECK(buffer_index_lock.owns_lock());
  if (buffer_empty_ || ran_out_of_space_in_thread_) {
    return;
  }
  // To write the data in the buffer, we will create either 1 or two spans to
  // pass into the Write() call, depending on if the current buffer chunk
  // includes the wrap point of the buffer_.
  // TODO(jkuszmaul): Consider writing smaller chunks than just "as much as
  // possible" so that we can free up space in the internal memory buffer as
  // fast as possible. Experimentation would be required to figure out what
  // actually made sense.
  std::array<absl::Span<const uint8_t>, 2> data;
  absl::Span<const absl::Span<const uint8_t>> queue;
  if (buffer_end_ > buffer_start_) {
    data[0] = {buffer_.data() + buffer_start_, buffer_end_ - buffer_start_};
    queue = absl::Span<const absl::Span<const uint8_t>>{data.data(), 1u};
  } else {
    data[0] = {buffer_.data() + buffer_start_, buffer_.size() - buffer_start_};
    if (buffer_end_ != 0) {
      data[1] = {buffer_.data(), buffer_end_};
      queue = absl::Span<const absl::Span<const uint8_t>>{data.data(), 2u};
    } else {
      queue = absl::Span<const absl::Span<const uint8_t>>{data.data(), 1u};
    }
  }
  const size_t new_buffer_start = buffer_end_;
  // Don't hold the lock while actually writing to disk.
  // Note that this does mean that buffer_end_ may be updated while doing the
  // Write() call; however, because buffer_start_ is not updated by
  // DoWriteAsync() we can trust that the data we are copying from will not be
  // affected.
  buffer_index_lock.unlock();
  const WriteResult result = FileHandler::DoWrite(queue);
  buffer_index_lock.lock();
  switch (result.code) {
    case WriteCode::kOk:
      CHECK_EQ(result.messages_written, queue.size())
          << ": Expected to write all of the data if there was space on disk.";
      break;
    case WriteCode::kOutOfSpace:
      ran_out_of_space_in_thread_ = true;
      break;
  }
  // Note: This isn't entirely correct in the case where we ran out of space.
  buffer_start_ = new_buffer_start;
  buffer_empty_ = buffer_start_ == buffer_end_;
  // Note: Ideally we would call UpdateMemoryBufferBytesAvailable, but we don't
  // currently manage asynchronous access to that object, and in practice
  // DoWriteAsync() should be called frequently enough to keep the statistics
  // usefully refreshed.
}

WriteResult BufferedFileHandler::DoWriteAsync(
    const absl::Span<const absl::Span<const uint8_t>> &queue) {
  size_t messages_written = 0;
  size_t bytes_written = 0;
  std::unique_lock<std::mutex> lock(buffer_index_mutex_);
  if (ran_out_of_space_in_thread_) {
    return {.code = WriteCode::kOutOfSpace,
            .messages_written = 0,
            .bytes_written = 0};
  }
  for (const absl::Span<const uint8_t> &message : queue) {
    size_t message_bytes_written = 0;
    // Copy as many bytes as possible at a time into the internal buffer; this
    // loop will iterate multiple times when either:
    // 1. The buffer is full and we must wait for the write thread to clear
    // space.
    // 2. When we reach the end of the buffer_ variable and have to do a
    // separate memcpy to point at the start of the memory chunk.
    while (true) {
      const size_t buffer_space = buffer_space_available();
      if (buffer_space == 0) {
        // Wait for more space to be available to write to.
        buffer_index_update_.wait(lock);
        if (ran_out_of_space_in_thread_) {
          return {.code = WriteCode::kOutOfSpace,
                  .messages_written = messages_written,
                  .bytes_written = bytes_written};
        }
        // Reset the while loop so that we recalculate buffer_space.
        continue;
      }
      // Memory span into which we will copy data from this message.
      absl::Span<uint8_t> target_buffer(
          buffer_.data() + buffer_end_,
          std::min(buffer_space, message.size() - message_bytes_written));
      const size_t space_until_buffer_wraps = buffer_.size() - buffer_end_;
      const bool wrap_required =
          target_buffer.size() > space_until_buffer_wraps;
      if (wrap_required) {
        target_buffer = target_buffer.subspan(0, space_until_buffer_wraps);
      }

      // Memory span that we will be copying from.
      const absl::Span<const uint8_t> copy_from =
          message.subspan(message_bytes_written, target_buffer.size());

      // Don't hold the lock while actually copying into the buffer, since this
      // may be expensive.
      // At this point we have not modified buffer_end_, so the file_writer_
      // thread will not attempt to read this data while we are writing it.
      // TODO(james.kuszmaul): Evaluate whether the lock/unlock actually is
      // worse for performance than just doing the memcopy.
      lock.unlock();
      std::copy(copy_from.cbegin(), copy_from.cend(), target_buffer.begin());
      lock.lock();
      message_bytes_written += target_buffer.size();
      bytes_written += target_buffer.size();
      DCHECK_NE(0u, message_bytes_written)
          << "buffer_end: " << buffer_end_ << " buffer size " << buffer_.size()
          << " buffer start " << buffer_start_ << " empty " << buffer_empty_
          << " buffer space " << buffer_space;
      buffer_end_ = (buffer_end_ + target_buffer.size()) % buffer_.size();
      buffer_empty_ = false;
      buffer_index_update_.notify_all();
      WriteStatistics()->UpdateMemoryBufferBytesAvailable(
          buffer_space_available());

      if (wrap_required) {
        continue;
      }

      if (message_bytes_written == message.size()) {
        ++messages_written;
        break;
      }
    }
  }
  return {.code = WriteCode::kOk,
          .messages_written = messages_written,
          .bytes_written = bytes_written};
}

WriteResult BufferedFileHandler::DoWrite(
    const absl::Span<const absl::Span<const uint8_t>> &queue) {
  if (memory_buffer_size_ > 0) {
    // DoWriteAsync() checks ran_out_of_space_in_thread_ itself.
    return DoWriteAsync(queue);
  }
  {
    // No writing thread here to latch it for us, so honor it on the way past.
    // Nothing sets it for an unbuffered handler except MarkRanOutOfSpace().
    std::unique_lock lock(buffer_index_mutex_);
    if (ran_out_of_space_in_thread_) {
      return {
          .code = WriteCode::kOutOfSpace,
          .messages_written = 0,
          .bytes_written = 0,
      };
    }
  }
  return FileHandler::DoWrite(queue);
}

bool FileHandler::ODirectEnabled() const { return odirect_enabled_; }

WriteResult FileHandler::Write(
    const absl::Span<const absl::Span<const uint8_t>> &queue) {
  const aos::monotonic_clock::time_point start_time =
      aos::monotonic_clock::now();
  WriteResult result = DoWrite(queue);
  const aos::monotonic_clock::time_point end_time = aos::monotonic_clock::now();
  WriteStatistics()->UpdateHandlerStats(
      end_time - start_time, result.bytes_written, result.messages_written);
  return result;
}

WriteCode BufferedFileHandler::Close() {
  closing_ = true;
  if (file_writer_.has_value()) {
    {
      std::unique_lock lock(buffer_index_mutex_);
      buffer_index_update_.notify_all();
    }
    file_writer_.value().join();
    file_writer_.reset();
  }

  // The writing thread sets ran_out_of_space_in_thread_ when one of its writes
  // comes back kOutOfSpace, which is how buffered data gets dropped.  Report
  // it: the descriptor closing cleanly says nothing about whether what we
  // buffered reached the disk, and closing is the caller's only chance to find
  // out.
  bool ran_out_of_space;
  {
    std::unique_lock lock(buffer_index_mutex_);
    ran_out_of_space = ran_out_of_space_in_thread_;
  }
  if (FileHandler::Close() == WriteCode::kOutOfSpace) {
    ran_out_of_space = true;
  }
  return ran_out_of_space ? WriteCode::kOutOfSpace : WriteCode::kOk;
}

FileBackend::FileBackend(std::string_view base_name, bool supports_odirect)
    : supports_odirect_(supports_odirect),
      base_name_(base_name),
      separator_(BaseNameSeparator(base_name_)) {}

std::unique_ptr<LogSink> FileBackend::RequestFile(
    const std::string_view id, const size_t memory_buffer_size) {
  CHECK_EQ(memory_buffer_size, 0u)
      << ": Memory buffer unsupported on regular FileBackend.";
  const std::string filename = absl::StrCat(base_name_, separator_, id);
  return std::make_unique<FileHandler>(filename, supports_odirect_);
}

std::vector<FileBackend::File> FileBackend::ListFiles() const {
  std::filesystem::path directory(base_name_);
  if (!is_directory(directory)) {
    directory = directory.parent_path();
  }
  internal::LocalFileOperations operations(directory);
  std::vector<internal::FileOperations::File> files;
  operations.FindLogs(&files);

  std::vector<File> names;
  // FindLogs() reports paths in generic (forward slash) form, so compare
  // against the generic form of the base name -- on Windows it may well be a
  // native path full of backslashes, and matching the two directly would fail
  // the CHECK below for every file.
  //
  // Both sides go through generic_string(), so they stay consistent even where
  // it rewrites more than the separators (it collapses "a//b" to "a/b").
  const std::string prefix = absl::StrCat(
      std::filesystem::path(base_name_).generic_string(), separator_);
  for (const auto &file : files) {
    CHECK(absl::StartsWith(file.name, prefix))
        << ": File " << file.name << ", prefix " << prefix;
    names.emplace_back(File{
        .name = file.name.substr(prefix.size()),
        .size = file.size,
    });
  }
  return names;
}

std::unique_ptr<DataDecoder> FileBackend::GetDecoder(
    const std::string_view id) const {
  const std::string filename = absl::StrCat(base_name_, separator_, id);
  CHECK(std::filesystem::exists(filename));
  return internal::ResolveDecoder(filename, /*quiet=*/true);
}

RenamableFileBackend::RenamableFileHandler::RenamableFileHandler(
    RenamableFileBackend *owner, std::string filename, bool supports_odirect,
    size_t memory_buffer_size)
    : BufferedFileHandler(std::move(filename), supports_odirect,
                          memory_buffer_size),
      owner_(owner) {}

RenamableFileBackend::RenamableFileBackend(std::string_view base_name,
                                           bool supports_odirect)
    : supports_odirect_(supports_odirect),
      base_name_(base_name),
      separator_(BaseNameSeparator(base_name_)) {}

RenamableFileBackend::~RenamableFileBackend() = default;

std::unique_ptr<LogSink> RenamableFileBackend::RequestFile(
    const std::string_view id, const size_t memory_buffer_size) {
  const std::string filename =
      absl::StrCat(base_name_, separator_, id, temp_suffix_);
  return std::make_unique<RenamableFileHandler>(
      this, filename, supports_odirect_, memory_buffer_size);
}

void RenamableFileBackend::EnableTempFiles() {
  use_temp_files_ = true;
  temp_suffix_ = kTempExtension;
}

std::optional<std::pair<std::string, std::string>>
RenamableFileBackend::ValidateAndSplitRenamePaths(
    std::string_view new_base_name) const {
  if (new_base_name == base_name_) {
    return std::nullopt;
  }
  CHECK(old_base_name_.empty())
      << "Only one change of base_name is supported. Was: " << old_base_name_;

  std::string current_directory = base_name_;
  std::string new_directory(new_base_name);

  auto current_path_split = current_directory.rfind("/");
  CHECK(current_path_split != std::string::npos)
      << "Could not find / in the current directory path";
  auto new_path_split = new_directory.rfind("/");
  CHECK(new_path_split != std::string::npos)
      << "Could not find / in the new directory path";

  CHECK(new_base_name.substr(new_path_split) ==
        current_directory.substr(current_path_split))
      << "Rename of file base from " << current_directory << " to "
      << new_directory << " is not supported.";

  current_directory.resize(current_path_split);
  new_directory.resize(new_path_split);
  return std::make_pair(current_directory, new_directory);
}

void RenamableFileBackend::SyncParentDirectories(
    const std::string &current_directory, const std::string &new_directory) {
  // Sync parent directories after successful rename if sync flag is set.
  if (absl::GetFlag(FLAGS_sync)) {
    // Find parent directories for syncing.
    auto current_parent_split = current_directory.rfind("/");
    if (current_parent_split != std::string::npos) {
      std::string current_parent_dir =
          current_directory.substr(0, current_parent_split);
      if (!current_parent_dir.empty()) {
        aos::util::SyncDirectory(std::filesystem::path(current_parent_dir));
      }
    }

    auto new_parent_split = new_directory.rfind("/");
    if (new_parent_split != std::string::npos) {
      std::string new_parent_dir = new_directory.substr(0, new_parent_split);
      if (!new_parent_dir.empty()) {
        // Only sync new parent directory if it's different from the current
        // parent directory.
        auto current_parent_split = current_directory.rfind("/");
        if (current_parent_split == std::string::npos ||
            new_parent_dir !=
                current_directory.substr(0, current_parent_split)) {
          aos::util::SyncDirectory(std::filesystem::path(new_parent_dir));
        }
      }
    }
  }
}

bool RenamableFileBackend::DirectoryExists(const std::string &directory) {
  std::error_code error;
  const bool exists = std::filesystem::exists(directory, error);
  // The error_code overload only sets error when the query couldn't be
  // answered at all.  A directory that simply isn't there (ENOENT, ENOTDIR, and
  // their Windows equivalents -- how a rename that already happened looks)
  // comes back as a clean false, so everything reaching here means we don't
  // know whether the directory exists rather than that it is missing.
  //
  // There is nothing to recover to.  Every remaining error says the log
  // destination is unusable -- unreadable by us (EACCES), unnameable (ELOOP,
  // ENAMETOOLONG), failing (EIO), or a network mount that went away (ESTALE)
  // -- and answering false would report "already renamed, nothing to do" for a
  // directory that may well be sitting right there.
  CHECK(!error) << ": Failed to check whether " << directory
                << " exists: " << error.message();
  return exists;
}

WriteCode RenamableFileBackend::RenameFileAfterClose(
    std::string_view filename) {
  // Fast check that we can skip rename.
  if (!use_temp_files_ && old_base_name_.empty()) {
    return WriteCode::kOk;
  }

  std::string current_filename(filename);

  // When changing the base name, we rename the log folder while there active
  // buffer writers. Therefore, the name of that active buffer may still refer
  // to the old file location rather than the new one.
  if (!old_base_name_.empty()) {
    // Check if the filename already has the new base name.
    if (current_filename.find(base_name_) == 0) {
      // File was created after the base directory was renamed, so it already
      // has the new base name.
      VLOG(1) << "File already has new base name: " << current_filename;
    } else {
      auto offset = current_filename.find(old_base_name_);
      if (offset != std::string::npos) {
        current_filename.replace(offset, old_base_name_.length(), base_name_);
      }
    }
  }

  std::string final_filename = current_filename;
  if (use_temp_files_) {
    CHECK(current_filename.size() > temp_suffix_.size());
    final_filename = current_filename.substr(
        0, current_filename.size() - temp_suffix_.size());
  }

  int result = rename(current_filename.c_str(), final_filename.c_str());

  bool ran_out_of_space = false;
  if (absl::GetFlag(FLAGS_sync)) {
    // Syncing the directory after rename ensures that the updated
    // directory entry (with the new file name) is written to disk.
    // This is sufficient for data integrity because renaming is an
    // atomic operation on most filesystems. Once the directory
    // entry is updated, the file is guaranteed to be accessible
    // under the new name, and the old name is no longer valid.
    aos::util::SyncDirectory(
        std::filesystem::path(final_filename).parent_path());
  }
  if (result != 0) {
    if (errno == ENOSPC) {
      ran_out_of_space = true;
    } else {
      PLOG(FATAL) << "Renaming " << current_filename << " to " << final_filename
                  << " failed";
    }
  } else {
    VLOG(1) << "Renamed " << current_filename << " -> " << final_filename;
  }
  return ran_out_of_space ? WriteCode::kOutOfSpace : WriteCode::kOk;
}

WriteCode RenamableFileBackend::RenamableFileHandler::Close() {
  if (!is_open()) {
    return WriteCode::kOk;
  }

  // If the base directory has been renamed, update the filename to point to the
  // new location.
  if (owner_->was_renamed()) {
    // Update the filename using the protected method.
    UpdateFilenameBase(owner_->old_base_name_, owner_->base_name_);
  }

  // Continue with the standard close logic.
  if (BufferedFileHandler::Close() == WriteCode::kOutOfSpace) {
    return WriteCode::kOutOfSpace;
  }
  if (owner_->RenameFileAfterClose(filename()) == WriteCode::kOutOfSpace) {
    return WriteCode::kOutOfSpace;
  }
  return WriteCode::kOk;
}

}  // namespace aos::logger
