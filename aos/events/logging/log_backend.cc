#include "aos/events/logging/log_backend.h"

#include <dirent.h>

#include <filesystem>
#include <numeric>

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

inline bool IsAlignedStart(const absl::Span<const uint8_t> span) {
  return (reinterpret_cast<size_t>(span.data()) % FileHandler::kSector) == 0;
}

inline bool IsAlignedLength(const absl::Span<const uint8_t> span) {
  return (span.size() % FileHandler::kSector) == 0;
}

}  // namespace

logger::QueueAligner::QueueAligner() {
  aligned_queue_.reserve(absl::GetFlag(FLAGS_queue_reserve));
}

void logger::QueueAligner::FillAlignedQueue(
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
  // If the filename already starts with the new base name, it's already been
  // updated.
  if (filename_.starts_with(new_base_name)) {
    return;
  }

  CHECK(filename_.starts_with(old_base_name))
      << "Expected old_base_name '" << old_base_name
      << "' to be at the start of filename '" << filename_ << "'";
  filename_.replace(0, old_base_name.length(), new_base_name);
}

WriteCode BufferedFileHandler::OpenForWrite() {
  if (memory_buffer_size_ > 0) {
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
  return FileHandler::OpenForWrite();
}

WriteCode FileHandler::OpenForWrite() {
  iovec_.reserve(10);
  if (!aos::util::MkdirPIfSpace(filename_, 0777, absl::GetFlag(FLAGS_sync))) {
    return WriteCode::kOutOfSpace;
  } else {
    fd_ = open(filename_.c_str(), O_RDWR | O_CLOEXEC | O_CREAT | O_EXCL, 0774);
    if (fd_ == -1 && errno == ENOSPC) {
      return WriteCode::kOutOfSpace;
    } else {
      PCHECK(fd_ != -1) << ": Failed to open " << filename_ << " for writing";
      VLOG(1) << "Opened " << filename_ << " for writing";
    }

    flags_ = fcntl(fd_, F_GETFL, 0);
    PCHECK(flags_ >= 0) << ": Failed to get flags for " << filename_;

    EnableDirect();

    CHECK(std::filesystem::exists(filename_));

    return WriteCode::kOk;
  }
}

void FileHandler::EnableDirect() {
  if (supports_odirect_ && !ODirectEnabled()) {
#ifdef O_DIRECT
    const int new_flags = flags_ | O_DIRECT;
    // Track if we failed to set O_DIRECT.  Note: Austin hasn't seen this call
    // fail.  The write call tends to fail instead.
    if (fcntl(fd_, F_SETFL, new_flags) == -1) {
      PLOG(WARNING) << "Failed to set O_DIRECT on " << filename_;
      supports_odirect_ = false;
    } else {
      flags_ = new_flags;
      odirect_enabled_ = true;
      VLOG(1) << "Enabled O_DIRECT on " << filename_;
    }
#elif defined(__APPLE__)
    if (fcntl(fd_, F_NOCACHE, 1) == -1) {
      PLOG(WARNING) << "Failed to set F_NOCACHE on " << filename_;
      supports_odirect_ = false;
    } else {
      odirect_enabled_ = true;
      VLOG(1) << "Enabled F_NOCACHE on " << filename_;
    }
#else
    // OSX likes aligned blocks to write efficiently, but does it implicitly
    // rather than explicitly.
    odirect_enabled_ = true;
    VLOG(1) << "Enabled O_DIRECT on " << filename_;
#endif
  }
}

void FileHandler::DisableDirect() {
  if (supports_odirect_ && ODirectEnabled()) {
#ifdef O_DIRECT
    flags_ = flags_ & (~O_DIRECT);
    PCHECK(fcntl(fd_, F_SETFL, flags_) != -1) << ": Failed to disable O_DIRECT";
#elif defined(__APPLE__)
    PCHECK(fcntl(fd_, F_NOCACHE, 0) != -1) << ": Failed to disable F_NOCACHE";
#endif
    odirect_enabled_ = false;
    VLOG(1) << "Disabled O_DIRECT on " << filename_;
  }
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
  // Write() call; however, because buffer_start_ is not updated by WriteAsync()
  // we can trust that the data we are copying from will not be affected.
  buffer_index_lock.unlock();
  const WriteResult result = FileHandler::Write(queue);
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
}

WriteResult BufferedFileHandler::WriteAsync(
    const absl::Span<const absl::Span<const uint8_t>> &queue) {
  size_t messages_written = 0;
  std::unique_lock<std::mutex> lock(buffer_index_mutex_);
  if (ran_out_of_space_in_thread_) {
    return {.code = WriteCode::kOutOfSpace, .messages_written = 0};
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
      const size_t buffer_space =
          (buffer_start_ > buffer_end_ || !buffer_empty_)
              ? buffer_start_ - buffer_end_
              : buffer_start_ + buffer_.size() - buffer_end_;
      if (buffer_space == 0) {
        // Wait for more space to be available to write to.
        buffer_index_update_.wait(lock);
        if (ran_out_of_space_in_thread_) {
          return {.code = WriteCode::kOutOfSpace,
                  .messages_written = messages_written};
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
      DCHECK_NE(0u, message_bytes_written)
          << "buffer_end: " << buffer_end_ << " buffer size " << buffer_.size()
          << " buffer start " << buffer_start_ << " empty " << buffer_empty_
          << " buffer space " << buffer_space;
      buffer_end_ = (buffer_end_ + target_buffer.size()) % buffer_.size();
      buffer_empty_ = false;
      buffer_index_update_.notify_all();

      if (wrap_required) {
        continue;
      }

      if (message_bytes_written == message.size()) {
        ++messages_written;
        break;
      }
    }
  }
  return {.code = WriteCode::kOk, .messages_written = messages_written};
}

WriteResult BufferedFileHandler::Write(
    const absl::Span<const absl::Span<const uint8_t>> &queue) {
  if (memory_buffer_size_ > 0) {
    return WriteAsync(queue);
  }
  return FileHandler::Write(queue);
}

bool FileHandler::ODirectEnabled() const { return odirect_enabled_; }

WriteResult FileHandler::Write(
    const absl::Span<const absl::Span<const uint8_t>> &queue) {
  iovec_.clear();
  CHECK_LE(queue.size(), static_cast<size_t>(IOV_MAX));

  queue_aligner_.FillAlignedQueue(queue);
  CHECK_LE(queue_aligner_.aligned_queue().size(), static_cast<size_t>(IOV_MAX));

  // Ok, we now need to figure out if we were aligned, and if we were, how much
  // of the data we are being asked to write is aligned.
  //
  // When writing with O_DIRECT, the kernel only will accept writes where the
  // offset into the file is a multiple of kSector, the data is aligned to
  // kSector in memory, and the length being written is a multiple of kSector.
  // Some of the callers use an aligned ResizeableBuffer to generate 512 byte
  // aligned buffers for this code to find and use.
  bool was_aligned = IsAligned(total_write_bytes_);
  VLOG(1) << "Started " << (was_aligned ? "aligned" : "unaligned")
          << " at offset " << total_write_bytes_ << " on " << filename();

  // Walk through aligned queue and batch writes based on aligned flag
  for (const auto &item : queue_aligner_.aligned_queue()) {
    if (was_aligned != item.aligned) {
      // Switching aligned context. Let's flush current batch.
      if (!iovec_.empty()) {
        // Flush current queue if we need.
        const WriteCode code = WriteV(was_aligned);
        if (code == WriteCode::kOutOfSpace) {
          // We cannot say anything about what number of messages was written
          // for sure.
          return {
              .code = code,
              .messages_written = queue.size(),
          };
        }
        iovec_.clear();
      }
      // Write queue is flushed. WriteV updates the total_write_bytes_.
      was_aligned = IsAligned(total_write_bytes_) && item.aligned;
    }
    iovec_.push_back(
        {.iov_base = const_cast<uint8_t *>(item.data), .iov_len = item.size});
  }

  WriteCode result_code = WriteCode::kOk;
  if (!iovec_.empty()) {
    // Flush current queue if we need.
    result_code = WriteV(was_aligned);
  }
  return {
      .code = result_code,
      .messages_written = queue.size(),
  };
}

WriteCode FileHandler::WriteV(bool aligned) {
  // Configure the file descriptor to match the mode we should be in.  This is
  // safe to over-call since it only does the syscall if needed.
  if (aligned) {
    EnableDirect();
  } else {
    DisableDirect();
  }

  VLOG(2) << "Flushing queue of " << iovec_.size() << " elements, "
          << (aligned ? "aligned" : "unaligned");

  CHECK_GT(iovec_.size(), 0u);
  const auto start = aos::monotonic_clock::now();

  // Validation of alignment assumptions.
  if (aligned) {
    CHECK(IsAligned(total_write_bytes_))
        << ": Failed after writing " << total_write_bytes_
        << " to the file, attempting aligned write with unaligned start.";

    for (const auto &iovec_item : iovec_) {
      absl::Span<const uint8_t> data(
          reinterpret_cast<const uint8_t *>(iovec_item.iov_base),
          iovec_item.iov_len);
      VLOG(2) << "  iov_base " << static_cast<void *>(iovec_item.iov_base)
              << ", iov_len " << iovec_item.iov_len;
      CHECK(IsAlignedStart(data) && IsAlignedLength(data));
    }
  }

  size_t iovecs_index = 0;
  size_t total_written = 0;
  // Iterate until we write all the bytes or get an error.
  while (true) {
    // Calculation of expected written size.
    size_t counted_size =
        std::accumulate(iovec_.begin() + iovecs_index, iovec_.end(), size_t(0),
                        [](size_t count, const struct iovec &next_iovec) {
                          return count + next_iovec.iov_len;
                        });

    VLOG(2) << "Going to write " << counted_size;
    CHECK_GT(counted_size, 0u);

    const ssize_t written =
        writev(fd_, iovec_.data() + iovecs_index, iovec_.size() - iovecs_index);
    VLOG(2) << "Wrote " << written << ", for iovec size " << iovec_.size();

    if (written == -1 && errno == ENOSPC) {
      PLOG(ERROR) << "Wrote " << written << " bytes of " << counted_size;
      return WriteCode::kOutOfSpace;
    }
    PCHECK(written >= 0) << ": write failed, got " << written << " for "
                         << filename_;
    total_written += written;
    if (written < static_cast<ssize_t>(counted_size)) {
      // Note that we have observed this condition (less data being written than
      // requested) when:
      // 1. We try to write a massive buffer (over 2 GiB typically).
      // 2. We run out of space on the filesystem.
      // For both cases, we want to remove the bytes that we have successfully
      // written from the iovecs_ vector and attempt to write the case. In the
      // first case, we will eventually finish writing everything. In the latter
      // case we will get an ENOSPC on any subsequent writes.
      //
      // Future work may also create situations where we e.g. attempt to write
      // to sockets where we may more frequently encounter incomplete writes.
      if (VLOG_IS_ON(1)) {
        PLOG(WARNING) << "Wrote " << written << " bytes of " << counted_size;
      }
      encountered_incomplete_write_ = true;
      ssize_t bytes_to_evict = written;
      while (iovec_.at(iovecs_index).iov_len <=
             static_cast<size_t>(bytes_to_evict)) {
        bytes_to_evict -= iovec_.at(iovecs_index).iov_len;
        ++iovecs_index;
        // Mostly a sanity check.
        // If iovecs_index > iovec_.size() then we wrote more bytes than we
        // had in iovec_. If iovecs_index == iovec_.size() then written
        // should strictly equal counted_size and we should not have ended up
        // here.
        CHECK(iovecs_index < iovec_.size());
      }
      iovec_.at(iovecs_index).iov_base =
          reinterpret_cast<uint8_t *>(iovec_.at(iovecs_index).iov_base) +
          bytes_to_evict;
      iovec_.at(iovecs_index).iov_len -= bytes_to_evict;
    } else {
      iovecs_index = iovec_.size();
      break;
    }
  }

  const auto end = aos::monotonic_clock::now();

  if (absl::GetFlag(FLAGS_sync)) {
#ifdef __linux__
    // Flush asynchronously and force the data out of the cache.
    sync_file_range(fd_, total_write_bytes_, total_written,
                    SYNC_FILE_RANGE_WRITE);
    if (last_synced_bytes_ != 0) {
      // Per Linus' recommendation online on how to do fast file IO, do a
      // blocking flush of the previous write chunk, and then tell the kernel to
      // drop the pages from the cache.  This makes sure we can't get too far
      // ahead.
      sync_file_range(fd_, last_synced_bytes_,
                      total_write_bytes_ - last_synced_bytes_,
                      SYNC_FILE_RANGE_WAIT_BEFORE | SYNC_FILE_RANGE_WRITE |
                          SYNC_FILE_RANGE_WAIT_AFTER);
      posix_fadvise(fd_, last_synced_bytes_,
                    total_write_bytes_ - last_synced_bytes_,
                    POSIX_FADV_DONTNEED);
    }
#elif defined(__APPLE__)
    if (fcntl(fd_, F_FULLFSYNC) == -1) {
      PLOG(WARNING) << "Failed to F_FULLFSYNC " << filename_;
    }
#endif
    last_synced_bytes_ = total_write_bytes_;
  }

  total_write_bytes_ += total_written;
  if (aligned) {
    written_aligned_ += total_written;
  }
  WriteStatistics()->UpdateStats(end - start, total_written, iovecs_index);
  return WriteCode::kOk;
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
  return FileHandler::Close();
}

WriteCode FileHandler::Close() {
  if (!is_open()) {
    return WriteCode::kOk;
  }
  bool ran_out_of_space = false;

  if (absl::GetFlag(FLAGS_sync)) {
    // Force everythig out at the end so we know that it hits disk.
#ifdef __linux__
    fdatasync(fd_);
#elif defined(__APPLE__)
    if (fcntl(fd_, F_FULLFSYNC) == -1) {
      PLOG(WARNING) << "Failed to F_FULLFSYNC " << filename_;
    }
#else
    fsync(fd_);
#endif
  }

  if (close(fd_) == -1) {
    if (errno == ENOSPC) {
      ran_out_of_space = true;
    } else {
      PLOG(ERROR) << "Closing log file failed";
    }
  }
  if (absl::GetFlag(FLAGS_sync)) {
    aos::util::SyncDirectory(std::filesystem::path(filename_).parent_path());
  }
  fd_ = -1;
  VLOG(1) << "Closed " << filename_;
  return ran_out_of_space ? WriteCode::kOutOfSpace : WriteCode::kOk;
}

FileBackend::FileBackend(std::string_view base_name, bool supports_odirect)
    : supports_odirect_(supports_odirect),
      base_name_(base_name),
      separator_(base_name_.back() == '/' ? "" : "_") {}

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
  internal::LocalFileOperations operations(directory.string());
  std::vector<internal::FileOperations::File> files;
  operations.FindLogs(&files);

  std::vector<File> names;
  const std::string prefix = absl::StrCat(base_name_, separator_);
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

RenamableFileBackend::RenamableFileBackend(std::string_view base_name,
                                           bool supports_odirect)
    : supports_odirect_(supports_odirect),
      base_name_(base_name),
      separator_(base_name_.back() == '/' ? "" : "_") {}

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

bool RenamableFileBackend::RenameLogBase(std::string_view new_base_name) {
  if (new_base_name == base_name_) {
    return true;
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
  DIR *dir = opendir(current_directory.c_str());
  if (dir) {
    closedir(dir);
    const int result = rename(current_directory.c_str(), new_directory.c_str());
    if (result != 0) {
      PLOG(ERROR) << "Unable to rename " << current_directory << " to "
                  << new_directory;
      return false;
    }

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
  } else {
    // Handle if directory was already renamed.
    dir = opendir(new_directory.c_str());
    if (!dir) {
      LOG(ERROR) << "Old directory " << current_directory
                 << " missing and new directory " << new_directory
                 << " not present.";
      return false;
    }
    closedir(dir);
  }
  old_base_name_ = base_name_;
  base_name_ = std::string(new_base_name);
  separator_ = base_name_.back() == '/' ? "" : "_";
  return true;
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
