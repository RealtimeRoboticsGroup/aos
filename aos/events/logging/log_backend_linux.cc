#include <fcntl.h>
#include <sys/uio.h>
#include <unistd.h>

#include <filesystem>
#include <numeric>

#include "absl/flags/declare.h"
#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/log/vlog_is_on.h"

#include "aos/events/logging/log_backend.h"
#include "aos/util/file.h"

ABSL_DECLARE_FLAG(bool, sync);

namespace aos::logger {

namespace {

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

std::pair<WriteCode, size_t> FileHandler::WriteV(bool aligned) {
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
      return std::make_pair(WriteCode::kOutOfSpace, 0);
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
  WriteStatistics()->UpdateDiskStats(end - start, total_written);
  return std::make_pair(WriteCode::kOk, total_written);
}

WriteCode FileHandler::OpenForWrite() {
  iovec_.reserve(10);
  if (!aos::util::MkdirPIfSpace(filename_, std::filesystem::perms::all,
                                absl::GetFlag(FLAGS_sync))) {
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

WriteResult FileHandler::DoWrite(
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
  size_t total_bytes_written = 0;

  // Walk through aligned queue and batch writes based on aligned flag
  for (const auto &item : queue_aligner_.aligned_queue()) {
    if (was_aligned != item.aligned) {
      // Switching aligned context. Let's flush current batch.
      if (!iovec_.empty()) {
        // Flush current queue if we need.
        const auto [code, bytes_written] = WriteV(was_aligned);
        total_bytes_written += bytes_written;
        if (code == WriteCode::kOutOfSpace) {
          // We cannot say anything about what number of messages was written
          // for sure.
          return {
              .code = code,
              .messages_written = queue.size(),
              .bytes_written = total_bytes_written,
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
    size_t bytes_written;
    // Flush current queue if we need.
    std::tie(result_code, bytes_written) = WriteV(was_aligned);
    total_bytes_written += bytes_written;
  }
  return {
      .code = result_code,
      .messages_written = queue.size(),
      .bytes_written = total_bytes_written,
  };
}

bool RenamableFileBackend::RenameLogBase(std::string_view new_base_name) {
  auto paths = ValidateAndSplitRenamePaths(new_base_name);
  if (!paths) {
    return true;
  }
  const auto &[current_directory, new_directory] = *paths;

  const bool dir_exists = DirectoryExists(current_directory);

  if (dir_exists) {
    const int result = rename(current_directory.c_str(), new_directory.c_str());
    if (result != 0) {
      // Out of space is the one condition the logger is built to ride out, so
      // it stays a reported failure: the base name is left alone and logging
      // carries on in the old directory until the writes themselves start
      // coming back kOutOfSpace and wind it down.
      //
      // Nothing else rename(2) can fail with here fixes itself.  They all say
      // this deployment is wrong rather than unlucky -- EXDEV for a new base
      // name on a different mount, EACCES or EROFS for a parent we can't write,
      // EEXIST or ENOTEMPTY for a target already sitting there -- and the
      // caller has no way to act on any of them (set_base_name() drops this
      // return value on the floor).
      PCHECK(errno == ENOSPC) << ": Unable to rename " << current_directory
                              << " to " << new_directory;
      PLOG(ERROR) << "Ran out of space renaming " << current_directory << " to "
                  << new_directory << "; logging continues at the old path";
      return false;
    }

    SyncParentDirectories(current_directory, new_directory);
  } else {
    // Neither name is on disk, so the directory being logged into has gone out
    // from under us -- this is not the "somebody already renamed it" case,
    // which is new_directory being present.  There is nothing left to rename
    // and nothing to keep logging into.
    CHECK(DirectoryExists(new_directory))
        << ": Old directory " << current_directory
        << " missing and new directory " << new_directory << " not present.";
  }
  old_base_name_ = base_name_;
  base_name_ = std::string(new_base_name);
  separator_ = base_name_.back() == '/' ? "" : "_";

  return true;
}

std::unique_ptr<RenamableFileBackend> MakeRenamableFileBackend(
    std::string_view base_name, bool supports_odirect) {
  return std::make_unique<RenamableFileBackend>(base_name, supports_odirect);
}

}  // namespace aos::logger
