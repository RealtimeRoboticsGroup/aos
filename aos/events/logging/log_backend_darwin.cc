#include "aos/events/logging/log_backend.h"

#include <fcntl.h>
#include <sys/uio.h>
#include <unistd.h>

#include <filesystem>
#include <numeric>
#include <vector>

#include "absl/flags/declare.h"
#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"

#include "aos/util/file.h"

ABSL_DECLARE_FLAG(bool, sync);

namespace aos::logger {

void FileHandler::EnableDirect() {
  if (supports_odirect_ && !ODirectEnabled()) {
    if (fcntl(fd_, F_NOCACHE, 1) == -1) {
      PLOG(WARNING) << "Failed to set F_NOCACHE on " << filename_;
      supports_odirect_ = false;
    } else {
      odirect_enabled_ = true;
      VLOG(1) << "Enabled F_NOCACHE on " << filename_;
    }
  }
}

void FileHandler::DisableDirect() {
  if (supports_odirect_ && ODirectEnabled()) {
    PCHECK(fcntl(fd_, F_NOCACHE, 0) != -1) << ": Failed to disable F_NOCACHE";
    odirect_enabled_ = false;
    VLOG(1) << "Disabled F_NOCACHE on " << filename_;
  }
}

WriteCode FileHandler::PlatformSyncImpl() {
  if (fcntl(fd_, F_FULLFSYNC) == -1) {
    if (errno == ENOSPC) {
      return WriteCode::kOutOfSpace;
    }
    // Not every filesystem implements F_FULLFSYNC, so anything else stays the
    // warning this backend has logged since before it was split out.
    PLOG(WARNING) << "Failed to F_FULLFSYNC " << filename_;
  }
  return WriteCode::kOk;
}

std::pair<WriteCode, size_t> FileHandler::WriteV(bool) {
  const auto start = aos::monotonic_clock::now();
  const ssize_t written = writev(fd_, iovec_.data(), iovec_.size());
  const auto end = aos::monotonic_clock::now();

  if (written == -1) {
    if (errno == ENOSPC) {
      return std::make_pair(WriteCode::kOutOfSpace, 0);
    }
    PLOG(FATAL) << "writev failed for " << filename_;
  }

  if (absl::GetFlag(FLAGS_sync)) {
    // --sync promises the data is on disk as we go ("sync the file after each
    // written block"), not just at Close().  Linux gets that from
    // sync_file_range() in its WriteV(); the Darwin equivalent is a full
    // F_FULLFSYNC, which is what this backend did before it was split out of
    // log_backend_linux.cc.  Without it a crash between here and Close() loses
    // writes we already reported as durable.
    //
    // Deliberately after `end` is sampled: the disk stats measure the write
    // itself, matching the pre-split behavior and the Linux backend.
    if (PlatformSync() == WriteCode::kOutOfSpace) {
      // These bytes did reach the file, they just aren't durable, so report
      // the count along with the failure and let DoWrite() wind the log down.
      WriteStatistics()->UpdateDiskStats(end - start, written);
      return std::make_pair(WriteCode::kOutOfSpace, written);
    }
  }

  WriteStatistics()->UpdateDiskStats(end - start, written);
  return std::make_pair(WriteCode::kOk, written);
}

WriteResult FileHandler::DoWrite(
    const absl::Span<const absl::Span<const uint8_t>> &queue) {
  // We can write up to kIovMax elements at a time.
  // We also limit each writev payload to 1 GiB to avoid overflow/macOS limits.
  constexpr size_t kMaxSingleWriteSize = 1024 * 1024 * 1024;

  size_t total_bytes_written = 0;
  size_t current_msg_index = 0;
  size_t current_msg_offset = 0;

  while (current_msg_index < queue.size()) {
    iovec_.clear();
    size_t batch_bytes = 0;

    // Build the batch of iovecs
    for (size_t i = current_msg_index;
         i < queue.size() && iovec_.size() < static_cast<size_t>(kIovMax);
         ++i) {
      const auto &msg = queue[i];
      size_t msg_len = msg.size();
      size_t offset = (i == current_msg_index) ? current_msg_offset : 0;
      size_t len_to_write = msg_len - offset;

      if (len_to_write == 0) {
        if (i == current_msg_index) {
          current_msg_index++;
          current_msg_offset = 0;
        }
        continue;
      }

      // Limit to 1 GiB total write size for the system call
      if (batch_bytes + len_to_write > kMaxSingleWriteSize) {
        if (iovec_.empty()) {
          // A single message larger than the cap doesn't fit even on its own,
          // so write the first kMaxSingleWriteSize of it and pick the rest up
          // on the next trip around the outer loop.  Every push_back below
          // grows batch_bytes by the same non-zero amount, so an empty iovec_
          // means the whole budget is still available.
          DCHECK_EQ(batch_bytes, 0u);
          iovec_.push_back(
              {.iov_base = const_cast<uint8_t *>(msg.data() + offset),
               .iov_len = kMaxSingleWriteSize});
          batch_bytes += kMaxSingleWriteSize;
        }
        break;
      }

      iovec_.push_back({.iov_base = const_cast<uint8_t *>(msg.data() + offset),
                        .iov_len = len_to_write});
      batch_bytes += len_to_write;
    }

    if (iovec_.empty()) {
      // All remaining messages in queue were empty or handled
      break;
    }

    const auto [code, written] = WriteV(false);
    if (code == WriteCode::kOutOfSpace) {
      // A failing writev() reports zero bytes; a failing sync after a
      // successful writev() reports what that writev() put in the file.
      // Either way those bytes are on the file, so count them before winding
      // down rather than under-reporting the batch.
      total_bytes_written += written;
      total_write_bytes_ += written;
      return {
          .code = WriteCode::kOutOfSpace,
          .messages_written = queue.size(),
          .bytes_written = total_bytes_written,
      };
    }

    if (written < batch_bytes) {
      encountered_incomplete_write_ = true;
    }

    total_bytes_written += written;
    total_write_bytes_ += written;

    // Advance our pointers based on how many bytes were actually written
    size_t bytes_to_consume = written;
    while (bytes_to_consume > 0 && current_msg_index < queue.size()) {
      size_t msg_len = queue[current_msg_index].size();
      size_t remaining_in_msg = msg_len - current_msg_offset;
      if (bytes_to_consume >= remaining_in_msg) {
        bytes_to_consume -= remaining_in_msg;
        current_msg_index++;
        current_msg_offset = 0;
      } else {
        current_msg_offset += bytes_to_consume;
        bytes_to_consume = 0;
      }
    }
  }

  return {
      .code = WriteCode::kOk,
      .messages_written = queue.size(),
      .bytes_written = total_bytes_written,
  };
}

}  // namespace aos::logger
