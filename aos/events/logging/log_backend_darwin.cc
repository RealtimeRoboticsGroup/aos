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

#include "aos/events/logging/log_backend.h"
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

void FileHandler::PlatformSync() {
  if (fcntl(fd_, F_FULLFSYNC) == -1) {
    PLOG(WARNING) << "Failed to F_FULLFSYNC " << filename_;
  }
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
          // Write a chunk of this large message
          size_t chunk_len = kMaxSingleWriteSize - batch_bytes;
          iovec_.push_back(
              {.iov_base = const_cast<uint8_t *>(msg.data() + offset),
               .iov_len = chunk_len});
          batch_bytes += chunk_len;
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
