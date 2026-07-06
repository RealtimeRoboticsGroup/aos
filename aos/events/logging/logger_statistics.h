#ifndef AOS_EVENTS_LOGGING_WRITE_STATS_H_
#define AOS_EVENTS_LOGGING_WRITE_STATS_H_

#include <chrono>
#include <cstddef>

#include "aos/events/logging/logger_statistics_static.h"

namespace aos::logger {

class WriteStatistics {
 public:
  // The maximum time for a single write call, or 0 if none have been performed.
  std::chrono::nanoseconds max_write_time() const { return max_write_time_; }
  // The number of bytes in the longest write call, or -1 if none have been
  // performed.
  int max_write_time_bytes() const { return max_write_time_bytes_; }
  // The number of buffers in the longest write call, or -1 if none have been
  // performed.
  int max_write_time_messages() const { return max_write_time_messages_; }
  // The total time spent in write calls.
  std::chrono::nanoseconds total_write_time() const {
    return total_write_time_;
  }

  // The total number of writes which have been performed.
  int total_write_count() const { return total_write_count_; }
  // The total number of messages which have been written.
  int total_write_messages() const { return total_write_messages_; }
  // The total number of bytes which have been written.
  int total_write_bytes() const { return total_write_bytes_; }

  void ResetStats();

  void UpdateStats(std::chrono::nanoseconds duration, std::ptrdiff_t written,
                   int messages);

  // Accumulates another statistics object into this object.
  void UpdateWithStats(const WriteStatistics &other);

  void PopulateFbs(fbs::WriteStatisticsStatic *fbs) const;

 private:
  // Total number of messages written.
  int total_write_messages_ = 0;
  // Total number of bytes passed to Write().
  int total_write_bytes_ = 0;
  // Total number of Write() calls.
  int total_write_count_ = 0;
  // Total time spent in the write calls.
  std::chrono::nanoseconds total_write_time_ = std::chrono::nanoseconds::zero();
  // Maximum time spent in any single write call.
  std::chrono::nanoseconds max_write_time_ = std::chrono::nanoseconds::zero();
  // Number of bytes received in the longest write call.
  int max_write_time_bytes_ = -1;
  // Number of messages received in the longest write call.
  int max_write_time_messages_ = -1;
};

class LoggerStatistics {
 public:
  LoggerStatistics() {}

  void ResetStats();

  // Updates the statistics associated with what we have actually written to
  // disk.
  void UpdateDiskStats(std::chrono::nanoseconds duration,
                       std::ptrdiff_t written) {
    // We don't track message counts for what we actually flush to disk.
    disk_stats_.UpdateStats(duration, written, /*messages*/ 0);
  }

  // Updates the statistics associated with what has been passed to the
  // FileHandler.
  void UpdateHandlerStats(std::chrono::nanoseconds duration,
                          std::ptrdiff_t written, int messages) {
    handler_stats_.UpdateStats(duration, written, messages);
  }

  // Update our total_encode_duration_ stat; used by the buffer writer rather
  // than the FileHandler classes.
  void UpdateEncodeDuration(std::chrono::nanoseconds duration) {
    total_encode_duration_ += duration;
  }

  void UpdateMemoryBufferBytesAvailable(size_t memory_buffer_bytes_available) {
    memory_buffer_bytes_available_ = memory_buffer_bytes_available;
    CHECK_LE(0u, memory_buffer_bytes_available);
  }

  // Accumulates another statistics object into this object.
  // This is meant for combining statistics across FileHandlers. Most statistics
  // are added together. The only current exception is the max_write_time_*
  // statistics capturing worst-case situations.
  void UpdateWithStats(const LoggerStatistics &other);

  // The total time spent encoding.
  std::chrono::nanoseconds total_encode_duration() const {
    return total_encode_duration_;
  }

  const WriteStatistics &disk_stats() const { return disk_stats_; }
  const WriteStatistics &handler_stats() const { return handler_stats_; }
  std::optional<size_t> memory_buffer_bytes_available() const {
    return memory_buffer_bytes_available_;
  }

  void PopulateFbs(fbs::LoggerStatisticsStatic *fbs) const;

 private:
  // Statistics for tracking what has actually been written to disk.
  // For non-async file writers, these numbers will typically match the
  // handler_stats_ numbers for total bytes written exactly.
  // Note that the disk stats will not track the total messages written, because
  // the asynchronous writer ends up combining messages into a single memory
  // buffer before passing them to the write thread.
  WriteStatistics disk_stats_;
  // Statistics for tracking what has been passed to the FileHandler. These
  // bytes may not yet be committed to disk.
  WriteStatistics handler_stats_;
  std::chrono::nanoseconds total_encode_duration_ =
      std::chrono::nanoseconds::zero();

  std::optional<size_t> memory_buffer_bytes_available_;
};
}  // namespace aos::logger
#endif  // AOS_EVENTS_LOGGING_WRITE_STATS_H_
