#include "aos/events/logging/logger_statistics.h"

#include "absl/log/die_if_null.h"

namespace aos::logger {

void WriteStatistics::ResetStats() {
  max_write_time_ = std::chrono::nanoseconds::zero();
  max_write_time_bytes_ = -1;
  max_write_time_messages_ = -1;
  total_write_time_ = std::chrono::nanoseconds::zero();
  total_write_count_ = 0;
  total_write_messages_ = 0;
  total_write_bytes_ = 0;
}

void WriteStatistics::UpdateStats(std::chrono::nanoseconds duration,
                                  ssize_t written, int messages) {
  if (duration > max_write_time_) {
    max_write_time_ = duration;
    max_write_time_bytes_ = written;
    max_write_time_messages_ = messages;
  }
  total_write_time_ += duration;
  ++total_write_count_;
  total_write_messages_ += messages;
  total_write_bytes_ += written;
}

// Accumulates another statistics object into this object.
void WriteStatistics::UpdateWithStats(const WriteStatistics &other) {
  if (other.max_write_time_ > max_write_time_) {
    max_write_time_ = other.max_write_time_;
    max_write_time_bytes_ = other.max_write_time_bytes_;
    max_write_time_messages_ = other.max_write_time_messages_;
  }
  total_write_messages_ += other.total_write_messages_;
  total_write_bytes_ += other.total_write_bytes_;
  total_write_count_ += other.total_write_count_;
  total_write_time_ += other.total_write_time_;
}

void WriteStatistics::PopulateFbs(fbs::WriteStatisticsStatic *fbs) const {
  fbs->set_total_write_messages(total_write_messages_);
  fbs->set_total_write_bytes(total_write_bytes_);
  fbs->set_total_write_count(total_write_count_);
  fbs->set_total_write_time_ns(total_write_time_.count());
  fbs->set_max_write_time_ns(max_write_time_.count());
  fbs->set_max_write_time_bytes(max_write_time_bytes_);
  fbs->set_max_write_time_messages(max_write_time_messages_);
}

void LoggerStatistics::ResetStats() {
  disk_stats_.ResetStats();
  handler_stats_.ResetStats();
  total_encode_duration_ = std::chrono::nanoseconds::zero();
  memory_buffer_bytes_available_.reset();
}

void LoggerStatistics::UpdateWithStats(const LoggerStatistics &other) {
  disk_stats_.UpdateWithStats(other.disk_stats_);
  handler_stats_.UpdateWithStats(other.handler_stats_);
  total_encode_duration_ += other.total_encode_duration_;
  if (other.memory_buffer_bytes_available_.has_value()) {
    memory_buffer_bytes_available_ =
        memory_buffer_bytes_available_.value_or(0) +
        other.memory_buffer_bytes_available_.value();
  }
}

void LoggerStatistics::PopulateFbs(fbs::LoggerStatisticsStatic *fbs) const {
  disk_stats_.PopulateFbs(ABSL_DIE_IF_NULL(fbs->add_disk_statistics()));
  handler_stats_.PopulateFbs(ABSL_DIE_IF_NULL(fbs->add_handler_statistics()));
  fbs->set_total_encode_duration_ns(total_encode_duration_.count());
  if (memory_buffer_bytes_available_.has_value()) {
    fbs->set_memory_buffer_bytes_available(
        memory_buffer_bytes_available_.value());
  }
}

}  // namespace aos::logger
