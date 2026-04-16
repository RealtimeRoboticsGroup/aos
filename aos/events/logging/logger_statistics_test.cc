#include "aos/events/logging/logger_statistics.h"

#include "gtest/gtest.h"

#include "aos/flatbuffers/builder.h"
#include "aos/json_to_flatbuffer.h"

namespace aos::logger::testing {
TEST(WriteStatistics, Accumulate) {
  WriteStatistics stats;
  const std::chrono::seconds kWriteTime{1};
  const size_t kWriteBytes = 100;
  const size_t kWriteMessages = 10;
  stats.UpdateStats(kWriteTime, kWriteBytes, kWriteMessages);
  EXPECT_EQ(kWriteTime, stats.total_write_time());
  EXPECT_EQ(kWriteBytes, stats.total_write_bytes());
  EXPECT_EQ(kWriteMessages, stats.total_write_messages());

  EXPECT_EQ(kWriteTime, stats.max_write_time());
  EXPECT_EQ(kWriteBytes, stats.max_write_time_bytes());
  EXPECT_EQ(kWriteMessages, stats.max_write_time_messages());

  stats.UpdateStats(kWriteTime, kWriteBytes, kWriteMessages);
  EXPECT_EQ(2 * kWriteTime, stats.total_write_time());
  EXPECT_EQ(2 * kWriteBytes, stats.total_write_bytes());
  EXPECT_EQ(2 * kWriteMessages, stats.total_write_messages());
  // Max times should have been unaffected by us recording the same data gain.
  EXPECT_EQ(kWriteTime, stats.max_write_time());
  EXPECT_EQ(kWriteBytes, stats.max_write_time_bytes());
  EXPECT_EQ(kWriteMessages, stats.max_write_time_messages());

  // If we record a longer interval we should see the max time go up.
  stats.UpdateStats(3 * kWriteTime, 3 * kWriteBytes, 3 * kWriteMessages);
  EXPECT_EQ(5 * kWriteTime, stats.total_write_time());
  EXPECT_EQ(5 * kWriteBytes, stats.total_write_bytes());
  EXPECT_EQ(5 * kWriteMessages, stats.total_write_messages());

  EXPECT_EQ(3 * kWriteTime, stats.max_write_time());
  EXPECT_EQ(3 * kWriteBytes, stats.max_write_time_bytes());
  EXPECT_EQ(3 * kWriteMessages, stats.max_write_time_messages());
}

TEST(WriteStatistics, ResetStats) {
  WriteStatistics stats;
  const std::chrono::seconds kWriteTime{1};
  const size_t kWriteBytes = 100;
  const size_t kWriteMessages = 10;
  stats.UpdateStats(kWriteTime, kWriteBytes, kWriteMessages);
  EXPECT_EQ(kWriteTime, stats.total_write_time());
  EXPECT_EQ(kWriteBytes, stats.total_write_bytes());
  EXPECT_EQ(kWriteMessages, stats.total_write_messages());

  EXPECT_EQ(kWriteTime, stats.max_write_time());
  EXPECT_EQ(kWriteBytes, stats.max_write_time_bytes());
  EXPECT_EQ(kWriteMessages, stats.max_write_time_messages());

  stats.ResetStats();
  EXPECT_EQ(std::chrono::seconds(0), stats.total_write_time());
  EXPECT_EQ(0, stats.total_write_bytes());
  EXPECT_EQ(0, stats.total_write_messages());
  EXPECT_EQ(std::chrono::seconds(0), stats.max_write_time());
  EXPECT_EQ(-1, stats.max_write_time_bytes());
  EXPECT_EQ(-1, stats.max_write_time_messages());
}

TEST(WriteStatistics, CombineStats) {
  WriteStatistics stats1, stats2;
  const std::chrono::seconds kWriteTime{1};
  const size_t kWriteBytes = 100;
  const size_t kWriteMessages = 10;
  stats1.UpdateStats(kWriteTime, kWriteBytes, kWriteMessages);
  stats2.UpdateStats(2 * kWriteTime, 2 * kWriteBytes, 2 * kWriteMessages);
  stats1.UpdateWithStats(stats2);
  EXPECT_EQ(3 * kWriteTime, stats1.total_write_time());
  EXPECT_EQ(3 * kWriteBytes, stats1.total_write_bytes());
  EXPECT_EQ(3 * kWriteMessages, stats1.total_write_messages());

  EXPECT_EQ(2 * kWriteTime, stats1.max_write_time());
  EXPECT_EQ(2 * kWriteBytes, stats1.max_write_time_bytes());
  EXPECT_EQ(2 * kWriteMessages, stats1.max_write_time_messages());
}

TEST(LoggerStatistics, Reset) {
  LoggerStatistics stats;
  const std::chrono::seconds kWriteTime{1};
  const size_t kWriteBytes = 100;
  const size_t kWriteMessages = 10;
  const size_t kMemoryBufferBytes = 123;
  stats.UpdateDiskStats(kWriteTime, kWriteBytes);
  // The total messages for the disk stats should always be zero.
  EXPECT_EQ(0, stats.disk_stats().total_write_messages());
  stats.UpdateHandlerStats(2 * kWriteTime, 2 * kWriteBytes, 2 * kWriteMessages);
  stats.UpdateEncodeDuration(kWriteTime);
  stats.UpdateMemoryBufferBytesAvailable(kMemoryBufferBytes);
  // Spot check some numbers to make sure we are connecting the right modifiers
  // to the right WriteStatistics objects.
  EXPECT_EQ(kWriteTime, stats.disk_stats().total_write_time());
  EXPECT_EQ(2 * kWriteTime, stats.handler_stats().total_write_time());
  EXPECT_EQ(kWriteTime, stats.total_encode_duration());
  ASSERT_TRUE(stats.memory_buffer_bytes_available());
  EXPECT_EQ(kMemoryBufferBytes, stats.memory_buffer_bytes_available().value());

  stats.ResetStats();
  EXPECT_EQ(std::chrono::seconds(0), stats.disk_stats().total_write_time());
  EXPECT_EQ(std::chrono::seconds(0), stats.handler_stats().total_write_time());
  EXPECT_EQ(std::chrono::seconds(0), stats.total_encode_duration());
  ASSERT_FALSE(stats.memory_buffer_bytes_available());
}

TEST(LoggerStatistics, CombineStats) {
  LoggerStatistics stats1, stats2;
  const std::chrono::seconds kWriteTime{1};
  const size_t kWriteBytes = 100;
  const size_t kWriteMessages = 10;
  const size_t kMemoryBufferBytes = 123;
  stats1.UpdateDiskStats(kWriteTime, kWriteBytes);
  stats2.UpdateDiskStats(kWriteTime, kWriteBytes);
  stats1.UpdateHandlerStats(kWriteTime, kWriteBytes, kWriteMessages);
  stats2.UpdateHandlerStats(kWriteTime, kWriteBytes, kWriteMessages);
  stats1.UpdateEncodeDuration(kWriteTime);
  stats2.UpdateEncodeDuration(kWriteTime);
  stats1.UpdateMemoryBufferBytesAvailable(kMemoryBufferBytes);
  stats2.UpdateMemoryBufferBytesAvailable(kMemoryBufferBytes);
  stats1.UpdateWithStats(stats2);

  // Spot check some numbers to make sure we are connecting the right modifiers
  // to the right WriteStatistics objects.
  EXPECT_EQ(2 * kWriteTime, stats1.disk_stats().total_write_time());
  EXPECT_EQ(2 * kWriteTime, stats1.handler_stats().total_write_time());
  EXPECT_EQ(2 * kWriteTime, stats1.total_encode_duration());
  EXPECT_EQ(2 * kMemoryBufferBytes,
            stats1.memory_buffer_bytes_available().value());
}

// Validate that we combine memory buffer byte availability by summing up
// values, and that we correctly combine even when one statistics value has
// nothing set.
TEST(LoggerStatistics, CombineStatsMemoryBuffer) {
  const size_t kMemoryBufferBytes = 123;
  LoggerStatistics stats1, stats2;
  // Try with neither stats* having anything populated.
  stats1.UpdateWithStats(stats2);
  EXPECT_FALSE(stats1.memory_buffer_bytes_available().has_value());
  // Try with just stats1 populated.
  stats1.UpdateMemoryBufferBytesAvailable(kMemoryBufferBytes);
  stats1.UpdateWithStats(stats2);
  ASSERT_TRUE(stats1.memory_buffer_bytes_available().has_value());
  EXPECT_EQ(kMemoryBufferBytes, stats1.memory_buffer_bytes_available().value());
  // Try with just stats2 populated.
  stats1.ResetStats();
  ASSERT_FALSE(stats1.memory_buffer_bytes_available().has_value());
  stats2.UpdateMemoryBufferBytesAvailable(kMemoryBufferBytes);
  stats1.UpdateWithStats(stats2);
  ASSERT_TRUE(stats1.memory_buffer_bytes_available().has_value());
  EXPECT_EQ(kMemoryBufferBytes, stats1.memory_buffer_bytes_available().value());

  // Try with both populated
  stats1.ResetStats();
  ASSERT_FALSE(stats1.memory_buffer_bytes_available().has_value());
  stats1.UpdateMemoryBufferBytesAvailable(kMemoryBufferBytes);
  stats1.UpdateWithStats(stats2);
  ASSERT_TRUE(stats1.memory_buffer_bytes_available().has_value());
  EXPECT_EQ(2 * kMemoryBufferBytes,
            stats1.memory_buffer_bytes_available().value());
}

TEST(LoggerStatistics, PopulateFlatbuffer) {
  LoggerStatistics stats;
  const std::chrono::seconds kWriteTime{1};
  const size_t kWriteBytes = 100;
  const size_t kWriteMessages = 10;
  stats.UpdateDiskStats(kWriteTime, kWriteBytes);
  stats.UpdateHandlerStats(2 * kWriteTime, 2 * kWriteBytes, 2 * kWriteMessages);
  stats.UpdateEncodeDuration(kWriteTime);
  stats.UpdateMemoryBufferBytesAvailable(123);

  aos::fbs::Builder<fbs::LoggerStatisticsStatic> builder;
  stats.PopulateFbs(builder.get());
  ASSERT_EQ(R"json({
 "disk_statistics": {
  "total_write_messages": 0,
  "total_write_bytes": 100,
  "total_write_count": 1,
  "total_write_time_ns": 1000000000,
  "max_write_time_ns": 1000000000,
  "max_write_time_bytes": 100,
  "max_write_time_messages": 0
 },
 "handler_statistics": {
  "total_write_messages": 20,
  "total_write_bytes": 200,
  "total_write_count": 1,
  "total_write_time_ns": 2000000000,
  "max_write_time_ns": 2000000000,
  "max_write_time_bytes": 200,
  "max_write_time_messages": 20
 },
 "total_encode_duration_ns": 1000000000,
 "memory_buffer_bytes_available": 123
})json",
            aos::FlatbufferToJson(builder, {.multi_line = true}));
}
}  // namespace aos::logger::testing
