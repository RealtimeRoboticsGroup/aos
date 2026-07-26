#include <cinttypes>

#include "absl/log/log.h"

#include "aos/containers/resizeable_buffer.h"
#include "aos/ipc_lib/lockless_queue_memory.h"
#include "aos/ipc_lib/lockless_queue_test_utils.h"
#include "aos/ipc_lib/queue_racer.h"

namespace aos::ipc_lib::testing {

namespace chrono = ::std::chrono;

// Verifies that initializing a queue to start at index N and then sending
// M messages puts it in a byte-for-byte equivalent state to a queue initialized
// to 0 and having N + M messages sent naturally.
TEST_F(LocklessQueueTest, InitializeVerification) {
  const uint32_t queue_size = config_.queue_size;
  const uint64_t M = 2 * queue_size;

  const std::vector<uint32_t> offsets_to_test = {10000, 10001, 12345,
                                                 15000, 20000, 27531};

  for (const uint32_t N : offsets_to_test) {
    LOG(INFO) << "Testing starting offset N = " << N;
    const monotonic_clock::time_point test_start_monotonic =
        monotonic_clock::now();
    const realtime_clock::time_point test_start_realtime =
        realtime_clock::now();
    // Queue 1: Start at 0, write N + M messages.
    // We align both memory regions to 128-byte boundaries so they have
    // identical absolute alignments modulo 128, ensuring RoundChannelData
    // behaves identically.
    aos::AllocatorResizeableBuffer<aos::AlignedReallocator<128>> raw_memory1;
    raw_memory1.resize(LocklessQueueMemorySize(config_));
    memset(raw_memory1.data(), 0, raw_memory1.size());
    LocklessQueue q1(
        reinterpret_cast<LocklessQueueMemory *>(raw_memory1.data()),
        reinterpret_cast<LocklessQueueMemory *>(raw_memory1.data()), config_);
    q1.Initialize();
    {
      QueueRacerConfiguration config{
          .num_threads = 1,
          .num_messages = N + M,
          .expected_send_results = {LocklessQueueSender::Result::GOOD},
          .channel_storage_duration = std::chrono::nanoseconds(0),
          .check_writes_and_reads = true,
          .initial_queue_index = 0,
      };
      QueueRacer racer(q1, config);
      racer.RunIteration(false, 0, false, true);
    }

    // Queue 2: Start at N, write M messages.
    aos::AllocatorResizeableBuffer<aos::AlignedReallocator<128>> raw_memory2;
    raw_memory2.resize(LocklessQueueMemorySize(config_));
    memset(raw_memory2.data(), 0, raw_memory2.size());
    LocklessQueue q2(
        reinterpret_cast<LocklessQueueMemory *>(raw_memory2.data()),
        reinterpret_cast<LocklessQueueMemory *>(raw_memory2.data()), config_);
    q2.Initialize();
    {
      QueueRacerConfiguration config{
          .num_threads = 1,
          .num_messages = N + M,
          .expected_send_results = {LocklessQueueSender::Result::GOOD},
          .channel_storage_duration = std::chrono::nanoseconds(0),
          .check_writes_and_reads = true,
          .initial_queue_index = N,
      };
      QueueRacer racer(q2, config);
      racer.RunIteration(false, 0, false, true);
    }

    // Validate that all sent timestamps are valid and were written by the run
    // above, rather than being stale or uninitialized.  Since the two queue
    // runs occur at slightly different times, their timestamps differ.  After
    // validating their correctness, we overwrite them to a constant value to
    // allow a byte-for-byte memory comparison.
    //
    // Bound this by when the sending actually finished rather than by a fixed
    // slop around the start.  The two runs above push tens of millions of
    // messages through, and how long that takes is a property of the machine --
    // notably of how expensive a clock read is, since every send takes two.  A
    // fixed window is simultaneously too loose to be a real check on a fast
    // machine and wrong on a slow one.
    const monotonic_clock::time_point test_end_monotonic =
        monotonic_clock::now();
    const realtime_clock::time_point test_end_realtime = realtime_clock::now();
    constexpr auto kSlop = std::chrono::seconds(1);
    auto written_by_this_run = [kSlop](auto time, auto start, auto end) {
      return time >= start - kSlop && time <= end + kSlop;
    };

    for (size_t i = 0; i < q1.memory()->num_messages(); ++i) {
      const Message *const msg1 =
          q1.memory()->GetMessage(Index(QueueIndex::Invalid(), i));
      Message *const msg2 =
          q2.memory()->GetMessage(Index(QueueIndex::Invalid(), i));

      const auto mono1 = msg1->monotonic_sent_time();
      const auto mono2 = msg2->monotonic_sent_time();
      const auto real1 = msg1->realtime_sent_time();
      const auto real2 = msg2->realtime_sent_time();

      // Validate that every timestamp in the queue memory is valid:
      // - If a message slot was written to during the test, its timestamp must
      // be near now.
      // - If a message slot was never written (e.g., unused sender/pinner
      // scratch
      //   spaces, or historical messages initialized prior to the wrap point),
      //   its timestamp must be one of the expected uninitialized/sentinel
      //   values:
      //   - min_time: The sentinel used for uninitialized non-atomic
      //   monotonic/realtime times.
      //   - max_time (kInvalid): The sentinel used for invalid/uninitialized
      //   times
      //     when atomic time points are enabled.
      //
      // The individual validity checks below (mono_is_sentinel ||
      // written_by_this_run) are not redundant with the XOR matching logic;
      // they verify that the values are actually valid sentinels or sent times,
      // preventing cases where both evaluate to a matching arbitrary/garbage
      // value (which would otherwise pass the XOR check).
      //
      // Additionally, confirm that both the old and new memory match for each
      // slot: they must be either both unwritten/sentinel (matching exactly) or
      // both recently written (near now).
      const bool mono_is_sentinel1 =
          (mono1 == monotonic_clock::min_time ||
           mono1 == monotonic_clock::time_point::max());
      const bool mono_is_sentinel2 =
          (mono2 == monotonic_clock::min_time ||
           mono2 == monotonic_clock::time_point::max());

      const bool mono1_written =
          written_by_this_run(mono1, test_start_monotonic, test_end_monotonic);
      const bool mono2_written =
          written_by_this_run(mono2, test_start_monotonic, test_end_monotonic);

      EXPECT_TRUE(mono_is_sentinel1 || mono1_written);
      EXPECT_TRUE(mono_is_sentinel2 || mono2_written);
      EXPECT_TRUE((mono1 == mono2) != (mono1_written && mono2_written));

      const bool real_is_sentinel1 =
          (real1 == realtime_clock::min_time ||
           real1 == realtime_clock::time_point::max());
      const bool real_is_sentinel2 =
          (real2 == realtime_clock::min_time ||
           real2 == realtime_clock::time_point::max());

      const bool real1_written =
          written_by_this_run(real1, test_start_realtime, test_end_realtime);
      const bool real2_written =
          written_by_this_run(real2, test_start_realtime, test_end_realtime);

      EXPECT_TRUE(real_is_sentinel1 || real1_written);
      EXPECT_TRUE(real_is_sentinel2 || real2_written);
      EXPECT_TRUE((real1 == real2) != (real1_written && real2_written));

#ifdef AOS_IPC_LIB_LOCKLESS_QUEUE_HAS_ATOMIC_TIME_POINT
      msg2->header.monotonic_sent_time.Store(msg1->monotonic_sent_time());
      msg2->header.realtime_sent_time.Store(msg1->realtime_sent_time());
#else
      msg2->header.monotonic_sent_time = msg1->monotonic_sent_time();
      msg2->header.realtime_sent_time = msg1->realtime_sent_time();
#endif
    }

    const size_t memory_size = LocklessQueueMemorySize(config_);
    const uint8_t *b1 = reinterpret_cast<const uint8_t *>(q1.memory());
    const uint8_t *b2 = reinterpret_cast<const uint8_t *>(q2.memory());
    int diff = memcmp(q1.memory(), q2.memory(), memory_size);
    if (diff != 0) {
      int diff_count = 0;
      for (size_t i = 0; i < memory_size; ++i) {
        if (b1[i] != b2[i]) {
          LOG(ERROR) << "N = " << N << " DIFF: offset " << i << ": mem1 = 0x"
                     << std::hex << static_cast<int>(b1[i]) << ", mem2 = 0x"
                     << static_cast<int>(b2[i]);
          if (++diff_count >= 100) {
            LOG(ERROR) << "N = " << N
                       << " DIFF: truncated after 100 differences";
            break;
          }
        }
      }
    }
    ASSERT_EQ(diff, 0);
  }
}

// Send enough messages to wrap the 32 bit send counter.
TEST_F(LocklessQueueTest, WrappedSend) {
  PinForTest pin_cpu;
  const uint32_t queue_size = queue().config().queue_size;
  const uint32_t max_index = QueueIndex::MaxIndex(0xffffffffu, queue_size);
  // Start 5000000 messages before wrapping.
  const uint32_t start_index = max_index - 5000000;
  // Send 10000000 messages total, wrapping across 0.
  const uint64_t kNumMessages = 10000000;

#ifdef AOS_IPC_LIB_LOCKLESS_QUEUE_HAS_ATOMIC_TIME_POINT
  const std::vector<LocklessQueueSender::Result> expected_results = {
      LocklessQueueSender::Result::GOOD};
#else
  const std::vector<LocklessQueueSender::Result> expected_results = {
      LocklessQueueSender::Result::GOOD,
      LocklessQueueSender::Result::MESSAGES_SENT_TOO_FAST};
#endif

  QueueRacerConfiguration config{
      .num_threads = 1,
      .num_messages = start_index + kNumMessages,
      .expected_send_results = expected_results,
      .channel_storage_duration = std::chrono::nanoseconds(0),
      .check_writes_and_reads = true,
      .initial_queue_index = start_index,
  };
  QueueRacer racer(queue(), config);

  // Ensure that the number of messages is indeed large enough to force a wrap
  // across max_index.
  ASSERT_GT(kNumMessages, max_index - start_index);

  const monotonic_clock::time_point start_time = monotonic_clock::now();
  EXPECT_NO_FATAL_FAILURE(racer.RunIteration(false, 0, false, true));

  // Validate that the final queue index matches the expected wrapped index,
  // confirming that we did indeed wrap around to a lower value.
  const uint32_t expected_final_index =
      (start_index + kNumMessages - 1) % max_index;
  EXPECT_EQ(racer.CurrentIndex(), expected_final_index);
  EXPECT_LT(expected_final_index, start_index);
  const monotonic_clock::time_point monotonic_now = monotonic_clock::now();
  double elapsed_seconds = chrono::duration_cast<chrono::duration<double>>(
                               monotonic_now - start_time)
                               .count();
  LOG(INFO) << "Took " << elapsed_seconds << " seconds to write "
            << kNumMessages << " messages, "
            << (static_cast<double>(kNumMessages) / elapsed_seconds)
            << " messages/s";
}

}  // namespace aos::ipc_lib::testing
