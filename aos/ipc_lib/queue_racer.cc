#include "aos/ipc_lib/queue_racer.h"

#include <stdio.h>

#include <algorithm>
#include <cinttypes>
#include <cstring>
#include <limits>
#include <optional>
#include <ostream>
#include <thread>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/types/span.h"
#include "gtest/gtest.h"

#include "aos/events/context.h"
#include "aos/ipc_lib/event.h"
#include "aos/ipc_lib/lockless_queue_memory.h"

namespace aos::ipc_lib {
namespace {

struct ThreadPlusCount {
  uint64_t thread;
  uint64_t count;
};

}  // namespace

struct ThreadState {
  ::std::thread thread;
  Event ready;
  uint64_t event_count = ::std::numeric_limits<uint64_t>::max();
};

QueueRacer::QueueRacer(LocklessQueue queue, int num_threads,
                       uint64_t num_messages)
    : queue_(queue),
      num_threads_(num_threads),
      num_messages_(num_messages),
      channel_storage_duration_(std::chrono::nanoseconds(1)),
      expected_send_results_({LocklessQueueSender::Result::GOOD}),
      check_writes_and_reads_(true),
      initial_queue_index_(0) {
  CHECK_LT(1u, std::thread::hardware_concurrency())
      << "Queue racing must be done on a multi-core executor.";
  Reset();
}

QueueRacer::QueueRacer(LocklessQueue queue,
                       const QueueRacerConfiguration &config)
    : queue_(queue),
      num_threads_(config.num_threads),
      num_messages_(config.num_messages),
      channel_storage_duration_(config.channel_storage_duration),
      expected_send_results_(config.expected_send_results),
      check_writes_and_reads_(config.check_writes_and_reads),
      initial_queue_index_(config.initial_queue_index) {
  CHECK_LT(1u, std::thread::hardware_concurrency())
      << "Queue racing must be done on a multi-core executor.";
  Reset();
}

void QueueRacer::Reset() {
  memset(reinterpret_cast<void *>(queue_.memory()), 0,
         LocklessQueueMemorySize(queue_.config()));
  queue_.Initialize();
  if (initial_queue_index_ != 0) {
    LocklessQueueMemory *const memory = queue_.memory();
    const uint32_t queue_size = memory->queue_size();
    const uint32_t max_index = QueueIndex::MaxIndex(0xffffffffu, queue_size);
    const uint64_t N = initial_queue_index_;
    const size_t msg_data_size = memory->message_data_size();

    // 1. Initialize queue slots.
    for (uint32_t j = 0; j < queue_size; ++j) {
      uint64_t qi;
      uint32_t msg_idx;
      if (N == 0 || N - 1 < j) {
        qi = j - queue_size;
        msg_idx = j;
      } else {
        uint64_t w = j + ((N - 1 - j) / queue_size) * queue_size;
        qi = w;
        msg_idx = (w + queue_size) % (queue_size + 1);
      }
      QueueIndex qi_wrapped =
          QueueIndex::Zero(queue_size).IncrementBy(qi % max_index);
      memory->GetQueue(j)->Store(Index(qi_wrapped, msg_idx));
    }

    // 2. Initialize sender scratch buffers.
    const int num_senders = memory->num_senders();
    for (int s = 0; s < num_senders; ++s) {
      uint32_t msg_idx;
      if (s == 0) {
        msg_idx = (N == 0) ? queue_size : (N - 1) % (queue_size + 1);
      } else {
        msg_idx = s + queue_size;
      }
      memory->GetSender(s)->scratch_index.RelaxedStore(
          Index(QueueIndex::Invalid(), msg_idx));
    }

    // 3. Initialize all message pool slots to exact historical state.
    const size_t num_messages = memory->num_messages();
    const uint32_t active_sender_scratch =
        (N == 0) ? queue_size : (N - 1) % (queue_size + 1);
    for (size_t m = 0; m < num_messages; ++m) {
      Message *msg = memory->GetMessage(Index(QueueIndex::Invalid(), m));
#ifdef AOS_IPC_LIB_LOCKLESS_QUEUE_HAS_ATOMIC_TIME_POINT
      msg->header.monotonic_sent_time.Invalidate();
      msg->header.realtime_sent_time.Invalidate();
#else
      msg->header.monotonic_sent_time = monotonic_clock::min_time;
      msg->header.realtime_sent_time = realtime_clock::min_time;
#endif
      if (m == active_sender_scratch) {
        msg->header.queue_index.Invalidate();
        msg->header.length = 0;
        msg->header.remote_queue_index = 0x00000000;
        msg->header.source_boot_uuid = UUID::Zero();
        memset(msg->data(msg_data_size), 0, msg_data_size);
      } else if (m <= queue_size) {
        uint64_t start = (m + 1) % (queue_size + 1);
        if (N > 0 && N - 1 >= start) {
          uint64_t w =
              start + ((N - 1 - start) / (queue_size + 1)) * (queue_size + 1);

          struct ThreadPlusCount {
            uint64_t thread;
            uint64_t count;
          };
          ThreadPlusCount tpc{0, w};

          msg->header.length = sizeof(ThreadPlusCount);
          msg->header.remote_queue_index = 0x00000000;
          msg->header.source_boot_uuid = UUID::FromSpan(
              absl::Span<const uint8_t>(reinterpret_cast<const uint8_t *>(&tpc),
                                        sizeof(ThreadPlusCount)));
          msg->header.queue_index.Store(
              QueueIndex::Zero(queue_size).IncrementBy(w % max_index));

#ifdef AOS_IPC_LIB_LOCKLESS_QUEUE_HAS_ATOMIC_TIME_POINT
          msg->header.monotonic_sent_time.Store(monotonic_clock::epoch());
          msg->header.realtime_sent_time.Store(realtime_clock::epoch());
#else
          msg->header.monotonic_sent_time = monotonic_clock::epoch();
          msg->header.realtime_sent_time = realtime_clock::epoch();
#endif

          msg->header.monotonic_remote_time = monotonic_clock::min_time;
          msg->header.monotonic_remote_transmit_time =
              monotonic_clock::min_time;
          msg->header.realtime_remote_time = realtime_clock::min_time;

          char *const data = msg->data(msg_data_size) + msg_data_size -
                             sizeof(ThreadPlusCount);
          memcpy(data, &tpc, sizeof(ThreadPlusCount));
        } else {
          msg->header.queue_index.Invalidate();
          msg->header.length = 0;
          msg->header.remote_queue_index = 0x00000000;
          msg->header.source_boot_uuid = UUID::Zero();
          memset(msg->data(msg_data_size), 0, msg_data_size);
        }
      } else {
        msg->header.queue_index.Invalidate();
        msg->header.length = 0;
        msg->header.remote_queue_index = 0x00000000;
        msg->header.source_boot_uuid = UUID::Zero();
        memset(msg->data(msg_data_size), 0, msg_data_size);
      }
    }

    // 4. Initialize next_queue_index.
    QueueIndex next_qi =
        QueueIndex::Zero(queue_size).IncrementBy(N % max_index);
    memory->next_queue_index.Store(next_qi);
  }
}

void QueueRacer::RunIteration(bool race_reads, int write_wrap_count,
                              bool set_should_read, bool should_read_result) {
  const bool will_wrap = num_messages_ * num_threads_ *
                             static_cast<uint64_t>(1 + write_wrap_count) >
                         queue_.config().queue_size;

  // Clear out shmem.
  Reset();
  started_writes_ = initial_queue_index_ * num_threads_;
  finished_writes_ = initial_queue_index_ * num_threads_;

  // Event used to start all the threads processing at once.
  Event run;

  ::std::atomic<bool> poll_index{true};

  // List of threads.
  ::std::vector<ThreadState> threads(num_threads_);

  ::std::thread queue_index_racer([this, &poll_index]() {
    LocklessQueueReader reader(queue_);

    // Track the number of times we wrap, and cache the modulo.
    uint64_t wrap_count = 0;
    // The reader thread tracks the latest queue index and detects wrap-around.
    // - If the queue starts empty (initial_queue_index_ == 0), the first read
    //   index will be 0. We initialize last_queue_index to 0 so that reading 0
    //   does not trigger wrap detection (since 0 < 0 is false).
    // - If the queue starts pre-populated (initial_queue_index_ > 0), the last
    //   message already in the queue is at initial_queue_index_ - 1. The reader
    //   will see this value on its first read, so we initialize
    //   last_queue_index to initial_queue_index_ - 1 to prevent that first read
    //   from triggering wrap detection.
    uint32_t last_queue_index =
        initial_queue_index_ > 0 ? initial_queue_index_ - 1 : 0;
    const uint32_t max_queue_index =
        QueueIndex::MaxIndex(0xffffffffu, queue_.config().queue_size);
    while (poll_index) {
      // We want to read everything backwards.  This will give us conservative
      // bounds.  And with enough time and randomness, we will see all the cases
      // we care to see.

      // These 3 numbers look at the same thing, but at different points of time
      // in the process.  The process (essentially) looks like:
      //
      // ++started_writes;
      // ++latest_queue_index;
      // ++finished_writes;
      //
      // We want to check that latest_queue_index is bounded by the number of
      // writes started and finished.  Basically, we can say that
      // finished_writes < latest_queue_index always.  And
      // latest_queue_index < started_writes.  And everything always increases.
      // So, if we let more time elapse between sampling finished_writes and
      // latest_queue_index, we will only be relaxing our bounds, not
      // invalidating the check.  The same goes for started_writes.
      //
      // So, grab them in order.
      const uint64_t finished_writes = finished_writes_.load();
      const QueueIndex latest_queue_index_queue_index = reader.LatestIndex();
      const uint64_t started_writes = started_writes_.load();

      const uint32_t latest_queue_index_uint32_t =
          latest_queue_index_queue_index.index();
      uint64_t latest_queue_index = latest_queue_index_uint32_t;

      if (latest_queue_index_queue_index != QueueIndex::Invalid()) {
        // If we got smaller, we wrapped.
        if (latest_queue_index_uint32_t < last_queue_index) {
          ++wrap_count;
        }
        // And apply it.
        latest_queue_index +=
            static_cast<uint64_t>(max_queue_index) * wrap_count;
        last_queue_index = latest_queue_index_uint32_t;
      }

      // For grins, check that we have always started more than we finished.
      // Should never fail.
      EXPECT_GE(started_writes, finished_writes);

      // If we are at the beginning, the queue needs to always return empty.
      if (started_writes == 0) {
        EXPECT_EQ(latest_queue_index_queue_index, QueueIndex::Invalid());
        EXPECT_EQ(finished_writes, 0);
      } else {
        if (finished_writes == 0) {
          // Plausible to be at the beginning, in which case we don't have
          // anything to check.
          if (latest_queue_index_queue_index != QueueIndex::Invalid()) {
            // Otherwise, we have started.  The queue can't have any more
            // entries than this.
            EXPECT_GE(started_writes, latest_queue_index + 1);
          }
        } else {
          EXPECT_NE(latest_queue_index_queue_index, QueueIndex::Invalid());
          // latest_queue_index is an index, not a count.  So it always reads 1
          // low.
          if (check_writes_and_reads_) {
            EXPECT_GE(latest_queue_index + 1, finished_writes);
          }
        }
      }
    }
  });

  // Build up each thread and kick it off.
  int thread_index = 0;
  for (ThreadState &t : threads) {
    if (will_wrap) {
      t.event_count = ::std::numeric_limits<uint64_t>::max();
    } else {
      t.event_count = initial_queue_index_;
    }
    t.thread = ::std::thread([this, &t, thread_index, &run,
                              write_wrap_count]() {
      LocklessQueueSender sender =
          LocklessQueueSender::Make(queue_, channel_storage_duration_).value();
      CHECK_GE(sender.size(), sizeof(ThreadPlusCount));

      // Signal that we are ready to start sending.
      t.ready.Set();

      // Wait until signaled to start running.
      run.Wait();

      // Gogogo!
      const uint64_t start_i = initial_queue_index_;
      const uint64_t end_i =
          start_i + (num_messages_ - start_i) * (1 + write_wrap_count);
      for (uint64_t i = start_i; i < end_i; ++i) {
        char *const data = static_cast<char *>(sender.Data()) + sender.size() -
                           sizeof(ThreadPlusCount);
        const char fill = (i + 55) & 0xFF;
        memset(data, fill, sizeof(ThreadPlusCount));
        {
          bool found_nonzero = false;
          for (size_t i = 0; i < sizeof(ThreadPlusCount); ++i) {
            if (data[i] != fill) {
              found_nonzero = true;
            }
          }
          CHECK(!found_nonzero) << ": Somebody else is writing to our buffer";
        }

        ThreadPlusCount tpc;
        tpc.thread = thread_index;
        tpc.count = i;

        memcpy(data, &tpc, sizeof(ThreadPlusCount));

        if ((i - start_i) % 0x800000 == 0x100000) {
          VLOG(1) << "Sent " << (i - start_i) << ", "
                  << (static_cast<double>(i - start_i) /
                      static_cast<double>((num_messages_ - start_i) *
                                          (1 + write_wrap_count)) *
                      100.0)
                  << " %";
        }

        ++started_writes_;
        auto result =
            sender.Send(sizeof(ThreadPlusCount), aos::monotonic_clock::min_time,
                        aos::realtime_clock::min_time,
                        aos::monotonic_clock::min_time, 0xffffffff,
                        UUID::FromSpan(absl::Span<const uint8_t>(
                            reinterpret_cast<const uint8_t *>(&tpc),
                            sizeof(ThreadPlusCount))),
                        nullptr, nullptr, nullptr);

        CHECK(std::find(expected_send_results_.begin(),
                        expected_send_results_.end(),
                        result) != expected_send_results_.end())
            << "Unexpected send result: " << result;

        // Blank out the new scratch buffer, to catch other people using it.
        {
          char *const new_data = static_cast<char *>(sender.Data()) +
                                 sender.size() - sizeof(ThreadPlusCount);
          const char new_fill = ~fill;
          memset(new_data, new_fill, sizeof(ThreadPlusCount));
        }
        ++finished_writes_;
      }
    });
    ++thread_index;
  }

  // Wait until all the threads are ready.
  for (ThreadState &t : threads) {
    t.ready.Wait();
  }

  // And start them racing.
  run.Set();

  // Let all the threads finish before reading if we are supposed to not be
  // racing reads.
  if (!race_reads) {
    for (ThreadState &t : threads) {
      t.thread.join();
    }
    poll_index = false;
    queue_index_racer.join();
  }

  if (check_writes_and_reads_) {
    CheckReads(race_reads, write_wrap_count, &threads, set_should_read,
               should_read_result);
  }

  // Reap all the threads.
  if (race_reads) {
    for (ThreadState &t : threads) {
      t.thread.join();
    }
    poll_index = false;
    queue_index_racer.join();
  }

  if (check_writes_and_reads_) {
    // Confirm that the number of writes matches the expected number of writes.
    ASSERT_EQ(num_threads_ * num_messages_ * (1 + write_wrap_count),
              started_writes_);
    ASSERT_EQ(num_threads_ * num_messages_ * (1 + write_wrap_count),
              finished_writes_);

    // And that every thread sent the right number of messages.
    for (ThreadState &t : threads) {
      if (will_wrap) {
        if (!race_reads) {
          // If we are wrapping, there is a possibility that a thread writes
          // everything *before* we can read any of it, and it all gets
          // overwritten.
          ASSERT_TRUE(t.event_count == ::std::numeric_limits<uint64_t>::max() ||
                      t.event_count == (1 + write_wrap_count) * num_messages_)
              << ": Got " << t.event_count << " events, expected "
              << (1 + write_wrap_count) * num_messages_;
        }
      } else {
        ASSERT_EQ(t.event_count, num_messages_);
      }
    }
  }
}

void QueueRacer::CheckReads(bool race_reads, int write_wrap_count,
                            ::std::vector<ThreadState> *threads,
                            bool set_should_read, bool should_read_result) {
  // Now read back the results to double check.
  LocklessQueueReader reader(queue_);
  const bool will_wrap = num_messages_ * num_threads_ * (1 + write_wrap_count) >
                         LocklessQueueSize(queue_.memory());

  monotonic_clock::time_point last_monotonic_sent_time =
      monotonic_clock::epoch();
  uint64_t initial_i = initial_queue_index_;
  if (will_wrap) {
    initial_i = initial_queue_index_ +
                (1 + write_wrap_count) *
                    (num_messages_ - initial_queue_index_) * num_threads_ -
                LocklessQueueSize(queue_.memory());
  }

  std::function<bool(const Context &)> nop;

  Context fetched_context;
  std::function<bool(const Context &)> should_read =
      [&should_read_result, &fetched_context](const Context &context) {
        fetched_context = context;
        return should_read_result;
      };

  for (uint64_t i = initial_i;
       i < initial_queue_index_ + (1 + write_wrap_count) *
                                      (num_messages_ - initial_queue_index_) *
                                      num_threads_;
       ++i) {
    monotonic_clock::time_point monotonic_sent_time;
    realtime_clock::time_point realtime_sent_time;
    monotonic_clock::time_point monotonic_remote_time;
    monotonic_clock::time_point monotonic_remote_transmit_time;
    realtime_clock::time_point realtime_remote_time;
    UUID source_boot_uuid;
    uint32_t remote_queue_index;
    size_t length;
    char read_data[1024];

    // Handle overflowing the message count for the wrap test.
    const uint32_t wrapped_i =
        i % static_cast<size_t>(QueueIndex::MaxIndex(
                0xffffffffu, LocklessQueueSize(queue_.memory())));
    LocklessQueueReader::Result read_result =
        set_should_read
            ? reader.Read(
                  wrapped_i, &monotonic_sent_time, &realtime_sent_time,
                  &monotonic_remote_time, &monotonic_remote_transmit_time,
                  &realtime_remote_time, &remote_queue_index, &source_boot_uuid,
                  &length, &(read_data[0]), std::ref(should_read))
            : reader.Read(wrapped_i, &monotonic_sent_time, &realtime_sent_time,
                          &monotonic_remote_time,
                          &monotonic_remote_transmit_time,
                          &realtime_remote_time, &remote_queue_index,
                          &source_boot_uuid, &length, &(read_data[0]), nop);

    // The code in lockless_queue.cc reads everything but data, checks that the
    // header hasn't changed, then reads the data.  So, if we succeed and both
    // end up not being corrupted, then we've confirmed everything works.
    //
    // Feed in both combos of should_read and whether or not to return true or
    // false from should_read.  By capturing the header values inside the
    // callback, we can also verify the state in the middle of the process to
    // make sure we have the right boundaries.
    if (race_reads) {
      if (read_result == LocklessQueueReader::Result::NOTHING_NEW) {
        --i;
        continue;
      }
    }

    if (race_reads && will_wrap) {
      if (read_result == LocklessQueueReader::Result::TOO_OLD) {
        continue;
      }
    }

    if (!set_should_read) {
      // Every message should be good.
      ASSERT_EQ(read_result, LocklessQueueReader::Result::GOOD)
          << ": i is " << i;
    } else {
      if (should_read_result) {
        ASSERT_EQ(read_result, LocklessQueueReader::Result::GOOD)
            << ": i is " << i;

        ASSERT_EQ(monotonic_sent_time, fetched_context.monotonic_event_time);
        ASSERT_EQ(realtime_sent_time, fetched_context.realtime_event_time);
        ASSERT_EQ(monotonic_remote_time, fetched_context.monotonic_remote_time);
        ASSERT_EQ(realtime_remote_time, fetched_context.realtime_remote_time);
        ASSERT_EQ(source_boot_uuid, fetched_context.source_boot_uuid);
        ASSERT_EQ(remote_queue_index, fetched_context.remote_queue_index);
        ASSERT_EQ(length, fetched_context.size);

        ASSERT_EQ(
            absl::Span<const uint8_t>(
                reinterpret_cast<const uint8_t *>(
                    read_data + LocklessQueueMessageDataSize(queue_.memory()) -
                    length),
                length),
            source_boot_uuid.span());
      } else {
        ASSERT_EQ(read_result, LocklessQueueReader::Result::FILTERED);
        monotonic_sent_time = fetched_context.monotonic_event_time;
        realtime_sent_time = fetched_context.realtime_event_time;
        monotonic_remote_time = fetched_context.monotonic_remote_time;
        realtime_remote_time = fetched_context.realtime_remote_time;
        source_boot_uuid = fetched_context.source_boot_uuid;
        remote_queue_index = fetched_context.remote_queue_index;
        length = fetched_context.size;
      }
    }

    // And, confirm that time never went backwards.
    ASSERT_GT(monotonic_sent_time, last_monotonic_sent_time);
    last_monotonic_sent_time = monotonic_sent_time;

    ASSERT_EQ(monotonic_remote_time, aos::monotonic_clock::min_time);
    ASSERT_EQ(realtime_remote_time, aos::realtime_clock::min_time);

    ThreadPlusCount tpc;
    ASSERT_EQ(source_boot_uuid.span().size(), sizeof(ThreadPlusCount));
    memcpy(&tpc, source_boot_uuid.span().data(),
           source_boot_uuid.span().size());

    if (will_wrap) {
      // The queue won't chang out from under us, so we should get some amount
      // of the tail end of the messages from a a thread.
      // Confirm that once we get our first message, they all show up.
      if ((*threads)[tpc.thread].event_count ==
          ::std::numeric_limits<uint64_t>::max()) {
        (*threads)[tpc.thread].event_count = tpc.count;
      }

      if (race_reads) {
        // Make sure nothing goes backwards.  Really not much we can do here.
        ASSERT_LE((*threads)[tpc.thread].event_count, tpc.count)
            << ": Thread " << tpc.thread;
        (*threads)[tpc.thread].event_count = tpc.count;
      } else {
        // Make sure nothing goes backwards.  Really not much we can do here.
        ASSERT_EQ((*threads)[tpc.thread].event_count, tpc.count)
            << ": Thread " << tpc.thread;
      }
    } else {
      // Confirm that we see every message counter from every thread.
      ASSERT_EQ((*threads)[tpc.thread].event_count, tpc.count)
          << ": Thread " << tpc.thread;
    }
    ++(*threads)[tpc.thread].event_count;
  }
}

}  // namespace aos::ipc_lib
