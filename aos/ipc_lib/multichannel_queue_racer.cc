#include "aos/ipc_lib/multichannel_queue_racer.h"

#include <condition_variable>
#include <shared_mutex>
#include <thread>

#include "absl/log/absl_check.h"
#include "absl/log/log.h"

#include "aos/containers/ring_buffer.h"
#include "aos/ipc_lib/event.h"

namespace aos::ipc_lib {

MultiChannelQueueRacer::MultiChannelQueueRacer(int num_threads,
                                               uint64_t num_messages)
    : num_threads_(num_threads),
      num_messages_(num_messages),
      config_{.num_watchers = 10,
              .num_senders = 10,
              .num_pinners = 10,
              .queue_size = 10000,
              .message_data_size = 128} {
  ABSL_CHECK_LT(1u, std::thread::hardware_concurrency())
      << "Queue racing must be done on a multi-core executor.";
  for (size_t index = 0; index < num_threads_; ++index) {
    queues_.emplace_back(config_);
  }
}

void MultiChannelQueueRacer::Run() {
  aos::Event event;
  std::atomic<bool> senders_done{false};
  std::vector<std::thread> threads;
  for (size_t thread_index = 0; thread_index < num_threads_; ++thread_index) {
    aos::Event thread_start_event;
    threads.emplace_back([this, thread_index, &event, &thread_start_event]() {
      LocklessQueueSender sender =
          LocklessQueueSender::Make(queues_[thread_index].queue,
                                    channel_storage_duration_)
              .value();
      const UUID boot_uuid = UUID::Zero();
      VLOG(1) << "sender " << thread_index << " is ready!";

      thread_start_event.Set();
      event.Wait();
      VLOG(1) << "sender " << thread_index << " is running!";
      for (size_t message_index = 0; message_index < num_messages_;
           ++message_index) {
        if (message_index % 100 == 0) {
          VLOG(2) << "Sending " << message_index << " on " << thread_index;
        }
        ABSL_CHECK(LocklessQueueSender::Result::GOOD ==
                   sender.Send(0, aos::monotonic_clock::min_time,
                               aos::realtime_clock::min_time,
                               aos::monotonic_clock::min_time, 0xffffffff,
                               boot_uuid, nullptr, nullptr, nullptr));
      }
    });
    thread_start_event.Wait();
  }
  uint64_t good_reads = 0;
  std::thread queue_readers([this, &event, &senders_done, &good_reads]() {
    struct ReaderState {
      LocklessQueueReader reader;
      QueueIndex last_queue_index = QueueIndex::Invalid();
      aos::RingBuffer<monotonic_clock::time_point, 2> recent_send_times = {};
    };
    std::vector<ReaderState> readers;
    for (size_t thread_index = 0; thread_index < num_threads_; ++thread_index) {
      readers.emplace_back(LocklessQueueReader(queues_[thread_index].queue));
      // Fill up the reader buffer for convenience.
      while (!readers.back().recent_send_times.full()) {
        readers.back().recent_send_times.Push(monotonic_clock::min_time);
      }
    }
    std::function<bool(const Context &)> should_read = [](const Context &) {
      return true;
    };
    VLOG(1) << "queue readers are ready!";
    // We are ready to go!
    // Don't notify the receivers until they are waiting on us.

    VLOG(1) << "Running!";
    event.Set();

    // Algorithm for detecting races:
    // Round-robin over all of the readers, fetching the latest message each
    // time.
    // Visual depiction of going through the round robin, reading the most
    // recent sent time each time.
    //
    // Iteration Count | Reader 1 | Reader 2 | ... | Reader X | ... |
    // --------------------------------------------------------------
    // 0               | 0.1 sec  | 0.2 sec  | ... | 0.0 sec  | ... |
    // 1               | 0.11 sec | 0.21 sec | ... | 0.32 sec | ... |
    // ...
    // K               | 1.0 sec  | 0.21 sec | ... | 0.8 sec  | ... |
    // ...
    //
    // Note that while we read the readers sequentially, they will be fetching
    // the *most recent* send time on their channel. As such, while send times
    // will never go down on a given queue, as we iterate from reader X-1 to
    // reader X we may observe a message that was sent earlier.
    //
    // The invariant that we can establish with this pattern is that, for some
    // new send time observed at iteration K on reader X, all send times that
    // were observed *prior* to reading reader X on iteration K - 1 must be
    // older than the (K, X) send time. This only applies when we observed a
    // new message on channel X between (K-1, X) and (K, X).

    while (!senders_done.load()) {
      for (size_t thread_index = 0; thread_index < num_threads_;
           ++thread_index) {
        ReaderState &reader = readers[thread_index];
        // Copy the most recent receive_time for if there is no new message.
        monotonic_clock::time_point receive_time =
            reader.recent_send_times[reader.recent_send_times.size() - 1];
        if (reader.last_queue_index != reader.reader.LatestIndex()) {
          reader.last_queue_index = reader.reader.LatestIndex();
          realtime_clock::time_point realtime_sent_time;
          monotonic_clock::time_point monotonic_remote_time;
          monotonic_clock::time_point monotonic_remote_transmit_time;
          realtime_clock::time_point realtime_remote_time;
          uint32_t remote_queue_index;
          UUID source_boot_uuid;
          size_t length;
          // There is a new message available; grab the timestamp and check that
          // it is newer than the oldest timestamps on every channel.
          const LocklessQueueReader::Result read_result = reader.reader.Read(
              reader.last_queue_index.index(), &receive_time,
              &realtime_sent_time, &monotonic_remote_time,
              &monotonic_remote_transmit_time, &realtime_remote_time,
              &remote_queue_index, &source_boot_uuid, &length, nullptr,
              should_read);
          if (LocklessQueueReader::Result::GOOD == read_result) {
            ++good_reads;
            // Now check that this is actually newer than everything in the
            // oldest slot of the buffer. Technically this also checks against
            // ourselves, but that invariant should also hold and there should
            // be minimal performance penalty to checking against ourselves as
            // well.
            for (const auto &other : readers) {
              ABSL_CHECK_LT(other.recent_send_times[0], receive_time)
                  << "thread " << thread_index << " queue "
                  << reader.last_queue_index.index();
            }
          }
        }
        reader.recent_send_times.Push(receive_time);
      }
    }
  });
  VLOG(1) << "Set up threads; waiting to finish!";
  for (std::thread &thread : threads) {
    thread.join();
  }
  senders_done.store(true);
  VLOG(1) << "Done sending data!";
  queue_readers.join();
#if defined(AOS_IPC_LIB_TEST_CAN_RELIABLY_TRIGGER_RACES)
  // Check that we actually received a non-trivial number of messages.
  ABSL_CHECK_LT(num_messages_ / 100 + 1, good_reads);
#endif
}
}  // namespace aos::ipc_lib
