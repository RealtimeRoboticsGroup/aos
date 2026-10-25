#ifndef AOS_IPC_LIB_MULTICHANNEL_QUEUE_RACER_H_
#define AOS_IPC_LIB_MULTICHANNEL_QUEUE_RACER_H_
#include <stdint.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <functional>
#include <vector>

#include "aos/ipc_lib/index.h"
#include "aos/ipc_lib/lockless_queue.h"
#include "aos/time/time.h"
#include "aos/uuid.h"

namespace aos::ipc_lib {

struct ThreadState;

// Class to test the queue by spinning up a bunch of writing threads and racing
// them together to all write at once.
// We then try to read messages across all of the channels in a single thread
// and validate that we observe everything in a consistent order.
class MultiChannelQueueRacer {
 public:
  MultiChannelQueueRacer(int num_threads, uint64_t num_messages);

  void Run();

 private:
  // TODO:
  // * Initialize a bunch of these for each queue that we want to run on.
  // * Go through and run a sender on each thread for each queue, with a single
  // thread
  //   reading all of the queues and validating that they arrive in order (TODO:
  //   figure out how to test exactly "arrive in order").
  // * TOOD: how to validate that we get things right when we read a late
  // message before the earlier senders have finished running.
  struct LocalMemoryQueue {
    LocalMemoryQueue(const LocklessQueueConfiguration &config)
        : memory((LocklessQueueMemorySize(config) + sizeof(uint64_t) - 1) /
                     sizeof(uint64_t),
                 0),
          queue(reinterpret_cast<LocklessQueueMemory *>(memory.data()),
                reinterpret_cast<LocklessQueueMemory *>(memory.data()),
                config) {}
    std::vector<uint64_t> memory;
    LocklessQueue queue;
  };

  const uint64_t num_threads_;
  const uint64_t num_messages_;
  const monotonic_clock::duration channel_storage_duration_ =
      std::chrono::nanoseconds(1);
  const LocklessQueueConfiguration config_;
  std::vector<LocalMemoryQueue> queues_;
  // Number of writes about to be started.
  ::std::atomic<uint64_t> started_writes_;
  // Number of writes completed.
  ::std::atomic<uint64_t> finished_writes_;
};

}  // namespace aos::ipc_lib
#endif  // AOS_IPC_LIB_MULTICHANNEL_QUEUE_RACER_H_
