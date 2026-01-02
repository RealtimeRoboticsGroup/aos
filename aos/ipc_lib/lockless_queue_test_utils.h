#ifndef AOS_IPC_LIB_LOCKLESS_QUEUE_TEST_UTILS_H_
#define AOS_IPC_LIB_LOCKLESS_QUEUE_TEST_UTILS_H_

#include "gtest/gtest.h"

#include "aos/events/epoll.h"
#include "aos/ipc_lib/event.h"
#include "aos/ipc_lib/lockless_queue.h"
#include "aos/realtime.h"

namespace aos::ipc_lib::testing {
class LocklessQueueTest : public ::testing::Test {
 public:
  static constexpr monotonic_clock::duration kChannelStorageDuration =
      std::chrono::milliseconds(500);

  // Temporarily pins the current thread to the first 2 available CPUs.
  // This speeds up the test on some machines a lot (~4x). It also preserves
  // opportunities for the 2 threads to race each other.
  class PinForTest {
   public:
    PinForTest();
    ~PinForTest() { SetCurrentThreadAffinity(old_); }

   private:
    CpuSet old_;
  };

  LocklessQueueTest();

  LocklessQueue queue() {
    return LocklessQueue(reinterpret_cast<LocklessQueueMemory *>(&(memory_[0])),
                         reinterpret_cast<LocklessQueueMemory *>(&(memory_[0])),
                         config_);
  }

  void Reset() { memset(&memory_[0], 0, LocklessQueueMemorySize(config_)); }

  // Runs until the signal is received.
  void RunUntilWakeup(Event *ready, int priority);

  // Use a type with enough alignment that we are guarenteed that everything
  // will be aligned properly on the target platform.
  ::std::vector<uint64_t> memory_;

  LocklessQueueConfiguration config_;
};
}  // namespace aos::ipc_lib::testing
#endif  // AOS_IPC_LIB_LOCKLESS_QUEUE_TEST_UTILS_H_
