#include "aos/ipc_lib/process_local_queue.h"

#ifndef _WIN32
#include <sys/wait.h>
#include <unistd.h>
#endif

#include <atomic>
#include <chrono>
#include <string>
#include <thread>

#include "gtest/gtest.h"

#include "aos/configuration.h"
#include "aos/ipc_lib/lockless_queue.h"
#include "aos/json_to_flatbuffer.h"

namespace aos::ipc_lib::testing {
namespace {

class ProcessLocalQueueTest : public ::testing::Test {
 protected:
  ProcessLocalQueueTest()
      : config_(configuration::MergeConfiguration(
            FlatbufferDetachedBuffer<Configuration>(
                JsonToFlatbuffer<Configuration>(R"({
  "channels": [
    { "name": "/test", "type": "aos.TestMessage", "frequency": 10,
      "max_size": 64 },
    { "name": "/other", "type": "aos.TestMessage", "frequency": 10,
      "max_size": 64 },
    { "name": "/big", "type": "aos.TestMessage", "frequency": 500,
      "max_size": 16384 }
  ]
})")))) {
    // Every test here starts from no queues, the way a shared-memory test
    // starts from an empty --shm_base.
    ProcessLocalQueue::ResetAll();
  }

  const Channel *channel(const char *name) const {
    return configuration::GetChannel(&config_.message(), name,
                                     "aos.TestMessage", "", nullptr);
  }

  // Puts one message in the queue behind `channel`.
  void SendOn(const Channel *channel) {
    ProcessLocalQueue queue(&config_.message(), channel);
    std::optional<LocklessQueueSender> sender =
        LocklessQueueSender::Make(queue.queue(), std::chrono::seconds(2));
    ASSERT_TRUE(sender.has_value());
    const char data[] = "hello";
    monotonic_clock::time_point monotonic_sent_time;
    realtime_clock::time_point realtime_sent_time;
    uint32_t queue_index;
    ASSERT_EQ(sender->Send(data, sizeof(data), monotonic_clock::min_time,
                           realtime_clock::min_time, monotonic_clock::min_time,
                           0xffffffffu, UUID::Zero(), &monotonic_sent_time,
                           &realtime_sent_time, &queue_index),
              LocklessQueueSender::Result::GOOD);
  }

  // Whether the queue behind `channel` holds anything.
  bool HasMessage(const Channel *channel) {
    ProcessLocalQueue queue(&config_.message(), channel);
    return LocklessQueueReader(queue.queue()).LatestIndex().valid();
  }

  FlatbufferDetachedBuffer<Configuration> config_;
};

using ProcessLocalQueueDeathTest = ProcessLocalQueueTest;

#ifndef _WIN32
// A fork() child shares these queues, so it may make one, even when another
// thread was in the middle of making or destroying one as it forked.  That
// thread held the registry's lock, which the child used to find held for good.
TEST_F(ProcessLocalQueueTest, ForkWhileAnotherThreadMakesAQueue) {
  // aos_sync's once-per-process setup, before the maker thread can be the one
  // to run it.  A fork in the middle of that call_once leaves it running for
  // good in the child, which is a different hang from the one this is about
  // (seen on macOS).
  {
    ProcessLocalQueue warm(&config_.message(), channel("/other"));
  }
  std::atomic<bool> stop{false};
  std::thread maker([this, &stop]() {
    while (!stop) {
      // A new mapping every time, pre-faulted under the lock.
      {
        ProcessLocalQueue big(&config_.message(), channel("/big"));
      }
      ProcessLocalQueue::ResetAll();
    }
  });
  for (int i = 0; i < 20; ++i) {
    const pid_t child = fork();
    ASSERT_GE(child, 0);
    if (child == 0) {
      alarm(5);
      {
        ProcessLocalQueue other(&config_.message(), channel("/other"));
      }
      _exit(0);
    }
    int status;
    ASSERT_EQ(waitpid(child, &status, 0), child);
    EXPECT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0)
        << "the child could not make a queue";
  }
  stop = true;
  maker.join();
}
#endif  // !_WIN32

// Every ProcessLocalQueue for a channel is the same memory, and different
// channels are different memory.
TEST_F(ProcessLocalQueueTest, SameChannelSameMemory) {
  ProcessLocalQueue first(&config_.message(), channel("/test"));
  ProcessLocalQueue second(&config_.message(), channel("/test"));
  ProcessLocalQueue other(&config_.message(), channel("/other"));

  EXPECT_EQ(first.memory(), second.memory());
  EXPECT_EQ(first.const_memory(), second.const_memory());
  EXPECT_NE(first.memory(), other.memory());

  // The two views are two mappings of the same pages: a write through one
  // reads back through the other.
  EXPECT_NE(static_cast<const void *>(first.memory()),
            static_cast<const void *>(first.const_memory()));
  absl::Span<char> writable = first.GetMutableSharedMemory();
  absl::Span<const char> readonly = first.GetConstSharedMemory();
  ASSERT_EQ(writable.size(), readonly.size());
  const char saved = writable.back();
  writable.back() = static_cast<char>(saved + 1);
  EXPECT_EQ(readonly.back(), static_cast<char>(saved + 1));
  writable.back() = saved;
}

// The queue outlives its users: what one sends, a later one finds.
TEST_F(ProcessLocalQueueTest, MessageOutlivesUsers) {
  EXPECT_FALSE(HasMessage(channel("/test")));
  SendOn(channel("/test"));
  EXPECT_TRUE(HasMessage(channel("/test")));
  EXPECT_FALSE(HasMessage(channel("/other")));
}

// ResetAll() is the only way to empty a queue, and it empties all
// of them.
TEST_F(ProcessLocalQueueTest, ResetEmptiesEverything) {
  SendOn(channel("/test"));
  SendOn(channel("/other"));
  ProcessLocalQueue::ResetAll();
  EXPECT_FALSE(HasMessage(channel("/test")));
  EXPECT_FALSE(HasMessage(channel("/other")));
}

// The read-only view really is read-only.
TEST_F(ProcessLocalQueueDeathTest, ReadOnlyViewFaultsOnWrite) {
  ProcessLocalQueue queue(&config_.message(), channel("/test"));
  char *const readonly =
      const_cast<char *>(queue.GetConstSharedMemory().data());
  EXPECT_DEATH({ readonly[0] = 'X'; }, "");
}

// A queue cannot be reset out from under a live user.
TEST_F(ProcessLocalQueueDeathTest, ResetWithLiveUserDies) {
  ProcessLocalQueue queue(&config_.message(), channel("/test"));
  EXPECT_DEATH(ProcessLocalQueue::ResetAll(),
               "live ProcessLocalQueue on /test aos.TestMessage");
}

// The same channel with a different queue configuration is a mistake, the
// way a shared-memory file of the wrong size is.
TEST_F(ProcessLocalQueueDeathTest, ChangedConfigurationDies) {
  ProcessLocalQueue queue(&config_.message(), channel("/test"));

  const FlatbufferDetachedBuffer<Configuration> bigger =
      configuration::MergeConfiguration(FlatbufferDetachedBuffer<Configuration>(
          JsonToFlatbuffer<Configuration>(R"({
  "channels": [
    { "name": "/test", "type": "aos.TestMessage", "frequency": 10,
      "max_size": 1024 }
  ]
})")));
  const Channel *const bigger_channel = configuration::GetChannel(
      &bigger.message(), "/test", "aos.TestMessage", "", nullptr);
  EXPECT_DEATH(ProcessLocalQueue(&bigger.message(), bigger_channel),
               "already exists in this process with a different "
               "configuration");
}

}  // namespace
}  // namespace aos::ipc_lib::testing
