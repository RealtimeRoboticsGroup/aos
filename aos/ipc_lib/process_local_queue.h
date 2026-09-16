#ifndef AOS_IPC_LIB_PROCESS_LOCAL_QUEUE_H_
#define AOS_IPC_LIB_PROCESS_LOCAL_QUEUE_H_

#include "aos/configuration.h"
#include "aos/ipc_lib/lockless_queue.h"
#include "aos/ipc_lib/queue_memory.h"

namespace aos::ipc_lib {

namespace internal {
struct ProcessLocalQueueEntry;
}  // namespace internal

// Queue memory that belongs to this process: shared between its threads,
// never with another program, and reclaimed by the OS when the process exits
// however it exits.  There are no names, files or kernel objects to clean up.
// A fork() child keeps both mappings, so it shares these queues with its
// parent the way it would shared memory.
//
// One registry per process, keyed by channel name and type, so every
// ProcessLocalQueue for a channel hands back the same block.  The block lives
// until the process exits, not until its last user is destroyed: a listener
// thread that restarts, or a test that builds a new event loop, still finds
// the last message, which is the contract shared memory has inside one
// process too.  Memory is bounded by the number of distinct channels, which
// is what shared memory costs as well.
//
// Neither --shm_base nor --permissions means anything here.
class ProcessLocalQueue : public QueueMemory {
 public:
  ProcessLocalQueue(const Configuration *configuration, const Channel *channel);
  ~ProcessLocalQueue() override;

  ProcessLocalQueue(const ProcessLocalQueue &) = delete;
  ProcessLocalQueue &operator=(const ProcessLocalQueue &) = delete;

  LocklessQueueMemory *memory() const override { return memory_; }
  const LocklessQueueMemory *const_memory() const override {
    return const_memory_;
  }
  const LocklessQueueConfiguration &config() const override { return config_; }

  // Frees every queue in the process, so the next ProcessLocalQueue on any
  // channel starts from empty memory -- the process-local equivalent of
  // wiping --shm_base.  Dies if any ProcessLocalQueue is still alive, because
  // its memory is about to go away.
  static void ResetAll();

 private:
  const LocklessQueueConfiguration config_;
  internal::ProcessLocalQueueEntry *entry_;
  LocklessQueueMemory *memory_;
  const LocklessQueueMemory *const_memory_;
};

}  // namespace aos::ipc_lib

#endif  // AOS_IPC_LIB_PROCESS_LOCAL_QUEUE_H_
