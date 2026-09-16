#ifndef AOS_IPC_LIB_QUEUE_MEMORY_H_
#define AOS_IPC_LIB_QUEUE_MEMORY_H_

#include "absl/types/span.h"

#include "aos/ipc_lib/lockless_queue.h"

namespace aos::ipc_lib {

// The memory one channel's lockless queue lives in, from whoever owns it.
//
// Everything above this -- LocklessQueue, its senders, readers, pinners and
// watchers -- takes a LocklessQueueMemory pointer and does not care where the
// bytes came from.  This is the seam that lets one event loop implementation
// run over shared memory (MemoryMappedQueue) or over memory only this process
// can see (ProcessLocalQueue).
//
// Every implementation hands back the same block twice: a writable view for
// senders and a read-only view for readers, so a reader that writes into the
// queue faults instead of corrupting it.
class QueueMemory {
 public:
  virtual ~QueueMemory() = default;

  virtual LocklessQueueMemory *memory() const = 0;
  virtual const LocklessQueueMemory *const_memory() const = 0;
  virtual const LocklessQueueConfiguration &config() const = 0;

  LocklessQueue queue() const {
    return LocklessQueue(const_memory(), memory(), config());
  }

  absl::Span<char> GetMutableSharedMemory() const {
    return absl::Span<char>(reinterpret_cast<char *>(memory()),
                            LocklessQueueMemorySize(config()));
  }

  absl::Span<const char> GetConstSharedMemory() const {
    return absl::Span<const char>(
        reinterpret_cast<const char *>(const_memory()),
        LocklessQueueMemorySize(config()));
  }
};

}  // namespace aos::ipc_lib

#endif  // AOS_IPC_LIB_QUEUE_MEMORY_H_
