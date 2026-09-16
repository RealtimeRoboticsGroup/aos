#include "aos/ipc_lib/process_local_queue.h"

#ifndef _WIN32
#include <pthread.h>
#endif

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "absl/base/no_destructor.h"
#include "absl/base/thread_annotations.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/ipc_lib/memory_mapped_queue.h"
#include "aos/ipc_lib/process_local_queue_internal.h"
#include "aos/ipc_lib/shm_mapping.h"

namespace aos::ipc_lib {

namespace internal {

struct ProcessLocalQueueEntry {
  LocklessQueueConfiguration config;
  ProcessLocalMapping mapping;
  // ProcessLocalQueues currently pointing at this entry.  Only
  // ResetAll() reads it; the memory itself never goes away on the
  // count reaching zero.
  int users = 0;
};

}  // namespace internal

namespace {

using internal::ProcessLocalQueueEntry;

// The registry.  Never destroyed: the queues are meant to outlive every user
// in the process, and running destructors at exit would race whatever threads
// are still touching them.  NoDestructor puts it in static storage rather
// than on the heap, so it is not a leak to anything that checks for one.
struct Registry {
  std::mutex mutex;
  std::map<std::pair<std::string, std::string>,
           std::unique_ptr<ProcessLocalQueueEntry>>
      entries;
};

Registry &registry() {
  static absl::NoDestructor<Registry> instance;
#ifndef _WIN32
  // A fork() child shares these queues, so it may well make or destroy one,
  // and a child forked while another thread held the mutex would find it
  // locked for good.  The window is wide: the first acquisition of a channel
  // maps and pre-faults the whole block under it.  So a fork waits for the
  // mutex, and both sides let it go.  The lock and unlock are in separate
  // functions, which the thread safety analysis can't follow.
  static const bool at_fork_registered = []() {
    ABSL_CHECK_EQ(
        pthread_atfork(
            []() ABSL_NO_THREAD_SAFETY_ANALYSIS { registry().mutex.lock(); },
            []() ABSL_NO_THREAD_SAFETY_ANALYSIS { registry().mutex.unlock(); },
            []() ABSL_NO_THREAD_SAFETY_ANALYSIS { registry().mutex.unlock(); }),
        0);
    return true;
  }();
  (void)at_fork_registered;
#endif
  return *instance;
}

ProcessLocalQueueEntry *AcquireEntry(const Channel *channel,
                                     const LocklessQueueConfiguration &config) {
  Registry &r = registry();
  std::unique_lock<std::mutex> locker(r.mutex);

  std::unique_ptr<ProcessLocalQueueEntry> &entry =
      r.entries[std::make_pair(std::string(channel->name()->string_view()),
                               std::string(channel->type()->string_view()))];
  if (entry == nullptr) {
    entry = std::make_unique<ProcessLocalQueueEntry>();
    entry->config = config;
    entry->mapping = internal::MapProcessLocal(LocklessQueueMemorySize(config));
    // Pre-fault both views once, here, so a later (possibly realtime) access
    // through either never takes a fault.  Shared memory does this per
    // mapping; this block is only ever mapped once per process.
    const long page_size = SystemPageSize();
    PageFaultDataWrite(static_cast<char *>(entry->mapping.writable),
                       entry->mapping.size, page_size);
    PageFaultDataRead(static_cast<const char *>(entry->mapping.readonly),
                      entry->mapping.size, page_size);
  } else if (entry->config != config) {
    // The shared-memory equivalent is the backing file's size not matching,
    // which MapShm() dies on.  Same answer here.
    ABSL_LOG(FATAL) << "Queue for "
                    << configuration::CleanedChannelToString(channel)
                    << " already exists in this process with a different "
                       "configuration.  Did the channel definition change?";
  }
  ++entry->users;
  return entry.get();
}

void ReleaseEntry(ProcessLocalQueueEntry *entry) {
  Registry &r = registry();
  std::unique_lock<std::mutex> locker(r.mutex);
  ABSL_CHECK_GT(entry->users, 0);
  --entry->users;
}

}  // namespace

ProcessLocalQueue::ProcessLocalQueue(const Configuration *configuration,
                                     const Channel *channel)
    : config_(MakeQueueConfiguration(configuration, channel)),
      entry_(AcquireEntry(channel, config_)),
      memory_(static_cast<LocklessQueueMemory *>(entry_->mapping.writable)),
      const_memory_(
          static_cast<const LocklessQueueMemory *>(entry_->mapping.readonly)) {
  InitializeLocklessQueueMemory(memory_, config_);
}

ProcessLocalQueue::~ProcessLocalQueue() { ReleaseEntry(entry_); }

void ProcessLocalQueue::ResetAll() {
  Registry &r = registry();
  std::unique_lock<std::mutex> locker(r.mutex);
  for (const auto &[key, entry] : r.entries) {
    ABSL_CHECK_EQ(entry->users, 0)
        << ": ResetAll() with a live ProcessLocalQueue on " << key.first << " "
        << key.second;
  }
  for (const auto &[key, entry] : r.entries) {
    internal::UnmapProcessLocal(entry->mapping);
  }
  r.entries.clear();
}

}  // namespace aos::ipc_lib
