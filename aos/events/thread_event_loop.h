#ifndef AOS_EVENTS_THREAD_EVENT_LOOP_H_
#define AOS_EVENTS_THREAD_EVENT_LOOP_H_

#include <memory>

#include "aos/events/queue_event_loop.h"

namespace aos {

// The event loop for a program whose AOS users are all its own threads:
// simulation, unit tests.  Its queues are ordinary memory this process owns
// (aos/ipc_lib/process_local_queue.h), shared between its threads and never
// with another program, so the OS reclaims them when it exits however it
// exits, and nothing can be left behind in /dev/shm or anywhere else.  A
// fork() child shares them, as it would shared memory.
//
// Inside the process the queues behave as shared memory does: they live until
// the process exits, so an event loop built after another one was destroyed
// still finds the last message.  Two ThreadEventLoops in one process see each
// other; a ThreadEventLoop and a ShmEventLoop on the same channel do not,
// even in the same process.  --shm_base and --permissions do not apply.
//
// There is one queue per channel name and type in the whole process, whatever
// Configuration each ThreadEventLoop was built from.  A second Configuration
// that shapes a channel's queue differently dies, and there is nothing like
// ShmEventLoop's per-loop --shm_base to keep two nodes of a multi-node
// configuration apart in one process.
//
// Everything else -- the API, wakeups, timers, realtime -- is ShmEventLoop's,
// through the QueueEventLoop they share.
class ThreadEventLoop : public QueueEventLoop {
 public:
  ThreadEventLoop(const Flatbuffer<Configuration> &configuration)
      : ThreadEventLoop(&configuration.message()) {}
  ThreadEventLoop(const Configuration *configuration);

  // Runs on somebody else's Aio; see QueueEventLoop for the contract.
  ThreadEventLoop(const Configuration *configuration, Aio *aio);

 private:
  std::unique_ptr<ipc_lib::QueueMemory> MakeQueueMemory(
      const Channel *channel) override;
};

}  // namespace aos

#endif  // AOS_EVENTS_THREAD_EVENT_LOOP_H_
