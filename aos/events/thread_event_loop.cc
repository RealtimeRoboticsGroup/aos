#include "aos/events/thread_event_loop.h"

#include "aos/ipc_lib/process_local_queue.h"

namespace aos {

ThreadEventLoop::ThreadEventLoop(const Configuration *configuration)
    : QueueEventLoop(configuration) {}

ThreadEventLoop::ThreadEventLoop(const Configuration *configuration, Aio *aio)
    : QueueEventLoop(configuration, aio) {}

std::unique_ptr<ipc_lib::QueueMemory> ThreadEventLoop::MakeQueueMemory(
    const Channel *channel) {
  return std::make_unique<ipc_lib::ProcessLocalQueue>(configuration(), channel);
}

}  // namespace aos
