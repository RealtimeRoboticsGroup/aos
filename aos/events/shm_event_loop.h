#ifndef AOS_EVENTS_SHM_EVENT_LOOP_H_
#define AOS_EVENTS_SHM_EVENT_LOOP_H_

#include <memory>
#include <string>

#include "aos/events/queue_event_loop.h"
#include "aos/ipc_lib/shm_base.h"

namespace aos {

// The event loop for a robot: its queues live in shared memory under
// --shm_base, so every process that maps the same files talks over the same
// channels, and they outlive every process that used them (see
// aos/ipc_lib/shm_mapping.h for why that is the contract).
//
// For a program whose AOS users are all its own threads -- simulation, unit
// tests -- see ThreadEventLoop, which leaves nothing behind.
class ShmEventLoop : public QueueEventLoop {
 public:
  ShmEventLoop(const Flatbuffer<Configuration> &configuration)
      : ShmEventLoop(&configuration.message()) {}
  ShmEventLoop(const Configuration *configuration);

  // Runs on somebody else's Aio; see QueueEventLoop for the contract.
  ShmEventLoop(const Flatbuffer<Configuration> &configuration, Aio *aio)
      : ShmEventLoop(&configuration.message(), aio) {}
  ShmEventLoop(const Configuration *configuration, Aio *aio);

  ~ShmEventLoop() override;

 private:
  std::unique_ptr<ipc_lib::QueueMemory> MakeQueueMemory(
      const Channel *channel) override;

  // Capture the --shm_base flag at construction time.  This makes it much
  // easier to make different shared memory regions for doing things like
  // multi-node tests.
  const std::string shm_base_;
};

}  // namespace aos

#endif  // AOS_EVENTS_SHM_EVENT_LOOP_H_
