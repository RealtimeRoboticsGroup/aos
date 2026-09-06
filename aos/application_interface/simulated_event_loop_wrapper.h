#ifndef AOS_APPLICATION_INTERFACE_SIMULATED_EVENT_LOOP_WRAPPER_H_
#define AOS_APPLICATION_INTERFACE_SIMULATED_EVENT_LOOP_WRAPPER_H_

#include <memory>

#include "aos/application_interface/application.h"
#include "aos/events/event_loop.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/util/status.h"

namespace aos {

// Holds lifetime ownership of instantiated event loops and applications.
struct ApplicationHandles {
  std::vector<std::unique_ptr<EventLoop>> event_loops;
  std::vector<std::unique_ptr<AosApplication>> applications;
};

// Instantiates registered applications listed in the factory's configuration
// via the `executable_name` field in the Application table, or falls back to
// the `name` field if `executable_name` is not set or empty. Every application
// in the configuration must have a linked translation unit registering this
// application key (or the key stripped of a leading "./") or this function
// will return a bad result. Event loops are created with names corresponding to
// the application names as required for proper channel mapping. In multi-node
// configurations, an application instance and event loop are created on each
// node specified in the application's `nodes` list. Callers pass their own
// event loop factory so that they can add other ad-hoc event loops for testing
// or other purposes. Returns ApplicationHandles to the caller so it can manage
// their lifetime.
Result<ApplicationHandles> InstantiateApplications(
    SimulatedEventLoopFactory &factory);

}  // namespace aos

#endif  // AOS_APPLICATION_INTERFACE_SIMULATED_EVENT_LOOP_WRAPPER_H_
