#include "aos/application_interface/simulated_event_loop_wrapper.h"

#include <memory>
#include <string_view>
#include <utility>
#include <vector>

#include "aos/application_interface/application.h"
#include "aos/application_interface/application_registry.h"
#include "aos/configuration.h"
#include "aos/events/event_loop.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/util/status.h"

namespace aos {

Result<ApplicationHandles> InstantiateApplications(
    SimulatedEventLoopFactory &factory) {
  const Configuration *config = factory.configuration();
  if (config == nullptr) {
    return MakeError("Invalid configuration in event loop factory");
  }

  ApplicationHandles handles;

  for (const Application *const app_config : *config->applications()) {
    const std::string_view name =
        app_config->has_name() ? app_config->name()->string_view() : "";
    const std::string_view executable_name =
        (app_config->has_executable_name() &&
         !app_config->executable_name()->string_view().empty())
            ? app_config->executable_name()->string_view()
            : name;

    if (executable_name.empty()) {
      return MakeError("Application missing both executable_name and name");
    }
    const ApplicationRegistry::FactoryFunc *factory_func{};
    auto factory_result =
        ApplicationRegistry::GetInstance().GetFactory(executable_name);
    if (!factory_result.has_value() && executable_name.starts_with("./")) {
      factory_result = ApplicationRegistry::GetInstance().GetFactory(
          executable_name.substr(2));
    }
    AOS_ASSIGN_OR_RETURN_ERROR(factory_func, std::move(factory_result));

    std::vector<const Node *> nodes;
    AOS_ASSIGN_OR_RETURN_ERROR(
        nodes, configuration::GetNodesForApplication(config, app_config));

    for (const Node *node : nodes) {
      std::unique_ptr<EventLoop> loop = factory.MakeEventLoop(name, node);
      std::unique_ptr<AosApplication> app_instance;
      AOS_ASSIGN_OR_RETURN_ERROR(app_instance, (*factory_func)(loop.get()));

      handles.event_loops.push_back(std::move(loop));
      handles.applications.push_back(std::move(app_instance));
    }
  }

  return handles;
}

}  // namespace aos
