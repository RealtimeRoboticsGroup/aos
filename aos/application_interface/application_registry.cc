#include "aos/application_interface/application_registry.h"

#include <string_view>
#include <utility>

#include "absl/log/check.h"
#include "absl/strings/str_cat.h"

#include "aos/events/event_loop.h"
#include "aos/util/status.h"

namespace aos {

ApplicationRegistry &ApplicationRegistry::GetInstance() {
  static ApplicationRegistry instance;
  return instance;
}

void ApplicationRegistry::Register(std::string_view implementation_name,
                                   FactoryFunc factory) {
  CHECK(!implementation_name.empty());
  CHECK(factory != nullptr);
  CHECK(factories_.emplace(implementation_name, std::move(factory)).second)
      << "Duplicate application registered!";
}

Result<const ApplicationRegistry::FactoryFunc *>
ApplicationRegistry::GetFactory(std::string_view implementation_name) const {
  auto it = factories_.find(implementation_name);
  if (it != factories_.cend()) {
    return &it->second;
  }
  return MakeError(
      absl::StrCat("ApplicationRegistry queried for non-existent key: ",
                   implementation_name));
}

}  // namespace aos
