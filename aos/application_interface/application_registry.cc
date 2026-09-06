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

void ApplicationRegistry::Register(std::string_view key, FactoryFunc factory) {
  CHECK(!key.empty());
  CHECK(factory != nullptr);
  CHECK(factories_.emplace(key, std::move(factory)).second)
      << "Duplicate application registered for key: " << key;
}

Result<const ApplicationRegistry::FactoryFunc *>
ApplicationRegistry::GetFactory(std::string_view key) const {
  auto it = factories_.find(key);
  if (it != factories_.cend()) {
    return &it->second;
  }
  return MakeError(
      absl::StrCat("ApplicationRegistry queried for non-existent key: ", key));
}

}  // namespace aos
