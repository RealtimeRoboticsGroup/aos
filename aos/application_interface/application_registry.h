#ifndef AOS_APPLICATION_INTERFACE_APPLICATION_REGISTRY_H_
#define AOS_APPLICATION_INTERFACE_APPLICATION_REGISTRY_H_

#include <memory>
#include <string_view>
#include <unordered_map>

#include "absl/functional/any_invocable.h"

#include "aos/application_interface/application.h"
#include "aos/events/event_loop.h"
#include "aos/util/status.h"

namespace aos {

// Singleton registry for mapping application implementation names to their
// factory functions.
class ApplicationRegistry {
 public:
  // We conceivably allow anything to be registered here, as long as it fits the
  // signature and is a non-mutable (i.e. pure) function. In practice, the
  // AOS_REGISTER_APPLICATION() macro in application_macro.h should be used for
  // almost all registrations.
  using FactoryFunc =
      absl::AnyInvocable<Result<std::unique_ptr<AosApplication>>(EventLoop *)
                             const>;

  // Gets the instance of this singleton object.
  static ApplicationRegistry &GetInstance();

  // Registers a factory function for a given application implementation name.
  void Register(std::string_view implementation_name, FactoryFunc factory);

  // Retrieves the factory function registered under `implementation_name` by
  // pointer, or a bad status if this key is not registered.
  Result<const FactoryFunc *> GetFactory(
      std::string_view implementation_name) const;

 private:
  ApplicationRegistry() = default;

  // We rely on the guarantee that unordered_map has stable value pointers since
  // we return pointers to them from GetFactory(). This shouldn't really be
  // important in practice because all registrations should happen during the
  // initialization phase and all calls to GetFactory() should happen later.
  std::unordered_map<std::string_view, FactoryFunc> factories_;
};

}  // namespace aos

#endif  // AOS_APPLICATION_INTERFACE_APPLICATION_REGISTRY_H_
