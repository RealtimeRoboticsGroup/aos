#ifndef AOS_APPLICATION_INTERFACE_APPLICATION_MACRO_H_
#define AOS_APPLICATION_INTERFACE_APPLICATION_MACRO_H_

#include <memory>
#include <string_view>

#include "aos/application_interface/application_registry.h"
#include "aos/events/event_loop.h"

namespace aos::internal {

// Registrar struct that registers T in the ApplicationRegistry with the given
// key upon construction.
template <typename T>
struct ApplicationRegistrar {
  explicit ApplicationRegistrar(const std::string_view key) {
    ApplicationRegistry::GetInstance().Register(
        key, [](EventLoop *const event_loop) { return T::Create(event_loop); });
  }
};

}  // namespace aos::internal

#define AOS_REGISTER_APPLICATION_INTERNAL(line, key, ClassName)           \
  namespace {                                                             \
  ::aos::internal::ApplicationRegistrar<ClassName> registrar_##line{key}; \
  }
#define AOS_REGISTER_APPLICATION_EXPAND(line, key, ClassName) \
  AOS_REGISTER_APPLICATION_INTERNAL(line, key, ClassName)

// Usage: AOS_REGISTER_APPLICATION("my_app", MyClass)
// Registers ClassName with ApplicationRegistry at static initialization time
// under the specified key.
// Note: ClassName must inherit from aos::AosApplication and have a static
// factory function Create() as described in application.h.
#define AOS_REGISTER_APPLICATION(key, ClassName) \
  AOS_REGISTER_APPLICATION_EXPAND(__LINE__, key, ClassName)

#endif  // AOS_APPLICATION_INTERFACE_APPLICATION_MACRO_H_
