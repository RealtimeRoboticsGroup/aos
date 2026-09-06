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

// Usage: AOS_REGISTER_APPLICATION(MyClass)
// Registers ClassName with ApplicationRegistry at static initialization time.
// Note: ClassName must inherit from aos::AosApplication and have a static
// factory function Create() as described in application.h.
#define AOS_REGISTER_APPLICATION(ClassName)                               \
  namespace {                                                             \
  ::aos::internal::ApplicationRegistrar<ClassName> registrar{#ClassName}; \
  }

#endif  // AOS_APPLICATION_INTERFACE_APPLICATION_MACRO_H_
