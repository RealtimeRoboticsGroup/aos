
#include "aos/application_interface/application_registry.h"

#include <memory>
#include <utility>

#include "absl/memory/memory.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "aos/application_interface/application.h"
#include "aos/application_interface/application_macro.h"
#include "aos/configuration_generated.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/json_to_flatbuffer.h"

namespace aos::testing {
namespace {

using ::testing::AllOf;
using ::testing::Eq;
using ::testing::ExplainMatchResult;
using ::testing::NotNull;
using ::testing::Pointee;
using ::testing::PrintToString;
using ::testing::Property;
using ::testing::WhenDynamicCastTo;

// Generic matcher for aos::Result
// TODO(mikebauer) Move this to a status test helpers library.
MATCHER_P(IsOkAndContains, inner_matcher, "") {
  if (!arg.has_value()) {
    *result_listener << "which contains an error: "
                     << PrintToString(arg.error());
    return false;
  }
  return ExplainMatchResult(inner_matcher, arg.value(), result_listener);
}

// Generic ASSERT macro for getting a value out of an Result.
// TODO(mikebauer) Move this to a status test helpers library.
#define ASSERT_HAS_VALUE_AND_ASSIGN(lhs, rexpr)                           \
  auto _result_##__LINE__ = (rexpr);                                      \
  ASSERT_TRUE(_result_##__LINE__.has_value())                             \
      << "Expected value, but got error: " << _result_##__LINE__.error(); \
  lhs = std::move(_result_##__LINE__.value())

// A test application matching the required interface for the registry.
class TestApp : public AosApplication {
 public:
  static aos::Result<std::unique_ptr<aos::AosApplication>> Create(
      aos::EventLoop *const event_loop) {
    return absl::WrapUnique(new TestApp(event_loop));
  }

  const EventLoop *event_loop() const { return event_loop_; }

 private:
  explicit TestApp(EventLoop *event_loop) : event_loop_(event_loop) {}

 private:
  EventLoop *event_loop_;
};

TEST(ApplicationRegistryTest, RegistersAndRetrievesFactory) {
  // SETUP
  const aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::JsonToFlatbuffer<aos::Configuration>("{ \"channels\": [] }");

  aos::SimulatedEventLoopFactory factory{&config.message()};
  std::unique_ptr<aos::EventLoop> event_loop =
      factory.MakeEventLoop("test_event_loop");

  // ACTION
  const auto &registry = ApplicationRegistry::GetInstance();

  // VERIFICATION
  ASSERT_HAS_VALUE_AND_ASSIGN(
      const ApplicationRegistry::FactoryFunc *const registered_factory,
      registry.GetFactory("aos::testing::TestApp"));
  ASSERT_NE(registered_factory, nullptr);
  // Check that the function isn't null.
  ASSERT_NE(*registered_factory, nullptr);

  // Actually call the function to make sure the factory calls Create above.
  const Result<std::unique_ptr<const AosApplication>> app =
      (*registered_factory)(event_loop.get());

  EXPECT_THAT(app,
              IsOkAndContains(Property(
                  &std::unique_ptr<const AosApplication>::get,
                  WhenDynamicCastTo<const TestApp *>(AllOf(
                      NotNull(), Pointee(Property(&TestApp::event_loop,
                                                  Eq(event_loop.get()))))))));
}

}  // namespace
}  // namespace aos::testing

AOS_REGISTER_APPLICATION(aos::testing::TestApp)
