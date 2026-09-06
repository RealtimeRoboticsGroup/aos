#include "aos/application_interface/simulated_event_loop_wrapper.h"

#include "absl/memory/memory.h"
#include "gtest/gtest.h"

#include "aos/application_interface/application.h"
#include "aos/application_interface/application_macro.h"
#include "aos/application_interface/application_registry.h"
#include "aos/application_interface/testdata/message_static.h"
#include "aos/configuration.h"
#include "aos/events/event_loop.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/testing/path.h"
#include "aos/util/status.h"

namespace aos::testing {

// Generic ASSERT macro for getting a value out of an Result.
// TODO(mikebauer) Move this to a status test helpers library.
#define ASSERT_HAS_VALUE_AND_ASSIGN(lhs, rexpr)                           \
  auto _result_##__LINE__ = (rexpr);                                      \
  ASSERT_TRUE(_result_##__LINE__.has_value())                             \
      << "Expected value, but got error: " << _result_##__LINE__.error(); \
  lhs = std::move(_result_##__LINE__.value())

class DummyApp : public AosApplication {
 public:
  static constexpr int kValueToPublish = 42;

  static aos::Result<std::unique_ptr<aos::AosApplication>> Create(
      aos::EventLoop *const event_loop) {
    return absl::WrapUnique(new DummyApp(event_loop));
  }

 private:
  DummyApp(aos::EventLoop *const event_loop) {
    sender_ =
        event_loop
            ->MakeSender<aos::application_interface::testdata::MessageStatic>(
                "/test");

    event_loop->OnRun([this]() {
      aos::Sender<aos::application_interface::testdata::MessageStatic>::
          StaticBuilder builder = sender_.MakeStaticBuilder();
      builder->set_value(kValueToPublish);
      builder.CheckOk(builder.Send());
    });
  }

  Sender<aos::application_interface::testdata::MessageStatic> sender_;
};

TEST(SimulatedEventLoopWrapperTest, InstantiatesRegisteredApplications) {
  // SETUP
  const aos::FlatbufferDetachedBuffer<Configuration> config =
      configuration::MergeWithConfig(
          &configuration::ReadConfig(
               "aos/application_interface/testdata/aos_config.bfbs")
               .message(),
          R"json({
    "applications": [
      {
        "name": "dummy_instance",
        "executable_name": "dummy_app"
      }
    ]
  })json");

  SimulatedEventLoopFactory factory(&config.message());

  // ACTION
  ASSERT_HAS_VALUE_AND_ASSIGN(const ApplicationHandles handles,
                              InstantiateApplications(factory));

  // VERIFICATION
  bool ran = false;
  std::unique_ptr<EventLoop> test_event_loop = factory.MakeEventLoop("test");
  test_event_loop->MakeWatcher(
      "/test",
      [&ran](const aos::application_interface::testdata::Message &message) {
        EXPECT_EQ(message.value(), DummyApp::kValueToPublish);
        ran = true;
      });
  factory.RunFor(std::chrono::seconds(1));
  EXPECT_TRUE(ran);
}

TEST(SimulatedEventLoopWrapperTest,
     InstantiatesRegisteredApplicationsMultinode) {
  // SETUP
  const aos::FlatbufferDetachedBuffer<Configuration> config =
      configuration::MergeWithConfig(
          &configuration::ReadConfig(
               "aos/application_interface/testdata/aos_multinode_config.bfbs")
               .message(),
          R"json({
    "applications": [
      {
        "name": "dummy_instance_a",
        "executable_name": "dummy_app",
        "nodes": ["a"],
        "maps": [
          {
            "match": {
              "name": "/test"
            },
            "rename": {
              "name": "/a/test"
            }
          }
        ]
      },
      {
        "name": "dummy_instance_b",
        "executable_name": "dummy_app",
        "nodes": ["b"],
        "maps": [
          {
            "match": {
              "name": "/test"
            },
            "rename": {
              "name": "/b/test"
            }
          }
        ]
      }
    ]
  })json");

  SimulatedEventLoopFactory factory(&config.message());

  // ACTION
  ASSERT_HAS_VALUE_AND_ASSIGN(const ApplicationHandles handles,
                              InstantiateApplications(factory));

  // VERIFICATION
  int ran_count = 0;

  // Watcher for a
  const Node *const node_a = configuration::GetNode(&config.message(), "a");
  const std::unique_ptr<EventLoop> test_event_loop_a =
      factory.MakeEventLoop("test_a", node_a);
  test_event_loop_a->MakeWatcher(
      "/a/test",
      [&ran_count](
          const aos::application_interface::testdata::Message &message) {
        EXPECT_EQ(message.value(), DummyApp::kValueToPublish);
        ++ran_count;
      });

  // Watcher for b
  const Node *const node_b = configuration::GetNode(&config.message(), "b");
  const std::unique_ptr<EventLoop> test_event_loop_b =
      factory.MakeEventLoop("test_b", node_b);
  test_event_loop_b->MakeWatcher(
      "/b/test",
      [&ran_count](
          const aos::application_interface::testdata::Message &message) {
        EXPECT_EQ(message.value(), DummyApp::kValueToPublish);
        ++ran_count;
      });

  factory.RunFor(std::chrono::seconds(1));
  EXPECT_EQ(ran_count, 2);
}

TEST(SimulatedEventLoopWrapperTest,
     InstantiatesSingleApplicationOnMultipleNodes) {
  // SETUP
  const aos::FlatbufferDetachedBuffer<Configuration> config =
      configuration::MergeWithConfig(
          &configuration::ReadConfig(
               "aos/application_interface/testdata/aos_multinode_config.bfbs")
               .message(),
          R"json({
    "maps": [
      {
        "match": {
          "name": "/test",
          "source_node": "a"
        },
        "rename": {
          "name": "/a/test"
        }
      },
      {
        "match": {
          "name": "/test",
          "source_node": "b"
        },
        "rename": {
          "name": "/b/test"
        }
      }
    ],
    "applications": [
      {
        "name": "dummy_instance",
        "executable_name": "dummy_app",
        "nodes": ["a", "b"]
      }
    ]
  })json");

  SimulatedEventLoopFactory factory(&config.message());

  // ACTION
  ASSERT_HAS_VALUE_AND_ASSIGN(const ApplicationHandles handles,
                              InstantiateApplications(factory));

  EXPECT_EQ(handles.event_loops.size(), 2u);
  EXPECT_EQ(handles.applications.size(), 2u);

  // VERIFICATION
  int ran_count = 0;

  // Watcher for a
  const Node *const node_a = configuration::GetNode(&config.message(), "a");
  const std::unique_ptr<EventLoop> test_event_loop_a =
      factory.MakeEventLoop("test_a", node_a);
  test_event_loop_a->MakeWatcher(
      "/a/test",
      [&ran_count](
          const aos::application_interface::testdata::Message &message) {
        EXPECT_EQ(message.value(), DummyApp::kValueToPublish);
        ++ran_count;
      });

  // Watcher for b
  const Node *const node_b = configuration::GetNode(&config.message(), "b");
  const std::unique_ptr<EventLoop> test_event_loop_b =
      factory.MakeEventLoop("test_b", node_b);
  test_event_loop_b->MakeWatcher(
      "/b/test",
      [&ran_count](
          const aos::application_interface::testdata::Message &message) {
        EXPECT_EQ(message.value(), DummyApp::kValueToPublish);
        ++ran_count;
      });

  factory.RunFor(std::chrono::seconds(1));
  EXPECT_EQ(ran_count, 2);
}

TEST(SimulatedEventLoopWrapperTest, ErrorOnMissingNodesInMultiNode) {
  // SETUP
  const aos::FlatbufferDetachedBuffer<Configuration> config =
      configuration::MergeWithConfig(
          &configuration::ReadConfig(
               "aos/application_interface/testdata/aos_multinode_config.bfbs")
               .message(),
          R"json({
    "applications": [
      {
        "name": "dummy_instance",
        "executable_name": "dummy_app"
      }
    ]
  })json");

  SimulatedEventLoopFactory factory(&config.message());

  // ACTION / VERIFICATION
  EXPECT_FALSE(InstantiateApplications(factory).has_value());
}

TEST(SimulatedEventLoopWrapperTest, ErrorOnUnknownNodeInMultiNode) {
  // SETUP
  const aos::FlatbufferDetachedBuffer<Configuration> config =
      configuration::MergeWithConfig(
          &configuration::ReadConfig(
               "aos/application_interface/testdata/aos_multinode_config.bfbs")
               .message(),
          R"json({
    "applications": [
      {
        "name": "dummy_instance",
        "executable_name": "dummy_app",
        "nodes": ["nonexistent_node"]
      }
    ]
  })json");

  SimulatedEventLoopFactory factory(&config.message());

  // ACTION / VERIFICATION
  EXPECT_FALSE(InstantiateApplications(factory).has_value());
}

TEST(SimulatedEventLoopWrapperTest, BadResultOnKeyMiss) {
  // SETUP
  const aos::FlatbufferDetachedBuffer<Configuration> config =
      configuration::MergeWithConfig(
          &configuration::ReadConfig(
               "aos/application_interface/testdata/aos_config.bfbs")
               .message(),
          R"json({
    "applications": [
      {
        "name": "dummy_instance",
        "executable_name": "not_a_real_app"
      }
    ]
  })json");

  SimulatedEventLoopFactory factory(&config.message());

  // ACTION / VERIFICATION
  EXPECT_FALSE(InstantiateApplications(factory).has_value());
}

TEST(SimulatedEventLoopWrapperTest, FallsBackToNameWhenExecutableNameUnset) {
  // SETUP
  const aos::FlatbufferDetachedBuffer<Configuration> config =
      configuration::MergeWithConfig(
          &configuration::ReadConfig(
               "aos/application_interface/testdata/aos_config.bfbs")
               .message(),
          R"json({
    "applications": [
      {
        "name": "dummy_app"
      }
    ]
  })json");

  SimulatedEventLoopFactory factory(&config.message());

  // ACTION
  ASSERT_HAS_VALUE_AND_ASSIGN(const ApplicationHandles handles,
                              InstantiateApplications(factory));

  EXPECT_EQ(handles.applications.size(), 1);
  EXPECT_EQ(handles.event_loops.size(), 1);

  // VERIFICATION
  bool ran = false;
  std::unique_ptr<EventLoop> test_event_loop = factory.MakeEventLoop("test");
  test_event_loop->MakeWatcher(
      "/test",
      [&ran](const aos::application_interface::testdata::Message &message) {
        EXPECT_EQ(message.value(), DummyApp::kValueToPublish);
        ran = true;
      });
  factory.RunFor(std::chrono::seconds(1));
  EXPECT_TRUE(ran);
}

TEST(SimulatedEventLoopWrapperTest, FallsBackToNameWhenExecutableNameEmpty) {
  // SETUP
  const aos::FlatbufferDetachedBuffer<Configuration> config =
      configuration::MergeWithConfig(
          &configuration::ReadConfig(
               "aos/application_interface/testdata/aos_config.bfbs")
               .message(),
          R"json({
    "applications": [
      {
        "name": "dummy_app",
        "executable_name": ""
      }
    ]
  })json");

  SimulatedEventLoopFactory factory(&config.message());

  // ACTION
  ASSERT_HAS_VALUE_AND_ASSIGN(const ApplicationHandles handles,
                              InstantiateApplications(factory));

  EXPECT_EQ(handles.applications.size(), 1);
  EXPECT_EQ(handles.event_loops.size(), 1);
}

TEST(SimulatedEventLoopWrapperTest, BadResultOnKeyMissFallbackToName) {
  // SETUP
  const aos::FlatbufferDetachedBuffer<Configuration> config =
      configuration::MergeWithConfig(
          &configuration::ReadConfig(
               "aos/application_interface/testdata/aos_config.bfbs")
               .message(),
          R"json({
    "applications": [
      {
        "name": "not_a_real_app"
      }
    ]
  })json");

  SimulatedEventLoopFactory factory(&config.message());

  // ACTION / VERIFICATION
  EXPECT_FALSE(InstantiateApplications(factory).has_value());
}

TEST(SimulatedEventLoopWrapperTest, ErrorOnEmptyNameAndExecutableName) {
  // SETUP
  const aos::FlatbufferDetachedBuffer<Configuration> config =
      configuration::MergeWithConfig(
          &configuration::ReadConfig(
               "aos/application_interface/testdata/aos_config.bfbs")
               .message(),
          R"json({
    "applications": [
      {
        "name": ""
      }
    ]
  })json");

  SimulatedEventLoopFactory factory(&config.message());

  // ACTION / VERIFICATION
  EXPECT_FALSE(InstantiateApplications(factory).has_value());
}

}  // namespace aos::testing

AOS_REGISTER_APPLICATION("dummy_app", aos::testing::DummyApp)
