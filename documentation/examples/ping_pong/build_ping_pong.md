# Getting Started with AOS

This page will walk through how to create a simple Ping/Pong application in AOS, essentially from scratch.

## Building the Code

For purposes of this exercise, we will assume that you have cloned the [aos](https://github.com/RealtimeRoboticsGroup/aos) repository. If you have set up AOS as an external repository in your own [Bazel](https://bazel.build/) workspace, the same steps apply. All AOS labels below are written as `@aos//...`, which works both from inside the AOS repository and from a repository that depends on AOS.

AOS nominally supports Debian Bookworm, although later versions of Debian and Ubuntu 20.04+ should work to build the code.

After cloning AOS and installing Bazel, then you can confirm that the core functionality is working by doing

```
bazel test //aos/events/...
```

Note that several tests may fail to run due to permissions issues, since they will attempt to exercise some of AOS code related to setting realtime schedulers and locking memory. You can resolve these by following the directions in [Realtime System Configuration](#realtime-system-configuration).

At a minimum, `bazel test //aos/events:simulated_event_loop_test` should reliably succeed.

Once you have the code building, you can start trying to write an application. Note that this example will just be walking through replicating the existing ping/pong processes that are defined in `//documentation/examples/ping_pong/`[^paths]. If you are unsure if you are doing something wrong, you can compare your code to the code already in the repository.

## Writing a Pair of Ping/Pong Applications

For this example, we will create a pair of applications where the `ping` application will send an `aos.examples.Ping` message at a fixed period. The `pong` application will listen for this message, and send an `aos.examples.Pong` message in response. The ping application will listen for this message and print to stderr whenever it gets a response.

We will do all our work in a new folder. Let us create a new top-level `foo` folder in the repository (the exact name/path is unimportant).

To keep this example tractable, it functions in a "single-node" world. Most AOS users actually operate in a multi-node world. This is discussed later.

<!-- TODO: link to said discussion. -->

### Defining FlatBuffer Messages

The FlatBuffer messages that our processes send/receive are effectively the API of the processes themselves, and so we will start by defining them. FlatBuffer messages are defined in `.fbs` files which define the schema for the message. For details on how FlatBuffers themselves work, see the [FlatBuffers website](https://google.github.io/flatbuffers/).

For the Ping message, create a `ping.fbs` file with the following contents:

```
namespace aos.examples;

table Ping {
  value:int (id: 0);
  send_time:long (id: 1);
}

root_type Ping;
```

Stepping through what each line means:

```
namespace aos.examples;
```

This specifies that all following definitions will be in the `aos.examples` namespace. For C++ code, this results in the generated flatbuffer code being in the `aos::examples` namespace (as one would expect). Other languages translate namespaces in different ways.

```
table Ping {
```

Defines the `Ping` table. Because this is in the `aos.examples` namespace, the fully-qualified name of this table will be `aos.examples.Ping`.

```
  value:int (id: 0);
```

Defines a 32-bit signed integer field named `value` in the flatbuffer. Since no default value is specified, the default default value of 0 will be used. The `(id: 0)` specifies the unique ID that the flatbuffer serialization will use for this field. The exact value is generally unimportant, except that IDs must be unique, IDs must start from zero and have no gaps (or else the flatbuffer compiler will yell at you), and because IDs are used for serialization, if you ever change the ID used for a field (or re-use the ID for a different field), you will break backwards compatibility with old data.

Details about the scalar types available in FlatBuffers can be found [here](https://google.github.io/flatbuffers/flatbuffers_guide_writing_schema.html) in the Types section.

```
  send_time:long (id: 1);
```

This defines a field named `send_time` that is a signed 64-bit integer.

```
}

root_type Pong;
```

The `root_type` declaration doesn't change any FlatBuffer serialization on its own, but the root type for a schema will see some additional codegen. For AOS, any table that is going to serve as a message must be the `root_type` of its `.fbs` file (there is only one message per `.fbs` file in this example, and no nested tables, so the issue is irrelevant here).

For the `aos.examples.Pong` message, we will create a `pong.fbs` that has essentially the same structure:

```
namespace aos.examples;

table Pong {
  value:int (id: 0);
  initial_send_time:long (id: 1);
}

root_type Pong;
```

The final part of defining the FlatBuffer messages is defining Bazel targets to actually do the codegen for the schemas. Since we have not yet created a BUILD file for this new directory, we will create a new `BUILD` file with the following contents:

```python
load("@aos//aos:defs.bzl", "static_flatbuffer")

static_flatbuffer(
    name = "ping_fbs",
    srcs = ["ping.fbs"],
)

static_flatbuffer(
    name = "pong_fbs",
    srcs = ["pong.fbs"],
)
```

If we step through the interesting lines in the `BUILD` file:

```python
load("@aos//aos:defs.bzl", "static_flatbuffer")
```

Imports the `static_flatbuffer` build rule so that we have it available. `@aos//aos:defs.bzl` is the public entry point for AOS build macros.

```python
static_flatbuffer(
    name = "ping_fbs",
```

Defines a target (named `ping_fbs`) that will codegen a C++ library for the specified `.fbs` file. This can then be depended on by C++ Bazel targets directly. The library provides two APIs for each table:

- The [static FlatBuffers API](flatbuffers.md) (`ping_static.h`), which provides an `aos.examples.PingStatic` class. This is what AOS code should use to _build_ messages.
- The standard FlatBuffers API (`ping_generated.h`, which `ping_static.h` includes), which provides the read-only `aos.examples.Ping` class used to _read_ messages.

You can also write messages with the upstream FlatBuffers API (`Sender::MakeBuilder()` and a `flatbuffers::FlatBufferBuilder`), but it is harder to use: tables, strings, and vectors must be built bottom-up, and each table must be finished before it can be referenced by its parent.

`static_flatbuffer` also always generates the reflection schema for the FlatBuffer file. This results in a `.bfbs` file being generated that contains a FlatBuffer of type `reflection.Schema` (defined in [reflection.fbs][reflection_fbs]). This is required for FlatBuffer files that will be used for AOS messages so that the AOS config can include reflection information.

```python
    srcs = ["ping.fbs"],
```

Specifies the `.fbs` file that we are generating code for.

For more details on the arguments available in the `static_flatbuffer` rule, see the docstring in [`aos/flatbuffers/generate.bzl`](https://github.com/RealtimeRoboticsGroup/aos/blob/main/aos/flatbuffers/generate.bzl).

To confirm that you have successfully set up your FlatBuffer files and BUILD targets, you can build the targets directly:

```bash
bazel build //foo:ping_fbs
```

This should generate C++ headers at `bazel-bin/foo/ping_static.h` and `bazel-bin/foo/ping_generated.h`.

### Writing the AOS Config

Every system running AOS will need a configuration. The configuration is a FlatBuffer message defined in [configuration.fbs](https://github.com/RealtimeRoboticsGroup/aos/blob/main/aos/configuration.fbs). Developers typically interact with the AOS config as a JSON file, which is then parsed into a FlatBuffer by various pieces of tooling.

The primary role of the config is to define all of the message channels that will exist. It also includes some top-level configuration information and a list of processes ("applications") that will be run.

For this system we will create a file named `pingpong.json` that has contents of:

```json
{
  "applications": [
    {
      "name": "ping"
    },
    {
      "name": "pong"
    }
  ],
  "channels": [
    {
      "name": "/test",
      "type": "aos.examples.Ping",
      "frequency": 4500
    },
    {
      "name": "/test",
      "type": "aos.examples.Pong",
      "frequency": 4500
    }
  ],
  "imports": ["../aos/events/aos.json"]
}
```

Going over the key lines:

```json
  "applications": [
    {
      "name": "ping"
    },
    {
      "name": "pong"
    }
  ],
```

This defines the set of applications running in the system. Since we have nothing particularly complex going on, we only need to specify the names of the applications. If, e.g., the name of the binary being run does not match the name of the application, then additional parameters are needed.

```json
    {
      "name": "/test",
      "type": "aos.examples.Ping",
      "frequency": 4500
    },
```

This defines a channel named `/test` with a type of `aos.examples.Ping` that will see messages at rate of at most 4500 Hz. Note that the pair of name + type must be unique per channel, but it is possible to both have multiple channels with the same name but different types and channels with different names but the same types. Note that the `type` field will always use the fully-qualified type name (i.e., it includes the namespace). The frequency is a strict maximum---messages cannot be sent any faster. See [Sent Too Fast](reference.md#sent-too-fast) for details on max frequencies.

There is also a `max_size` channel value that allows you to increase the allowable size (in bytes) for messages sent on the channel.

<!-- TODO: Link to multi-node configuration. -->

```json
  "imports": [
    "../aos/events/aos.json"
  ]
```

This imports the `aos.json` config, which defines some common channels that are potentially relevant for any AOS system. Note that imports are relative.

Next, we need to use the `aos_config` Bazel rule to generate the processed config that can actually be used on a real system. Internally, the `aos_config` rule is feeding the hand-written configurations through the `config_flattener` which does a few things:

- Resolves all imports.
- Imports the message schemas for each channel into the config.
- Generates the resulting flattened config in multiple useful formats.

To do this, we add the following to our `BUILD` file:

```python
# Typically, this load() statement will go at the top of the file.
load("@aos//aos:defs.bzl", "aos_config")

aos_config(
    name = "pingpong_config",
    src = "pingpong.json",
    flatbuffers = [
        ":ping_fbs",
        ":pong_fbs",
    ],
    deps = ["@aos//aos/events:aos_config"],
)
```

In particular, note that the list of `flatbuffers` must include _all_ the FlatBuffers used by the config (so that we can get at the reflection schemas that `static_flatbuffer` generates, as described in the previous section). We also have a `deps` entry for the config that we are importing.

If we build this config

```bash
bazel build //foo:pingpong_config
```

We will see that it generates three output files

```
bazel-bin/foo/pingpong_config.json
bazel-bin/foo/pingpong_config.stripped.json
bazel-bin/foo/pingpong_config.bfbs
```

The suffixes of each file mean:

- `.json`: This is the full resulting config, in human-readable JSON.
- `.stripped.json`: This is the flattened config, but with the message schemas removed. This is not used by any application code, but can be convenient for debugging since it is a much smaller file than the full configs.
- `.bfbs`: The full resulting config, but serialized as a binary FlatBuffer. This contains exactly the same information as the `.json`, but is much smaller and is much faster to parse.

Both the `.json` and `.bfbs` files can be used by applications, but most applications will attempt to use the `.bfbs` file if present due to the lower startup penalty from not having to read a massive JSON file.

### Writing the Ping Application

Now that we have the message types and channels all defined, we have the basic structure that we need in order to actually write our applications, starting with `ping`. To do this, we will end up creating four files:

- `ping_lib.h`: The header file for the application itself.
- `ping_lib.cc`: The source file implementing our application.
- `ping.cc`: The file with `main` that will start up the `ping` application.
- `ping_lib_test.cc`: Contains the unit tests for the code in `ping_lib.*`.tests

The contents of `ping_lib.h` will be:

```cpp
#ifndef FOO_PING_LIB_H_
#define FOO_PING_LIB_H_

#include "aos/events/event_loop.h"
#include "foo/ping_static.h"
#include "foo/pong_static.h"

namespace aos {

// Class which sends out a Ping message every X ms, and times the response.
class Ping {
 public:
  Ping(EventLoop *event_loop);

 private:
  // Sends out the ping message with an incrementing count.
  void SendPing();

  // Receives the reply and measures the latency.
  void HandlePong(const examples::Pong &pong);

  aos::EventLoop *event_loop_;
  aos::Sender<examples::PingStatic> sender_;
  // Timer handle which sends the Ping message.
  aos::TimerHandler *timer_handle_;
  // Number of pings sent.
  int count_ = 0;
};

}  // namespace aos

#endif  // FOO_PING_LIB_H_
```

And `ping_lib.cc` will be:

```cpp
#include "foo/ping_lib.h"

#include "absl/flags/flag.h"
#include "absl/log/absl_log.h"

#include "aos/json_to_flatbuffer.h"

ABSL_FLAG(int32_t, sleep_ms, 10, "Time to sleep between pings");

namespace aos {

namespace chrono = std::chrono;

Ping::Ping(EventLoop *event_loop)
    : event_loop_(event_loop),
      sender_(event_loop_->MakeSender<examples::PingStatic>("/test")) {
  timer_handle_ = event_loop_->AddTimer([this]() { SendPing(); });
  timer_handle_->set_name("ping");

  event_loop_->MakeWatcher(
      "/test", [this](const examples::Pong &pong) { HandlePong(pong); });

  event_loop_->OnRun([this]() {
    timer_handle_->Schedule(
        event_loop_->monotonic_now(),
        chrono::milliseconds(absl::GetFlag(FLAGS_sleep_ms)));
  });
}

void Ping::SendPing() {
  ++count_;
  aos::Sender<examples::PingStatic>::StaticBuilder builder =
      sender_.MakeStaticBuilder();
  builder->set_value(count_);
  builder->set_send_time(
      event_loop_->monotonic_now().time_since_epoch().count());
  builder.CheckOk(builder.Send());
}

void Ping::HandlePong(const examples::Pong &pong) {
  const aos::monotonic_clock::time_point monotonic_send_time(
      chrono::nanoseconds(pong.initial_send_time()));
  const aos::monotonic_clock::time_point monotonic_now =
      event_loop_->monotonic_now();

  const chrono::nanoseconds round_trip_time =
      monotonic_now - monotonic_send_time;

  if (pong.value() == count_) {
    ABSL_LOG(INFO) << "Elapsed time " << round_trip_time.count() << " ns "
                   << FlatbufferToJson(&pong);
  }
}

}  // namespace aos
```

Stepping over some of the more relevant code:

In `ping_lib.h`:

```cpp
  #include "foo/ping_static.h"
  #include "foo/pong_static.h"
```

These are the generated headers for each of the FlatBuffer messages that we defined. For a `.fbs` file named `foo.fbs`, the resulting static API header will be named `foo_static.h`. It includes `foo_generated.h`, so both the `Foo` (reading) and `FooStatic` (building) classes are available.

```cpp
  Ping(EventLoop *event_loop);
```

By convention, the top-level class for an AOS application will take, as the first argument to its constructor, an `aos::EventLoop*`. The [EventLoop](reference.md#eventloop-interface) will be used for all the application's interactions with the rest of the AOS system.

In `ping_lib.cc`:

```cpp
ABSL_FLAG(int32_t, sleep_ms, 10, "Time to sleep between pings");
```

This creates a command-line flag `--sleep_ms` which can be accessed in the code as `absl::GetFlag(FLAGS_sleep_ms)`, using the [Abseil flags](https://abseil.io/docs/cpp/guides/flags) library. Abseil flags is the default command-line flag management library used throughout AOS, although for testability we generally discourage using flags for configuration of normal application behavior.

```cpp
      sender_(event_loop_->MakeSender<examples::PingStatic>("/test")) {
```

Here we create a sender for the `aos.examples.Ping` message on the `/test` channel. Note that the sender is templated on `examples::PingStatic` rather than `examples::Ping`; this is what lets us build messages with the static API below. Senders need to be created before the EventLoop starts running. The `MakeSender` call will check for whether the requested channel actually exists in the AOS configuration, and die if no such channel is available. There is a `TryMakeSender` available for those situations where it makes sense to gate logic on the existence of a channel.

```cpp
  timer_handle_ = event_loop_->AddTimer([this]() { SendPing(); });
  timer_handle_->set_name("ping");
```

This creates a timer (and gives it a name, which is optional but helpful for debugging). Timers in AOS call callbacks (in this case a lambda that calls `SendPing();`) at requested times/intervals per the `Schedule` call (which is done below).

```cpp
  event_loop_->MakeWatcher(
      "/test", [this](const examples::Pong &pong) { HandlePong(pong); });
```

This sets up the `HandlePong()` method to be called every time a new message is received on the `/test` `aos.examples.Pong` channel.

```cpp
  event_loop_->OnRun([this]() {
    timer_handle_->Schedule(
        event_loop_->monotonic_now(),
        chrono::milliseconds(absl::GetFlag(FLAGS_sleep_ms)));
  });
```

This makes it so that, once execution starts on the EventLoop, the timer (which calls `SendPing`) will start getting called at the current time (`event_loop_->monotonic_now()`) and every `--sleep_ms` milliseconds thereafter.

`OnRun` can be called many times to register many handlers to be called when the `EventLoop` starts. The `OnRun` handler allows you to delay certain actions until execution is "actually" happening. In many real situations, there is little practical difference between just doing all your setup in the constructor of your class vs. in the `OnRun`. However, in situations where you do long-running initialization work in your constructor (e.g., you need to pre-seed a complex solver), it can be desirable to delay certain work until the EventLoop actually starts running. There are also some subtle differences in guarantees provided by the EventLoop when it is running vs. when it is not.

<!-- TODO: Link to explain those "subtle differences" -->

```cpp
void Ping::SendPing() {
  ++count_;
  aos::Sender<examples::PingStatic>::StaticBuilder builder =
      sender_.MakeStaticBuilder();
  builder->set_value(count_);
  builder->set_send_time(
      event_loop_->monotonic_now().time_since_epoch().count());
  builder.CheckOk(builder.Send());
}
```

Here we actually send out our Ping message. `MakeStaticBuilder` claims a buffer in the channel's shared memory and returns a `StaticBuilder` which constructs the `PingStatic` table in place. We use `->` on the builder to access the table, set the count and current time on it, and then actually send it out, checking to ensure that there were no errors encountered while attempting to send the message. Unlike the standard FlatBuffers API, fields can be set in any order and changed again before sending.

See [FlatBuffers](flatbuffers.md) for more details on working with FlatBuffers in AOS.

```cpp
void Ping::HandlePong(const examples::Pong &pong) {
  const aos::monotonic_clock::time_point monotonic_send_time(
      chrono::nanoseconds(pong.initial_send_time()));
  const aos::monotonic_clock::time_point monotonic_now =
      event_loop_->monotonic_now();

  const chrono::nanoseconds round_trip_time =
      monotonic_now - monotonic_send_time;

  if (pong.value() == count_) {
    ABSL_LOG(INFO) << "Elapsed time " << round_trip_time.count() << " ns "
                   << FlatbufferToJson(&pong);
  }
}
```

Here we handle each incoming Pong message, calculate the round-trip time and print it out only if the Pong message actually corresponds to our most recent Ping. Note that `ABSL_LOG(INFO)` is [Abseil logging](https://abseil.io/docs/cpp/guides/logging), which AOS uses throughout.

Finally, in order to build this we need a `cc_library` entry in our BUILD file:

```python
cc_library(
    name = "ping_lib",
    srcs = ["ping_lib.cc"],
    hdrs = ["ping_lib.h"],
    deps = [
        ":ping_fbs",
        ":pong_fbs",
        "@aos//aos:json_to_flatbuffer",
        "@aos//aos/events:event_loop",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/log:absl_log",
    ],
)
```

### Writing Tests for the Ping application

Because we are good Software Engineers, we must now write tests for the application we just created. In AOS, because of how the EventLoop interface is designed, we can readily create a simulated environment where all events and clocks progress deterministically.

To test the `Ping` class, we can write a `ping_lib_test.cc` using [gtest](http://google.github.io/googletest/) that looks like:

```cpp
#include "foo/ping_lib.h"

#include "aos/events/simulated_event_loop.h"
#include "aos/testing/path.h"
#include "gtest/gtest.h"

namespace aos::testing {

namespace chrono = std::chrono;

class PingPongTest : public ::testing::Test {
 public:
  PingPongTest()
      // Using ArtifactPath to account for the file structure in the Bazel test
      // environment, read the AOS config generated by aos_config bazel rule
      // (note how the name of this config file is identical to the name of the
      // aos_config rule, not to the name of the hand-written file.
      : config_(aos::configuration::ReadConfig(
            ArtifactPath("foo/pingpong_config.json"))),
        // Construct a SimulatedEventLoopFactory according to the config. This
        // factory will handle coordinating the simulation.
        event_loop_factory_(&config_.message()),
        // Actually create the EventLoop that the Ping application will use. We
        // pass in a name that should match the name that is used in the AOS
        // config (although doesn't strictly have to).
        ping_event_loop_(event_loop_factory_.MakeEventLoop("ping")),
        ping_(ping_event_loop_.get()),
        // Create a separate "test" EventLoop (the name doesn't typically
        // matter) which will be used for interacting with the Ping application.
        test_event_loop_(event_loop_factory_.MakeEventLoop("test")) {}

  // Config and factory.
  // The factory creates connected event loops.  Each application needs
  // a separate event loop (because you can't send a message to yourself in a
  // single event loop).  The created event loops can then send messages to each
  // other and trigger callbacks to be called, or fetchers to receive data.
  aos::FlatbufferDetachedBuffer<aos::Configuration> config_;
  SimulatedEventLoopFactory event_loop_factory_;

  // Event loop and application for Ping.
  std::unique_ptr<EventLoop> ping_event_loop_;
  Ping ping_;

  // Event loop for running our tests.
  std::unique_ptr<EventLoop> test_event_loop_;
};

// Tests that the ping application sends a bunch of pings with unique counters.
TEST_F(PingPongTest, SendsPings) {
  // We keep this test relatively straightforward--we just check that the Ping
  // application is sending out Ping messages with counters that continually
  // increment by one. In order to do this, we use the test EventLoop that we
  // craeted to create a Watcher on the appropriate channel and check each Ping
  // message as it comes in.
  int last_count = 0;
  test_event_loop_->MakeWatcher(
      "/test", [&last_count](const aos::examples::Ping &ping) {
        EXPECT_EQ(ping.value(), last_count + 1)
            << "Count was expected to increment by exactly 1.";
        last_count = ping.value();
      });
  // RunFor runs all the EventLoop's in the factory simulataneously,
  // coordinating events and messages on all the channels.
  event_loop_factory_.RunFor(chrono::seconds(10));
  // At the end, ensure that we actually got Ping messages--if we don't test
  // this, then we could end up never receiving *any* messages and the rest of
  // the test would pass.
  EXPECT_LT(0, last_count) << "Expected at least 1 Ping message.";
}
}  // namespace aos::testing
```

Let us know if having the comments explaining the code inline works better or worse than having it broken out into its own text like the previous code snippets.

And then, in order to actually execute the test, we need to create a `cc_test` target in our BUILD file:

```python
cc_test(
    name = "ping_lib_test",
    srcs = ["ping_lib_test.cc"],
    data = [":pingpong_config.json"],
    deps = [
        ":ping_lib",
        "@aos//aos/events:simulated_event_loop",
        "@aos//aos/testing:googletest",
        "@aos//aos/testing:path",
    ],
)
```

Note that for files that are used by the test when it is executing (namely, the config file), we need to add them to the `data` entry of the test rule rather than to the `deps` (which are the C++ dependencies needed by the compiler).

We can then run the test by doing

```bash
bazel test //foo:ping_lib_test
```

### Writing the Ping main()

In order to create a `ping` binary that can actually be run on a real system, we will create a `ping.cc`:

```cpp
#include "aos/configuration.h"
#include "foo/ping_lib.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "absl/flags/flag.h"

// Provide a --config flag that can be used to point to the config that the
// application will use. Generally defaulted to the actual path that will be
// used on the real system (most applications default to a name of
// aos_config.json, by convention).
ABSL_FLAG(std::string, config, "foo/pingpong_config.json", "Path to the config.");

int main(int argc, char **argv) {
  // Various common initialization steps, including command line flag parsing.
  aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  // Create a shared-memory based EventLoop using the provided config.
  // This is currently the only EventLoop implementation for using on realtime
  // systems.
  aos::ShmEventLoop event_loop(&config.message());

  aos::Ping ping(&event_loop);

  // Actually run the EventLoop. This will block until event_loop.Exit() is
  // called or we receive a signal to exit (e.g., a Ctrl-C on the command line).
  event_loop.Run();

  return 0;
}
```

Essentially all of this code is boilerplate that you will see in most binaries. There are sometimes minor variations, but ideally as much code as possible is in the actual library code where it can be tested.

We will also need a `cc_binary` bazel target to build:

```python
cc_binary(
    name = "ping",
    srcs = ["ping.cc"],
    data = [":pingpong_config"],
    deps = [
        ":ping_lib",
        "@aos//aos:configuration",
        "@aos//aos:init",
        "@aos//aos/events:shm_event_loop",
        "@com_google_absl//absl/flags:flag",
    ],
)
```

Note that we add a `data` dependency on the AOS config. This makes it so that when we do `bazel run //foo:ping`, bazel will ensure that the AOS config is built and available to the `ping` binary at runtime.

### Defining the Pong application

We will not go over Pong in as much detail, and instead just provided the suggested contents of the `pong_lib.h`, `pong_lib.cc`, and `pong.cc`:

`pong_lib.h`:

```cpp
#ifndef FOO_PONG_LIB_H_
#define FOO_PONG_LIB_H_

#include "aos/events/event_loop.h"
#include "foo/ping_static.h"
#include "foo/pong_static.h"

namespace aos {

// Class which replies to a Ping message with a Pong message immediately.
class Pong {
 public:
  Pong(EventLoop *event_loop);

 private:
  EventLoop *event_loop_;
  aos::Sender<examples::PongStatic> sender_;
  int32_t last_value_ = 0;
  int32_t last_send_time_ = 0;
};

}  // namespace aos

#endif  // FOO_PONG_LIB_H_
```

`pong_lib.cc`:

```cpp
#include "foo/pong_lib.h"

#include "aos/events/event_loop.h"

namespace aos {

Pong::Pong(EventLoop *event_loop)
    : event_loop_(event_loop),
      sender_(event_loop_->MakeSender<examples::PongStatic>("/test")) {
  event_loop_->MakeWatcher("/test", [this](const examples::Ping &ping) {
    last_value_ = ping.value();
    last_send_time_ = ping.send_time();
    aos::Sender<examples::PongStatic>::StaticBuilder builder =
        sender_.MakeStaticBuilder();
    builder->set_value(ping.value());
    builder->set_initial_send_time(ping.send_time());
    builder.CheckOk(builder.Send());
  });
}

}  // namespace aos
```

`pong.cc`:

Should be identical to `ping.cc`, except for including a using `pong_lib.h` and `Pong` instead of `ping_lib.h` and `Ping`. Making these changes is left as an exercise to the reader.

And the following BUILD file entries should be created:

```python
cc_library(
    name = "pong_lib",
    srcs = ["pong_lib.cc"],
    hdrs = ["pong_lib.h"],
    deps = [
        ":ping_fbs",
        ":pong_fbs",
        "@aos//aos/events:event_loop",
    ],
)

cc_binary(
    name = "pong",
    srcs = ["pong.cc"],
    data = [":pingpong_config"],
    deps = [
        ":pong_lib",
        "@aos//aos:configuration",
        "@aos//aos:init",
        "@aos//aos/events:shm_event_loop",
        "@com_google_absl//absl/flags:flag",
    ],
)
```

### Running our applications "for real"

We are now in a state where we have all the pieces we need to actually run the ping and pong applications. We just need to get everything coordinated to actually run them. Note that in a real system, you would typically have some build rules and scripts that manage deploying everything---here, we will just manually build and run things to get them working.

Let us open three separate terminals, and in the first two run:

1. `bazel run //foo:ping`
2. `bazel run //foo:pong`

We should immediately see the `ping` terminal printing to `stderr` for every pong message it is receiving. If we want to actually see the message flow itself, we can use the `aos_dump` utility to view raw messages on individual channels in the third terminal:

```bash
# Build the config so that aos_dump can read it.
bazel build //foo:pingpong_config
bazel run @aos//aos:aos_dump -- --config=$PWD/bazel-bin/foo/pingpong_config.json
```

`aos_dump` with no extra arguments (beyond the config, which we only need because we are in a special case) will print a list of available channels. Copy a channel name and type from that list to watch it:

```
bazel run @aos//aos:aos_dump -- --config=$PWD/bazel-bin/foo/pingpong_config.json /test aos.examples.Ping
```

And you now have a simple, running, AOS application! There are lots of things that this hasn't covered, but hopefully it has provided a basic overview of how to get things working.

## Realtime System Configuration

In order to be able to run all the AOS tests, as well as to be able to work with running realtime code locally (or on a test target), you should add the following lines to `/etc/security/limits.d/rt.conf`, replacing "USERNAME" with the username you're running under. You'll probably need to do this as root, e.g., `sudo nano /etc/security/limits.d/rt.conf`

```
USERNAME - nice -20
USERNAME - rtprio 95
USERNAME - memlock unlimited
```

If you prefer to just run a single command, then

```
echo -e "${USER} - nice -20\n${USER} - rtprio 95\n${USER} - memlock unlimited" | sudo tee /etc/security/limits.d/rt.conf
```

should also work.

In order for these changes to take effect, you will need to reboot your machine (strictly speaking, you only need a new session, which can be done by, e.g., logging out and logging back in, or by creating a new ssh session). Confirm that the limits have taken effect using `ulimit -a`.

[github]: https://github.com/RealtimeRoboticsGroup/aos
[reflection_fbs]: https://github.com/google/flatbuffers/blob/master/reflection/reflection.fbs

[^paths]: Note that `//` refers to the root of the current repository in Bazel. In some places, we will also use this syntax to refer to paths in the repository, to be clear that we are referencing a path relative to the root of the repository.
