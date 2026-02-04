#include <chrono>
#include <memory>

#include "absl/flags/flag.h"

#include "aos/configuration.h"
#include "aos/events/event_loop.h"
#include "aos/events/logging/log_writer.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/flatbuffers.h"
#include "aos/init.h"
#include "aos/logging/log_message_generated.h"
#include "aos/testing/path.h"
#include "aos/testing/ping_pong/ping_lib.h"
#include "aos/testing/ping_pong/pong_lib.h"

ABSL_FLAG(std::string, output_folder, "",
          "Name of folder to write the generated logfile to.");

namespace {

// Inject an arbitrary LogMessageFbs message into the log by sending one out on
// the corresponding AOS channel.
void InjectLogMessage(aos::EventLoop *event_loop) {
  aos::Sender<aos::logging::LogMessageFbs> sender =
      event_loop->MakeSender<aos::logging::LogMessageFbs>("/aos");
  aos::Sender<aos::logging::LogMessageFbs>::Builder builder =
      sender.MakeBuilder();

  flatbuffers::Offset<flatbuffers::String> name_offset =
      builder.fbb()->CreateString("test");
  flatbuffers::Offset<flatbuffers::String> message_offset =
      builder.fbb()->CreateString("test message");

  aos::logging::LogMessageFbsBuilder log_message_builder =
      builder.MakeBuilder<aos::logging::LogMessageFbs>();

  log_message_builder.add_message(message_offset);
  log_message_builder.add_source_pid(12345);
  log_message_builder.add_level(aos::logging::Level::INFO);
  log_message_builder.add_name(name_offset);
  log_message_builder.add_send_failures(0);

  builder.CheckOk(builder.Send(log_message_builder.Finish()));
}

}  // namespace

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  const aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(aos::testing::ArtifactPath(
          "aos/testing/ping_pong/pingpong_config.json"));

  aos::SimulatedEventLoopFactory event_loop_factory(&config.message());

  // Inject a message before startup. The times here are somewhat arbitrary. We
  // want to simulate a message very early after boot. Then the log starts at 10
  // seconds after boot.
  event_loop_factory.RunFor(std::chrono::milliseconds(500));
  std::unique_ptr<aos::EventLoop> injector_event_loop =
      event_loop_factory.MakeEventLoop("injector");
  InjectLogMessage(injector_event_loop.get());
  event_loop_factory.RunFor(std::chrono::milliseconds(9500));

  // Event loop and app for Ping
  std::unique_ptr<aos::EventLoop> ping_event_loop =
      event_loop_factory.MakeEventLoop("ping");
  aos::Ping ping(ping_event_loop.get());

  // Event loop and app for Pong
  std::unique_ptr<aos::EventLoop> pong_event_loop =
      event_loop_factory.MakeEventLoop("pong");
  aos::Pong pong(pong_event_loop.get());

  std::unique_ptr<aos::EventLoop> log_writer_event_loop =
      event_loop_factory.MakeEventLoop("log_writer");
  aos::logger::Logger writer(log_writer_event_loop.get());
  writer.StartLoggingOnRun(absl::GetFlag(FLAGS_output_folder));

  event_loop_factory.RunFor(std::chrono::seconds(10));
  return 0;
}
