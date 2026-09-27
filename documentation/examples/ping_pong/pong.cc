#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/log.h"

#include "aos/configuration.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "documentation/examples/ping_pong/ping_generated.h"
#include "documentation/examples/ping_pong/pong_generated.h"
#include "documentation/examples/ping_pong/pong_lib.h"

ABSL_FLAG(std::string, config, "pingpong_config.json", "Path to the config.");

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  aos::EventLoop::SetDefaultVersionString("pong_version");

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  ::aos::ShmEventLoop event_loop(&config.message());

  aos::Pong ping(&event_loop);

  event_loop.Run();

  return 0;
}
