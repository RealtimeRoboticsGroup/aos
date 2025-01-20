#include "absl/flags/flag.h"

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frc/imu_fdcan/can_translator_lib.h"

ABSL_FLAG(std::string, channel, "/can", "The CAN channel to use");

using frc::imu_fdcan::CANTranslator;

int main(int argc, char **argv) {
  ::aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("aos_config.json");

  ::aos::ShmEventLoop event_loop(&config.message());

  CANTranslator translator(&event_loop, absl::GetFlag(FLAGS_channel));

  event_loop.Run();

  return 0;
}
