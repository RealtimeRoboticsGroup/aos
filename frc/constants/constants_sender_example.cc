#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"

#include "aos/configuration.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "frc/constants/constants_sender_lib.h"
#include "frc/constants/testdata/constants_data_generated.h"
#include "frc/constants/testdata/constants_list_generated.h"

ABSL_FLAG(std::string, config, "frc/constants/testdata/aos_config.json",
          "Path to the config.");
ABSL_FLAG(std::string, constants_path,
          "frc/constants/testdata/test_constants.json",
          "Path to the constant file");
// This is just a sample binary
int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));
  aos::ShmEventLoop event_loop(&config.message());
  frc::constants::ConstantSender<frc::constants::testdata::ConstantsData,
                                 frc::constants::testdata::ConstantsList>
      constants_sender(&event_loop, absl::GetFlag(FLAGS_constants_path));
  event_loop.Run();

  return 0;
}
