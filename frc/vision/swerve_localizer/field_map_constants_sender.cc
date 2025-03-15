#include "Eigen/Core"
#include "Eigen/Geometry"
#include "absl/flags/flag.h"

#include "aos/configuration.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "frc/vision/field_map_generated.h"
#include "frc/vision/swerve_localizer/field_map_constants_sender_lib.h"
#include "frc/vision/target_map_static.h"

ABSL_FLAG(std::string, config, "aos_config.json", "Path to the AOS config.");
ABSL_FLAG(std::string, field_map_json, "frc2025r2.fmap",
          "Path to the constant file");

namespace frc::vision::swerve_localizer {

int Main() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));
  aos::ShmEventLoop event_loop(&config.message());

  // Convert the field map JSON from limelight's format to our FieldMap format.
  aos::FlatbufferDetachedBuffer<FieldMap> field_map =
      aos::JsonFileToFlatbuffer<FieldMap>(absl::GetFlag(FLAGS_field_map_json));

  SendFieldMap(&event_loop, &field_map.message(),
               absl::GetFlag(FLAGS_field_map_json));

  // Don't need to call Run(), just exit.
  return 0;
}

}  // namespace frc::vision::swerve_localizer

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  return frc::vision::swerve_localizer::Main();
}
