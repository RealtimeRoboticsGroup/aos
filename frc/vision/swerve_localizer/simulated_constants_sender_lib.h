#ifndef FRC_VISION_SWERVE_LOCALIZER_SIMULATED_CONSTANTS_SENDER_LIB_H_
#define FRC_VISION_SWERVE_LOCALIZER_SIMULATED_CONSTANTS_SENDER_LIB_H_

#include "frc/vision/swerve_localizer/field_map_constants_sender_lib.h"
#include "aos/events/simulated_event_loop.h"
#include "frc/vision/camera_constants_generated.h"
#include "frc/vision/camera_constants_list_generated.h"

namespace frc::vision::swerve_localizer {

class SimulatedConstantsSender {
 public:
  SimulatedConstantsSender(aos::SimulatedEventLoopFactory *factory, int team,
                           const std::string &constants_path) {
    for (const aos::Node *node : factory->nodes()) {
      std::unique_ptr<aos::EventLoop> event_loop =
          factory->MakeEventLoop("constants_sender", node);
      frc::constants::NameConstantSender<CameraConstants, CameraConstantsList>
          sender(event_loop.get(), constants_path, team, "blackbeard", "/constants");
    }
  }
};

class SimulatedFieldMapSender {
 public:
  SimulatedFieldMapSender(aos::SimulatedEventLoopFactory *factory,
                          std::string_view field_map_json_path) {
    aos::FlatbufferDetachedBuffer<FieldMap> field_map =
        aos::JsonFileToFlatbuffer<FieldMap>(field_map_json_path);

    for (const aos::Node *node : factory->nodes()) {
      std::unique_ptr<aos::EventLoop> event_loop =
          factory->MakeEventLoop("field_constants_sender", node);
      SendFieldMap(event_loop.get(), &field_map.message(), "simulated_map");
    }
  }
};

}  // namespace frc::vision::swerve_localizer

#endif  // FRC_VISION_SWERVE_LOCALIZER_SIMULATED_CONSTANTS_SENDER_LIB_H_
