#ifndef FRC_VISION_SWERVE_LOCALIZER_FIELD_MAP_CONSTANTS_SENDER_LIB_H_
#define FRC_VISION_SWERVE_LOCALIZER_FIELD_MAP_CONSTANTS_SENDER_LIB_H_

#include <string_view>

#include "aos/events/event_loop.h"
#include "frc/vision/field_map_generated.h"

namespace frc::vision::swerve_localizer {

void SendFieldMap(aos::EventLoop *event_loop, const FieldMap *field_map,
                  std::string_view field_name);

}  // namespace frc::vision::swerve_localizer

#endif  // FRC_VISION_SWERVE_LOCALIZER_FIELD_MAP_CONSTANTS_SENDER_LIB_H_
