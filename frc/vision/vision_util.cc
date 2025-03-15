#include "frc/vision/vision_util.h"

#include "absl/log/check.h"
#include "absl/log/log.h"

namespace frc::vision {

const frc::vision::calibration::CameraCalibration *FindCameraCalibration(
    const frc::vision::CameraConstants &calibration_data,
    std::string_view node_name, int camera_number) {
  CHECK(calibration_data.has_calibration());
  for (const frc::vision::calibration::CameraCalibration *candidate :
       *calibration_data.calibration()) {
    if (candidate->node_name()->string_view() != node_name ||
        candidate->camera_number() != camera_number) {
      continue;
    }
    return candidate;
  }
  LOG(FATAL) << ": Failed to find camera calibration for " << node_name
             << " and camera number " << camera_number;
}

}  // namespace frc::vision
