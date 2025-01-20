#ifndef FRC_VISION_TARGET_MAP_UTILS_H_
#define FRC_VISION_TARGET_MAP_UTILS_H_

#include <Eigen/Dense>

#include "frc/vision/target_map_generated.h"

namespace frc::vision {
// Converts a TargetPoseFbs into a transformation matrix.
Eigen::Matrix<double, 4, 4> PoseToTransform(
    const frc::vision::TargetPoseFbs *pose);
}  // namespace frc::vision

#endif  // FRC_VISION_TARGET_MAP_UTILS_H_
