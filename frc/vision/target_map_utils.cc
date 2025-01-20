#include "frc/vision/target_map_utils.h"

namespace frc::vision {
Eigen::Matrix<double, 4, 4> PoseToTransform(
    const frc::vision::TargetPoseFbs *pose) {
  const frc::vision::Position *position = pose->position();
  const frc::vision::Quaternion *quaternion = pose->orientation();
  return (Eigen::Translation3d(
              Eigen::Vector3d(position->x(), position->y(), position->z())) *
          Eigen::Quaterniond(quaternion->w(), quaternion->x(), quaternion->y(),
                             quaternion->z())
              .normalized())
      .matrix();
}
}  // namespace frc::vision
