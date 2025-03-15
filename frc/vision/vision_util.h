#ifndef FRC_VISION_VISION_UTIL_H_
#define FRC_VISION_VISION_UTIL_H_

#include <map>
#include <string_view>

#include "opencv2/imgproc.hpp"

#include "frc/vision/camera_constants_generated.h"

namespace frc::vision {

// Generate unique colors for each camera
const auto kOrinColors = std::map<std::string, cv::Scalar>{
    {"/camera0/gray", cv::Scalar(255, 0, 255)},
    {"/camera1/gray", cv::Scalar(255, 255, 0)},
    {"/camera2/gray", cv::Scalar(0, 255, 255)},
    {"/camera3/gray", cv::Scalar(255, 165, 0)},
};

const frc::vision::calibration::CameraCalibration *FindCameraCalibration(
    const frc::vision::CameraConstants &calibration_data,
    std::string_view node_name, int camera_number);

}  // namespace frc::vision

#endif  // FRC_VISION_VISION_UTIL_H_
