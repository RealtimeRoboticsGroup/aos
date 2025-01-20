#ifndef FRC_VISION_CALIBRATION_ACCUMULATOR_H_
#define FRC_VISION_CALIBRATION_ACCUMULATOR_H_

#include <vector>

#include "Eigen/Dense"

#include "aos/events/simulated_event_loop.h"
#include "aos/time/time.h"
#include "frc/control_loops/quaternion_utils.h"
#include "frc/vision/charuco_lib.h"
#include "frc/vision/foxglove_image_converter_lib.h"
#include "frc/wpilib/imu_batch_generated.h"

namespace frc::vision {

// This class provides an interface for an application to be notified of all
// camera and IMU samples in order with the correct timestamps.
class CalibrationDataObserver {
 public:
  // Observes a camera sample at the corresponding time t, and with the
  // corresponding rotation and translation vectors rt.
  virtual void UpdateCamera(aos::distributed_clock::time_point t,
                            std::pair<Eigen::Vector3d, Eigen::Vector3d> rt) = 0;

  // Observes an IMU sample at the corresponding time t, and with the
  // corresponding angular velocity and linear acceleration vectors wa.
  virtual void UpdateIMU(aos::distributed_clock::time_point t,
                         std::pair<Eigen::Vector3d, Eigen::Vector3d> wa) = 0;

  // Observes a turret sample at the corresponding time t, and with the
  // corresponding state.
  virtual void UpdateTurret(aos::distributed_clock::time_point t,
                            Eigen::Vector2d state) = 0;
};

// Class to both accumulate and replay camera and IMU data in time order.
class CalibrationData {
 public:
  // Adds a camera/charuco detection to the list at the provided time.
  // This has only been tested with a charuco board.
  void AddCameraPose(aos::distributed_clock::time_point distributed_now,
                     Eigen::Vector3d rvec, Eigen::Vector3d tvec);

  // Adds an IMU point to the list at the provided time.
  void AddImu(aos::distributed_clock::time_point distributed_now,
              Eigen::Vector3d gyro, Eigen::Vector3d accel);

  // Adds a turret reading (position; velocity) to the list at the provided
  // time.
  void AddTurret(aos::distributed_clock::time_point distributed_now,
                 Eigen::Vector2d state);

  // Processes the data points by calling UpdateCamera and UpdateIMU in time
  // order.
  void ReviewData(CalibrationDataObserver *observer) const;

  size_t camera_samples_size() const { return rot_trans_points_.size(); }

  size_t imu_samples_size() const { return imu_points_.size(); }

  size_t turret_samples_size() const { return turret_points_.size(); }

 private:
  std::vector<std::pair<aos::distributed_clock::time_point,
                        std::pair<Eigen::Vector3d, Eigen::Vector3d>>>
      imu_points_;

  // Store pose samples as timestamp, along with
  // pair of rotation, translation vectors
  std::vector<std::pair<aos::distributed_clock::time_point,
                        std::pair<Eigen::Vector3d, Eigen::Vector3d>>>
      rot_trans_points_;

  // Turret state as a timestamp and [x, v].
  std::vector<std::pair<aos::distributed_clock::time_point, Eigen::Vector2d>>
      turret_points_;
};

class CalibrationFoxgloveVisualizer {
 public:
  CalibrationFoxgloveVisualizer(aos::EventLoop *event_loop,
                                std::string_view camera_channel = "/camera");

  static aos::FlatbufferDetachedBuffer<aos::Configuration>
  AddVisualizationChannels(const aos::Configuration *config,
                           const aos::Node *node);

  void HandleCharuco(const aos::monotonic_clock::time_point eof,
                     std::vector<std::vector<cv::Point2f>> charuco_corners) {
    auto builder = annotations_sender_.MakeBuilder();
    builder.CheckOk(builder.Send(
        BuildAnnotations(builder.fbb(), eof, charuco_corners,
                         std::vector<double>{0.0, 1.0, 0.0, 1.0}, 2.0)));
  }

 private:
  aos::EventLoop *event_loop_;
  FoxgloveImageConverter image_converter_;

  aos::Sender<foxglove::ImageAnnotations> annotations_sender_;
};

// Class to register image and IMU callbacks in AOS and route them to the
// corresponding CalibrationData class.
class Calibration {
 public:
  Calibration(aos::SimulatedEventLoopFactory *event_loop_factory,
              aos::EventLoop *image_event_loop, aos::EventLoop *imu_event_loop,
              std::string_view hostname,
              const calibration::CameraCalibration *intrinsics_calibration,
              TargetType target_type, std::string_view image_channel,
              CalibrationData *data);

  // Processes a charuco detection that is returned from charuco_lib.
  // For a valid detection(s), it stores camera observation
  // Also optionally displays and saves annotated images based on visualize and
  // save_path flags, respectively
  void HandleCharuco(cv::Mat rgb_image,
                     const aos::monotonic_clock::time_point eof,
                     std::vector<cv::Vec4i> /*charuco_ids*/,
                     std::vector<std::vector<cv::Point2f>> /*charuco_corners*/,
                     bool valid, std::vector<Eigen::Vector3d> rvecs_eigen,
                     std::vector<Eigen::Vector3d> tvecs_eigen);

  // Processes an IMU reading by storing for later processing
  void HandleIMU(const frc::IMUValues *imu);

 private:
  aos::EventLoop *image_event_loop_;
  aos::NodeEventLoopFactory *image_factory_;
  aos::EventLoop *imu_event_loop_;
  aos::NodeEventLoopFactory *imu_factory_;

  CharucoExtractor charuco_extractor_;
  ImageCallback image_callback_;

  CalibrationData *data_;

  std::unique_ptr<aos::EventLoop> visualizer_event_loop_;
  CalibrationFoxgloveVisualizer visualizer_;

  frc::IMUValuesT last_value_;
};

}  // namespace frc::vision

#endif  // FRC_VISION_CALIBRATION_ACCUMULATOR_H_
