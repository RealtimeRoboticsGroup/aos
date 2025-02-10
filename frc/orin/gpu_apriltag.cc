#include "frc/orin/gpu_apriltag.h"

#include <chrono>

#include "absl/flags/flag.h"
#include "opencv2/highgui.hpp"
#include "third_party/apriltag/apriltag.h"
#include "third_party/apriltag/apriltag_pose.h"
#include "third_party/apriltag/tag16h5.h"
#include "third_party/apriltag/tag36h11.h"

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/logging/logging.h"
#include "aos/realtime.h"
#include "frc/constants/constants_sender_lib.h"
#include "frc/orin/apriltag.h"
#include "frc/vision/calibration_generated.h"
#include "frc/vision/charuco_lib.h"
#include "frc/vision/vision_util_lib.h"

ABSL_FLAG(bool, debug, false, "If true, write debug images.");
ABSL_FLAG(
    double, max_expected_distortion, 0.314,
    "Maximum expected value for unscaled distortion factors. Will scale "
    "distortion factors so that this value (and a higher distortion) maps to "
    "1.0.");
ABSL_FLAG(double, min_decision_margin, 50.0,
          "Minimum decision margin (confidence) for an apriltag detection");
ABSL_FLAG(int32_t, pixel_border, 150,
          "Size of image border within which to reject detected corners");
ABSL_FLAG(uint64_t, pose_estimation_iterations, 50,
          "Number of iterations for apriltag pose estimation.");
ABSL_FLAG(std::string, image_format, "YUYV422", "Image format to use.");

namespace frc::apriltag {

// Set max age on image for processing at 20 ms.  For 60Hz, we should be
// processing at least every 16.7ms
constexpr aos::monotonic_clock::duration kMaxImageAge =
    std::chrono::milliseconds(50);

namespace chrono = std::chrono;

vision::ImageFormat ImageFormatFromString(std::string_view format) {
  size_t i = 0;
  while (true) {
    if (vision::EnumNamesImageFormat()[i] == nullptr) {
      LOG(FATAL) << "Invalid image format: " << format;
    }
    if (vision::EnumNamesImageFormat()[i] == format) {
      LOG(INFO) << "Using image format: " << format;
      return vision::EnumValuesImageFormat()[i];
    }
    ++i;
  }
}

ApriltagDetector::ApriltagDetector(
    aos::EventLoop *event_loop, std::string_view channel_name,
    const frc::vision::calibration::CameraCalibration *calibration,
    size_t width, size_t height)
    : tag_family_(tag36h11_create()),
      tag_detector_(MakeTagDetector(tag_family_)),
      node_name_(event_loop->node()->name()->string_view()),
      calibration_(calibration),
      intrinsics_(frc::vision::CameraIntrinsics(calibration_)),
      extrinsics_(frc::vision::CameraExtrinsics(calibration_)),
      dist_coeffs_(frc::vision::CameraDistCoeffs(calibration_)),
      distortion_camera_matrix_(GetCameraMatrix(calibration_)),
      distortion_coefficients_(GetDistCoeffs(calibration_)),
      image_format_(ImageFormatFromString(absl::GetFlag(FLAGS_image_format))),
      gpu_detector_(width, height, tag_detector_, distortion_camera_matrix_,
                    distortion_coefficients_, image_format_),
      image_callback_(
          event_loop, channel_name,
          [this](const vision::CameraImage &image,
                 const aos::monotonic_clock::time_point eof) {
            HandleImage(image, eof);
          },
          kMaxImageAge),
      target_map_sender_(
          event_loop->MakeSender<frc::vision::TargetMap>(channel_name)),
      image_annotations_sender_(
          event_loop->MakeSender<foxglove::ImageAnnotations>(channel_name)),
      rejections_(0) {
  projection_matrix_ = cv::Mat::zeros(3, 4, CV_64F);
  intrinsics_.rowRange(0, 3).colRange(0, 3).copyTo(
      projection_matrix_.rowRange(0, 3).colRange(0, 3));
}

ApriltagDetector::~ApriltagDetector() {
  apriltag_detector_destroy(tag_detector_);
  free(tag_family_);
}

apriltag_detector_t *ApriltagDetector::MakeTagDetector(
    apriltag_family_t *tag_family) {
  apriltag_detector_t *tag_detector = apriltag_detector_create();

  apriltag_detector_add_family_bits(tag_detector, tag_family, 1);

  tag_detector->nthreads = 6;
  tag_detector->wp = workerpool_create(tag_detector->nthreads);
  tag_detector->qtp.min_white_black_diff = 5;
  tag_detector->debug = absl::GetFlag(FLAGS_debug);

  return tag_detector;
}

flatbuffers::Offset<frc::vision::TargetPoseFbs>
ApriltagDetector::BuildTargetPose(const Detection &detection,
                                  flatbuffers::FlatBufferBuilder *fbb) {
  const auto T =
      Eigen::Translation3d(detection.pose.t->data[0], detection.pose.t->data[1],
                           detection.pose.t->data[2]);
  const auto position_offset =
      frc::vision::CreatePosition(*fbb, T.x(), T.y(), T.z());

  // Aprilrobotics stores the rotation matrix in row-major order
  const auto orientation = Eigen::Quaterniond(
      Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(detection.pose.R->data));
  const auto orientation_offset = frc::vision::CreateQuaternion(
      *fbb, orientation.w(), orientation.x(), orientation.y(), orientation.z());

  return frc::vision::CreateTargetPoseFbs(
      *fbb, detection.det.id, position_offset, orientation_offset,
      detection.det.decision_margin, detection.pose_error,
      detection.distortion_factor, detection.pose_error_ratio);
}

bool ApriltagDetector::UndistortDetection(apriltag_detection_t *det) const {
  // Copy the undistorted points into det
  bool converged = true;
  for (size_t i = 0; i < 4; i++) {
    double u = det->p[i][0];
    double v = det->p[i][1];

    converged &= GpuDetector::UnDistort(&u, &v, &distortion_camera_matrix_,
                                        &distortion_coefficients_);
    det->p[i][0] = u;
    det->p[i][1] = v;
  }
  return converged;
}

double ApriltagDetector::ComputeDistortionFactor(
    const std::vector<cv::Point2f> &orig_corners,
    const std::vector<cv::Point2f> &corners) {
  CHECK_EQ(orig_corners.size(), 4ul);
  CHECK_EQ(corners.size(), 4ul);

  double avg_distance = 0.0;
  for (size_t i = 0; i < corners.size(); i++) {
    avg_distance += cv::norm(orig_corners[i] - corners[i]);
  }
  avg_distance /= corners.size();

  // Normalize avg_distance by dividing by the image diagonal,
  // and then the maximum expected distortion
  double distortion_factor =
      avg_distance /
      cv::norm(cv::Point2d(image_size_.width, image_size_.height));
  return std::min(
      distortion_factor / absl::GetFlag(FLAGS_max_expected_distortion), 1.0);
}

std::vector<cv::Point2f> ApriltagDetector::MakeCornerVector(
    const apriltag_detection_t *det) {
  std::vector<cv::Point2f> corner_points;
  corner_points.emplace_back(det->p[0][0], det->p[0][1]);
  corner_points.emplace_back(det->p[1][0], det->p[1][1]);
  corner_points.emplace_back(det->p[2][0], det->p[2][1]);
  corner_points.emplace_back(det->p[3][0], det->p[3][1]);

  return corner_points;
}

void ApriltagDetector::DestroyPose(apriltag_pose_t *pose) const {
  matd_destroy(pose->R);
  matd_destroy(pose->t);
}

void ApriltagDetector::HandleImage(const vision::CameraImage &image,
                                   aos::monotonic_clock::time_point eof) {
  const aos::monotonic_clock::time_point start_time =
      aos::monotonic_clock::now();

  CHECK(image.format() == image_format_)
      << ": Image format doesn't match --image_format="
      << absl::GetFlag(FLAGS_image_format);
  CHECK(image.has_data());
  CHECK_EQ(gpu_detector_.width(), static_cast<size_t>(image.cols()));
  CHECK_EQ(gpu_detector_.height(), static_cast<size_t>(image.rows()));
  uint8_t *image_device = nullptr;
  CHECK_CUDA(cudaHostGetDevicePointer(
      &image_device,
      static_cast<void *>(const_cast<uint8_t *>(image.data()->data())), 0));
  CHECK_NE(image_device, nullptr);

  gpu_detector_.Detect(image.data()->data(), image_device);
  image_size_ = cv::Size(image.cols(), image.rows());

  const zarray_t *detections = gpu_detector_.Detections();

  aos::monotonic_clock::time_point end_time = aos::monotonic_clock::now();

  const uint32_t min_x = absl::GetFlag(FLAGS_pixel_border);
  const uint32_t max_x = image.cols() - absl::GetFlag(FLAGS_pixel_border);
  const uint32_t min_y = absl::GetFlag(FLAGS_pixel_border);
  const uint32_t max_y = image.rows() - absl::GetFlag(FLAGS_pixel_border);

  // Define variables for storing / visualizing the output
  std::vector<Detection> results;
  auto builder = image_annotations_sender_.MakeBuilder();
  std::vector<flatbuffers::Offset<foxglove::PointsAnnotation>> foxglove_corners;

  for (int i = 0; i < zarray_size(detections); ++i) {
    apriltag_detection_t *gpu_detection;

    zarray_get(detections, i, &gpu_detection);

    bool valid = gpu_detection->decision_margin >
                 absl::GetFlag(FLAGS_min_decision_margin);

    if (valid) {
      // Reject tags that are too close to the boundary, since they often
      // lead to corrupt matches since part of the tag is cut off
      if (gpu_detection->p[0][0] < min_x || gpu_detection->p[0][0] > max_x ||
          gpu_detection->p[1][0] < min_x || gpu_detection->p[1][0] > max_x ||
          gpu_detection->p[2][0] < min_x || gpu_detection->p[2][0] > max_x ||
          gpu_detection->p[3][0] < min_x || gpu_detection->p[3][0] > max_x ||
          gpu_detection->p[0][1] < min_y || gpu_detection->p[0][1] > max_y ||
          gpu_detection->p[1][1] < min_y || gpu_detection->p[1][1] > max_y ||
          gpu_detection->p[2][1] < min_y || gpu_detection->p[2][1] > max_y ||
          gpu_detection->p[3][1] < min_y || gpu_detection->p[3][1] > max_y) {
        VLOG(1) << "Rejecting detection because corner is outside pixel border";

        // Send rejected corner points to foxglove in red
        std::vector<cv::Point2f> rejected_corner_points =
            MakeCornerVector(gpu_detection);
        foxglove_corners.push_back(frc::vision::BuildPointsAnnotation(
            builder.fbb(), eof, rejected_corner_points,
            std::vector<double>{1.0, 0.0, 0.0, 0.5}));
        rejections_++;
        continue;
      }

      AOS_LOG(INFO,
              "Found GPU %s tag number %d hamming %d margin %f  (%f, %f), (%f, "
              "%f), (%f, %f), (%f, %f) in %f ms\n",
              valid ? "valid" : "invalid", gpu_detection->id,
              gpu_detection->hamming, gpu_detection->decision_margin,
              gpu_detection->p[0][0], gpu_detection->p[0][1],
              gpu_detection->p[1][0], gpu_detection->p[1][1],
              gpu_detection->p[2][0], gpu_detection->p[2][1],
              gpu_detection->p[3][0], gpu_detection->p[3][1],
              std::chrono::duration<float, std::milli>(end_time - start_time)
                  .count());

      VLOG(1) << "Found tag number " << gpu_detection->id
              << " hamming: " << gpu_detection->hamming
              << " margin: " << gpu_detection->decision_margin;

      // First create an apriltag_detection_info_t struct using your known
      // parameters.
      apriltag_detection_info_t info;
      info.tagsize = 6.5 * 0.0254;

      info.fx = intrinsics_.at<float>(0, 0);
      info.fy = intrinsics_.at<float>(1, 1);
      info.cx = intrinsics_.at<float>(0, 2);
      info.cy = intrinsics_.at<float>(1, 2);

      // Send original corner points in green
      std::vector<cv::Point2f> orig_corner_points =
          MakeCornerVector(gpu_detection);
      foxglove_corners.push_back(frc::vision::BuildPointsAnnotation(
          builder.fbb(), eof, orig_corner_points,
          std::vector<double>{0.0, 1.0, 0.0, 0.5}));

      bool converged = UndistortDetection(gpu_detection);

      if (!converged) {
        VLOG(1) << "Rejecting detection because Undistort failed to coverge";

        // Send corner points rejected to to lack of convergence in orange
        std::vector<cv::Point2f> rejected_corner_points =
            MakeCornerVector(gpu_detection);
        foxglove_corners.push_back(frc::vision::BuildPointsAnnotation(
            builder.fbb(), eof, rejected_corner_points,
            std::vector<double>{1.0, 0.65, 0.0, 0.5}));
        rejections_++;
        continue;
      }

      // We're setting this here to use the undistorted corner points in pose
      // estimation.
      info.det = gpu_detection;

      const aos::monotonic_clock::time_point before_pose_estimation =
          aos::monotonic_clock::now();

      apriltag_pose_t pose_1;
      apriltag_pose_t pose_2;
      double pose_error_1;
      double pose_error_2;
      estimate_tag_pose_orthogonal_iteration(
          &info, &pose_error_1, &pose_1, &pose_error_2, &pose_2,
          absl::GetFlag(FLAGS_pose_estimation_iterations));

      const aos::monotonic_clock::time_point after_pose_estimation =
          aos::monotonic_clock::now();
      VLOG(1) << "Took "
              << chrono::duration<double>(after_pose_estimation -
                                          before_pose_estimation)
                     .count()
              << " seconds for pose estimation";
      VLOG(1) << "Pose err 1: " << std::setprecision(20) << std::fixed
              << pose_error_1 << " " << (pose_error_1 < 1e-6 ? "Good" : "Bad");
      VLOG(1) << "Pose err 2: " << std::setprecision(20) << std::fixed
              << pose_error_2 << " " << (pose_error_2 < 1e-6 ? "Good" : "Bad");

      // Send undistorted corner points in pink
      std::vector<cv::Point2f> corner_points = MakeCornerVector(gpu_detection);
      foxglove_corners.push_back(frc::vision::BuildPointsAnnotation(
          builder.fbb(), eof, corner_points,
          std::vector<double>{1.0, 0.75, 0.8, 1.0}));

      double distortion_factor =
          ComputeDistortionFactor(orig_corner_points, corner_points);

      // We get two estimates for poses.
      // Choose the one with the lower pose estimation error
      bool use_pose_1 = (pose_error_1 < pose_error_2);
      auto best_pose = (use_pose_1 ? pose_1 : pose_2);
      auto secondary_pose = (use_pose_1 ? pose_2 : pose_1);
      double best_pose_error = (use_pose_1 ? pose_error_1 : pose_error_2);
      double secondary_pose_error = (use_pose_1 ? pose_error_2 : pose_error_1);

      CHECK_NE(best_pose_error, std::numeric_limits<double>::infinity())
          << "Got no valid pose estimations, this should not be possible.";
      double pose_error_ratio = best_pose_error / secondary_pose_error;

      // Destroy the secondary pose if we got one
      if (secondary_pose_error != std::numeric_limits<double>::infinity()) {
        DestroyPose(&secondary_pose);
      }

      results.emplace_back(Detection{.det = *gpu_detection,
                                     .pose = best_pose,
                                     .pose_error = best_pose_error,
                                     .distortion_factor = distortion_factor,
                                     .pose_error_ratio = pose_error_ratio});

      VLOG(1) << "Found tag number " << gpu_detection->id
              << " hamming: " << gpu_detection->hamming
              << " margin: " << gpu_detection->decision_margin;
    } else {
      rejections_++;
    }
  }

  const auto corners_offset = builder.fbb()->CreateVector(foxglove_corners);
  foxglove::ImageAnnotations::Builder annotation_builder(*builder.fbb());
  annotation_builder.add_points(corners_offset);
  builder.CheckOk(builder.Send(annotation_builder.Finish()));

  auto map_builder = target_map_sender_.MakeBuilder();
  std::vector<flatbuffers::Offset<frc::vision::TargetPoseFbs>> target_poses;
  for (auto &detection : results) {
    auto *fbb = map_builder.fbb();
    auto pose = BuildTargetPose(detection, fbb);
    DestroyPose(&detection.pose);
    target_poses.emplace_back(pose);
  }
  const auto target_poses_offset =
      map_builder.fbb()->CreateVector(target_poses);
  auto target_map_builder = map_builder.MakeBuilder<frc::vision::TargetMap>();
  target_map_builder.add_target_poses(target_poses_offset);
  target_map_builder.add_monotonic_timestamp_ns(eof.time_since_epoch().count());
  target_map_builder.add_rejections(rejections_);
  map_builder.CheckOk(map_builder.Send(target_map_builder.Finish()));

  // TODO: Do we need to clean this up?
  // apriltag_detections_destroy(detections);

  end_time = aos::monotonic_clock::now();

  if (absl::GetFlag(FLAGS_debug)) {
    timeprofile_display(tag_detector_->tp);
  }

  VLOG(2) << "Took " << chrono::duration<double>(end_time - start_time).count()
          << " seconds to detect overall";
}

}  // namespace frc::apriltag
