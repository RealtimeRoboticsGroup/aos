#include "y2023/vision/aprilrobotics.h"

#include "absl/flags/flag.h"
#include "opencv2/highgui.hpp"

#include "y2023/vision/vision_util.h"

ABSL_FLAG(
    bool, debug, false,
    "If true, dump a ton of debug and crash on the first valid detection.");

ABSL_FLAG(double, min_decision_margin, 50.0,
          "Minimum decision margin (confidence) for an apriltag detection");
ABSL_FLAG(int32_t, pixel_border, 10,
          "Size of image border within which to reject detected corners");
ABSL_FLAG(
    double, max_expected_distortion, 0.314,
    "Maximum expected value for unscaled distortion factors. Will scale "
    "distortion factors so that this value (and a higher distortion) maps to "
    "1.0.");
ABSL_FLAG(uint64_t, pose_estimation_iterations, 50,
          "Number of iterations for apriltag pose estimation.");

namespace y2023::vision {

namespace chrono = std::chrono;

// Set max age on image for processing at 20 ms.  For 60Hz, we should be
// processing at least every 16.7ms
constexpr aos::monotonic_clock::duration kMaxImageAge =
    std::chrono::milliseconds(20);

AprilRoboticsDetector::AprilRoboticsDetector(aos::EventLoop *event_loop,
                                             std::string_view channel_name,
                                             bool flip_image)
    : calibration_data_(event_loop),
      image_size_(0, 0),
      flip_image_(flip_image),
      node_name_(event_loop->node()->name()->string_view()),
      ftrace_(),
      image_callback_(
          event_loop, channel_name,
          [this](cv::Mat image_color_mat,
                 const aos::monotonic_clock::time_point eof) {
            HandleImage(image_color_mat, eof);
          },
          kMaxImageAge),
      target_map_sender_(
          event_loop->MakeSender<frc971::vision::TargetMap>("/camera")),
      image_annotations_sender_(
          event_loop->MakeSender<foxglove::ImageAnnotations>("/camera")),
      rejections_(0) {
  tag_family_ = tag16h5_create();
  tag_detector_ = apriltag_detector_create();

  apriltag_detector_add_family_bits(tag_detector_, tag_family_, 1);
  tag_detector_->nthreads = 6;
  tag_detector_->wp = workerpool_create(tag_detector_->nthreads);
  tag_detector_->qtp.min_white_black_diff = 5;
  tag_detector_->debug = absl::GetFlag(FLAGS_debug);

  std::string hostname = aos::network::GetHostname();

  // Check team string is valid
  calibration_ = FindCameraCalibration(
      calibration_data_.constants(), event_loop->node()->name()->string_view());

  extrinsics_ = CameraExtrinsics(calibration_);
  intrinsics_ = CameraIntrinsics(calibration_);
  // Create an undistort projection matrix using the intrinsics
  projection_matrix_ = cv::Mat::zeros(3, 4, CV_64F);
  intrinsics_.rowRange(0, 3).colRange(0, 3).copyTo(
      projection_matrix_.rowRange(0, 3).colRange(0, 3));

  dist_coeffs_ = CameraDistCoeffs(calibration_);

  image_callback_.set_format(frc971::vision::ImageCallback::Format::GRAYSCALE);
}

AprilRoboticsDetector::~AprilRoboticsDetector() {
  apriltag_detector_destroy(tag_detector_);
  free(tag_family_);
}

void AprilRoboticsDetector::SetWorkerpoolAffinities() {
  for (int i = 0; i < tag_detector_->wp->nthreads; i++) {
    cpu_set_t affinity;
    CPU_ZERO(&affinity);
    CPU_SET(i, &affinity);
    pthread_setaffinity_np(tag_detector_->wp->threads[i], sizeof(affinity),
                           &affinity);
    struct sched_param param;
    param.sched_priority = 20;
    int res = pthread_setschedparam(tag_detector_->wp->threads[i], SCHED_FIFO,
                                    &param);
    PCHECK(res == 0) << "Failed to set priority of threadpool threads";
  }
}

void AprilRoboticsDetector::HandleImage(cv::Mat image_grayscale,
                                        aos::monotonic_clock::time_point eof) {
  image_size_ = image_grayscale.size();

  DetectionResult result = DetectTags(image_grayscale, eof);

  auto builder = target_map_sender_.MakeBuilder();
  std::vector<flatbuffers::Offset<frc971::vision::TargetPoseFbs>> target_poses;
  for (auto &detection : result.detections) {
    auto *fbb = builder.fbb();
    auto pose = BuildTargetPose(detection, fbb);
    DestroyPose(&detection.pose);
    target_poses.emplace_back(pose);
  }
  const auto target_poses_offset = builder.fbb()->CreateVector(target_poses);
  auto target_map_builder = builder.MakeBuilder<frc971::vision::TargetMap>();
  target_map_builder.add_target_poses(target_poses_offset);
  target_map_builder.add_monotonic_timestamp_ns(eof.time_since_epoch().count());
  target_map_builder.add_rejections(result.rejections);
  builder.CheckOk(builder.Send(target_map_builder.Finish()));
}

flatbuffers::Offset<frc971::vision::TargetPoseFbs>
AprilRoboticsDetector::BuildTargetPose(const Detection &detection,
                                       flatbuffers::FlatBufferBuilder *fbb) {
  const auto T =
      Eigen::Translation3d(detection.pose.t->data[0], detection.pose.t->data[1],
                           detection.pose.t->data[2]);
  const auto position_offset =
      frc971::vision::CreatePosition(*fbb, T.x(), T.y(), T.z());

  // Aprilrobotics stores the rotation matrix in row-major order
  const auto orientation = Eigen::Quaterniond(
      Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(detection.pose.R->data));
  const auto orientation_offset = frc971::vision::CreateQuaternion(
      *fbb, orientation.w(), orientation.x(), orientation.y(), orientation.z());

  return frc971::vision::CreateTargetPoseFbs(
      *fbb, detection.det.id, position_offset, orientation_offset,
      detection.det.decision_margin, detection.pose_error,
      detection.distortion_factor, detection.pose_error_ratio);
}

void AprilRoboticsDetector::UndistortDetection(
    apriltag_detection_t *det) const {
  // 4 corners
  constexpr size_t kRows = 4;
  // 2d points
  constexpr size_t kCols = 2;

  cv::Mat distorted_points(kRows, kCols, CV_64F, det->p);
  cv::Mat undistorted_points = cv::Mat::zeros(kRows, kCols, CV_64F);

  // Undistort the april tag points
  cv::undistortPoints(distorted_points, undistorted_points, intrinsics_,
                      dist_coeffs_, cv::noArray(), projection_matrix_);

  // Copy the undistorted points into det
  for (size_t i = 0; i < kRows; i++) {
    for (size_t j = 0; j < kCols; j++) {
      det->p[i][j] = undistorted_points.at<double>(i, j);
    }
  }
}

double AprilRoboticsDetector::ComputeDistortionFactor(
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

std::vector<cv::Point2f> AprilRoboticsDetector::MakeCornerVector(
    const apriltag_detection_t *det) {
  std::vector<cv::Point2f> corner_points;
  corner_points.emplace_back(det->p[0][0], det->p[0][1]);
  corner_points.emplace_back(det->p[1][0], det->p[1][1]);
  corner_points.emplace_back(det->p[2][0], det->p[2][1]);
  corner_points.emplace_back(det->p[3][0], det->p[3][1]);

  return corner_points;
}

void AprilRoboticsDetector::DestroyPose(apriltag_pose_t *pose) const {
  matd_destroy(pose->R);
  matd_destroy(pose->t);
}

AprilRoboticsDetector::DetectionResult AprilRoboticsDetector::DetectTags(
    cv::Mat image, aos::monotonic_clock::time_point eof) {
  cv::Mat color_image;
  cvtColor(image, color_image, cv::COLOR_GRAY2RGB);
  const aos::monotonic_clock::time_point start_time =
      aos::monotonic_clock::now();

  image_u8_t im = {
      .width = image.cols,
      .height = image.rows,
      .stride = image.cols,
      .buf = image.data,
  };
  const uint32_t min_x = absl::GetFlag(FLAGS_pixel_border);
  const uint32_t max_x = image.cols - absl::GetFlag(FLAGS_pixel_border);
  const uint32_t min_y = absl::GetFlag(FLAGS_pixel_border);
  const uint32_t max_y = image.rows - absl::GetFlag(FLAGS_pixel_border);

  ftrace_.FormatMessage("Starting detect\n");
  zarray_t *detections = apriltag_detector_detect(tag_detector_, &im);
  ftrace_.FormatMessage("Done detecting\n");

  std::vector<Detection> results;

  auto builder = image_annotations_sender_.MakeBuilder();
  std::vector<flatbuffers::Offset<foxglove::PointsAnnotation>> foxglove_corners;

  for (int i = 0; i < zarray_size(detections); i++) {
    apriltag_detection_t *det;
    zarray_get(detections, i, &det);

    if (det->decision_margin > absl::GetFlag(FLAGS_min_decision_margin)) {
      if (det->p[0][0] < min_x || det->p[0][0] > max_x ||
          det->p[1][0] < min_x || det->p[1][0] > max_x ||
          det->p[2][0] < min_x || det->p[2][0] > max_x ||
          det->p[3][0] < min_x || det->p[3][0] > max_x ||
          det->p[0][1] < min_y || det->p[0][1] > max_y ||
          det->p[1][1] < min_y || det->p[1][1] > max_y ||
          det->p[2][1] < min_y || det->p[2][1] > max_y ||
          det->p[3][1] < min_y || det->p[3][1] > max_y) {
        VLOG(1) << "Rejecting detection because corner is outside pixel border";
        // Send rejected corner points in red
        std::vector<cv::Point2f> rejected_corner_points = MakeCornerVector(det);
        foxglove_corners.push_back(frc971::vision::BuildPointsAnnotation(
            builder.fbb(), eof, rejected_corner_points,
            std::vector<double>{1.0, 0.0, 0.0, 0.5}));
        continue;
      }
      VLOG(1) << "Found tag number " << det->id << " hamming: " << det->hamming
              << " margin: " << det->decision_margin;

      // First create an apriltag_detection_info_t struct using your known
      // parameters.
      apriltag_detection_info_t info;
      info.det = det;
      info.tagsize = 0.1524;

      info.fx = intrinsics_.at<double>(0, 0);
      info.fy = intrinsics_.at<double>(1, 1);
      info.cx = intrinsics_.at<double>(0, 2);
      info.cy = intrinsics_.at<double>(1, 2);

      // Send original corner points in green
      std::vector<cv::Point2f> orig_corner_points = MakeCornerVector(det);
      foxglove_corners.push_back(frc971::vision::BuildPointsAnnotation(
          builder.fbb(), eof, orig_corner_points,
          std::vector<double>{0.0, 1.0, 0.0, 0.5}));

      UndistortDetection(det);

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
      VLOG(1) << "Pose err 1: " << pose_error_1;
      VLOG(1) << "Pose err 2: " << pose_error_2;

      // Send undistorted corner points in pink
      std::vector<cv::Point2f> corner_points = MakeCornerVector(det);
      foxglove_corners.push_back(frc971::vision::BuildPointsAnnotation(
          builder.fbb(), eof, corner_points,
          std::vector<double>{1.0, 0.75, 0.8, 1.0}));

      double distortion_factor =
          ComputeDistortionFactor(orig_corner_points, corner_points);

      // We get two estimates for poses.
      // Choose the one with the lower estimation error
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

      results.emplace_back(Detection{.det = *det,
                                     .pose = best_pose,
                                     .pose_error = best_pose_error,
                                     .distortion_factor = distortion_factor,
                                     .pose_error_ratio = pose_error_ratio});

      if (absl::GetFlag(FLAGS_visualize)) {
        // Draw raw (distorted) corner points in green
        cv::line(color_image, orig_corner_points[0], orig_corner_points[1],
                 cv::Scalar(0, 255, 0), 2);
        cv::line(color_image, orig_corner_points[1], orig_corner_points[2],
                 cv::Scalar(0, 255, 0), 2);
        cv::line(color_image, orig_corner_points[2], orig_corner_points[3],
                 cv::Scalar(0, 255, 0), 2);
        cv::line(color_image, orig_corner_points[3], orig_corner_points[0],
                 cv::Scalar(0, 255, 0), 2);

        // Draw undistorted corner points in red
        cv::line(color_image, corner_points[0], corner_points[1],
                 cv::Scalar(0, 0, 255), 2);
        cv::line(color_image, corner_points[2], corner_points[1],
                 cv::Scalar(0, 0, 255), 2);
        cv::line(color_image, corner_points[2], corner_points[3],
                 cv::Scalar(0, 0, 255), 2);
        cv::line(color_image, corner_points[0], corner_points[3],
                 cv::Scalar(0, 0, 255), 2);
      }

      VLOG(1) << "Found tag number " << det->id << " hamming: " << det->hamming
              << " margin: " << det->decision_margin;

    } else {
      rejections_++;
    }
  }
  if (absl::GetFlag(FLAGS_visualize)) {
    // Display the result
    // Rotate by 180 degrees to make it upright
    if (flip_image_) {
      cv::rotate(color_image, color_image, 1);
    }
    cv::imshow(absl::StrCat("AprilRoboticsDetector Image ", node_name_),
               color_image);
  }

  const auto corners_offset = builder.fbb()->CreateVector(foxglove_corners);
  foxglove::ImageAnnotations::Builder annotation_builder(*builder.fbb());
  annotation_builder.add_points(corners_offset);
  builder.CheckOk(builder.Send(annotation_builder.Finish()));

  apriltag_detections_destroy(detections);

  const aos::monotonic_clock::time_point end_time = aos::monotonic_clock::now();

  if (absl::GetFlag(FLAGS_debug)) {
    timeprofile_display(tag_detector_->tp);
  }

  VLOG(1) << "Took " << chrono::duration<double>(end_time - start_time).count()
          << " seconds to detect overall";

  return {.detections = results, .rejections = rejections_};
}

}  // namespace y2023::vision
