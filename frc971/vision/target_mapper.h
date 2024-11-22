#ifndef FRC971_VISION_TARGET_MAPPER_H_
#define FRC971_VISION_TARGET_MAPPER_H_

#include <unordered_map>

#include "absl/strings/str_format.h"
#include "ceres/ceres.h"

#include "aos/events/simulated_event_loop.h"
#include "frc971/vision/ceres/types.h"
#include "frc971/vision/target_map_generated.h"
#include "frc971/vision/vision_util_lib.h"
#include "frc971/vision/visualize_robot.h"

ABSL_DECLARE_FLAG(int32_t, min_target_id);
ABSL_DECLARE_FLAG(int32_t, max_target_id);
ABSL_DECLARE_FLAG(double, outlier_std_devs);

namespace frc971::vision {

// Estimates positions of vision targets (ex. April Tags) using
// target detections relative to a robot (which were computed using robot
// positions at the time of those detections). Solves SLAM problem to estimate
// target locations using deltas between consecutive target detections.
class TargetMapper {
 public:
  using TargetId = int;
  using ConfidenceMatrix = Eigen::Matrix<double, 6, 6>;

  struct TargetPose {
    TargetId id;
    ceres::examples::Pose3d pose;
  };

  // target_poses_path is the path to a TargetMap json with initial guesses for
  // the actual locations of the targets on the field.
  // target_constraints are the deltas between consecutive target detections,
  // and are usually prepared by the DataAdapter class below.
  TargetMapper(std::string_view target_poses_path,
               const ceres::examples::VectorOfConstraints &target_constraints);
  // Alternate constructor for tests.
  // Takes in the actual intial guesses instead of a file containing them
  TargetMapper(const ceres::examples::MapOfPoses &target_poses,
               const ceres::examples::VectorOfConstraints &target_constraints);

  // Solves for the target map. If output_dir is set, the map will be saved to
  // output_dir/field_name.json
  void Solve(std::string_view field_name,
             std::optional<std::string_view> output_dir = std::nullopt);

  // Prints target poses into a TargetMap flatbuffer json
  std::string MapToJson(std::string_view field_name) const;

  // Builds a TargetPoseFbs from a TargetPose
  static flatbuffers::Offset<TargetPoseFbs> TargetPoseToFbs(
      const TargetMapper::TargetPose &target_pose,
      flatbuffers::FlatBufferBuilder *fbb);

  // Converts a TargetPoseFbs to a TargetPose
  static TargetMapper::TargetPose TargetPoseFromFbs(
      const TargetPoseFbs &target_pose_fbs);

  static std::optional<TargetPose> GetTargetPoseById(
      std::vector<TargetPose> target_poses, TargetId target_id);

  // Version that gets based on internal target_poses
  std::optional<TargetPose> GetTargetPoseById(TargetId target_id) const;

  ceres::examples::MapOfPoses target_poses() { return target_poses_; }

  // Cost function for the secondary solver finding out where the whole map fits
  // in the world
  template <typename S>
  bool operator()(const S *const translation, const S *const rotation,
                  S *residual) const;

  void DumpConstraints(std::string_view path) const;
  void DumpStats(std::string_view path) const;
  void PrintDiffs() const;

 private:
  // Error in an estimated pose
  struct PoseError {
    double angle;
    double distance;
  };

  // Stores info on how much all the constraints differ from our solved target
  // map
  struct Stats {
    // Average error for translation and rotation
    PoseError avg_err;
    // Standard deviation for translation and rotation error
    PoseError std_dev;
    // Maximum error for translation and rotation
    PoseError max_err;
  };

  // Compute the error of a single constraint
  PoseError ComputeError(const ceres::examples::Constraint3d &constraint) const;
  // Compute cumulative stats for all constraints
  Stats ComputeStats() const;
  // Removes constraints with very large errors
  void RemoveOutlierConstraints();

  void CountConstraints();

  // Constructs the nonlinear least squares optimization problem from the
  // pose graph constraints.
  void BuildTargetPoseOptimizationProblem(
      const ceres::examples::VectorOfConstraints &constraints,
      ceres::examples::MapOfPoses *poses, ceres::Problem *problem);

  // Constructs the nonlinear least squares optimization problem for the solved
  // -> actual pose solver.
  std::unique_ptr<ceres::CostFunction> BuildMapFittingOptimizationProblem(
      ceres::Problem *problem);

  // Create and display a visualization of the graph connectivity of the
  // constraints
  void DisplayConstraintGraph();

  // Create and display a visualization of the map solution (vs. the input map)
  void DisplaySolvedVsInitial();

  // Returns true if the solve was successful.
  bool SolveOptimizationProblem(ceres::Problem *problem);

  ceres::examples::MapOfPoses ideal_target_poses_;
  ceres::examples::MapOfPoses target_poses_;
  ceres::examples::VectorOfConstraints target_constraints_;

  // Counts of each pair of target ids we observe, so we can scale cost based on
  // the inverse of this and remove bias towards certain pairs
  std::map<std::pair<TargetId, TargetId>, size_t> constraint_counts_;

  // Transformation moving the target map we solved for to where it actually
  // should be in the world
  Eigen::Translation3d T_frozen_actual_;
  Eigen::Quaterniond R_frozen_actual_;

  const double kFieldWidth_ = 20.0;  // 20 meters across
  const int kImageWidth_ = 1000;
  const int kImageHeight_ =
      kImageWidth_ * 3.0 / 4.0;  // Roughly matches field aspect ratio
  mutable VisualizeRobot vis_robot_;

  Stats stats_with_outliers_;
};

// Transforms robot position and target detection data into target constraints
// to be used for mapping.
class DataAdapter {
 public:
  // Pairs target detection with a time point
  struct TimestampedDetection {
    aos::distributed_clock::time_point time;
    // Pose of target relative to robot
    Eigen::Affine3d H_robot_target;
    // Horizontal distance from camera to target, used for confidence
    // calculation
    double distance_from_camera;
    // A measure of how much distortion affected this detection from 0-1.
    double distortion_factor;
    TargetMapper::TargetId id;
  };

  // Pairs consecutive target detections that are not too far apart in time into
  // constraints. Meant to be used on a system without a position measurement.
  // Assumes timestamped_target_detections is in chronological order.
  // max_dt is the maximum time between two target detections to match them up.
  // If too much time passes, the recoding device (box of pis) could have moved
  // too much
  static ceres::examples::VectorOfConstraints MatchTargetDetections(
      const std::vector<TimestampedDetection> &timestamped_target_detections,
      aos::distributed_clock::duration max_dt = std::chrono::milliseconds(10));

  // Computes inverse of covariance matrix, assuming there was a target
  // detection between robot movement over the given time period. Ceres calls
  // this matrix the "information"
  static TargetMapper::ConfidenceMatrix ComputeConfidence(
      const TimestampedDetection &detection_start,
      const TimestampedDetection &detection_end);

  // Computes the constraint between the start and end pose of the targets: the
  // relative pose between the start and end target locations in the frame of
  // the start target.
  static ceres::examples::Constraint3d ComputeTargetConstraint(
      const TimestampedDetection &target_detection_start,
      const TimestampedDetection &target_detection_end,
      const TargetMapper::ConfidenceMatrix &confidence);
};

}  // namespace frc971::vision

namespace ceres::examples {
template <typename Sink>
void AbslStringify(Sink &sink, ceres::examples::Pose3d pose) {
  auto rpy = frc971::vision::PoseUtils::QuaternionToEulerAngles(pose.q);
  absl::Format(&sink,
               "{x: %.3f, y: %.3f, z: %.3f, roll: %.3f, pitch: "
               "%.3f, yaw: %.3f}",
               pose.p(0), pose.p(1), pose.p(2), rpy(0), rpy(1), rpy(2));
}

template <typename Sink>
void AbslStringify(Sink &sink, ceres::examples::Constraint3d constraint) {
  absl::Format(&sink, "{id_begin: %d, id_end: %d, pose: ", constraint.id_begin,
               constraint.id_end);
  AbslStringify(sink, constraint.t_be);
  sink.Append("}");
}
}  // namespace ceres::examples

#endif  // FRC971_VISION_TARGET_MAPPER_H_
