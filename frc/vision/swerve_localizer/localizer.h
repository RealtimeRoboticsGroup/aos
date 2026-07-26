#ifndef FRC_VISION_SWERVE_LOCALIZER_LOCALIZER_H_
#define FRC_VISION_SWERVE_LOCALIZER_LOCALIZER_H_

#include <array>
#include <map>

#include "frc/constants/constants_sender_lib.h"
#include "frc/control_loops/drivetrain/localization/localizer_output_generated.h"
#include "frc/control_loops/drivetrain/localization/utils.h"
#include "frc/control_loops/drivetrain/localizer_generated.h"
#include "frc/vision/camera_constants_generated.h"
#include "frc/vision/swerve_localizer/chassis_speeds_static.h"
#include "frc/vision/swerve_localizer/hybrid_ekf.h"
#include "frc/vision/swerve_localizer/pose2d_static.h"
#include "frc/vision/swerve_localizer/status_generated.h"
#include "frc/vision/swerve_localizer/visualization_static.h"
#include "frc/vision/target_map_generated.h"

namespace frc::vision::swerve_localizer {

class Localizer {
 public:
  static constexpr size_t kNumCameras = 4;
  using Pose = frc::control_loops::Pose;
  typedef Eigen::Matrix<double, 4, 4> Transform;
  typedef frc::vision::swerve_localizer::HybridEkf<double> HybridEkf;
  typedef HybridEkf::State State;
  typedef HybridEkf::Output Output;
  typedef HybridEkf::Input Input;
  typedef HybridEkf::StateIdx StateIdx;
  typedef HybridEkf::InputIdx InputIdx;
  Localizer(aos::EventLoop *event_loop);

 private:
  class Corrector : public HybridEkf::ExpectedObservationFunctor {
   public:
    // Indices used for each of the members of the output vector for this
    // Corrector.
    enum OutputIdx {
      kHeading = 0,
      kDistance = 1,
      kSkew = 2,
    };
    Corrector(const State &state_at_capture, const double heading,
              const Transform &H_field_target, const Transform &H_robot_camera,
              const Transform &H_camera_target);

    using HMatrix = Eigen::Matrix<double, Localizer::HybridEkf::kNOutputs,
                                  Localizer::HybridEkf::kNStates>;

    Output H(const State &, const Input &) final;
    HMatrix DHDX(const State &) final { return H_; }
    const Eigen::Vector3d &observed() const { return observed_; }
    const Eigen::Vector3d &expected() const { return expected_; }
    const Pose &expected_robot_pose() const { return expected_robot_pose_; }
    const Pose &expected_rio_heading_camera() const {
      return expected_rio_heading_camera_;
    }
    const Pose &expected_camera_pose() const { return expected_camera_; }
    const Pose &observed_camera_pose() const { return observed_camera_; }

    static Eigen::Vector3d HeadingDistanceSkew(const Pose &relative_pose);

    static Corrector CalculateHeadingDistanceSkewH(
        const State &state_at_capture, const double rio_heading,
        const Transform &H_field_target, const Transform &H_robot_camera,
        const Transform &H_camera_target);

    static void PopulateMeasurement(const Eigen::Vector3d &vector,
                                    MeasurementStatic *builder) {
      builder->set_heading(vector(kHeading));
      builder->set_distance(vector(kDistance));
      builder->set_skew(vector(kSkew));
    }

   private:
    Corrector(const Pose &expected_robot_pose, const Pose &observed_camera,
              const Pose &expected_camera,
              const Pose &expected_rio_heading_camera,
              const Eigen::Vector3d &expected, const Eigen::Vector3d &observed,
              const HMatrix &H)
        : expected_robot_pose_(expected_robot_pose),
          observed_camera_(observed_camera),
          expected_camera_(expected_camera),
          expected_rio_heading_camera_(expected_rio_heading_camera),
          expected_(expected),
          observed_(observed),
          H_(H) {}
    // For debugging.
    const Pose expected_robot_pose_;
    const Pose observed_camera_;
    const Pose expected_camera_;
    const Pose expected_rio_heading_camera_;
    // Actually used.
    const Eigen::Vector3d expected_;
    const Eigen::Vector3d observed_;
    const HMatrix H_;
  };

  // A corrector that just does x/y/theta based corrections rather than doing
  // heading/distance/skew corrections.
  class XyzCorrector : public HybridEkf::ExpectedObservationFunctor {
   public:
    // Indices used for each of the members of the output vector for this
    // Corrector.
    enum OutputIdx {
      kX = 0,
      kY = 1,
      kTheta = 2,
    };
    XyzCorrector(const State &state_at_capture, const Eigen::Vector3d &Z)
        : state_at_capture_(state_at_capture), Z_(Z) {
      H_.setZero();
      H_(kX, StateIdx::kX) = 1;
      H_(kY, StateIdx::kY) = 1;
      H_(kTheta, StateIdx::kTheta) = 1;
    }
    Output H(const State &, const Input &) final;
    Eigen::Matrix<double, HybridEkf::kNOutputs, HybridEkf::kNStates> DHDX(
        const State &) final {
      return H_;
    }

   private:
    Eigen::Matrix<double, HybridEkf::kNOutputs, HybridEkf::kNStates> H_;
    const State state_at_capture_;
    const Eigen::Vector3d &Z_;
  };

  struct CameraState {
    aos::Sender<VisualizationStatic> debug_sender;
    Transform extrinsics = Transform::Zero();
    aos::util::ArrayErrorCounter<RejectionReason, RejectionCount>
        rejection_counter;
    size_t total_candidate_targets = 0;
    size_t total_accepted_targets = 0;
  };

  // Returns true if we should use a lower weight for the specified april tag.
  // This is used for tags where we do not trust the placement as much.
  bool DeweightAprilTag(uint64_t target_id);
  static std::array<CameraState, kNumCameras> MakeCameras(
      const CameraConstants &constants, aos::EventLoop *event_loop);
  void HandleTarget(int camera_index,
                    const aos::monotonic_clock::time_point capture_time,
                    const frc::vision::TargetPoseFbs &target,
                    TargetEstimateDebugStatic *debug_builder);
  void HandleChassisSpeeds(
      const aos::monotonic_clock::time_point sample_time_orin,
      const ChassisSpeeds &speeds);
  void RejectImage(int camera_index, RejectionReason reason,
                   TargetEstimateDebugStatic *builder);

  void SendOutput();
  static flatbuffers::Offset<LocalizerState> PopulateState(
      const State &X_hat, flatbuffers::FlatBufferBuilder *fbb);
  void SendStatus();
  static flatbuffers::Offset<CumulativeStatistics> StatisticsForCamera(
      const CameraState &camera, flatbuffers::FlatBufferBuilder *fbb);
  static void StatisticsForCamera(const CameraState &camera,
                                  CumulativeStatisticsStatic *builder);

  bool UseAprilTag(uint64_t target_id);
  void HandleControl(
      const frc::control_loops::drivetrain::LocalizerControl &msg);

  aos::EventLoop *const event_loop_;
  // Need:
  //   Camera calibration (intrinsics and extrinsics)
  //   target_map:frc.vision.TargetMap (id: 0);
  frc::constants::ConstantsFetcher<CameraConstants> constants_fetcher_;
  frc::constants::ConstantsFetcher<TargetMap> target_map_fetcher_;
  std::array<CameraState, kNumCameras> cameras_;
  const std::map<uint64_t, Transform> target_poses_;

  HybridEkf ekf_;
  HybridEkf::ExpectedObservationAllocator<Corrector> observations_;
  HybridEkf::ExpectedObservationAllocator<XyzCorrector> xyz_observations_;

  frc::control_loops::drivetrain::LocalizationUtils utils_;

  aos::Sender<Status> status_sender_;
  aos::Sender<frc::controls::LocalizerOutput> output_sender_;
  aos::monotonic_clock::time_point t_ = aos::monotonic_clock::min_time;

  size_t total_candidate_targets_ = 0;
  size_t total_accepted_targets_ = 0;
  size_t heading_resets_ = 0;
  size_t total_chassis_speeds_ = 0;

  double prev_absolute_velocity_x_ = 0;
  double prev_absolute_velocity_y_ = 0;
  double prev_absolute_velocity_omega_ = 0;
  aos::monotonic_clock::time_point prev_timestamp_s_ =
      aos::monotonic_clock::min_time;

  // For the status message.
  std::optional<Eigen::Vector2d> last_encoder_readings_;

  aos::Fetcher<frc::control_loops::drivetrain::LocalizerControl>
      control_fetcher_;
  aos::Fetcher<frc::vision::swerve_localizer::Pose2d> roborio_pose_fetcher_;

  double roborio_pose_last_theta_ = 0;

  Eigen::Vector3d average_pose_ = Eigen::Vector3d::Zero();
};

}  // namespace frc::vision::swerve_localizer

#endif  // FRC_VISION_SWERVE_LOCALIZER_LOCALIZER_H_
