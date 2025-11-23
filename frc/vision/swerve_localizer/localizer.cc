#include "frc/vision/swerve_localizer/localizer.h"

#include "absl/flags/flag.h"

#include "aos/containers/sized_array.h"
#include "frc/control_loops/drivetrain/localizer_generated.h"
#include "frc/control_loops/pose.h"
#include "frc/math/flatbuffers_matrix.h"
#include "frc/vision/target_map_utils.h"

ABSL_FLAG(double, max_pose_error, 1e-5,
          "Throw out target poses with a higher pose error than this");
ABSL_FLAG(double, max_distortion, 1000.0, "");
ABSL_FLAG(double, max_pose_error_ratio, 0.4,
          "Throw out target poses with a higher pose error ratio than this");
ABSL_FLAG(double, distortion_noise_scalar, 4.0,
          "Scale the target pose distortion factor by this when computing "
          "the noise.");
ABSL_FLAG(
    double, max_implied_yaw_error, 5.0,
    "Reject target poses that imply a robot yaw of more than this many degrees "
    "off from our estimate.");
ABSL_FLAG(
    double, max_implied_teleop_yaw_error, 30.0,
    "Reject target poses that imply a robot yaw of more than this many degrees "
    "off from our estimate.");
ABSL_FLAG(double, max_distance_to_target, 5.0,
          "Reject target poses that have a 3d distance of more than this "
          "many meters.");
ABSL_FLAG(double, max_auto_image_robot_speed, 5.0,
          "Reject target poses when the robot is travelling faster than "
          "this speed in auto.");
ABSL_FLAG(
    bool, do_xytheta_corrections, false,
    "If set, uses the x/y/theta corrector rather than a heading/distance/skew "
    "one. This is better conditioned currently, but is theoretically worse due "
    "to not capturing noise effectively.");
ABSL_FLAG(bool, always_use_extra_tags, true,
          "If set, we will use the \"deweighted\" tags even in auto mode (this "
          "affects april tags whose field positions we do not trust as much).");

namespace frc::vision::swerve_localizer {

namespace {
constexpr std::array<std::string_view, Localizer::kNumCameras>
    kDetectionChannels{"/camera0/gray", "/camera1/gray", "/camera2/gray",
                       "/camera3/gray"};

size_t CameraIndexForName(std::string_view name) {
  for (size_t index = 0; index < kDetectionChannels.size(); ++index) {
    if (name == kDetectionChannels.at(index)) {
      return index;
    }
  }
  LOG(FATAL) << "No camera channel named " << name;
}

std::map<uint64_t, Localizer::Transform> GetTargetLocations(
    const TargetMap &constants) {
  CHECK(constants.has_target_poses());
  std::map<uint64_t, Localizer::Transform> transforms;
  for (const frc::vision::TargetPoseFbs *target : *constants.target_poses()) {
    CHECK(target->has_id());
    CHECK(target->has_position());
    CHECK(target->has_orientation());
    CHECK_EQ(0u, transforms.count(target->id()));
    transforms[target->id()] = PoseToTransform(target);
  }
  return transforms;
}

// Returns the "nominal" covariance of localizer---i.e., the values to which it
// tends to converge during normal operation. By initializing the localizer's
// covariance this way, we reduce the likelihood that the first few corrections
// we receive will result in insane jumps in robot state.
Eigen::Matrix<double, Localizer::HybridEkf::kNStates,
              Localizer::HybridEkf::kNStates>
NominalCovariance() {
  Eigen::Matrix<double, Localizer::HybridEkf::kNStates,
                Localizer::HybridEkf::kNStates>
      P_transpose;
  // Grabbed from when the robot was in a steady-state.
  P_transpose << 1.0e-2, 0.0, 0.0, 0.0, 1.0e-2, 0.0, 0.0, 0.0, 1.0e-4;
  return P_transpose.transpose();
}
}  // namespace

std::array<Localizer::CameraState, Localizer::kNumCameras>
Localizer::MakeCameras(const CameraConstants &constants,
                       aos::EventLoop *event_loop) {
  CHECK(constants.has_calibration());
  std::array<Localizer::CameraState, Localizer::kNumCameras> cameras;
  for (const calibration::CameraCalibration *calibration :
       *constants.calibration()) {
    CHECK(!calibration->has_turret_extrinsics())
        << "The 2024 robot does not have cameras on a turret.";
    CHECK(calibration->has_node_name());
    const std::string channel_name =
        absl::StrFormat("/camera%d/gray", calibration->camera_number());
    const size_t index = CameraIndexForName(channel_name);
    // We default-construct the extrinsics matrix to all-zeros; use that to
    // sanity-check whether we have populated the matrix yet or not.
    CHECK(cameras.at(index).extrinsics.norm() == 0)
        << "Got multiple calibrations for "
        << calibration->node_name()->string_view();
    CHECK(calibration->has_fixed_extrinsics());
    cameras.at(index).extrinsics =
        frc::control_loops::drivetrain::FlatbufferToTransformationMatrix(
            *calibration->fixed_extrinsics());
    cameras.at(index).debug_sender =
        event_loop->MakeSender<VisualizationStatic>(channel_name);
  }
  for (const CameraState &camera : cameras) {
    CHECK(camera.extrinsics.norm() != 0) << "Missing a camera calibration.";
  }
  return cameras;
}

Localizer::Localizer(aos::EventLoop *event_loop)
    : event_loop_(event_loop),
      constants_fetcher_(event_loop),
      target_map_fetcher_(event_loop),
      cameras_(MakeCameras(constants_fetcher_.constants(), event_loop)),
      target_poses_(GetTargetLocations(target_map_fetcher_.constants())),
      // Force the dt to 1 ms (the nominal IMU frequency) since we have observed
      // issues with timing on the orins.
      // TODO(james): Ostensibly, we should be able to use the timestamps from
      // the IMU board itself for exactly this; however, I am currently worried
      // about the impacts of clock drift in using that.
      ekf_(),
      observations_(&ekf_),
      xyz_observations_(&ekf_),
      utils_(event_loop),
      status_sender_(event_loop->MakeSender<Status>("/localizer")),
      output_sender_(
          event_loop->MakeSender<frc::controls::LocalizerOutput>("/localizer")),
      control_fetcher_(
          event_loop_
              ->MakeFetcher<frc::control_loops::drivetrain::LocalizerControl>(
                  "/drivetrain")),
      roborio_pose_fetcher_(
          event_loop_->MakeFetcher<frc::vision::swerve_localizer::Pose2d>(
              "/drivetrain")) {
  for (size_t camera_index = 0; camera_index < kNumCameras; ++camera_index) {
    const std::string_view channel_name = kDetectionChannels.at(camera_index);
    const aos::Channel *const channel =
        event_loop->GetChannel<frc::vision::TargetMap>(channel_name);
    CHECK(channel != nullptr);
    event_loop->MakeWatcher(
        channel_name,
        [this, camera_index](const frc::vision::TargetMap &targets) {
          CHECK(targets.has_target_poses());
          CHECK(targets.has_monotonic_timestamp_ns());
          const aos::monotonic_clock::time_point orin_capture_time(
              std::chrono::nanoseconds(targets.monotonic_timestamp_ns()));
          if (orin_capture_time > event_loop_->context().monotonic_event_time) {
            VLOG(1) << "Rejecting image due to being from future at "
                    << event_loop_->monotonic_now() << " with timestamp of "
                    << orin_capture_time << " and event time pf "
                    << event_loop_->context().monotonic_event_time;
            cameras_.at(camera_index)
                .rejection_counter.IncrementError(
                    RejectionReason::IMAGE_FROM_FUTURE);
            return;
          }
          auto debug_builder =
              cameras_.at(camera_index).debug_sender.MakeStaticBuilder();
          auto target_debug_list = debug_builder->add_targets();
          // The static_length should already be 20.
          CHECK(target_debug_list->reserve(20));
          for (const frc::vision::TargetPoseFbs *target :
               *targets.target_poses()) {
            VLOG(1) << "Handling target from " << camera_index;
            HandleTarget(camera_index, orin_capture_time, *target,
                         target_debug_list->emplace_back());
          }
          StatisticsForCamera(cameras_.at(camera_index),
                              debug_builder->add_statistics());
          debug_builder.CheckOk(debug_builder.Send());
          SendStatus();
        });
  }

  event_loop_->AddPhasedLoop([this](int) { SendOutput(); },
                             std::chrono::milliseconds(20));

  event_loop_->MakeWatcher("/drivetrain", [this](const ChassisSpeeds &speeds) {
    // TODO(austin): Periodicaly, even if there is no speed message.
    // Or, maybe we just don't care if someone manages to go the same speed for
    // long enough to be a problem?  That would solve stale readings pushing us
    // off the field.
    HandleChassisSpeeds(event_loop_->context().monotonic_event_time, speeds);
  });

  event_loop_->MakeWatcher(
      "/drivetrain",
      [this](const frc::control_loops::drivetrain::LocalizerControl &control) {
        HandleControl(control);
      });

  // Priority should be lower than the imu reading process, but non-zero.
  event_loop->SetRuntimeRealtimePriority(10);
  event_loop->OnRun([this, event_loop]() {
    ekf_.ResetInitialState(event_loop->monotonic_now(),
                           HybridEkf::State::Zero(), NominalCovariance());
    if (control_fetcher_.Fetch()) {
      HandleControl(*control_fetcher_.get());
    }
  });
}

void Localizer::HandleControl(
    const frc::control_loops::drivetrain::LocalizerControl &control) {
  // This is triggered whenever we need to force the X/Y/(maybe theta)
  // position of the robot to a particular point---e.g., during pre-match
  // setup, or when commanded by a button on the driverstation.

  // For some forms of reset, we choose to keep our current yaw estimate
  // rather than overriding it from the control message.
  const double theta = control.keep_current_theta()
                           ? ekf_.X_hat(StateIdx::kTheta)
                           : control.theta();
  ekf_.ResetInitialState(
      t_, (HybridEkf::State() << control.x(), control.y(), theta).finished(),
      NominalCovariance());
  VLOG(1) << "Reset state";
}

void Localizer::HandleChassisSpeeds(
    const aos::monotonic_clock::time_point sample_time_orin,
    const ChassisSpeeds &speeds) {
  roborio_pose_fetcher_.Fetch();
  if (roborio_pose_fetcher_.get() == nullptr) {
    return;
  }

  const Eigen::Rotation2D<double> rotation(roborio_pose_fetcher_->theta());
  const Eigen::Vector2d velocity(speeds.vx(), speeds.vy());

  const Eigen::Vector2d absolute_velocity = rotation * velocity;
  VLOG(1) << speeds.vx() << ", " << speeds.vy() << ", theta "
          << roborio_pose_fetcher_->theta() << " -> "
          << absolute_velocity.transpose();

  // Now, angle is +- M_PI
  const double theta_error = aos::math::NormalizeAngle(
      ekf_.X_hat(StateIdx::kTheta) - roborio_pose_fetcher_->theta());

  if (std::abs(theta_error) > 0.4) {
    ++heading_resets_;
    // TODO(austin): Count this and display it.
    VLOG(1) << "Resetting, theta too far off, was "
            << ekf_.X_hat(StateIdx::kTheta) << " expected "
            << roborio_pose_fetcher_->theta() << " for an error of "
            << theta_error;
    ekf_.ResetInitialState(t_,
                           (HybridEkf::State() << average_pose_.x(),
                            average_pose_.y(), roborio_pose_fetcher_->theta())
                               .finished(),
                           NominalCovariance());
  }

  t_ = sample_time_orin;
  // We don't actually use the down estimator currently, but it's really
  // convenient for debugging.
  ekf_.UpdateSpeeds(absolute_velocity.x(), absolute_velocity.y(),
                    speeds.omega(), t_);
  SendStatus();
}

void Localizer::RejectImage(int camera_index, RejectionReason reason,
                            TargetEstimateDebugStatic *builder) {
  if (builder != nullptr) {
    builder->set_accepted(false);
    builder->set_rejection_reason(reason);
  }
  cameras_.at(camera_index).rejection_counter.IncrementError(reason);
}

// Only use april tags present in the target map; this method has also been used
// (in the past) for ignoring april tags that tend to produce problematic
// readings.
bool Localizer::UseAprilTag(uint64_t target_id) {
  if (target_poses_.count(target_id) == 0 || target_id == 4 || target_id == 5 ||
      target_id == 14 || target_id == 15 || target_id == 3 || target_id == 16) {
    return false;
  }
  return true;
}

bool Localizer::DeweightAprilTag(uint64_t target_id) {
  switch (target_id) {
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
    case 17:
    case 18:
    case 19:
    case 20:
    case 21:
    case 22:
      return false;
    default:
      return true;
  }

  /*
  const flatbuffers::Vector<uint64_t> *ignore_tags = nullptr;

  switch (utils_.Alliance()) {
    case aos::Alliance::kRed:
      ignore_tags =
          constants_fetcher_.constants().common()->ignore_targets()->red();
      CHECK(ignore_tags != nullptr);
      break;
    case aos::Alliance::kBlue:
      ignore_tags =
          constants_fetcher_.constants().common()->ignore_targets()->blue();
      CHECK(ignore_tags != nullptr);
      break;
    case aos::Alliance::kInvalid:
      return false;
  }
  return std::find(ignore_tags->begin(), ignore_tags->end(), target_id) !=
         ignore_tags->end();
         */
}

namespace {
// converts a camera transformation matrix from treating the +z axis from
// pointing straight out the lens to having the +x pointing straight out the
// lens, with +Z going "up" (i.e., -Y in the normal convention) and +Y going
// leftwards (i.e., -X in the normal convention).
Localizer::Transform ZToXCamera(const Localizer::Transform &transform) {
  return transform *
         Eigen::Matrix4d{
             {0, -1, 0, 0}, {0, 0, -1, 0}, {1, 0, 0, 0}, {0, 0, 0, 1}};
}
}  // namespace

void Localizer::HandleTarget(
    int camera_index, const aos::monotonic_clock::time_point capture_time,
    const frc::vision::TargetPoseFbs &target,
    TargetEstimateDebugStatic *debug_builder) {
  ++total_candidate_targets_;
  ++cameras_.at(camera_index).total_candidate_targets;
  const uint64_t target_id = target.id();

  if (debug_builder == nullptr) {
    AOS_LOG(ERROR, "Dropped message from debug vector.");
  } else {
    debug_builder->set_camera(camera_index);
    debug_builder->set_image_age_sec(aos::time::DurationInSeconds(
        event_loop_->monotonic_now() - capture_time));
    debug_builder->set_image_monotonic_timestamp_ns(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            capture_time.time_since_epoch())
            .count());
    debug_builder->set_april_tag(target_id);
  }
  VLOG(2) << aos::FlatbufferToJson(&target);
  if (!UseAprilTag(target_id)) {
    VLOG(1) << "Rejecting target due to invalid ID " << target_id;
    RejectImage(camera_index, RejectionReason::NO_SUCH_TARGET, debug_builder);
    return;
  }
  double april_tag_noise_scalar = 1.0;
  if (DeweightAprilTag(target_id)) {
    if (!absl::GetFlag(FLAGS_always_use_extra_tags) &&
        utils_.MaybeInAutonomous()) {
      VLOG(1) << "Rejecting target due to auto invalid ID " << target_id;
      RejectImage(camera_index, RejectionReason::NO_SUCH_TARGET, debug_builder);
      return;
    } else {
      if (utils_.MaybeInAutonomous()) {
        april_tag_noise_scalar = 1.5;
      } else {
        april_tag_noise_scalar = 1.5;
      }
    }
  }

  const Transform &H_field_target = target_poses_.at(target_id);
  const Transform &H_robot_camera = cameras_.at(camera_index).extrinsics;

  const Transform H_camera_target = PoseToTransform(&target);

  // In order to do the EKF correction, we determine the expected state based
  // on the state at the time the image was captured; however, we insert the
  // correction update itself at the current time. This is technically not
  // quite correct, but saves substantial CPU usage & code complexity by
  // making it so that we don't have to constantly rewind the entire EKF
  // history.
  const std::optional<State> state_at_capture =
      ekf_.LastStateBeforeTime(capture_time);
  const std::optional<Input> input_at_capture =
      ekf_.LastInputBeforeTime(capture_time);

  if (!state_at_capture.has_value()) {
    VLOG(1) << "Rejecting image due to being too old.";
    return RejectImage(camera_index, RejectionReason::IMAGE_TOO_OLD,
                       debug_builder);
  } else if (target.pose_error() > absl::GetFlag(FLAGS_max_pose_error)) {
    VLOG(1) << "Rejecting target due to high pose error "
            << target.pose_error();
    return RejectImage(camera_index, RejectionReason::HIGH_POSE_ERROR,
                       debug_builder);
  } else if (target.pose_error_ratio() >
             absl::GetFlag(FLAGS_max_pose_error_ratio)) {
    VLOG(1) << "Rejecting target due to high pose error ratio "
            << target.pose_error_ratio();
    return RejectImage(camera_index, RejectionReason::HIGH_POSE_ERROR_RATIO,
                       debug_builder);
  }

  const double robot_speed =
      std::hypot(input_at_capture.value()(InputIdx::kVx),
                 input_at_capture.value()(InputIdx::kVy));

  roborio_pose_fetcher_.Fetch();
  double rio_theta;
  if (roborio_pose_fetcher_.get() == nullptr) {
    rio_theta = state_at_capture.value()(StateIdx::kTheta);
  } else {
    rio_theta = roborio_pose_fetcher_->theta();
  }

  Corrector corrector(state_at_capture.value(), rio_theta, H_field_target,
                      H_robot_camera, H_camera_target);
  const double distance_to_target = corrector.observed()(Corrector::kDistance);

  // Heading, distance, skew at 1 meter.
  Eigen::Matrix<double, 3, 1> noises(0.03, 0.25, 0.15);
  noises *= 2.0;
  const double distance_noise_scalar =
      std::min(1.0, std::pow(distance_to_target, 2.0));
  noises(Corrector::kDistance) *= distance_noise_scalar;
  noises(Corrector::kSkew) *= distance_noise_scalar;
  // TODO(james): This is leftover from last year; figure out if we want it.
  // Scale noise by the distortion factor for this detection
  noises *= (1.0 + absl::GetFlag(FLAGS_distortion_noise_scalar) *
                       target.distortion_factor());
  noises *= april_tag_noise_scalar;
  noises *= (1.0 + std::abs(robot_speed));

  Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
  R.diagonal() = noises.cwiseAbs2();
  const Eigen::Vector3d camera_position =
      corrector.observed_camera_pose().abs_pos();
  // Calculate the camera-to-robot transformation matrix ignoring the
  // pitch/roll of the camera.
  const Transform H_camera_robot_stripped =
      frc::control_loops::Pose(ZToXCamera(H_robot_camera))
          .AsTransformationMatrix()
          .inverse();
  const frc::control_loops::Pose measured_pose(
      corrector.observed_camera_pose().AsTransformationMatrix() *
      H_camera_robot_stripped);
  if (debug_builder != nullptr) {
    debug_builder->set_camera_x(camera_position.x());
    debug_builder->set_camera_y(camera_position.y());
    debug_builder->set_camera_theta(
        corrector.observed_camera_pose().abs_theta());
    debug_builder->set_implied_robot_x(measured_pose.rel_pos().x());
    debug_builder->set_implied_robot_y(measured_pose.rel_pos().y());
    debug_builder->set_implied_robot_theta(measured_pose.rel_theta());

    Corrector::PopulateMeasurement(corrector.expected(),
                                   debug_builder->add_expected_observation());
    Corrector::PopulateMeasurement(corrector.observed(),
                                   debug_builder->add_actual_observation());
    Corrector::PopulateMeasurement(noises, debug_builder->add_modeled_noise());
  }

  VLOG(1) << "Got " << corrector.observed_camera_pose().abs_theta()
          << " expected " << corrector.expected_camera_pose().abs_theta()
          << " rio " << corrector.expected_rio_heading_camera().abs_theta()
          << " absolute rio " << rio_theta << " heading "
          << state_at_capture.value()(StateIdx::kTheta);

  const double camera_yaw_error = aos::math::NormalizeAngle(
      corrector.expected_rio_heading_camera().abs_theta() -
      corrector.observed_camera_pose().abs_theta());
  constexpr double kDegToRad = M_PI / 180.0;
  const double yaw_threshold =
      (utils_.MaybeInAutonomous()
           ? absl::GetFlag(FLAGS_max_implied_yaw_error)
           : absl::GetFlag(FLAGS_max_implied_teleop_yaw_error)) *
      kDegToRad;

  if (target.distortion_factor() > absl::GetFlag(FLAGS_max_distortion)) {
    VLOG(1) << "Rejecting target due to high distortion.";
    return RejectImage(camera_index, RejectionReason::HIGH_DISTORTION,
                       debug_builder);
  } else if (utils_.MaybeInAutonomous() &&
             (std::abs(robot_speed) >
              absl::GetFlag(FLAGS_max_auto_image_robot_speed))) {
    return RejectImage(camera_index, RejectionReason::ROBOT_TOO_FAST,
                       debug_builder);
  } else if (std::abs(camera_yaw_error) > yaw_threshold) {
    average_pose_ = average_pose_ * 0.9 + 0.1 * measured_pose.rel_pos();
    return RejectImage(camera_index, RejectionReason::HIGH_IMPLIED_YAW_ERROR,
                       debug_builder);
  } else if (distance_to_target > absl::GetFlag(FLAGS_max_distance_to_target)) {
    return RejectImage(camera_index, RejectionReason::HIGH_DISTANCE_TO_TARGET,
                       debug_builder);
  }

  average_pose_ = average_pose_ * 0.9 + 0.1 * measured_pose.rel_pos();

  const Input U = ekf_.MostRecentInput();
  VLOG(1) << "previous state " << ekf_.X_hat().transpose();
  const State prior_state = ekf_.X_hat();
  // For the correction step, instead of passing in the measurement directly,
  // we pass in (0, 0, 0) as the measurement and then for the expected
  // measurement (Zhat) we calculate the error between the pose implied by
  // the camera measurement and the current estimate of the
  // pose. This doesn't affect any of the math, it just makes the code a bit
  // more convenient to write given the Correct() interface we already have.
  if (absl::GetFlag(FLAGS_do_xytheta_corrections)) {
    Eigen::Vector3d Z(measured_pose.rel_pos().x(), measured_pose.rel_pos().y(),
                      measured_pose.rel_theta());
    Eigen::Matrix<double, 3, 1> xyz_noises(0.2, 0.2, 0.5);
    xyz_noises *= distance_noise_scalar;
    xyz_noises *= april_tag_noise_scalar;
    // Scale noise by the distortion factor for this detection
    xyz_noises *= (1.0 + absl::GetFlag(FLAGS_distortion_noise_scalar) *
                             target.distortion_factor());

    Eigen::Matrix3d R_xyz = Eigen::Matrix3d::Zero();
    R_xyz.diagonal() = xyz_noises.cwiseAbs2();
    xyz_observations_.CorrectKnownH(Eigen::Vector3d::Zero(), &U,
                                    XyzCorrector(state_at_capture.value(), Z),
                                    R_xyz, t_);
  } else {
    observations_.CorrectKnownH(Eigen::Vector3d::Zero(), &U, corrector, R, t_);
  }
  ++total_accepted_targets_;
  ++cameras_.at(camera_index).total_accepted_targets;
  VLOG(1) << "new state " << ekf_.X_hat().transpose();
  if (debug_builder != nullptr) {
    debug_builder->set_correction_x(ekf_.X_hat()(StateIdx::kX) -
                                    prior_state(StateIdx::kX));
    debug_builder->set_correction_y(ekf_.X_hat()(StateIdx::kY) -
                                    prior_state(StateIdx::kY));
    debug_builder->set_correction_theta(ekf_.X_hat()(StateIdx::kTheta) -
                                        prior_state(StateIdx::kTheta));
    debug_builder->set_accepted(true);
    debug_builder->set_expected_robot_x(ekf_.X_hat()(StateIdx::kX));
    debug_builder->set_expected_robot_y(ekf_.X_hat()(StateIdx::kY));
    debug_builder->set_expected_robot_theta(
        aos::math::NormalizeAngle(ekf_.X_hat()(StateIdx::kTheta)));
  }
}

void Localizer::SendOutput() {
  auto builder = output_sender_.MakeBuilder();
  frc::controls::LocalizerOutput::Builder output_builder =
      builder.MakeBuilder<frc::controls::LocalizerOutput>();
  output_builder.add_monotonic_timestamp_ns(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          event_loop_->context().monotonic_event_time.time_since_epoch())
          .count());
  output_builder.add_x(ekf_.X_hat(StateIdx::kX));
  output_builder.add_y(ekf_.X_hat(StateIdx::kY));
  output_builder.add_theta(ekf_.X_hat(StateIdx::kTheta));
  output_builder.add_zeroed(true);
  output_builder.add_image_accepted_count(total_accepted_targets_);
  output_builder.add_heading_resets(heading_resets_);

  // The output message is year-agnostic, and retains "pi" naming for histrocial
  // reasons.
  output_builder.add_all_pis_connected(true);
  builder.CheckOk(builder.Send(output_builder.Finish()));
}

flatbuffers::Offset<LocalizerState>
Localizer::PopulateState(const State &X_hat,
                         flatbuffers::FlatBufferBuilder *fbb) {
  LocalizerState::Builder builder(*fbb);
  builder.add_x(X_hat(StateIdx::kX));
  builder.add_y(X_hat(StateIdx::kY));
  builder.add_theta(aos::math::NormalizeAngle(X_hat(StateIdx::kTheta)));
  return builder.Finish();
}

flatbuffers::Offset<CumulativeStatistics> Localizer::StatisticsForCamera(
    const CameraState &camera, flatbuffers::FlatBufferBuilder *fbb) {
  const auto counts_offset = camera.rejection_counter.PopulateCounts(fbb);
  CumulativeStatistics::Builder stats_builder(*fbb);
  stats_builder.add_total_accepted(camera.total_accepted_targets);
  stats_builder.add_total_candidates(camera.total_candidate_targets);
  stats_builder.add_rejection_reasons(counts_offset);
  return stats_builder.Finish();
}

void Localizer::StatisticsForCamera(const CameraState &camera,
                                    CumulativeStatisticsStatic *builder) {
  camera.rejection_counter.PopulateCountsStaticFbs(
      builder->add_rejection_reasons());
  builder->set_total_accepted(camera.total_accepted_targets);
  builder->set_total_candidates(camera.total_candidate_targets);
}

void Localizer::SendStatus() {
  auto builder = status_sender_.MakeBuilder();
  std::array<flatbuffers::Offset<CumulativeStatistics>, kNumCameras>
      stats_offsets;
  for (size_t ii = 0; ii < kNumCameras; ++ii) {
    stats_offsets.at(ii) = StatisticsForCamera(cameras_.at(ii), builder.fbb());
  }
  auto stats_offset =
      builder.fbb()->CreateVector(stats_offsets.data(), stats_offsets.size());
  auto state_offset = PopulateState(ekf_.X_hat(), builder.fbb());
  // covariance is a square; we use the number of rows in the state as the rows
  // and cols of the covariance.
  auto covariance_offset =
      frc::FromEigen<State::RowsAtCompileTime, State::RowsAtCompileTime>(
          ekf_.P(), builder.fbb());
  Status::Builder status_builder = builder.MakeBuilder<Status>();
  status_builder.add_state(state_offset);
  status_builder.add_statistics(stats_offset);
  status_builder.add_ekf_covariance(covariance_offset);
  builder.CheckOk(builder.Send(status_builder.Finish()));
}

Eigen::Vector3d Localizer::Corrector::HeadingDistanceSkew(
    const Pose &relative_pose) {
  const double heading = relative_pose.heading();
  const double distance = relative_pose.xy_norm();
  const double skew =
      ::aos::math::NormalizeAngle(relative_pose.rel_theta() - heading);
  return {heading, distance, skew};
}

// This approximates the Jacobian of a vector of [heading, distance, skew]
// of a target with respect to the full state of a drivetrain EKF.
// Note that the only nonzero values in the returned matrix will be in the
// columns corresponding to the X, Y, and Theta components of the state.
// This is suitable for use as the H matrix in the kalman updates of the EKF,
// although due to the approximation it should not be used to actually
// calculate the expected measurement.
// target_pose is the global pose of the target that we have identified.
// camera_pose is the current estimate of the global pose of
//   the camera that can see the target.
template <typename Scalar>
Eigen::Matrix<double, 3, HybridEkf<Scalar>::kNStates>
HMatrixForCameraHeadingDistanceSkew(
    const frc::control_loops::TypedPose<Scalar> &target_pose,
    const frc::control_loops::TypedPose<Scalar> &camera_pose) {
  // For all of the below calculations, we will assume to a first
  // approximation that:
  //
  // dcamera_theta / dtheta ~= 1
  // dcamera_x / dx ~= 1
  // dcamera_y / dy ~= 1
  //
  // For cameras sufficiently far from the robot's origin, or if the robot were
  // spinning extremely rapidly, this would not hold.

  // To calculate dheading/d{x,y,theta}:
  // heading = arctan2(target_pos - camera_pos) - camera_theta
  Eigen::Matrix<Scalar, 3, 1> target_pos = target_pose.abs_pos();
  Eigen::Matrix<Scalar, 3, 1> camera_pos = camera_pose.abs_pos();
  Scalar diffx = target_pos.x() - camera_pos.x();
  Scalar diffy = target_pos.y() - camera_pos.y();
  Scalar norm2 = diffx * diffx + diffy * diffy;
  Scalar dheadingdx = diffy / norm2;
  Scalar dheadingdy = -diffx / norm2;
  Scalar dheadingdtheta = -1.0;

  // To calculate ddistance/d{x,y}:
  // distance = sqrt(diffx^2 + diffy^2)
  Scalar distance = ::std::sqrt(norm2);
  Scalar ddistdx = -diffx / distance;
  Scalar ddistdy = -diffy / distance;

  // Skew = target.theta - camera.theta - heading
  //      = target.theta - arctan2(target_pos - camera_pos)
  Scalar dskewdx = -dheadingdx;
  Scalar dskewdy = -dheadingdy;
  Eigen::Matrix<Scalar, 3, HybridEkf<Scalar>::kNStates> H;
  H.setZero();
  H(0, HybridEkf<Scalar>::kX) = dheadingdx;
  H(0, HybridEkf<Scalar>::kY) = dheadingdy;
  H(0, HybridEkf<Scalar>::kTheta) = dheadingdtheta;
  H(1, HybridEkf<Scalar>::kX) = ddistdx;
  H(1, HybridEkf<Scalar>::kY) = ddistdy;
  H(2, HybridEkf<Scalar>::kX) = dskewdx;
  H(2, HybridEkf<Scalar>::kY) = dskewdy;
  return H;
}

Localizer::Corrector Localizer::Corrector::CalculateHeadingDistanceSkewH(
    const State &state_at_capture, double rio_heading,
    const Transform &H_field_target, const Transform &H_robot_camera,
    const Transform &H_camera_target) {
  const Transform H_field_camera = H_field_target * H_camera_target.inverse();
  const Pose expected_robot_pose(
      {state_at_capture(StateIdx::kX), state_at_capture(StateIdx::kY), 0.0},
      state_at_capture(StateIdx::kTheta));
  const Pose rio_heading_robot_pose(
      {state_at_capture(StateIdx::kX), state_at_capture(StateIdx::kY), 0.0},
      rio_heading);
  // Observed position on the field, reduced to just the 2-D pose.
  const Pose observed_camera(ZToXCamera(H_field_camera));
  const Pose expected_camera(expected_robot_pose.AsTransformationMatrix() *
                             ZToXCamera(H_robot_camera));
  const Pose expected_rio_heading_camera(
      rio_heading_robot_pose.AsTransformationMatrix() *
      ZToXCamera(H_robot_camera));
  const Pose nominal_target(ZToXCamera(H_field_target));
  const Pose observed_target = nominal_target.Rebase(&observed_camera);
  const Pose expected_target = nominal_target.Rebase(&expected_camera);
  return Localizer::Corrector{
      expected_robot_pose,
      observed_camera,
      expected_camera,
      expected_rio_heading_camera,
      HeadingDistanceSkew(expected_target),
      HeadingDistanceSkew(observed_target),
      HMatrixForCameraHeadingDistanceSkew(nominal_target, observed_camera)};
}

Localizer::Corrector::Corrector(const State &state_at_capture,
                                const double rio_heading,
                                const Transform &H_field_target,
                                const Transform &H_robot_camera,
                                const Transform &H_camera_target)
    : Corrector(CalculateHeadingDistanceSkewH(state_at_capture, rio_heading,
                                              H_field_target, H_robot_camera,
                                              H_camera_target)) {}

Localizer::Output Localizer::Corrector::H(const State &, const Input &) {
  return expected_ - observed_;
}

Localizer::Output Localizer::XyzCorrector::H(const State &, const Input &) {
  CHECK(Z_.allFinite());
  Eigen::Vector3d Zhat = H_ * state_at_capture_ - Z_;
  // Rewrap angle difference to put it back in range.
  Zhat(2) = aos::math::NormalizeAngle(Zhat(2));
  VLOG(1) << "Zhat " << Zhat.transpose() << " Z_ " << Z_.transpose()
          << " state " << (H_ * state_at_capture_).transpose();
  return Zhat;
}

}  // namespace frc::vision::swerve_localizer
