#include <networktables/BooleanTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructTopic.h>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "absl/flags/flag.h"

#include "aos/configuration.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "frc/constants/constants_sender_lib.h"
#include "frc/control_loops/drivetrain/localization/localizer_output_generated.h"
#include "frc/geometry/Pose2d.h"
#include "frc/vision/camera_constants_generated.h"
#include "frc/vision/field_map_generated.h"
#include "frc/vision/swerve_localizer/status_generated.h"
#include "frc/vision/target_map_generated.h"

ABSL_FLAG(std::string, config, "aos_config.json",
          "File path of aos configuration");
ABSL_FLAG(std::string, field_map, "frc2025r2.fmap",
          "File path of the field map to use");
ABSL_FLAG(double, max_distance, 4.0, "Max distance to accept targets.");

ABSL_FLAG(std::string, server, "roborio",
          "Server (IP address or hostname) to connect to.");

namespace frc::vision {

const calibration::CameraCalibration *FindCameraCalibration(
    const CameraConstants &calibration_data, std::string_view node_name,
    int camera_number) {
  CHECK(calibration_data.has_calibration());
  for (const calibration::CameraCalibration *candidate :
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

class NetworkTablesPublisher {
 public:
  NetworkTablesPublisher(aos::EventLoop *event_loop,
                         std::string_view table_name, const FieldMap *field_map)
      : event_loop_(event_loop),
        table_(nt::NetworkTableInstance::GetDefault().GetTable(table_name)),
        fused_pose2d_topic_(table_->GetStructTopic<frc::Pose2d>("fused_pose")),
        pose2d_topic_(table_->GetStructTopic<frc::Pose2d>("apriltag_pose")),
        cam0_detection_topic_(table_->GetBooleanTopic("cam0_has_detections")),
        fused_pose2d_publisher_(fused_pose2d_topic_.Publish(
            {.periodic = 0.02, .keepDuplicates = true})),
        pose2d_publisher_(pose2d_topic_.Publish({.keepDuplicates = true})),
        cam0_detection_publisher_(
            cam0_detection_topic_.Publish({.keepDuplicates = false})),
        fieldwidth_(field_map->fieldwidth()),
        fieldlength_(field_map->fieldlength()),
        calibration_data_(event_loop) {
    last_detection_times_.resize(4);
    for (size_t i = 0; i < 4; i++) {
      last_detection_times_[i] = aos::monotonic_clock::min_time;
      const calibration::CameraCalibration *calibration =
          FindCameraCalibration(calibration_data_.constants(),
                                event_loop->node()->name()->string_view(), i);
      CHECK(calibration->has_fixed_extrinsics());
      CHECK(calibration->fixed_extrinsics()->has_data());
      CHECK_EQ(calibration->fixed_extrinsics()->data()->size(), 16u);
      event_loop_->MakeWatcher(
          absl::StrCat("/camera", i, "/gray"),
          [this, calibration, i](const TargetMap &target_map) {
            HandleTargetMap(i, calibration, target_map);
          });
    }
    event_loop_->MakeWatcher(
        "/localizer",
        [this](const frc::controls::LocalizerOutput &localizer_output) {
          Publish(
              &fused_pose2d_publisher_,
              Eigen::Vector3d(localizer_output.x(), localizer_output.y(), 0.0) +
                  Eigen::Vector3d(fieldlength_ / 2.0, fieldwidth_ / 2.0, 0.0),
              localizer_output.theta());
        });

    size_t max_id = 0u;
    for (const Fiducial *fiducial : *field_map->fiducials()) {
      max_id = std::max(max_id, static_cast<size_t>(fiducial->id()));
    }

    // Make sure there aren't any holes in the ids
    CHECK_EQ(max_id, field_map->fiducials()->size());

    // Now, fill in the tag transformations table.
    tag_transformations_.resize(max_id + 1);
    for (const Fiducial *fiducial : *field_map->fiducials()) {
      CHECK(fiducial->has_transform());
      CHECK_EQ(fiducial->transform()->size(), 16u);

      VLOG(1) << "Fiducial: " << fiducial->id();
      Eigen::Affine3d transformation;
      for (size_t i = 0; i < 16u; ++i) {
        transformation.matrix().data()[i] = fiducial->transform()->Get(i);
      }

      transformation.matrix().transposeInPlace();

      tag_transformations_[fiducial->id()] = transformation;

      VLOG(1)
          << "  Tag at: "
          << (transformation * Eigen::Matrix<double, 3, 1>::Zero()).transpose();
    }

    aos::TimerHandler *update_lights =
        event_loop_->AddTimer([this]() { UpdateLights(); });
    event_loop_->OnRun([this, update_lights]() {
      update_lights->Schedule(event_loop_->monotonic_now(),
                              std::chrono::milliseconds(100));
    });
  }

 private:
  std::vector<aos::monotonic_clock::time_point> last_detection_times_;

  void UpdateLights() {
    const bool recent_detections =
        (last_detection_times_[0] + std::chrono::milliseconds(100) >
         event_loop_->context().monotonic_event_time);

    cam0_detection_publisher_.Set(recent_detections);
  }

  void HandleTargetMap(int camera_number,
                       const calibration::CameraCalibration *calibration,
                       const TargetMap &target_map) {
    // TODO(austin): Handle multiple targets better.
    const TargetPoseFbs *target_pose = nullptr;
    double min_distance = 1e6;
    for (size_t i = 0; i < target_map.target_poses()->size(); ++i) {
      const TargetPoseFbs *target_pose_i = target_map.target_poses()->Get(i);
      const Eigen::Vector3d translation_vector_i(
          target_pose_i->position()->x(), target_pose_i->position()->y(),
          target_pose_i->position()->z());
      VLOG(2) << "Got target pose: " << translation_vector_i.norm() << " for "
              << i;
      if (target_pose == nullptr ||
          translation_vector_i.norm() < min_distance) {
        target_pose = target_pose_i;
        min_distance = translation_vector_i.norm();
      }
    }

    VLOG(1) << "Got map for " << calibration->camera_number() << " with "
            << target_map.target_poses()->size() << " targets, min distance of "
            << min_distance;
    if (target_pose == nullptr ||
        min_distance > absl::GetFlag(FLAGS_max_distance)) {
      return;
    }
    last_detection_times_[camera_number] =
        event_loop_->context().monotonic_event_time;

    const Eigen::Vector3d translation_vector(target_pose->position()->x(),
                                             target_pose->position()->y(),
                                             target_pose->position()->z());
    const Eigen::Translation3d translation(translation_vector);

    const Eigen::Quaternion<double> orientation(
        target_pose->orientation()->w(), target_pose->orientation()->x(),
        target_pose->orientation()->y(), target_pose->orientation()->z());

    // This tag_to_camera exposed on networktables.
    const Eigen::Affine3d tag_to_camera = translation * orientation;
    const Eigen::Affine3d tag_to_field =
        tag_transformations_[target_pose->id()];

    // The map is in the photonvision tag coordinate system, and the detections
    // are in the aprilrobotics tag coordinate system. Convert.
    const Eigen::Matrix3d april_to_photon_matrix =
        (Eigen::Matrix3d() << 0, 0, -1, 1, 0, 0, 0, -1, 0).finished();
    const Eigen::Quaternion<double> april_to_photon(april_to_photon_matrix);

    // Chain them all together to get camera -> field.
    const Eigen::Affine3d camera_to_field =
        tag_to_field * april_to_photon * tag_to_camera.inverse();

    const double age_ms =
        std::chrono::duration<double, std::milli>(
            event_loop_->monotonic_now() -
            aos::monotonic_clock::time_point(
                std::chrono::nanoseconds(target_map.monotonic_timestamp_ns())))
            .count();

    const Eigen::Matrix<double, 4, 4> camera_to_robot_matrix =
        Eigen::Map<const Eigen::Matrix<float, 4, 4, Eigen::RowMajor>>(
            calibration->fixed_extrinsics()->data()->data())
            .cast<double>();

    Eigen::Affine3d camera_to_robot;
    camera_to_robot.matrix() = camera_to_robot_matrix;

    VLOG(2) << "Cam " << calibration->camera_number()
            << " fixed extrinsics are: " << camera_to_robot.matrix();

    const Eigen::Affine3d robot_to_field =
        camera_to_field * camera_to_robot.inverse();

    // Project the heading onto the plane of the field by rotating a unit
    // vector and backing out the heading.
    const Eigen::Vector3d projected_z =
        robot_to_field.rotation().matrix() * Eigen::Vector3d::UnitX();
    const double yaw = std::atan2(projected_z.y(), projected_z.x());

    VLOG(1) << "Cam" << calibration->camera_number() << ", tag "
            << target_pose->id() << ", t: " << translation_vector.transpose()
            << " min distance " << min_distance << " at "
            << (robot_to_field * Eigen::Vector3d::Zero()).transpose() << " yaw "
            << yaw << " age: " << age_ms << "ms";

    Publish(&pose2d_publisher_,
            robot_to_field * Eigen::Vector3d::Zero() +
                Eigen::Vector3d(fieldlength_ / 2.0, fieldwidth_ / 2.0, 0.0),
            yaw);
  }

  void Publish(nt::StructPublisher<frc::Pose2d> *publisher,
               Eigen::Vector3d translation, double yaw) {
    publisher->Set(Pose2d{units::meter_t{translation.x()},
                          units::meter_t{translation.y()},
                          frc::Rotation2d{units::radian_t{yaw}}});
  }

  aos::EventLoop *event_loop_;

  std::shared_ptr<nt::NetworkTable> table_;
  nt::StructTopic<frc::Pose2d> fused_pose2d_topic_;
  nt::StructTopic<frc::Pose2d> pose2d_topic_;
  nt::BooleanTopic cam0_detection_topic_;

  std::vector<Eigen::Affine3d> tag_transformations_;
  nt::StructPublisher<frc::Pose2d> fused_pose2d_publisher_;
  nt::StructPublisher<frc::Pose2d> pose2d_publisher_;

  nt::BooleanPublisher cam0_detection_publisher_;
  double fieldwidth_;
  double fieldlength_;

  const frc::constants::ConstantsFetcher<CameraConstants> calibration_data_;
};

int Main() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  // TODO(austin): Really should publish this as a message.
  aos::FlatbufferDetachedBuffer<FieldMap> field_map =
      aos::JsonFileToFlatbuffer<FieldMap>(absl::GetFlag(FLAGS_field_map));

  frc::constants::WaitForConstants<CameraConstants>(&config.message());

  aos::ShmEventLoop event_loop(&config.message());

  nt::NetworkTableInstance instance = nt::NetworkTableInstance::GetDefault();
  instance.SetServer(absl::GetFlag(FLAGS_server));
  instance.StartClient4("rtrg_frc_apriltag");

  NetworkTablesPublisher publisher(&event_loop, "orin", &field_map.message());

  event_loop.Run();

  return 0;
}

}  // namespace frc::vision

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  return frc::vision::Main();
}
