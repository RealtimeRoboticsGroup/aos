#include <networktables/DoubleArrayTopic.h>
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
#include "frc/geometry/Pose2d.h"
#include "frc/vision/field_map_generated.h"
#include "frc/vision/target_map_generated.h"

ABSL_FLAG(std::string, config, "aos_config.json",
          "File path of aos configuration");
ABSL_FLAG(std::string, field_map, "frc2025r2.fmap",
          "File path of the field map to use");

ABSL_FLAG(std::string, server, "localhost",
          "Server (IP address or hostname) to connect to.");

namespace frc::vision {

class NetworkTablesPublisher {
 public:
  NetworkTablesPublisher(aos::EventLoop *event_loop,
                         std::string_view table_name, const FieldMap *field_map)
      : event_loop_(event_loop),
        table_(nt::NetworkTableInstance::GetDefault().GetTable(table_name)),
        pose_topic_(table_->GetDoubleArrayTopic("botpose_wpiblue")),
        pose2d_topic_(table_->GetStructTopic<frc::Pose2d>("apriltag_pose")),
        pose_publisher_(pose_topic_.Publish({.keepDuplicates = true})),
        pose2d_publisher_(pose2d_topic_.Publish({.keepDuplicates = true})),
        fieldwidth_(field_map->fieldwidth()),
        fieldlength_(field_map->fieldlength()) {
    for (size_t i = 0; i < 4; i++) {
      event_loop_->MakeWatcher(absl::StrCat("/camera", i, "/gray"),
                               [this, i](const TargetMap &target_map) {
                                 HandleTargetMap(i, target_map);
                               });
    }

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
  }

 private:
  void HandleTargetMap(int i, const TargetMap &target_map) {
    VLOG(1) << "Got map for " << i;

    if (target_map.target_poses()->size() == 0) {
      Publish(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0, 0, 0, 0, 0,
              0.0);
      return;
    }

    // TODO(austin): What do we do with multiple targets?  Need to fuse them
    // somehow.
    const TargetPoseFbs *target_pose = target_map.target_poses()->Get(0);

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

    VLOG(1) << "Cam" << i << ", tag " << target_pose->id()
            << ", t: " << translation_vector.transpose() << " at "
            << (camera_to_field * Eigen::Vector3d::Zero()).transpose()
            << " age: " << age_ms << "ms";

    Eigen::Vector3d field_ypr = camera_to_field.rotation().eulerAngles(0, 1, 2);

    // Project the heading onto the plane of the field by rotating a unit
    // vector and backing out the heading.
    const Eigen::Vector3d projected_z =
        camera_to_field.rotation().matrix() * Eigen::Vector3d::UnitZ();
    const double yaw = std::atan2(projected_z.y(), projected_z.x());

    Publish(camera_to_field * Eigen::Vector3d::Zero() +
                Eigen::Vector3d(fieldlength_ / 2.0, fieldwidth_ / 2.0, 0.0),
            field_ypr, age_ms, target_map.target_poses()->size(), 0,
            translation_vector.norm(), 0, yaw);
  }

  void Publish(Eigen::Vector3d translation, Eigen::Vector3d ypr,
               double latency_ms, int tag_count, double tag_span_m,
               double tag_dist_m, double tag_area_percent, double yaw) {
    std::array<double, 11> pose{
        translation.x(),  translation.y(),
        translation.z(),  ypr.x(),
        ypr.y(),          ypr.z(),
        latency_ms,       static_cast<double>(tag_count),
        tag_span_m,       tag_dist_m,
        tag_area_percent,
    };

    pose_publisher_.Set(pose);
    pose2d_publisher_.Set(Pose2d{units::meter_t{translation.x()},
                                 units::meter_t{translation.y()},
                                 frc::Rotation2d{units::radian_t{yaw}}});
  }

  aos::EventLoop *event_loop_;

  std::shared_ptr<nt::NetworkTable> table_;
  nt::DoubleArrayTopic pose_topic_;
  nt::StructTopic<frc::Pose2d> pose2d_topic_;

  std::vector<Eigen::Affine3d> tag_transformations_;
  nt::DoubleArrayPublisher pose_publisher_;
  nt::StructPublisher<frc::Pose2d> pose2d_publisher_;
  double fieldwidth_;
  double fieldlength_;
};

int Main() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  // TODO(austin): Really should publish this as a message.
  aos::FlatbufferDetachedBuffer<FieldMap> field_map =
      aos::JsonFileToFlatbuffer<FieldMap>(absl::GetFlag(FLAGS_field_map));

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
