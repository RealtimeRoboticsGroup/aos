#include <networktables/DoubleArrayTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "absl/flags/flag.h"

#include "aos/configuration.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "frc/vision/target_map_generated.h"
#include "frc/vision/field_map_generated.h"

ABSL_FLAG(std::string, config, "aos_config.json",
          "File path of aos configuration");
ABSL_FLAG(std::string, field_map, "frc2025r2.fmap",
          "File path of the field map to use");

ABSL_FLAG(std::string, server, "roborio",
          "Server (IP address or hostname) to connect to.");

namespace frc::vision {

class NetworkTablesPublisher {
 public:
  NetworkTablesPublisher(aos::EventLoop *event_loop,
                         std::string_view table_name, const FieldMap *field_map)
      : event_loop_(event_loop),
        table_(nt::NetworkTableInstance::GetDefault().GetTable(table_name)),
        pose_topic_(table_->GetDoubleArrayTopic("botpose_wpiblue")),
        pose_publisher_(pose_topic_.Publish({.keepDuplicates = true})) {
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

    tag_transformations_.resize(max_id + 1);

    for (const Fiducial *fiducial : *field_map->fiducials()) {
      CHECK(fiducial->has_transform());
      CHECK_EQ(fiducial->transform()->size(), 16u);

      // LOG(INFO) << "Fiducial: " << aos::FlatbufferToJson(fiducial);
      LOG(INFO) << "Fiducial: " << fiducial->id();
      Eigen::Affine3d transformation;
      for (size_t i = 0; i < 16u; ++i) {
        transformation.matrix().data()[i] = fiducial->transform()->Get(i);
      }

      Eigen::Matrix4d matrix = transformation.matrix();
      transformation.matrix().transposeInPlace();
      // LOG(INFO) << "Transform: " << transformation.matrix();
      // LOG(INFO) << "  Translation: " <<
      // transformation.translation().transpose(); LOG(INFO) << "  Rotation: "
      // << transformation.rotation(); LOG(INFO) << "  Quaternion: "
      //<< Eigen::Quaterniond(transformation.rotation());

      tag_transformations_[fiducial->id()] = transformation;

      Eigen::Matrix<double, 3, 1> zero = Eigen::Matrix<double, 3, 1>::Zero();
      // Transform converts a tag coordinate to field coordinates.
      LOG(INFO) << "  Tag at: " << (transformation * zero).transpose();

      // [x+5]   [1 0 0 5][x]
      // [y+3]   [0 1 0 3][y]
      // [z+0]   [0 0 1 0][z]
      // [1]     [0 0 0 1][1]
    }
  }

 private:
  void HandleTargetMap(int i, const TargetMap &target_map) {
    VLOG(1) << "Got map for " << i;
    if (target_map.target_poses()->size() == 0) {
      Publish(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0, 0, 0, 0, 0);
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

    const Eigen::Affine3d camera_to_tag = translation * orientation;
    const Eigen::Affine3d tag_to_field =
        tag_transformations_[3];
        //tag_transformations_[target_pose->id()];

    const Eigen::Affine3d camera_to_field = camera_to_tag * tag_to_field;
    (void) camera_to_field;

    // TODO(austin): Is this the right set of euler angles?
    const Eigen::Vector3d ypr =
        orientation.toRotationMatrix().eulerAngles(0, 1, 2);

    const double age_ms =
        std::chrono::duration<double, std::milli>(
            event_loop_->monotonic_now() -
            aos::monotonic_clock::time_point(
                std::chrono::nanoseconds(target_map.monotonic_timestamp_ns())))
            .count();

      Eigen::Matrix<double, 3, 1> zero = Eigen::Matrix<double, 3, 1>::Zero();
      // Transform converts a tag coordinate to field coordinates.
      //
//I0221 02:02:45.253626    1424 network_tables_publisher.cc:124] Cam3 tag at: 2.78681 4.02961 1.30175 1m in x: 2.78681 3.02961 1.30175
//I0221 02:02:45.253701    1424 network_tables_publisher.cc:130] Cam3 tag at: 2.78681 4.02961 1.30175 1m in y: 3.78681 4.02961 1.30175
//I0221 02:02:45.253718    1424 network_tables_publisher.cc:136] Cam3 tag at: 2.78681 4.02961 1.30175 1m in z: 2.78681 4.02961 2.30175
//I0221 02:02:45.253728    1424 network_tables_publisher.cc:143] Cam3, tag 2, t: -0.113401 -0.105817  0.559166 at -0.113401 -0.105817  0.559166

//  0 1 0       [1 0 0]
// -1 0 0 = R * [0 1 0]
//  0 0 1       [0 0 1]

      // Z is up
      // X is perpendicular to the plane of the tag.
      LOG(INFO)
          << "Cam" << i << " tag at: "
          << (tag_to_field * Eigen::Matrix<double, 3, 1>(0, 0, 0)).transpose()
          << " 1m in x: "
          << (tag_to_field * Eigen::Matrix<double, 3, 1>(1, 0, 0))
                 .transpose();
      LOG(INFO)
          << "Cam" << i << " tag at: "
          << (tag_to_field * Eigen::Matrix<double, 3, 1>(0, 0, 0)).transpose()
          << " 1m in y: "
          << (tag_to_field * Eigen::Matrix<double, 3, 1>(0, 1, 0))
                 .transpose();
      LOG(INFO)
          << "Cam" << i << " tag at: "
          << (tag_to_field * Eigen::Matrix<double, 3, 1>(0, 0, 0)).transpose()
          << " 1m in z: "
          << (tag_to_field * Eigen::Matrix<double, 3, 1>(0, 0, 1))
                 .transpose();

      LOG(INFO) << "Cam" << i << ", tag " << target_pose->id()
                << ", t: " << translation_vector.transpose() << " at "
                << (camera_to_tag * Eigen::Vector3d::Zero()).transpose();

      // LOG(INFO) << "Cam" << i << ", tag " << target_pose->id()
      //<< ", t: " << translation_vector.transpose() << " at "
      //<< (camera_to_field * Eigen::Vector3d::Zero()).transpose()
      //<< " age: " << age_ms << "ms";

      Publish(translation_vector, ypr, age_ms,
              target_map.target_poses()->size(), 0, translation_vector.norm(),
              0);
  }

  void Publish(Eigen::Vector3d translation, Eigen::Vector3d ypr,
               double latency_ms, int tag_count, double tag_span_m,
               double tag_dist_m, double tag_area_percent) {
    std::array<double, 11> pose{
        translation.x(),  translation.y(),
        translation.z(),  ypr.x(),
        ypr.y(),          ypr.z(),
        latency_ms,       static_cast<double>(tag_count),
        tag_span_m,       tag_dist_m,
        tag_area_percent,
    };

    pose_publisher_.Set(pose);
  }

  aos::EventLoop *event_loop_;

  std::shared_ptr<nt::NetworkTable> table_;
  nt::DoubleArrayTopic pose_topic_;

  std::vector<Eigen::Affine3d> tag_transformations_;
  nt::DoubleArrayPublisher pose_publisher_;
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
