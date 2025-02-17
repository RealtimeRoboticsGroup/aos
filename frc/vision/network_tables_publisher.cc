#include <networktables/DoubleArrayTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "absl/flags/flag.h"

#include "aos/configuration.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frc/vision/target_map_generated.h"

ABSL_FLAG(std::string, config, "aos_config.json",
          "File path of aos configuration");
ABSL_FLAG(std::string, server, "roborio",
          "Server (IP address or hostname) to connect to.");

namespace frc::vision {

class NetworkTablesPublisher {
 public:
  NetworkTablesPublisher(aos::EventLoop *event_loop,
                         std::string_view table_name)
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

    const Eigen::Vector3d translation(target_pose->position()->x(),
                                      target_pose->position()->y(),
                                      target_pose->position()->z());

    const Eigen::Quaternion<double> orientation(
        target_pose->orientation()->w(), target_pose->orientation()->x(),
        target_pose->orientation()->y(), target_pose->orientation()->z());

    // TODO(austin): Is this the right set of euler angles?
    const Eigen::Vector3d ypr =
        orientation.toRotationMatrix().eulerAngles(0, 1, 2);

    const double age_ms =
        std::chrono::duration<double, std::milli>(
            event_loop_->monotonic_now() -
            aos::monotonic_clock::time_point(
                std::chrono::nanoseconds(target_map.monotonic_timestamp_ns())))
            .count();

    Publish(translation, ypr, age_ms, target_map.target_poses()->size(), 0,
            translation.norm(), 0);
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
  nt::DoubleArrayPublisher pose_publisher_;
};

int Main() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  aos::ShmEventLoop event_loop(&config.message());

  nt::NetworkTableInstance instance = nt::NetworkTableInstance::GetDefault();
  instance.SetServer(absl::GetFlag(FLAGS_server));
  instance.StartClient4("rtrg_frc_apriltag");

  NetworkTablesPublisher publisher(&event_loop, "orin");

  event_loop.Run();

  return 0;
}

}  // namespace frc::vision

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  return frc::vision::Main();
}
