#include "Eigen/Core"
#include "Eigen/Geometry"
#include "absl/flags/flag.h"

#include "aos/configuration.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "frc/vision/field_map_generated.h"
#include "frc/vision/target_map_static.h"

ABSL_FLAG(std::string, config, "aos_config.json", "Path to the AOS config.");
ABSL_FLAG(std::string, field_map_json, "frc2025r2.fmap",
          "Path to the constant file");

namespace frc::vision::swerve_localizer {

int Main() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));
  aos::ShmEventLoop event_loop(&config.message());
  aos::Sender<TargetMapStatic> target_map_sender =
      event_loop.MakeSender<TargetMapStatic>("/constants");

  aos::Sender<TargetMapStatic>::StaticBuilder builder =
      target_map_sender.MakeStaticBuilder();

  // Convert the field map JSON from limelight's format to our FieldMap format.
  aos::FlatbufferDetachedBuffer<FieldMap> field_map =
      aos::JsonFileToFlatbuffer<FieldMap>(absl::GetFlag(FLAGS_field_map_json));

  auto field_name = builder->add_field_name();
  CHECK(field_name->reserve(absl::GetFlag(FLAGS_field_map_json).size() + 1));
  field_name->SetString(absl::GetFlag(FLAGS_field_map_json));

  auto target_poses = builder->add_target_poses();
  CHECK(target_poses->reserve(field_map.message().fiducials()->size()));

  // Now, fill in the tag transformations table.
  for (const Fiducial *fiducial : *field_map.message().fiducials()) {
    CHECK(fiducial->has_transform());
    CHECK_EQ(fiducial->transform()->size(), 16u);

    VLOG(1) << "Fiducial: " << fiducial->id();
    Eigen::Affine3d transformation;
    for (size_t i = 0; i < 16u; ++i) {
      transformation.matrix().data()[i] = fiducial->transform()->Get(i);
    }

    transformation.matrix().transposeInPlace();

    TargetPoseFbsStatic *target_pose = target_poses->emplace_back();
    CHECK(target_pose != nullptr);

    target_pose->set_id(fiducial->id());

    PositionStatic *position = target_pose->add_position();
    QuaternionStatic *orientation = target_pose->add_orientation();

    Eigen::Vector3d translation = transformation.translation();

    position->set_x(translation.x());
    position->set_y(translation.y());
    position->set_z(translation.z());

    Eigen::Quaterniond rotation = Eigen::Quaterniond(transformation.rotation());
    orientation->set_w(rotation.w());
    orientation->set_x(rotation.x());
    orientation->set_y(rotation.y());
    orientation->set_z(rotation.z());

    VLOG(1)
        << "  Tag at: "
        << (transformation * Eigen::Matrix<double, 3, 1>::Zero()).transpose();
  }

  // And publish the converted result.
  builder.CheckOk(builder.Send());

  // Don't need to call Run().
  return 0;
}

}  // namespace frc::vision::swerve_localizer

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  return frc::vision::swerve_localizer::Main();
}
