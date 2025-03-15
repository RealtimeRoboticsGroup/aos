#include "Eigen/Core"
#include "Eigen/Geometry"
#include "absl/flags/flag.h"

#include "aos/events/event_loop.h"
#include "aos/json_to_flatbuffer.h"
#include "frc/vision/field_map_generated.h"
#include "frc/vision/target_map_static.h"

namespace frc::vision::swerve_localizer {

void SendFieldMap(aos::EventLoop *event_loop, const FieldMap *field_map,
                  std::string_view field_name) {
  aos::Sender<TargetMapStatic> target_map_sender =
      event_loop->MakeSender<TargetMapStatic>("/constants");

  aos::Sender<TargetMapStatic>::StaticBuilder builder =
      target_map_sender.MakeStaticBuilder();

  // Convert the field map JSON from limelight's format to our FieldMap format.
  auto field_name_string = builder->add_field_name();
  CHECK(field_name_string->reserve(field_name.size() + 1));
  field_name_string->SetString(field_name);

  auto target_poses = builder->add_target_poses();
  CHECK(target_poses->reserve(field_map->fiducials()->size()));

  // Now, fill in the tag transformations table.
  for (const Fiducial *fiducial : *field_map->fiducials()) {
    CHECK(fiducial->has_transform());
    CHECK_EQ(fiducial->transform()->size(), 16u);

    VLOG(1) << "Fiducial: " << fiducial->id();
    Eigen::Affine3d photonvision_transformation;
    for (size_t i = 0; i < 16u; ++i) {
      photonvision_transformation.matrix().data()[i] =
          fiducial->transform()->Get(i);
    }

    photonvision_transformation.matrix().transposeInPlace();

    // The map is in the photonvision tag coordinate system, and the detections
    // are in the aprilrobotics tag coordinate system. Convert.
    const Eigen::Matrix3d april_to_photon_matrix =
        (Eigen::Matrix3d() << 0, 0, -1, 1, 0, 0, 0, -1, 0).finished();
    const Eigen::Quaternion<double> april_to_photon(april_to_photon_matrix);

    Eigen::Affine3d aprilrobotics_transformation =
        photonvision_transformation * april_to_photon;

    TargetPoseFbsStatic *target_pose = target_poses->emplace_back();
    CHECK(target_pose != nullptr);

    target_pose->set_id(fiducial->id());

    PositionStatic *position = target_pose->add_position();
    QuaternionStatic *orientation = target_pose->add_orientation();

    Eigen::Vector3d translation = aprilrobotics_transformation.translation();

    position->set_x(translation.x());
    position->set_y(translation.y());
    position->set_z(translation.z());

    Eigen::Quaterniond rotation =
        Eigen::Quaterniond(aprilrobotics_transformation.rotation());
    orientation->set_w(rotation.w());
    orientation->set_x(rotation.x());
    orientation->set_y(rotation.y());
    orientation->set_z(rotation.z());

    VLOG(1) << "  Tag at: "
            << (aprilrobotics_transformation *
                Eigen::Matrix<double, 3, 1>::Zero())
                   .transpose();
  }

  // And publish the converted result.
  builder.CheckOk(builder.Send());
}

}  // namespace frc::vision::swerve_localizer
