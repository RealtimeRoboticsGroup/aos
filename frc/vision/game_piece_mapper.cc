#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "absl/flags/flag.h"
#include "absl/log/log.h"

#include "aos/configuration.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frc/constants/constants_sender_lib.h"
#include "frc/control_loops/drivetrain/localization/localizer_output_generated.h"
#include "frc/vision/camera_constants_generated.h"
#include "frc/vision/coral_detection_static.h"
#include "frc/vision/game_piece_locations_static.h"

ABSL_FLAG(std::string, config, "aos_config.json",
          "File path of aos configuration");

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

class GamePieceMapper {
 public:
  GamePieceMapper(aos::EventLoop *event_loop)
      : event_loop_(event_loop),
        game_piece_locations_sender_(
            event_loop->MakeSender<frc::vision::GamePieceLocationsStatic>(
                "/camera1/coral")),
        calibration_data_(event_loop),
        localizer_output_fetcher_(
            event_loop_->MakeFetcher<frc::controls::LocalizerOutput>(
                "/localizer")) {
    event_loop_->MakeWatcher(
        "/camera1/coral",
        [this](const frc::vision::BoundingBoxes &bounding_boxes) {
          BoundingBoxes(bounding_boxes);
        });

    const calibration::CameraCalibration *calibration =
        FindCameraCalibration(calibration_data_.constants(),
                              event_loop->node()->name()->string_view(), 1);
    CHECK(calibration->has_fixed_extrinsics());
    CHECK(calibration->fixed_extrinsics()->has_data());
    CHECK_EQ(calibration->fixed_extrinsics()->data()->size(), 16u);
    camera_calibration_ = calibration;
  }

  void BoundingBoxes(const frc::vision::BoundingBoxes &bounding_boxes) {
    const Eigen::Matrix<double, 4, 4> camera_to_robot_matrix =
        Eigen::Map<const Eigen::Matrix<float, 4, 4, Eigen::RowMajor>>(
            camera_calibration_->fixed_extrinsics()->data()->data())
            .cast<double>();

    Eigen::Matrix<double, 3, 3> intrinsics =
        Eigen::Map<const Eigen::Matrix<float, 3, 3, Eigen::RowMajor>>(
            camera_calibration_->intrinsics()->data())
            .cast<double>();
    //intrinsics(0, 2) = 800;
    //intrinsics(1, 2) = 754;

    const Eigen::Matrix<double, 3, 4> camera_projective_transform =
        (Eigen::Matrix<double, 3, 4>() << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0)
            .finished();

    Eigen::Affine3d camera_to_robot;
    camera_to_robot.matrix() = camera_to_robot_matrix;

    //camera_to_robot = CameraToRobot();

    localizer_output_fetcher_.Fetch();
    if (localizer_output_fetcher_.get() == nullptr) {
      return;
    }

    if (!bounding_boxes.has_boxes()) return;

    double x = localizer_output_fetcher_->x();
    double y = localizer_output_fetcher_->y();
    double theta = localizer_output_fetcher_->theta();

    //x = 0.0;
    //y = 0.0;

    //theta = 0.0;

    const Eigen::Affine3d robot_to_field =
        Eigen::Translation3d(Eigen::Vector3d(x, y, 0.0)) *
        Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());

    //LOG(INFO) << "X: " << x << ", Y: " << y << ", Theta: " << theta;

    const Eigen::Affine3d camera_to_field = robot_to_field * camera_to_robot;
    const Eigen::Affine3d field_to_camera = camera_to_field.inverse();

    VLOG(1) << bounding_boxes.has_boxes();
    //LOG(INFO) << "Robot to field " << robot_to_field.matrix();
    //LOG(INFO) << "Robot to field " << robot_to_field * Eigen::Vector3d(0, 0, 0);
    //LOG(INFO) << "Robot to field " << robot_to_field * Eigen::Vector3d(1, 0, 0);

    //LOG(INFO) << "Camera matrix " << intrinsics;
    //LOG(INFO) << "Projective " << camera_projective_transform;

    Eigen::Matrix<double, 3, 4> field_to_pixel =
        intrinsics * camera_projective_transform *
        (field_to_camera * Eigen::Translation3d(Eigen::Vector3d(0, 0, 0.05)))
            .matrix();

    //LOG(INFO) << "field -> pixel" << field_to_pixel;

    Eigen::Matrix<double, 3, 3> field_to_pixel_xys;
    field_to_pixel_xys.block<3, 2>(0, 0) = field_to_pixel.block<3, 2>(0, 0);
    field_to_pixel_xys.block<3, 1>(0, 2) = field_to_pixel.block<3, 1>(0, 3);

    const Eigen::Matrix<double, 3, 1> field_to_pixel_z =
        field_to_pixel.block<3, 1>(0, 2);

    //LOG(INFO) << "ftp xys -> " << field_to_pixel_xys;

    {
      aos::Sender<frc::vision::GamePieceLocationsStatic>::StaticBuilder builder =
          game_piece_locations_sender_.MakeStaticBuilder();
      auto locations = builder->add_locations();
      CHECK(locations->reserve(bounding_boxes.boxes()->size()));
      for (const frc::vision::BoundingBox *box : *bounding_boxes.boxes()) {
        float u0 = box->x0();
        float v0 = box->y0();

        float uc = box->x0() + box->width() / 2.0;
        float vc = box->y0() + box->height() / 2.0;

        float u1 = box->x0() + box->width();
        float v1 = box->y0() + box->height();

        Eigen::Matrix<double, 3, 1> xys0 =
            field_to_pixel_xys.inverse() * Eigen::Vector3d(u0, v0, 1.0);
        Eigen::Matrix<double, 3, 1> xysc =
            field_to_pixel_xys.inverse() * Eigen::Vector3d(uc, vc, 1.0);
        Eigen::Matrix<double, 3, 1> xys1 =
            field_to_pixel_xys.inverse() * Eigen::Vector3d(u1, v1, 1.0);

        VLOG(1) << "xys0: " << xys0.transpose() << " xysc: " << xysc.transpose()
                  << " xys1: " << xys1.transpose();

        VLOG(1) << "Center: " << uc << ", " << vc;

        //if (xys0.z() < 0.0 || xysc.z() < 0.0 || xys1.z() < 0.0) {
          // Behind us...  Ignore it.
          //continue;
        //}


        Eigen::Vector3d xy0 = xys0 / xys0.z();
        Eigen::Vector3d xyc = xysc / xysc.z();
        Eigen::Vector3d xy1 = xys1 / xys1.z();

        // TODO(austin): Do the behind us calc...

        VLOG(1) << "xy0: " << xy0.transpose() << " xyc: " << xyc.transpose()
                << " xy1: " << xy1.transpose();
        frc::vision::GamePieceLocationStatic *location_static =
            locations->emplace_back();
        location_static->set_class_id(box->class_id());
        location_static->set_confidence(box->confidence());
        location_static->set_x(xyc.x());
        location_static->set_y(xyc.y());
        location_static->set_width(xy1.x() - xy0.x());
        location_static->set_height(xy1.y() - xy0.y());
      }
      builder.CheckOk(builder.Send());
    }
  }

 private:
  aos::EventLoop *event_loop_;

  aos::Sender<frc::vision::GamePieceLocationsStatic>
      game_piece_locations_sender_;
  const calibration::CameraCalibration *camera_calibration_;

  const frc::constants::ConstantsFetcher<CameraConstants> calibration_data_;

  aos::Fetcher<frc::controls::LocalizerOutput> localizer_output_fetcher_;
};

void Main() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  frc::constants::WaitForConstants<CameraConstants>(&config.message());

  aos::ShmEventLoop event_loop(&config.message());
  event_loop.SetRuntimeRealtimePriority(6);

  GamePieceMapper game_piece_mapper(&event_loop);

  event_loop.Run();
}

}  // namespace frc::vision

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  frc::vision::Main();
  return 0;
}
