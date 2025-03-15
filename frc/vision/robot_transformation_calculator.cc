#include "Eigen/Core"
#include "Eigen/Geometry"
#include "absl/log/log.h"

#include "aos/init.h"

// A tool to convert camera mounting locations in CAD to transformation
// matricies.
int Main() {
  // Converts a coordinate n the camera frame to the robot frame.
  Eigen::Matrix3d camera_to_robot_matrix;
  camera_to_robot_matrix << 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;

  Eigen::Quaterniond camera_to_robot(camera_to_robot_matrix);
  LOG(INFO) << "Camera to robot: \n" << camera_to_robot_matrix;
  LOG(INFO) << "Camera to robot: \n" << camera_to_robot.matrix();
  LOG(INFO) << "X: "
            << (camera_to_robot * Eigen::Vector3d::UnitX()).transpose();
  LOG(INFO) << "Y: "
            << (camera_to_robot * Eigen::Vector3d::UnitY()).transpose();
  LOG(INFO) << "Z: "
            << (camera_to_robot * Eigen::Vector3d::UnitZ()).transpose();

  Eigen::IOFormat json_format(Eigen::FullPrecision, 0, ", ", "\n", "      ",
                              ",");

  // Converts from the camera frame to the robot frame.
  const double angle0 = 0.0;
  const double angledown0 = 15.0;
  const Eigen::Affine3d orientation0 =
      Eigen::Translation3d(-0.120, 0.133, 0.710) *
      Eigen::AngleAxisd(angle0 * M_PI / 180, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(angledown0 * M_PI / 180, Eigen::Vector3d::UnitY()) *
      camera_to_robot;

  LOG(INFO) << "Orientation0 (cam to robot): \n"
            << orientation0.matrix().format(json_format);
  LOG(INFO) << "T: "
            << (orientation0 * Eigen::Vector3d(0.0, 0.0, 0.0)).transpose()
            << " angle " << angle0 << " or " << angle0 + 360.;

  LOG(INFO) << "Camera0 to robot: \n" << orientation0.matrix();
  const Eigen::Vector3d z = orientation0 * Eigen::Vector3d::Zero();
  LOG(INFO) << "0: " << (z).transpose();
  LOG(INFO) << "X: "
            << (orientation0 * Eigen::Vector3d::UnitX() - z).transpose();
  LOG(INFO) << "Y: "
            << (orientation0 * Eigen::Vector3d::UnitY() - z).transpose();
  LOG(INFO) << "Z: "
            << (orientation0 * Eigen::Vector3d::UnitZ() - z).transpose();

  const double angle1 = 36.0;
  const double angledown1 = 0.0;
  const Eigen::Affine3d orientation1 =
      Eigen::Translation3d(0.177, -0.210, 0.959) *
      Eigen::AngleAxisd(angle1 * M_PI / 180, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(angledown1 * M_PI / 180, Eigen::Vector3d::UnitY()) *
      camera_to_robot;

  LOG(INFO) << "Orientation1: \n" << orientation1.matrix().format(json_format);
  LOG(INFO) << "T: "
            << (orientation1 * Eigen::Vector3d(0.0, 0.0, 0.0)).transpose()
            << " angle " << angle1 << " or " << angle1 + 360.;

  const double angle2 = -36.0;
  const double angledown2 = 0.0;
  const Eigen::Affine3d orientation2 =
      Eigen::Translation3d(-0.177, -0.317, 0.959) *
      Eigen::AngleAxisd(angle2 * M_PI / 180, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(angledown2 * M_PI / 180, Eigen::Vector3d::UnitY()) *
      camera_to_robot;

  LOG(INFO) << "Orientation2: \n" << orientation2.matrix().format(json_format);
  LOG(INFO) << "T: "
            << (orientation2 * Eigen::Vector3d(0.0, 0.0, 0.0)).transpose()
            << " angle " << angle2 << " or " << angle2 + 360.;

  const double angle3 = 180.;
  const double angledown3 = -28.0;
  const Eigen::Affine3d orientation3 =
      Eigen::Translation3d(0.120, -0.264, 1.031) *
      Eigen::AngleAxisd(angle3 * M_PI / 180, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(angledown3 * M_PI / 180, Eigen::Vector3d::UnitY()) *
      camera_to_robot;

  LOG(INFO) << "Orientation3: \n" << orientation3.matrix().format(json_format);
  LOG(INFO) << "T: "
            << (orientation3 * Eigen::Vector3d(0.0, 0.0, 0.0)).transpose()
            << " angle " << angle3 << " or " << angle3 + 360.;
  return 0;
}

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  return Main();
}
