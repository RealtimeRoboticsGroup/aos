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
  const Eigen::Affine3d orientation0 =
      Eigen::Translation3d(0.1687, -0.2156, 0.7979) *
      Eigen::AngleAxisd(-260.0 * M_PI / 180, Eigen::Vector3d::UnitZ()) *
      camera_to_robot;

  LOG(INFO) << "Orientation0 (cam to robot): \n"
            << orientation0.matrix().format(json_format);
  LOG(INFO) << "T: " << orientation0 * Eigen::Vector3d(0.0, 0.0, 0.0);

  const Eigen::Affine3d orientation1 =
      Eigen::Translation3d(0.1832, -0.2219, 0.4074) *
      Eigen::AngleAxisd(-345. * M_PI / 180, Eigen::Vector3d::UnitZ()) *
      camera_to_robot;

  LOG(INFO) << "Orientation1: \n" << orientation1.matrix().format(json_format);
  LOG(INFO) << "T: " << orientation1 * Eigen::Vector3d(0.0, 0.0, 0.0);

  const Eigen::Affine3d orientation2 =
      Eigen::Translation3d(-0.1844, -0.2333, 0.4067) *
      Eigen::AngleAxisd(-185. * M_PI / 180, Eigen::Vector3d::UnitZ()) *
      camera_to_robot;

  LOG(INFO) << "Orientation2: \n" << orientation2.matrix().format(json_format);
  LOG(INFO) << "T: " << orientation2 * Eigen::Vector3d(0.0, 0.0, 0.0);

  const Eigen::Affine3d orientation3 =
      Eigen::Translation3d(-0.1594, -0.2926, 0.7630) *
      Eigen::AngleAxisd(-70. * M_PI / 180, Eigen::Vector3d::UnitZ()) *
      camera_to_robot;

  LOG(INFO) << "Orientation3: \n" << orientation3.matrix().format(json_format);
  LOG(INFO) << "T: " << orientation3 * Eigen::Vector3d(0.0, 0.0, 0.0);
  return 0;
}

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  return Main();
}
