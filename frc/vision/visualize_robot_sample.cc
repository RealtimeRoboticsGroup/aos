#include <math.h>

#include "Eigen/Dense"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "opencv2/aruco.hpp"
#include "opencv2/aruco/charuco.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "aos/init.h"
#include "aos/logging/logging.h"
#include "aos/time/time.h"
#include "frc/vision/visualize_robot.h"

namespace frc::vision {

// Show / test the basics of visualizing the robot frames
void Main(int /*argc*/, char ** /* argv */) {
  VisualizeRobot vis_robot;

  int image_width = 500;
  cv::Mat image_mat =
      cv::Mat::zeros(cv::Size(image_width, image_width), CV_8UC3);
  vis_robot.SetImage(image_mat);
  double focal_length = 1000.0;
  vis_robot.SetDefaultViewpoint(image_width, focal_length);

  // Go around the clock and plot the coordinate frame at different rotations
  for (int i = 0; i < 12; i++) {
    double angle = M_PI * double(i) / 6.0;
    Eigen::Vector3d trans;
    trans << 1.0 * cos(angle), 1.0 * sin(angle), 0.0;

    Eigen::Affine3d offset_rotate_origin =
        Eigen::Translation3d(trans) *
        Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX());

    vis_robot.DrawFrameAxes(offset_rotate_origin, std::to_string(i));
  }

  // Display the result
  cv::imshow("Display", image_mat);
  cv::waitKey();
}
}  // namespace frc::vision

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  frc::vision::Main(argc, argv);
}
