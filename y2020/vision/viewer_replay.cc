#include "absl/flags/flag.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "aos/events/logging/log_reader.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"
#include "frc971/vision/vision_generated.h"

ABSL_FLAG(std::string, node, "pi1", "Node name to replay.");
ABSL_FLAG(std::string, image_save_prefix, "/tmp/img",
          "Prefix to use for saving images from the logfile.");

namespace frc971::vision {
namespace {

void ViewerMain(int argc, char *argv[]) {
  // open logfiles
  aos::logger::LogReader reader(
      aos::logger::SortParts(aos::logger::FindLogs(argc, argv)));
  reader.Register();
  const aos::Node *node = nullptr;
  if (aos::configuration::MultiNode(reader.configuration())) {
    node = aos::configuration::GetNode(reader.configuration(),
                                       absl::GetFlag(FLAGS_node));
  }
  std::unique_ptr<aos::EventLoop> event_loop =
      reader.event_loop_factory()->MakeEventLoop("player", node);

  int image_count = 0;
  event_loop->MakeWatcher("/camera", [&image_count](const CameraImage &image) {
    cv::Mat image_mat(image.rows(), image.cols(), CV_8U);
    CHECK(image_mat.isContinuous());
    const int number_pixels = image.rows() * image.cols();
    for (int i = 0; i < number_pixels; ++i) {
      reinterpret_cast<uint8_t *>(image_mat.data)[i] =
          image.data()->data()[i * 2];
    }

    cv::imshow("Display", image_mat);
    if (!absl::GetFlag(FLAGS_image_save_prefix).empty()) {
      cv::imwrite("/tmp/img" + std::to_string(image_count++) + ".png",
                  image_mat);
    }
    cv::waitKey(1);
  });

  reader.event_loop_factory()->Run();
}

}  // namespace
}  // namespace frc971::vision

// Quick and lightweight grayscale viewer for images
int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  frc971::vision::ViewerMain(argc, argv);
}
