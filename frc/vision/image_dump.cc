#include "absl/flags/flag.h"

#include "aos/events/logging/log_reader.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"
#include "aos/sha256.h"
#include "aos/util/file.h"
#include "frc/vision/vision_generated.h"

ABSL_FLAG(std::string, node, "orin", "The orin node name");
ABSL_FLAG(std::string, path, "/media/austin/a7ea7e92-b2f9-4e20-8134-54e7a3d73a5c/home/austin/frc4646/images/", "The directory to write into");

class ImageDump {
  public:
   ImageDump(aos::EventLoop *event_loop) : event_loop_(event_loop) {
     event_loop_->MakeWatcher(
         "/camera0",
         [this](const frc::vision::CameraImage &image) { LogImage(image, 0); });
     event_loop_->MakeWatcher(
         "/camera3",
         [this](const frc::vision::CameraImage &image) { LogImage(image, 3); });
     event_loop_->MakeWatcher(
         "/camera2",
         [this](const frc::vision::CameraImage &image) { LogImage(image, 2); });
   }

   void LogImage(const frc::vision::CameraImage &image, int camera) {
     CHECK(image.format() == frc::vision::ImageFormat::MJPEG);
     CHECK(image.has_data());
     std::string_view image_data(
         reinterpret_cast<const char *>(image.data()->data()),
         image.data()->size());
     std::string sha256 = aos::Sha256(image_data);

     std::string path =
         absl::StrCat(absl::GetFlag(FLAGS_path), "/", sha256.substr(0, 2), "/",
                      sha256, "-", camera, ".jpg");
     LOG(INFO) << "Writing " << path;

     CHECK(aos::util::MkdirPIfSpace(path, 0755));
     aos::util::WriteStringToFileOrDie(path, image_data, 0644);
   }

  private:
    aos::EventLoop *event_loop_;
};

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  // sort logfiles
  const std::vector<aos::logger::LogFile> logfiles =
      aos::logger::SortParts(aos::logger::FindLogs(argc, argv));


  // open logfiles
  aos::logger::LogReader reader(logfiles);

  aos::SimulatedEventLoopFactory event_loop_factory(reader.configuration());
  reader.RegisterWithoutStarting(&event_loop_factory);

  const aos::Node *node = aos::configuration::GetNode(
      event_loop_factory.configuration(), absl::GetFlag(FLAGS_node));

  reader.OnStart(node, [&event_loop_factory, node]() {
    aos::NodeEventLoopFactory *node_factory =
        event_loop_factory.GetNodeEventLoopFactory(node);

    node_factory->AlwaysStart<ImageDump>("image_dump");
  });

  event_loop_factory.Run();

  reader.Deregister();
}
