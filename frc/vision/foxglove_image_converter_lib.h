#ifndef FRC_VISION_FOXGLOVE_IMAGE_CONVERTER_H_
#define FRC_VISION_FOXGLOVE_IMAGE_CONVERTER_H_
#include "external/com_github_foxglove_schemas/CompressedImage_generated.h"

#include "aos/events/event_loop.h"
#include "frc/vision/charuco_lib.h"
#include "frc/vision/vision_generated.h"

namespace frc::vision {
// Empirically, from 2022 logs:
// PNG is an ~2x space savings relative to raw images.
// JPEG is an ~10x space savings relative to PNG.
// Both perform significantly better than attempting to perform in-browser
// conversion with a user-script in Foxglove Studio.
enum class ImageCompression { kJpeg, kPng };

std::string_view ExtensionForCompression(ImageCompression compression);

flatbuffers::Offset<foxglove::CompressedImage> CompressImage(
    const cv::Mat image, const aos::monotonic_clock::time_point eof,
    flatbuffers::FlatBufferBuilder *fbb, ImageCompression compression);

// This class provides a simple converter that will take an AOS CameraImage
// channel and output
class FoxgloveImageConverter {
 public:
  // Watches for frc.vision.CameraImage messages on the input_channel and
  // sends foxglove.CompressedImage messages on the output_channel, using the
  // specified image compression algorithm.
  FoxgloveImageConverter(aos::EventLoop *event_loop,
                         std::string_view input_channel,
                         std::string_view output_channel,
                         ImageCompression compression);

 private:
  aos::EventLoop *event_loop_;
  ImageCallback image_callback_;
  aos::Sender<foxglove::CompressedImage> sender_;
};
}  // namespace frc::vision
#endif  // FRC_VISION_FOXGLOVE_IMAGE_CONVERTER_H_
