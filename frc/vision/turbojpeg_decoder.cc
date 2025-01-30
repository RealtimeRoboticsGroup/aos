#include <stdio.h>
#include <stdlib.h>
#include <turbojpeg.h>

#include <chrono>

#include "absl/flags/flag.h"
#include "absl/log/log.h"
#include "absl/strings/str_cat.h"

#include "aos/configuration.h"
#include "aos/events/event_loop.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/realtime.h"
#include "frc/vision/vision_generated.h"

ABSL_FLAG(std::string, config, "aos_config.json",
          "File path of aos configuration");
ABSL_FLAG(std::string, channel, "/camera", "Channel name for the camera.");

namespace frc::vision {

class TurboJpegDecoder {
 public:
  TurboJpegDecoder(aos::EventLoop *event_loop)
      : event_loop_(event_loop),
        handle_(tjInitDecompress()),
        camera_output_sender_(event_loop_->MakeSender<CameraImage>(
            absl::StrCat(absl::GetFlag(FLAGS_channel), "/gray"))) {
    CHECK(handle_) << "Error initializing turbojpeg decompressor.";
    event_loop_->MakeWatcher(
        absl::GetFlag(FLAGS_channel),
        [this](const CameraImage &image) { ProcessImage(image); });
  }

  ~TurboJpegDecoder() { tjDestroy(handle_); }

 private:
  void ProcessImage(const CameraImage &image) {
    int width, height, subsamp, colorspace;

    {
      aos::ScopedNotRealtime nrt;
      CHECK_EQ(tjDecompressHeader3(handle_, image.data()->data(),
                                   image.data()->size(), &width, &height,
                                   &subsamp, &colorspace),
               0)
          << "Error decompressing header: " << tjGetErrorStr();
    }

    auto builder = camera_output_sender_.MakeBuilder();

    // Allocate space directly in the flatbuffer.
    uint8_t *image_data_ptr = nullptr;
    flatbuffers::Offset<flatbuffers::Vector<uint8_t>> data_offset =
        builder.fbb()->CreateUninitializedVector(width * height, 1,
                                                 &image_data_ptr);

    {
      aos::ScopedNotRealtime nrt;
      CHECK_EQ(tjDecompress2(handle_, image.data()->data(),
                             image.data()->size(), image_data_ptr, width,
                             0 /* pitch */, height, TJPF_GRAY, 0),
               0)
          << "Error decompressing image: " << tjGetErrorStr();
    }

    CameraImage::Builder camera_image_builder(*builder.fbb());

    camera_image_builder.add_rows(height);
    camera_image_builder.add_cols(width);
    camera_image_builder.add_data(data_offset);
    camera_image_builder.add_monotonic_timestamp_ns(
        image.monotonic_timestamp_ns());
    camera_image_builder.add_format(frc::vision::ImageFormat::GRAYSCALE);

    builder.CheckOk(builder.Send(camera_image_builder.Finish()));

    VLOG(1) << "Decompressed " << image.data()->size() << " bytes to " << width
            << "x" << height << " in "
            << std::chrono::duration<double>(
                   event_loop_->monotonic_now() -
                   event_loop_->context().monotonic_event_time)
                   .count()
            << "sec";
  }

  aos::EventLoop *event_loop_;
  tjhandle handle_;
  aos::Sender<CameraImage> camera_output_sender_;
};

int Main() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  aos::ShmEventLoop event_loop(&config.message());

  event_loop.SetRuntimeRealtimePriority(5);

  TurboJpegDecoder turbo_jpeg_decoder(&event_loop);

  event_loop.Run();

  return 0;
}

}  // namespace frc::vision

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  return frc::vision::Main();
}
