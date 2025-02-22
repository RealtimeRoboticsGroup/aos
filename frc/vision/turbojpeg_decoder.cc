#include <string.h>

#include <chrono>

#include "absl/flags/flag.h"
#include "absl/log/log.h"
#include "absl/strings/str_cat.h"

#include "aos/configuration.h"
#include "aos/containers/inlined_vector.h"
#include "aos/events/event_loop.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/realtime.h"
#include "frc/vision/turbojpeg_decoder_status_static.h"
#include "frc/vision/vision_generated.h"
#include "turbojpeg.h"

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
            absl::StrCat(absl::GetFlag(FLAGS_channel), "/gray"))),
        status_sender_(event_loop_->MakeSender<TurboJpegDecoderStatusStatic>(
            absl::StrCat(absl::GetFlag(FLAGS_channel), "/gray"))) {
    CHECK(handle_) << "Error initializing turbojpeg decompressor.";
    aos::TimerHandler *status_timer =
        event_loop_->AddTimer([this]() { SendStatus(); });
    event_loop_->OnRun([this, status_timer]() {
      status_timer->Schedule(event_loop_->monotonic_now(),
                             std::chrono::seconds(1));
    });
    event_loop_->MakeWatcher(
        absl::GetFlag(FLAGS_channel),
        [this](const CameraImage &image) { ProcessImage(image); });
  }

  ~TurboJpegDecoder() { tjDestroy(handle_); }

 private:
  void ProcessImage(const CameraImage &image) {
    CHECK(image.format() == ImageFormat::MJPEG)
        << ": Expected MJPEG format but got: "
        << EnumNameImageFormat(image.format());

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
      if (tjDecompress2(handle_, image.data()->data(), image.data()->size(),
                        image_data_ptr, width, 0 /* pitch */, height, TJPF_GRAY,
                        0) != 0) {
        ++failed_decodes_;
        const char *const error = tjGetErrorStr();
        const size_t truncated_len =
            strnlen(error, last_error_message_.capacity());
        last_error_message_.resize(truncated_len);
        memcpy(last_error_message_.data(), error, truncated_len);
        VLOG(1) << "Error decompressing image: " << error;
        return;
      }
    }
    ++successful_decodes_;

    CameraImage::Builder camera_image_builder(*builder.fbb());

    camera_image_builder.add_rows(height);
    camera_image_builder.add_cols(width);
    camera_image_builder.add_data(data_offset);
    camera_image_builder.add_monotonic_timestamp_ns(
        image.monotonic_timestamp_ns());
    camera_image_builder.add_format(frc::vision::ImageFormat::MONO8);

    builder.CheckOk(builder.Send(camera_image_builder.Finish()));

    VLOG(1) << "Decompressed " << image.data()->size() << " bytes to " << width
            << "x" << height << " in "
            << std::chrono::duration<double>(
                   event_loop_->monotonic_now() -
                   event_loop_->context().monotonic_event_time)
                   .count()
            << "sec";
  }

  void SendStatus() {
    auto builder = status_sender_.MakeStaticBuilder();
    builder->set_successful_decodes(successful_decodes_);
    builder->set_failed_decodes(failed_decodes_);
    if (failed_decodes_ > 0) {
      auto error_fbs = builder->add_last_error_message();
      CHECK(error_fbs->reserve(last_error_message_.size()));
      error_fbs->SetString(std::string_view(last_error_message_.data(),
                                            last_error_message_.size()));
    }
    builder.CheckOk(builder.Send());
  }

  aos::EventLoop *event_loop_;
  tjhandle handle_;
  aos::Sender<CameraImage> camera_output_sender_;

  aos::Sender<TurboJpegDecoderStatusStatic> status_sender_;
  int successful_decodes_ = 0;
  int failed_decodes_ = 0;
  aos::InlinedVector<char, 128> last_error_message_;
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
