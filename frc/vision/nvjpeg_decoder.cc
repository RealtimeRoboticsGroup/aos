// NVJPG Hardware JPEG Decoder for Jetson Orin.
//
// Drop-in alternative to turbojpeg_decoder that decodes through libnvjpeg on
// the Orin's dedicated NVJPG hardware engine.  Dies loudly at startup (or on
// any frame) if the engine is unavailable -- no silent CPU fallback; CPU
// decode is turbojpeg_decoder's job.  Mirrors turbojpeg_decoder's interface
// exactly -- same flags, same MONO8 output and TurboJpegDecoderStatus on the
// same /cameraX/gray channel -- so swapping decoders is just an
// executable_name change in the config.
//
// The hardware decoder is most effective under sustained multi-camera load,
// where the engine does the decode work while CPU cores stay free for
// AprilTag detection and pose estimation.  Measured on an Orin Nano
// (JetPack 6.2), 1280x800 4:2:2 UVC frames: 2.3 ms/frame wall time with
// ~0.2 ms of CPU, vs 2.8 ms/frame all-CPU for turbojpeg.

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
#include "frc/vision/nvjpeg_decoder_lib.h"
#include "frc/vision/turbojpeg_decoder_status_static.h"
#include "frc/vision/vision_generated.h"

ABSL_FLAG(std::string, config, "aos_config.json",
          "File path of aos configuration");
ABSL_FLAG(std::string, channel, "/camera", "Channel name for the camera.");

ABSL_FLAG(uint32_t, skip, 0,
          "Number of images to skip to reduce the framerate of inference to "
          "reduce GPU load.");

namespace frc::vision {

class NvjpegDecoder {
 public:
  NvjpegDecoder(aos::EventLoop *event_loop)
      : event_loop_(event_loop),
        camera_output_sender_(event_loop_->MakeSender<CameraImage>(
            absl::StrCat(absl::GetFlag(FLAGS_channel), "/gray"))),
        status_sender_(event_loop_->MakeSender<TurboJpegDecoderStatusStatic>(
            absl::StrCat(absl::GetFlag(FLAGS_channel), "/gray"))) {
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

 private:
  void ProcessImage(const CameraImage &image) {
    CHECK(image.format() == ImageFormat::MJPEG)
        << ": Expected MJPEG format but got: "
        << EnumNameImageFormat(image.format());

    if (skip_ != 0) {
      --skip_;
      return;
    } else {
      skip_ = absl::GetFlag(FLAGS_skip);
    }

    // Decode straight into the outgoing flatbuffer: the inbound camera image
    // carries its dimensions, so the output vector can be allocated up front
    // and the engine's rows land directly in the message.
    if (image.rows() <= 0 || image.cols() <= 0) {
      ++failed_decodes_;
      constexpr std::string_view kError = "camera image is missing dimensions";
      last_error_message_.resize(kError.size());
      memcpy(last_error_message_.data(), kError.data(), kError.size());
      VLOG(1) << kError;
      return;
    }
    const uint32_t rows = static_cast<uint32_t>(image.rows());
    const uint32_t cols = static_cast<uint32_t>(image.cols());
    const size_t gray_size = static_cast<size_t>(rows) * cols;

    auto builder = camera_output_sender_.MakeBuilder();
    uint8_t *image_data_ptr = nullptr;
    flatbuffers::Offset<flatbuffers::Vector<uint8_t>> data_offset =
        builder.fbb()->CreateUninitializedVector(gray_size, 1, &image_data_ptr);

    // On failure the builder is dropped without Send, like turbojpeg_decoder.
    NvJpegDecoderLib::Result result;
    {
      aos::ScopedNotRealtime nrt;
      if (!hw_decoder_.DecodeToGray(image.data()->data(), image.data()->size(),
                                    image_data_ptr, gray_size, &result)) {
        ++failed_decodes_;
        constexpr std::string_view kError = "NVJPG hardware decode failed";
        last_error_message_.resize(kError.size());
        memcpy(last_error_message_.data(), kError.data(), kError.size());
        VLOG(1) << kError;
        return;
      }
    }
    if (result.width != cols || result.height != rows) {
      ++failed_decodes_;
      constexpr std::string_view kError =
          "decoded dimensions do not match the camera image";
      last_error_message_.resize(kError.size());
      memcpy(last_error_message_.data(), kError.data(), kError.size());
      VLOG(1) << kError << ": " << result.width << "x" << result.height
              << " vs " << cols << "x" << rows;
      return;
    }
    ++successful_decodes_;

    CameraImage::Builder camera_image_builder(*builder.fbb());

    camera_image_builder.add_rows(rows);
    camera_image_builder.add_cols(cols);
    camera_image_builder.add_data(data_offset);
    camera_image_builder.add_monotonic_timestamp_ns(
        image.monotonic_timestamp_ns());
    camera_image_builder.add_format(frc::vision::ImageFormat::MONO8);

    builder.CheckOk(builder.Send(camera_image_builder.Finish()));

    VLOG(1) << "NVJPG decoded " << image.data()->size() << " bytes to "
            << result.width << "x" << result.height << " in "
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
    // Reset counters for next status message.
    successful_decodes_ = 0;
    failed_decodes_ = 0;
  }

  aos::EventLoop *event_loop_;
  NvJpegDecoderLib hw_decoder_;
  aos::Sender<CameraImage> camera_output_sender_;

  aos::Sender<TurboJpegDecoderStatusStatic> status_sender_;
  int successful_decodes_ = 0;
  int failed_decodes_ = 0;
  aos::InlinedVector<char, 128> last_error_message_;

  size_t skip_ = 0;
};

int Main() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  aos::ShmEventLoop event_loop(&config.message());

  event_loop.SetRuntimeRealtimePriority(5);

  NvjpegDecoder nvjpeg_decoder(&event_loop);

  event_loop.Run();

  return 0;
}

}  // namespace frc::vision

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  return frc::vision::Main();
}
