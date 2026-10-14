#include "frc/vision/nvjpeg_decoder_service.h"

#include <string.h>

#include <algorithm>
#include <chrono>

#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "absl/strings/str_cat.h"

#include "aos/realtime.h"

ABSL_FLAG(std::string, channel, "/camera", "Channel name for the camera.");

ABSL_FLAG(uint32_t, skip, 0,
          "Number of images to skip to reduce the framerate of inference to "
          "reduce GPU load.");

ABSL_FLAG(uint32_t, max_image_age_ms, 50,
          "Skip images older than this.  Keeps a (re)starting decoder from "
          "burning through a queued backlog faster than the output channel's "
          "configured frequency allows.");

namespace frc::vision {

NvjpegDecoder::NvjpegDecoder(aos::EventLoop *event_loop)
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

void NvjpegDecoder::ProcessImage(const CameraImage &image) {
  ABSL_CHECK(image.format() == ImageFormat::MJPEG)
      << ": Expected MJPEG format but got: "
      << EnumNameImageFormat(image.format());

  // Skip frames that are already stale, most importantly the backlog a
  // watcher drains when this process starts (or restarts) while the
  // camera is streaming.  Decoding a backlog as fast as the engine
  // allows would exceed the channel's configured frequency, so AOS would
  // refuse most of the sends -- and the detector would drop old frames
  // anyway.
  if (event_loop_->monotonic_now() -
          event_loop_->context().monotonic_event_time >
      std::chrono::milliseconds(absl::GetFlag(FLAGS_max_image_age_ms))) {
    // No logging: this path runs realtime, where allocation is fatal.
    // The skip is visible as skipped_stale_frames on the status channel.
    ++skipped_stale_frames_;
    return;
  }

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
    ABSL_VLOG(1) << kError;
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
      // The library only VLOGs failures; the status channel carries the
      // reason (truncated to the message field's static_length).
      const std::string_view error = hw_decoder_.last_error();
      const size_t truncated_len =
          std::min(error.size(), last_error_message_.capacity());
      last_error_message_.resize(truncated_len);
      memcpy(last_error_message_.data(), error.data(), truncated_len);
      return;
    }
  }
  if (result.width != cols || result.height != rows) {
    ++failed_decodes_;
    constexpr std::string_view kError =
        "decoded dimensions do not match the camera image";
    last_error_message_.resize(kError.size());
    memcpy(last_error_message_.data(), kError.data(), kError.size());
    ABSL_VLOG(1) << kError << ": " << result.width << "x" << result.height
                 << " vs " << cols << "x" << rows;
    return;
  }
  ++successful_decodes_;

  CheckDecode(image, image_data_ptr, rows, cols);

  CameraImage::Builder camera_image_builder(*builder.fbb());

  camera_image_builder.add_rows(rows);
  camera_image_builder.add_cols(cols);
  camera_image_builder.add_data(data_offset);
  camera_image_builder.add_monotonic_timestamp_ns(
      image.monotonic_timestamp_ns());
  camera_image_builder.add_format(frc::vision::ImageFormat::MONO8);

  // AOS refuses a send that would exceed the output channel's configured
  // frequency and counts it in the timing report.  Drop that frame rather
  // than crash-looping the decoder mid-match; the frame already counted as
  // a successful decode, and the lost send is counted in dropped_sends.
  // (No logging here: this runs realtime, where allocation is fatal.)
  const aos::RawSender::Error send_error =
      builder.Send(camera_image_builder.Finish());
  if (send_error == aos::RawSender::Error::kMessagesSentTooFast) {
    ++dropped_sends_;
    return;
  }
  builder.CheckOk(send_error);

  const aos::monotonic_clock::time_point now = event_loop_->monotonic_now();
  // Track the worst-case age of published pixels so an age-gate mismatch
  // with the detector shows up on the status channel instead of as a
  // silent vision outage.
  const int64_t age_ms = (std::chrono::duration_cast<std::chrono::nanoseconds>(
                              now.time_since_epoch())
                              .count() -
                          image.monotonic_timestamp_ns()) /
                         1000000;
  max_publish_age_ms_ = std::max(max_publish_age_ms_, age_ms);

  ABSL_VLOG(1) << "NVJPG decoded " << image.data()->size() << " bytes to "
               << result.width << "x" << result.height << " in "
               << std::chrono::duration<double>(
                      event_loop_->monotonic_now() -
                      event_loop_->context().monotonic_event_time)
                      .count()
               << "sec";
}

void NvjpegDecoder::SendStatus() {
  auto builder = status_sender_.MakeStaticBuilder();
  builder->set_successful_decodes(successful_decodes_);
  builder->set_failed_decodes(failed_decodes_);
  builder->set_skipped_stale_frames(skipped_stale_frames_);
  builder->set_dropped_sends(dropped_sends_);
  builder->set_max_publish_age_ms(max_publish_age_ms_);
  if (failed_decodes_ > 0) {
    auto error_fbs = builder->add_last_error_message();
    ABSL_CHECK(error_fbs->reserve(last_error_message_.size()));
    error_fbs->SetString(std::string_view(last_error_message_.data(),
                                          last_error_message_.size()));
  }
  builder.CheckOk(builder.Send());
  // Reset counters for next status message.
  successful_decodes_ = 0;
  failed_decodes_ = 0;
  skipped_stale_frames_ = 0;
  dropped_sends_ = 0;
  max_publish_age_ms_ = 0;
}

}  // namespace frc::vision
