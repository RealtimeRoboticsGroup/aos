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

#include <algorithm>
#include <chrono>
#include <optional>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
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
          "reduce GPU load.  NOTE: the hardware pipeline delivers each frame "
          "on the NEXT submission, so every skipped frame also adds a full "
          "frame period to the age of published images -- keep the "
          "detector's --max_image_age_ms comfortably above "
          "(skip + 1) * frame_period plus decode time, or vision goes "
          "silently dark.");

ABSL_FLAG(uint32_t, max_image_age_ms, 50,
          "Skip images older than this.  Keeps a (re)starting decoder from "
          "burning through a queued backlog faster than the output channel's "
          "configured frequency allows.");

namespace frc::vision {

class NvjpegDecoder {
 public:
  NvjpegDecoder(aos::EventLoop *event_loop)
      : event_loop_(event_loop),
        camera_output_sender_(event_loop_->MakeSender<CameraImage>(
            absl::StrCat(absl::GetFlag(FLAGS_channel), "/gray"))),
        status_sender_(event_loop_->MakeSender<TurboJpegDecoderStatusStatic>(
            absl::StrCat(absl::GetFlag(FLAGS_channel), "/gray"))) {
    // AOS kills senders that exceed frequency * channel_storage_duration
    // sends within the storage window, so derive our own send budget from
    // the output channel's configured limits and stay one under it.  The
    // helper resolves the per-channel override against the config-wide
    // default (the raw flatbuffer accessor cannot distinguish unset from an
    // explicit 2s because the field default is 2000000000).  The send-time
    // ring buffer is preallocated here: the event loop runs realtime, where
    // allocation is fatal.
    const aos::Channel *out_channel = camera_output_sender_.channel();
    send_window_ = aos::configuration::ChannelStorageDuration(
        event_loop_->configuration(), out_channel);
    const int64_t max_sends_in_window = std::max<int64_t>(
        1, static_cast<int64_t>(
               out_channel->frequency() *
               std::chrono::duration<double>(send_window_).count()) -
               1);
    send_times_.resize(max_sends_in_window, aos::monotonic_clock::min_time);

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
    ABSL_CHECK(image.format() == ImageFormat::MJPEG)
        << ": Expected MJPEG format but got: "
        << EnumNameImageFormat(image.format());

    // Skip frames that are already stale, most importantly the backlog a
    // watcher drains when this process starts (or restarts) while the
    // camera is streaming.  Decoding a backlog as fast as the engine
    // allows would exceed the channel's configured frequency and get this
    // process killed for sending too fast -- and the detector would drop
    // old frames anyway.
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
        // The library rebuilt its hardware stream; the pipelined frame it
        // was holding is gone with it.
        pending_timestamp_ns_.reset();
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
      pending_timestamp_ns_.reset();
      constexpr std::string_view kError =
          "decoded dimensions do not match the camera image";
      last_error_message_.resize(kError.size());
      memcpy(last_error_message_.data(), kError.data(), kError.size());
      ABSL_VLOG(1) << kError << ": " << result.width << "x" << result.height
                   << " vs " << cols << "x" << rows;
      {
        // The frame we just submitted is being discarded: rebuild the
        // hardware stream so its pixels cannot be read back later.
        // jpeg_create_decompress allocates, which is fatal in this realtime
        // loop without the escape hatch.
        aos::ScopedNotRealtime nrt;
        hw_decoder_.ResetStream();
      }
      return;
    }
    ++successful_decodes_;

    // The hardware pipelines: the pixels we just got are normally the frame
    // submitted on the PREVIOUS call (see nvjpeg_decoder_lib.h), so publish
    // them with that call's timestamp.  Right after (re)start there is no
    // previous frame and the output is the current one.
    const int64_t publish_timestamp_ns =
        pending_timestamp_ns_.value_or(image.monotonic_timestamp_ns());
    pending_timestamp_ns_ = image.monotonic_timestamp_ns();

    // An all-zero frame is the pipeline's warmup bubble (or a genuinely
    // black image, which is equally useless to the detector): skip
    // publishing but keep the timestamp pairing above.  Sampling is the
    // cheap first pass; the full scan below runs only on candidate-zero
    // frames (warmup bubbles and genuinely black images), so steady-state
    // cost is unchanged.  The warmup behavior (bubble count and position
    // vary run to run, so it cannot be skipped by decode index) is
    // empirical against the closed-source engine on JetPack 6 / L4T
    // r36.4; re-profile it when we migrate to JetPack 7.
    bool all_zero = true;
    for (size_t i = 0; i < gray_size; i += 997) {
      if (image_data_ptr[i] != 0) {
        all_zero = false;
        break;
      }
    }
    if (all_zero) {
      // Confirm with a full scan: a frame the sample missed nonzero bytes
      // in must still be published.  (Realtime path: no allocation, no
      // logging.)
      for (size_t i = 0; i < gray_size; ++i) {
        if (image_data_ptr[i] != 0) {
          all_zero = false;
          break;
        }
      }
    }
    if (all_zero) {
      // Deliberate: a genuinely black scene (lens cap, dark pit) makes the
      // gray channel go silent rather than carry black frames.  The
      // skipped_zero_frames status counter is the discriminator between
      // "black scene" and "dead channel".  No logging on this realtime path.
      ++skipped_zero_frames_;
      return;
    }

    // Stay under the channel's configured send budget; dropping a frame
    // beats getting killed for sending too fast.  send_times_ is a ring of
    // the last N ACCEPTED send times where N is the budget: if the oldest
    // entry is still inside the window, sending now would be the N+1th send
    // in it.  The slot is consumed only after AOS accepts the send, so a
    // rejected send does not shrink the future budget.
    const aos::monotonic_clock::time_point now = event_loop_->monotonic_now();
    const aos::monotonic_clock::time_point oldest = send_times_[send_index_];
    if (oldest != aos::monotonic_clock::min_time &&
        now - oldest <= send_window_) {
      ++dropped_sends_;
      return;
    }

    CameraImage::Builder camera_image_builder(*builder.fbb());

    camera_image_builder.add_rows(rows);
    camera_image_builder.add_cols(cols);
    camera_image_builder.add_data(data_offset);
    camera_image_builder.add_monotonic_timestamp_ns(publish_timestamp_ns);
    camera_image_builder.add_format(frc::vision::ImageFormat::MONO8);

    // kMessagesSentTooFast should be impossible given the budget above (it
    // can still happen right after a restart while the queue holds the
    // previous instance's sends), but a dropped frame must never crash-loop
    // the decoder mid-match.  The frame already counted as a successful
    // decode; the lost send is counted in dropped_sends.  (No logging here:
    // this runs realtime, where allocation is fatal.)
    const aos::RawSender::Error send_error =
        builder.Send(camera_image_builder.Finish());
    if (send_error == aos::RawSender::Error::kMessagesSentTooFast) {
      ++dropped_sends_;
      return;
    }
    builder.CheckOk(send_error);
    send_times_[send_index_] = now;
    send_index_ = (send_index_ + 1) % send_times_.size();

    // Track the worst-case age of published pixels (pipeline delay
    // included) so an age-gate mismatch with the detector shows up on the
    // status channel instead of as a silent vision outage.
    const int64_t age_ms =
        (std::chrono::duration_cast<std::chrono::nanoseconds>(
             now.time_since_epoch())
             .count() -
         publish_timestamp_ns) /
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

  void SendStatus() {
    auto builder = status_sender_.MakeStaticBuilder();
    builder->set_successful_decodes(successful_decodes_);
    builder->set_failed_decodes(failed_decodes_);
    builder->set_skipped_zero_frames(skipped_zero_frames_);
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
    skipped_zero_frames_ = 0;
    skipped_stale_frames_ = 0;
    dropped_sends_ = 0;
    max_publish_age_ms_ = 0;
  }

  aos::EventLoop *event_loop_;
  NvJpegDecoderLib hw_decoder_;
  aos::Sender<CameraImage> camera_output_sender_;

  // Timestamp of the frame most recently submitted to the hardware, i.e.
  // the frame whose pixels the NEXT decode will hand back.
  std::optional<int64_t> pending_timestamp_ns_;

  // Preallocated ring of the most recent send times (capacity = the send
  // budget within send_window_); no allocation in the realtime loop.
  std::vector<aos::monotonic_clock::time_point> send_times_;
  size_t send_index_ = 0;
  std::chrono::nanoseconds send_window_{0};

  aos::Sender<TurboJpegDecoderStatusStatic> status_sender_;
  int successful_decodes_ = 0;
  int failed_decodes_ = 0;
  int skipped_zero_frames_ = 0;
  int skipped_stale_frames_ = 0;
  int dropped_sends_ = 0;
  int64_t max_publish_age_ms_ = 0;
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
