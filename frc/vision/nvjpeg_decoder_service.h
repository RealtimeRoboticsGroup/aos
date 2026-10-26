#ifndef FRC_VISION_NVJPEG_DECODER_SERVICE_H_
#define FRC_VISION_NVJPEG_DECODER_SERVICE_H_

#include <cstdint>

#include "aos/containers/inlined_vector.h"
#include "aos/events/event_loop.h"
#include "frc/vision/nvjpeg_decoder_lib.h"
#include "frc/vision/turbojpeg_decoder_status_static.h"
#include "frc/vision/vision_generated.h"

namespace frc::vision {

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
// (JetPack 6, L4T r36.4), 1280x800 4:2:2 UVC frames at 120 Hz: 3.56 ms
// per frame and 0.19 cores per camera, vs 3.3 ms and 0.40 cores for
// turbojpeg.
//
// libnvjpeg is closed-source and has returned stale or delayed pixels while
// reporting success on other call sequences.  This class does not check its
// output itself, so the nvjpeg_decoder service adds no CPU decode load:
// nvjpeg_seq_test checks the library against turbojpeg on real camera
// frames, and nvjpeg_decoder_verify overrides CheckDecode to also decode
// every frame with turbojpeg and die if the hardware output does not match.
//
// There is deliberately no startup self-test on synthetic frames: decoding
// them while the rest of the stack starts made libnvjpeg hang in about 8% of
// decoder starts on the bench, and 0 of 96 without it.
class NvjpegDecoder {
 public:
  explicit NvjpegDecoder(aos::EventLoop *event_loop);
  virtual ~NvjpegDecoder() = default;

 protected:
  // Called with every successfully decoded frame before it is published, on
  // the realtime path.  |gray| holds rows * cols pixels.
  virtual void CheckDecode(const CameraImage & /*image*/,
                           const uint8_t * /*gray*/, uint32_t /*rows*/,
                           uint32_t /*cols*/) {}

 private:
  void ProcessImage(const CameraImage &image);
  void SendStatus();

  aos::EventLoop *event_loop_;
  NvJpegDecoderLib hw_decoder_;
  aos::Sender<CameraImage> camera_output_sender_;

  aos::Sender<TurboJpegDecoderStatusStatic> status_sender_;
  int successful_decodes_ = 0;
  int failed_decodes_ = 0;
  int skipped_stale_frames_ = 0;
  int dropped_sends_ = 0;
  int64_t max_publish_age_ms_ = 0;
  aos::InlinedVector<char, 128> last_error_message_;

  size_t skip_ = 0;
};

}  // namespace frc::vision

#endif  // FRC_VISION_NVJPEG_DECODER_SERVICE_H_
