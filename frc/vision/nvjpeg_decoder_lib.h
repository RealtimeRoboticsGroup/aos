#ifndef FRC_VISION_NVJPEG_DECODER_LIB_H_
#define FRC_VISION_NVJPEG_DECODER_LIB_H_

#include <cstddef>
#include <cstdint>
#include <string_view>

// Thin wrapper around the Jetson NVJPG hardware JPEG decoder.
//
// Decodes through libnvjpeg.so -- NVIDIA's TEGRA_ACCELERATE build of
// libjpeg-8b shipped on the Orin image -- in hardware MJPEG stream mode,
// which drives the NVJPG engine through the tegra-drm render node
// (/dev/dri/renderD*) and /dev/nvmap on JetPack 6.  This is the same path
// NVIDIA's own nvjpegdec GStreamer plugin uses.  There is deliberately no
// CPU fallback: construction dies if no engine is bound to the tegra-nvjpg
// driver (or /dev/nvmap is inaccessible), and any decode dies if libnvjpeg
// reports it did not use the engine.  CPU decode is turbojpeg_decoder's
// job, selected explicitly in the config.
//
// Output is the luma (Y) plane only, i.e. grayscale MONO8.
//
// PIPELINING -- the caller MUST handle this: the engine decodes as a
// pipelined stream, so in steady state the pixels written to |gray_out|
// are the frame submitted on the PREVIOUS DecodeToGray call, not the one
// passed in.  The first call after construction (or after ANY false
// return, all of which rebuild the stream, discarding the in-flight
// frame) returns its own frame; the second typically
// returns an all-zero warmup frame that should be dropped.  Callers that
// care about timestamps must therefore pair each output with the previous
// call's timestamp, and skip all-zero frames.  All frames in one stream
// must share dimensions (true for our per-camera decoder processes).
//
// Usage (per camera stream):
//   NvJpegDecoderLib decoder;
//   NvJpegDecoderLib::Result result;
//   if (decoder.DecodeToGray(jpeg_data, jpeg_size, gray_out, max_size,
//                            &result)) {
//     // result.width, result.height contain the decoded dimensions.
//     // gray_out contains width*height bytes of grayscale data --
//     // normally the PREVIOUS call's frame; all-zero means warmup.
//   }
namespace frc::vision {

class NvJpegDecoderLib {
 public:
  struct Result {
    uint32_t width = 0;
    uint32_t height = 0;
  };

  NvJpegDecoderLib();
  ~NvJpegDecoderLib();

  NvJpegDecoderLib(const NvJpegDecoderLib &) = delete;
  NvJpegDecoderLib &operator=(const NvJpegDecoderLib &) = delete;

  // Decodes a JPEG image and extracts the Y-plane (grayscale) into
  // |gray_out|.
  //
  // |jpeg_data|: pointer to the compressed JPEG data.
  // |jpeg_size|: size of the compressed JPEG data in bytes.
  // |gray_out|:  pointer to the destination buffer for grayscale output.
  //              Must be at least width*height bytes.
  // |max_out_size|: size of the gray_out buffer.
  // |result|:    populated with decoded width/height on success.
  //
  // Returns true on success, false on decode failure.
  bool DecodeToGray(const uint8_t *jpeg_data, size_t jpeg_size,
                    uint8_t *gray_out, size_t max_out_size, Result *result);

  // Tears down and recreates the decoder's hardware stream.  Call after
  // discarding a submitted frame (e.g. metadata mismatch) so a stale frame
  // cannot be read back later; the next decode is synchronous, followed by
  // the usual all-zero warmup frame.
  void ResetStream();

  // Human-readable reason for the most recent DecodeToGray() failure (empty
  // before the first failure); valid until the next DecodeToGray() call.
  // Per-frame failures log at VLOG(1) only -- the status channel, fed from
  // this, is the durable record -- so callers are expected to surface this
  // message there.
  std::string_view last_error() const;

 private:
  // Tears down the decompress object and rebuilds it from scratch, starting
  // a fresh hardware stream.
  void RebuildDecompress();

  // Per-frame failure path: formats the reason into the buffer last_error()
  // reads, logs it at VLOG(1), and rebuilds the stream so the next frame
  // starts clean.  Returns false so call sites can `return FailFrame(...)`.
  bool FailFrame(const char *format, ...) __attribute__((format(printf, 2, 3)));

  // Internal state (libjpeg decompress object, error manager, scratch rows).
  struct Impl;
  Impl *impl_;
};

}  // namespace frc::vision

#endif  // FRC_VISION_NVJPEG_DECODER_LIB_H_
