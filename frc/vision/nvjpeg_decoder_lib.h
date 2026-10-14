#ifndef FRC_VISION_NVJPEG_DECODER_LIB_H_
#define FRC_VISION_NVJPEG_DECODER_LIB_H_

#include <cstddef>
#include <cstdint>
#include <string_view>

// Thin wrapper around the Jetson NVJPG hardware JPEG decoder.
//
// Decodes through libnvjpeg.so -- NVIDIA's TEGRA_ACCELERATE build of
// libjpeg-8b shipped on the Orin image -- in hardware MJPEG stream mode,
// taking the decoded image from the dmabuf the library hands back (the path
// NVIDIA's own NvJPEGDecoder::decodeToFd uses).  The engine is reached
// through the tegra-drm render node (/dev/dri/renderD*) and /dev/nvmap on
// JetPack 6.  There is deliberately no CPU fallback: construction dies if no
// engine is bound to the tegra-nvjpg driver (or /dev/nvmap is inaccessible),
// and any decode dies if libnvjpeg reports it did not use the engine.  CPU
// decode is turbojpeg_decoder's job, selected explicitly in the config.
//
// Output is the luma (Y) plane only, i.e. grayscale MONO8, of the frame
// passed in: there is no pipeline delay and no warmup frame.
//
// Only baseline YCbCr 4:2:2 JPEGs -- what the UVC cameras send -- are
// accepted; anything else fails before it reaches the library, because
// libnvjpeg crashes on some formats (4:4:4) instead of reporting an error.
// All frames in one stream must share dimensions (true for our per-camera
// decoder processes); a size change fails that frame and restarts the
// stream.
//
// libnvjpeg is closed-source and has silently returned stale or empty
// pixels on other call sequences while reporting success, so callers should
// check the output against an independent decode (nvjpeg_seq_test does on
// real camera frames, and nvjpeg_decoder_verify on every live frame).
//
// Usage (per camera stream):
//   NvJpegDecoderLib decoder;
//   NvJpegDecoderLib::Result result;
//   if (decoder.DecodeToGray(jpeg_data, jpeg_size, gray_out, max_size,
//                            &result)) {
//     // result.width, result.height contain the decoded dimensions.
//     // gray_out contains width*height bytes of grayscale data.
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

  // Tears down and recreates the decoder's hardware stream, so the next
  // frame starts from clean library state and may have different
  // dimensions.
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

  // Copies the Y plane of the dmabuf libnvjpeg returned in |fd| into
  // |gray_out|.  Returns false (through FailFrame) if the buffer cannot be
  // read or does not match the decoded dimensions.
  bool CopyLumaFromDmabuf(int fd, uint32_t width, uint32_t height,
                          uint8_t *gray_out);

  // Per-frame failure path: formats the reason into the buffer last_error()
  // reads, logs it at VLOG(1), and rebuilds the stream so the next frame
  // starts clean.  Returns false so call sites can `return FailFrame(...)`.
  bool FailFrame(const char *format, ...) __attribute__((format(printf, 2, 3)));

  // Internal state (libjpeg decompress object, error manager, output
  // surface descriptor).
  struct Impl;
  Impl *impl_;
};

}  // namespace frc::vision

#endif  // FRC_VISION_NVJPEG_DECODER_LIB_H_
