#ifndef FRC_VISION_NVJPEG_DECODER_LIB_H_
#define FRC_VISION_NVJPEG_DECODER_LIB_H_

#include <cstddef>
#include <cstdint>

// Thin wrapper around the Jetson NVJPG hardware JPEG decoder.
//
// Decodes through libnvjpeg.so -- NVIDIA's TEGRA_ACCELERATE build of
// libjpeg-8b shipped on the Orin image -- which drives the NVJPG engine
// through the tegra-drm render node (/dev/dri/renderD*) and /dev/nvmap
// on JetPack 6.  This is the same path NVIDIA's own nvjpegdec GStreamer
// plugin uses.  There is deliberately no CPU fallback: construction dies
// if no engine is bound to the tegra-nvjpg driver (or /dev/nvmap is
// inaccessible), and the first decode dies if libnvjpeg reports it did
// not use the engine.  CPU decode is turbojpeg_decoder's job, selected
// explicitly in the config.
//
// Output is the luma (Y) plane only, i.e. grayscale MONO8.
//
// Usage:
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

 private:
  // Internal state (libjpeg decompress object, error manager, scratch rows).
  struct Impl;
  Impl *impl_;
};

}  // namespace frc::vision

#endif  // FRC_VISION_NVJPEG_DECODER_LIB_H_
