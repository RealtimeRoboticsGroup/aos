#include "frc/vision/nvjpeg_decoder_lib.h"

#include <glob.h>
#include <setjmp.h>
#include <unistd.h>

#include <cerrno>
#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "nvbufsurface.h"

// libnvjpeg.so implements the libjpeg-8b API with NVIDIA's TEGRA_ACCELERATE
// extensions.  TEGRA_ACCELERATE (set via local_defines in the BUILD file)
// selects the extended struct layout that matches the library's ABI.
extern "C" {
#include <jpeglib.h>
}

// JPEG decode happens through libnvjpeg's hardware MJPEG stream mode
// (cinfo->mjpeg_decode = TRUE) with the output delivered as a dmabuf, the
// sequence NVIDIA's NvJPEGDecoder::decodeToFd uses on JetPack 6:
//
//   mjpeg_decode = TRUE -> jpeg_mem_src -> jpeg_read_header ->
//   IsVendorbuf = TRUE, pVendor_buf = &NvBufSurface -> jpeg_start_decompress
//   -> one jpeg_read_raw_data(NULL) -> jpeg_finish_decompress -> cinfo->fd
//   -> map the dmabuf, copy Y, unmap
//
// Every piece of that is load-bearing (verified empirically on L4T r36.4 by
// comparing each output to turbojpeg's decode of the same and the previous
// frame; nvjpeg_seq_test keeps checking the same-frame match):
//
//   * Without pVendor_buf, cinfo->fd stays -1.
//   * Without mjpeg_decode, only the FIRST decode of a decompress object's
//     lifetime produces pixels; later ones return a stale image while
//     reporting success.  (Letting jpeg_read_raw_data copy rows into our
//     own buffers has the same first-frame-only bug.)
//   * Reading the stream-mode surface through cinfo->jpegTegraMgr instead of
//     the dmabuf returns the PREVIOUS submission's frame (a one-frame
//     pipeline delay, plus an all-zero warmup frame after each restart).
//   * The fd stays the same number from frame to frame but fronts a
//     rotating set of buffers, so it must be looked up and mapped every
//     frame; a mapping kept across frames reads the wrong frame.
//
// On JetPack 6 / L4T 36 there is no /dev/nvhost-nvjpg chardev (that was the
// JetPack 5 interface): libnvjpeg reaches the engine through the tegra-drm
// render node (/dev/dri/renderD*) and /dev/nvmap, with the engine's client
// driver (tegra-nvjpg) living inside tegra-drm.ko.
//
// libnvjpeg has also been seen to block forever inside a decode
// (jpeg_read_raw_data -> jpegTegraDecoderRenderWait), but only while four
// decoder processes decoded synthetic frames in a startup self-test as the
// rest of the stack came up.  With that self-test gone, 2000 decoder starts
// and a 4 h, four-camera 120 Hz soak on the bench never hung.  If a hang
// shows up again, the process stays RUNNING but never publishes: look for
// a decoder whose main thread sleeps in a futex instead of the event loop's
// epoll wait.
//
// The library would decode on the CPU when the engine is unavailable; we
// deliberately fail loudly instead (at construction when no engine is bound
// to the tegra-nvjpg driver, and on any frame where libnvjpeg reports it
// did not use the engine) -- CPU decode is turbojpeg_decoder's job,
// selected explicitly in the config.

namespace frc::vision {
namespace {

// An NVJPG engine bound to its driver looks like
// /sys/bus/platform/drivers/tegra-nvjpg/15380000.nvjpg (the Orin has two).
constexpr char kNvjpgDriverGlob[] =
    "/sys/bus/platform/drivers/tegra-nvjpg/*.nvjpg";

// libjpeg reports errors through a callback that must not return; the
// standard client pattern is to longjmp back to the caller.
struct DecodeErrorMgr {
  struct jpeg_error_mgr pub;
  jmp_buf setjmp_buffer;
  char message[JMSG_LENGTH_MAX];
};

void ErrorExit(j_common_ptr cinfo) {
  DecodeErrorMgr *err = reinterpret_cast<DecodeErrorMgr *>(cinfo->err);
  (*cinfo->err->format_message)(cinfo, err->message);
  longjmp(err->setjmp_buffer, 1);
}

// Non-fatal libjpeg warnings (corrupt-data recovery etc.) -- keep them out
// of stderr but visible at higher verbosity.  The cameras' MJPEG frames
// routinely end in zero padding instead of an EOI marker, which triggers a
// harmless "Premature end of JPEG file" here on every frame.
void OutputMessage(j_common_ptr cinfo) {
  char message[JMSG_LENGTH_MAX];
  (*cinfo->err->format_message)(cinfo, message);
  ABSL_VLOG(1) << "libnvjpeg: " << message;
}

}  // namespace

struct NvJpegDecoderLib::Impl {
  struct jpeg_decompress_struct cinfo;
  DecodeErrorMgr err;

  // Handed to libnvjpeg through cinfo.pVendor_buf; without it the library
  // does not return an output dmabuf.  Zeroed before every decode, as
  // NvJPEGDecoder::decodeToFd passes a fresh one each call.
  NvBufSurface vendor_surface;

  bool logged_first_decode = false;

  // Set while RebuildDecompress runs so DecodeToGray's error handler can
  // tell a failure inside the rebuild itself apart from an ordinary decode
  // error (the former is unrecoverable and must not recurse).
  bool rebuilding = false;

  // Dimensions of the current hardware stream (0 until the first successful
  // decode after a (re)build).  The hardware stream is set up for these, so
  // a mid-stream dimension change fails that frame and restarts the stream.
  uint32_t stream_width = 0;
  uint32_t stream_height = 0;
};

NvJpegDecoderLib::NvJpegDecoderLib() : impl_(new Impl()) {
  memset(&impl_->cinfo, 0, sizeof(impl_->cinfo));
  memset(&impl_->err, 0, sizeof(impl_->err));
  impl_->cinfo.err = jpeg_std_error(&impl_->err.pub);
  impl_->err.pub.error_exit = ErrorExit;
  impl_->err.pub.output_message = OutputMessage;

  if (setjmp(impl_->err.setjmp_buffer)) {
    ABSL_LOG(FATAL) << "libnvjpeg failed to initialize: " << impl_->err.message;
  }
  jpeg_create_decompress(&impl_->cinfo);

  // libnvjpeg silently decodes on the CPU when the engine is unavailable.
  // Fail loudly instead: CPU decode is turbojpeg_decoder's job, selected
  // explicitly in the config, never an accident of a missing driver.
  glob_t engines;
  size_t engine_count = 0;
  if (glob(kNvjpgDriverGlob, 0, nullptr, &engines) == 0) {
    engine_count = engines.gl_pathc;
    globfree(&engines);
  }
  if (engine_count == 0) {
    ABSL_LOG(FATAL)
        << "No NVJPG engine is bound to the tegra-nvjpg driver ("
        << kNvjpgDriverGlob
        << " matched nothing).  Is the tegra-drm kernel module "
           "loaded?  Check lsmod / 'sudo modprobe tegra-drm' on the "
           "Orin.  For CPU decode, switch the config template to "
           "turbojpeg_decoder instead.";
  }
  if (access("/dev/nvmap", R_OK | W_OK) != 0) {
    ABSL_LOG(FATAL) << "/dev/nvmap is not accessible: " << strerror(errno)
                    << " -- libnvjpeg needs it (does this user have the video "
                       "group?)";
  }
  ABSL_LOG(INFO) << engine_count
                 << " NVJPG engine(s) bound; hardware decode available";
}

NvJpegDecoderLib::~NvJpegDecoderLib() {
  if (impl_ != nullptr) {
    if (setjmp(impl_->err.setjmp_buffer) == 0) {
      jpeg_destroy_decompress(&impl_->cinfo);
    }
    delete impl_;
  }
}

// Tears the decompress object down and rebuilds it: the next frame starts a
// fresh hardware stream.  After any failure the stream state is suspect, so
// every failure path goes through here.
void NvJpegDecoderLib::RebuildDecompress() {
  impl_->rebuilding = true;
  jpeg_decompress_struct *cinfo = &impl_->cinfo;
  jpeg_abort_decompress(cinfo);
  jpeg_destroy_decompress(cinfo);
  memset(cinfo, 0, sizeof(*cinfo));
  cinfo->err = &impl_->err.pub;
  jpeg_create_decompress(cinfo);
  impl_->stream_width = 0;
  impl_->stream_height = 0;
  impl_->rebuilding = false;
}

void NvJpegDecoderLib::ResetStream() {
  // destroy/create should never fail, so a longjmp out of libjpeg here is
  // fatal (same stance as the constructor).
  if (setjmp(impl_->err.setjmp_buffer)) {
    ABSL_LOG(FATAL) << "libnvjpeg failed to reset the decode stream: "
                    << impl_->err.message;
  }
  RebuildDecompress();
}

std::string_view NvJpegDecoderLib::last_error() const {
  return impl_->err.message;
}

bool NvJpegDecoderLib::FailFrame(const char *format, ...) {
  va_list args;
  va_start(args, format);
  vsnprintf(impl_->err.message, sizeof(impl_->err.message), format, args);
  va_end(args);
  ABSL_VLOG(1) << "JPEG decode failed: " << impl_->err.message;
  RebuildDecompress();
  return false;
}

bool NvJpegDecoderLib::CopyLumaFromDmabuf(int fd, uint32_t width,
                                          uint32_t height, uint8_t *gray_out) {
  if (fd < 0) {
    return FailFrame("libnvjpeg returned no output buffer (fd %d)", fd);
  }
  // Looked up and mapped every frame: the fd number is stable but the
  // buffer behind it rotates.
  NvBufSurface *surface = nullptr;
  if (NvBufSurfaceFromFd(fd, reinterpret_cast<void **>(&surface)) != 0 ||
      surface == nullptr || surface->numFilled < 1) {
    return FailFrame("no NvBufSurface behind output fd %d", fd);
  }
  const NvBufSurfaceParams &params = surface->surfaceList[0];
  // The buffer may be padded past the image (300 wide decodes into a
  // 304-wide buffer), but never smaller.
  if (params.width < width || params.height < height ||
      params.planeParams.pitch[0] < width) {
    return FailFrame("output buffer is %ux%u (pitch %u), image is %ux%u",
                     params.width, params.height, params.planeParams.pitch[0],
                     width, height);
  }
  if (NvBufSurfaceMap(surface, 0, 0, NVBUF_MAP_READ) != 0) {
    return FailFrame("failed to map output buffer fd %d", fd);
  }
  NvBufSurfaceSyncForCpu(surface, 0, 0);
  const uint8_t *luma = static_cast<const uint8_t *>(params.mappedAddr.addr[0]);
  const size_t pitch = params.planeParams.pitch[0];
  for (uint32_t row = 0; row < height; ++row) {
    memcpy(gray_out + static_cast<size_t>(row) * width,
           luma + static_cast<size_t>(row) * pitch, width);
  }
  NvBufSurfaceUnMap(surface, 0, 0);
  return true;
}

bool NvJpegDecoderLib::DecodeToGray(const uint8_t *jpeg_data, size_t jpeg_size,
                                    uint8_t *gray_out, size_t max_out_size,
                                    Result *result) {
  jpeg_decompress_struct *cinfo = &impl_->cinfo;

  if (setjmp(impl_->err.setjmp_buffer)) {
    // libjpeg hit a fatal decode error and longjmp'd back here.
    if (impl_->rebuilding) {
      // The failure happened inside RebuildDecompress itself
      // (destroy/create should never fail); re-entering the rebuild on a
      // half-initialized object cannot recover, so die loudly.
      ABSL_LOG(FATAL) << "libnvjpeg failed while rebuilding the decode stream: "
                      << impl_->err.message;
    }
    // The hardware stream context is in an undefined state after an error,
    // so tear the decompress object down and rebuild it: the next frame
    // starts a fresh stream.  The message is already in the last_error()
    // buffer (ErrorExit formatted it there).
    RebuildDecompress();
    ABSL_VLOG(1) << "JPEG decode failed: " << impl_->err.message;
    return false;
  }

  // Hardware MJPEG stream-decode mode; this is what NVIDIA's own nvjpegdec
  // GStreamer element uses for frame-after-frame camera streams.  Without
  // it, every decode after the first returns a stale image.
  cinfo->mjpeg_decode = TRUE;

  jpeg_mem_src(cinfo, const_cast<unsigned char *>(jpeg_data),
               static_cast<unsigned long>(jpeg_size));
  jpeg_read_header(cinfo, TRUE);

  const uint32_t width = cinfo->image_width;
  const uint32_t height = cinfo->image_height;
  // Only the cameras' format reaches the library: it crashes (in
  // jpeg_start_decompress) on some others, e.g. 4:4:4, instead of reporting
  // an error, and nothing else has been verified to decode correctly.
  if (cinfo->progressive_mode || cinfo->arith_code ||
      cinfo->data_precision != 8) {
    return FailFrame("Only baseline 8-bit Huffman JPEG is supported");
  }
  if (cinfo->jpeg_color_space != JCS_YCbCr || cinfo->num_components != 3 ||
      cinfo->comp_info[0].h_samp_factor != 2 ||
      cinfo->comp_info[0].v_samp_factor != 1 ||
      cinfo->comp_info[1].h_samp_factor != 1 ||
      cinfo->comp_info[1].v_samp_factor != 1 ||
      cinfo->comp_info[2].h_samp_factor != 1 ||
      cinfo->comp_info[2].v_samp_factor != 1) {
    return FailFrame(
        "Unsupported JPEG format (color space %d, %d components, Y %dx%d); "
        "only baseline YCbCr 4:2:2 is accepted",
        static_cast<int>(cinfo->jpeg_color_space), cinfo->num_components,
        cinfo->comp_info[0].h_samp_factor, cinfo->comp_info[0].v_samp_factor);
  }

  // The hardware stream is set up for its first frame's dimensions; on a
  // mid-stream change (camera re-enumerated at a new resolution?) fail this
  // frame and restart the stream rather than find out what the library
  // does with it.
  if (impl_->stream_width != 0 &&
      (width != impl_->stream_width || height != impl_->stream_height)) {
    return FailFrame("JPEG dimensions changed mid-stream: %ux%u vs %ux%u",
                     width, height, impl_->stream_width, impl_->stream_height);
  }
  if (static_cast<size_t>(width) * height > max_out_size) {
    return FailFrame("JPEG %ux%u exceeds output buffer (%zu bytes)", width,
                     height, max_out_size);
  }

  cinfo->out_color_space = JCS_YCbCr;
  cinfo->IsVendorbuf = TRUE;
  memset(&impl_->vendor_surface, 0, sizeof(impl_->vendor_surface));
  cinfo->pVendor_buf =
      reinterpret_cast<unsigned char *>(&impl_->vendor_surface);

  jpeg_start_decompress(cinfo);
  // In vendor-buffer mode one call decodes the whole frame into the dmabuf;
  // there are no rows to hand over.
  jpeg_read_raw_data(cinfo, nullptr,
                     cinfo->comp_info[0].v_samp_factor * DCTSIZE);

  // Read hardware state before finish -- finish/abort may reset it.
  //
  // tegra_acceleration is libnvjpeg's own report of the path it took.  The
  // engine being bound is not enough -- refuse to masquerade as a hardware
  // decoder while burning CPU.
  if (cinfo->tegra_acceleration == 0) {
    ABSL_LOG(FATAL)
        << "libnvjpeg decoded on the CPU even though an NVJPG engine "
           "is bound -- refusing to run without hardware decode.  For "
           "CPU decode, switch the config template to "
           "turbojpeg_decoder instead.";
  }

  jpeg_finish_decompress(cinfo);

  if (!CopyLumaFromDmabuf(cinfo->fd, width, height, gray_out)) {
    return false;
  }

  if (!impl_->logged_first_decode) {
    impl_->logged_first_decode = true;
    ABSL_LOG(INFO) << "First JPEG decoded (" << width << "x" << height
                   << ") on the NVJPG engine";
  }

  impl_->stream_width = width;
  impl_->stream_height = height;

  result->width = width;
  result->height = height;
  return true;
}

}  // namespace frc::vision
