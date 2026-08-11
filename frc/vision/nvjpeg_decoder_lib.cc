#include "frc/vision/nvjpeg_decoder_lib.h"

#include <glob.h>
#include <setjmp.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <vector>

#include "absl/log/check.h"
#include "absl/log/log.h"

// libnvjpeg.so implements the libjpeg-8b API with NVIDIA's TEGRA_ACCELERATE
// extensions.  TEGRA_ACCELERATE (set via local_defines in the BUILD file)
// selects the extended struct layout that matches the library's ABI.
extern "C" {
#include <jpeglib.h>
}

// JPEG decode happens through libnvjpeg's hardware MJPEG stream mode
// (cinfo->mjpeg_decode = TRUE), reading the decoded luma plane from the
// hardware output surface (cinfo->jpegTegraMgr->buff[0] / pitch[0]):
//
//   mjpeg_decode = TRUE -> jpeg_mem_src -> jpeg_read_header ->
//   raw_data_out = TRUE -> jpeg_start_decompress ->
//   jpeg_read_raw_data per iMCU row group (into discard rows, only to
//   advance the scanline state machine) -> copy Y from jpegTegraMgr ->
//   jpeg_finish_decompress
//
// The obvious-looking alternative -- letting jpeg_read_raw_data copy the
// rows -- is broken in this library: it only produces pixels for the FIRST
// decode of a decompress object's lifetime, and silently writes nothing on
// every later decode while still reporting success (verified empirically on
// L4T r36.4 with nvjpeg_seq_test; a same-file-in-a-loop benchmark cannot
// see this, which is how it originally slipped through).
//
// In stream mode the engine pipelines: the surface read back after
// submitting frame N contains frame N-1 (the first decode after object
// creation is synchronous, and the second typically yields an all-zero
// warmup frame).  See the header for the caller-visible contract.
//
// On JetPack 6 / L4T 36 there is no /dev/nvhost-nvjpg chardev (that was the
// JetPack 5 interface): libnvjpeg reaches the engine through the tegra-drm
// render node (/dev/dri/renderD*) and /dev/nvmap, with the engine's client
// driver (tegra-nvjpg) living inside tegra-drm.ko.
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
  VLOG(1) << "libnvjpeg: " << message;
}

}  // namespace

struct NvJpegDecoderLib::Impl {
  struct jpeg_decompress_struct cinfo;
  DecodeErrorMgr err;

  // One row of DCT-padded scratch.  All raw-data rows land here: the real
  // pixels are read from the hardware surface, the raw-data calls exist
  // only to run the scanline state machine forward.
  std::vector<uint8_t> discard_row;

  bool logged_first_decode = false;

  // Set while RebuildDecompress runs so DecodeToGray's error handler can
  // tell a failure inside the rebuild itself apart from an ordinary decode
  // error (the former is unrecoverable and must not recurse).
  bool rebuilding = false;

  // Dimensions of the current hardware stream (0 until the first successful
  // decode after a (re)build).  The output surface is sized for these, so a
  // mid-stream dimension change must restart the stream before readback.
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
    LOG(FATAL) << "libnvjpeg failed to initialize: " << impl_->err.message;
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
    LOG(FATAL) << "No NVJPG engine is bound to the tegra-nvjpg driver ("
               << kNvjpgDriverGlob
               << " matched nothing).  Is the tegra-drm kernel module "
                  "loaded?  Check lsmod / 'sudo modprobe tegra-drm' on the "
                  "Orin.  For CPU decode, switch the config template to "
                  "turbojpeg_decoder instead.";
  }
  if (access("/dev/nvmap", R_OK | W_OK) != 0) {
    LOG(FATAL) << "/dev/nvmap is not accessible: " << strerror(errno)
               << " -- libnvjpeg needs it (does this user have the video "
                  "group?)";
  }
  LOG(INFO) << engine_count
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
// fresh hardware stream (its first decode is synchronous, then the pipeline
// refills through the usual warmup frame).  The rebuild deliberately
// discards the pipelined in-flight frame -- after any failure the stream
// state is suspect, and guaranteed-correct timestamp pairing afterwards is
// worth the one or two extra lost frames on what should be a rare event.
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
    LOG(FATAL) << "libnvjpeg failed to reset the decode stream: "
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
  VLOG(1) << "JPEG decode failed: " << impl_->err.message;
  RebuildDecompress();
  return false;
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
      LOG(FATAL) << "libnvjpeg failed while rebuilding the decode stream: "
                 << impl_->err.message;
    }
    // The hardware stream context is in an undefined state after an error,
    // so tear the decompress object down and rebuild it: the next frame
    // starts a fresh stream (its first decode is synchronous, then the
    // pipeline refills through the usual warmup frame).  The message is
    // already in the last_error() buffer (ErrorExit formatted it there).
    RebuildDecompress();
    VLOG(1) << "JPEG decode failed: " << impl_->err.message;
    return false;
  }

  // Hardware MJPEG stream-decode mode; this is what NVIDIA's own nvjpegdec
  // GStreamer element uses for frame-after-frame camera streams.
  cinfo->mjpeg_decode = TRUE;

  jpeg_mem_src(cinfo, const_cast<unsigned char *>(jpeg_data),
               static_cast<unsigned long>(jpeg_size));
  jpeg_read_header(cinfo, TRUE);

  const uint32_t width = cinfo->image_width;
  const uint32_t height = cinfo->image_height;
  // The output surface is sized for the stream's dimensions; a mid-stream
  // dimension change (camera re-enumerated at a new resolution?) would make
  // the readback below use the wrong geometry, so restart the stream.
  if (impl_->stream_width != 0 &&
      (width != impl_->stream_width || height != impl_->stream_height)) {
    return FailFrame("JPEG dimensions changed mid-stream: %ux%u vs %ux%u",
                     width, height, impl_->stream_width, impl_->stream_height);
  }
  if (static_cast<size_t>(width) * height > max_out_size) {
    return FailFrame("JPEG %ux%u exceeds output buffer (%zu bytes)", width,
                     height, max_out_size);
  }
  if (cinfo->progressive_mode) {
    return FailFrame("Progressive JPEG is not supported");
  }
  const bool grayscale_source = (cinfo->jpeg_color_space == JCS_GRAYSCALE);
  if (!grayscale_source && cinfo->jpeg_color_space != JCS_YCbCr) {
    return FailFrame("Unsupported JPEG color space %d",
                     static_cast<int>(cinfo->jpeg_color_space));
  }

  cinfo->raw_data_out = TRUE;
  cinfo->out_color_space = grayscale_source ? JCS_GRAYSCALE : JCS_YCbCr;

  jpeg_start_decompress(cinfo);

  // Raw-data mode hands us one iMCU row group per call, DCT-padded on both
  // axes.  Y always carries the max sampling factor for the formats we
  // accept, so the group height equals the luma rows per group.
  const int lines_per_group = cinfo->max_v_samp_factor * DCTSIZE;
  const int y_rows_per_group = cinfo->comp_info[0].v_samp_factor * DCTSIZE;
  if (y_rows_per_group != lines_per_group || y_rows_per_group > 4 * DCTSIZE) {
    return FailFrame("Unsupported subsampling (Y %dx%d, max_v %d)",
                     cinfo->comp_info[0].h_samp_factor,
                     cinfo->comp_info[0].v_samp_factor,
                     cinfo->max_v_samp_factor);
  }

  size_t scratch_row_size =
      static_cast<size_t>(cinfo->comp_info[0].width_in_blocks) * DCTSIZE;
  for (int c = 1; c < cinfo->num_components; ++c) {
    scratch_row_size = std::max<size_t>(
        scratch_row_size,
        static_cast<size_t>(cinfo->comp_info[c].width_in_blocks) * DCTSIZE);
  }
  impl_->discard_row.resize(scratch_row_size);

  // Every row of every plane goes to the discard row: these calls only
  // advance the library's scanline state so jpeg_finish_decompress is
  // legal.  The pixels come from the hardware surface below.
  JSAMPROW y_rows[4 * DCTSIZE];
  JSAMPROW cb_rows[4 * DCTSIZE];
  JSAMPROW cr_rows[4 * DCTSIZE];
  JSAMPARRAY planes[3] = {y_rows, cb_rows, cr_rows};
  for (int i = 0; i < 4 * DCTSIZE; ++i) {
    y_rows[i] = impl_->discard_row.data();
    cb_rows[i] = impl_->discard_row.data();
    cr_rows[i] = impl_->discard_row.data();
  }

  while (cinfo->output_scanline < cinfo->output_height) {
    if (jpeg_read_raw_data(cinfo, planes, lines_per_group) == 0) {
      // Treat like a decode error: rebuild so the stream restarts cleanly.
      return FailFrame("jpeg_read_raw_data made no progress at row %u",
                       cinfo->output_scanline);
    }
  }

  // Read hardware state before finish -- finish/abort may reset it.
  //
  // tegra_acceleration is libnvjpeg's own report of the path it took.  The
  // engine being bound is not enough -- refuse to masquerade as a hardware
  // decoder while burning CPU.
  if (cinfo->tegra_acceleration == 0) {
    LOG(FATAL) << "libnvjpeg decoded on the CPU even though an NVJPG engine "
                  "is bound -- refusing to run without hardware decode.  For "
                  "CPU decode, switch the config template to "
                  "turbojpeg_decoder instead.";
  }

  // The decoded luma plane lives in the hardware output surface.  In stream
  // mode this surface holds the PREVIOUS submission's frame once the
  // pipeline is primed (and an all-zero warmup frame on the second decode);
  // the caller handles that pairing.
  CHECK(cinfo->jpegTegraMgr != nullptr &&
        cinfo->jpegTegraMgr->buff[0] != nullptr)
      << ": hardware decode reported but no output surface is present";
  const uint8_t *surface = cinfo->jpegTegraMgr->buff[0];
  const size_t pitch = cinfo->jpegTegraMgr->pitch[0];
  CHECK_GE(pitch, static_cast<size_t>(width))
      << ": hardware surface pitch is narrower than the image";
  for (uint32_t row = 0; row < height; ++row) {
    memcpy(gray_out + static_cast<size_t>(row) * width,
           surface + static_cast<size_t>(row) * pitch, width);
  }

  jpeg_finish_decompress(cinfo);

  if (!impl_->logged_first_decode) {
    impl_->logged_first_decode = true;
    LOG(INFO) << "First JPEG decoded (" << width << "x" << height
              << ") on the NVJPG engine";
  }

  impl_->stream_width = width;
  impl_->stream_height = height;

  result->width = width;
  result->height = height;
  return true;
}

}  // namespace frc::vision
