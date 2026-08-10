#include "frc/vision/nvjpeg_decoder_lib.h"

#include <glob.h>
#include <setjmp.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <vector>

#include "absl/log/log.h"

// libnvjpeg.so implements the libjpeg-8b API with NVIDIA's TEGRA_ACCELERATE
// extensions.  TEGRA_ACCELERATE (set via local_defines in the BUILD file)
// selects the extended struct layout that matches the library's ABI.
extern "C" {
#include <jpeglib.h>
}

// JPEG decode happens through the standard libjpeg raw-data-out interface:
//
//   jpeg_mem_src -> jpeg_read_header -> raw_data_out = TRUE,
//   out_color_space = JCS_YCbCr -> jpeg_start_decompress ->
//   jpeg_read_raw_data per iMCU row group -> jpeg_finish_decompress
//
// With libnvjpeg, jpeg_start_decompress runs the whole frame through the
// NVJPG engine and jpeg_read_raw_data copies the decoded planes into our row
// pointers.  This is the exact call set NVIDIA's nvjpegdec GStreamer plugin
// makes against the same library.
//
// On JetPack 6 / L4T 36 there is no /dev/nvhost-nvjpg chardev (that was the
// JetPack 5 interface): libnvjpeg reaches the engine through the tegra-drm
// render node (/dev/dri/renderD*) and /dev/nvmap, with the engine's client
// driver (tegra-nvjpg) living inside tegra-drm.ko.  Verified by stracing a
// hardware decode on an Orin Nano running the 36.4.3 image.
//
// The library would decode on the CPU when the engine is unavailable; we
// deliberately fail loudly instead (at construction when no engine is bound
// to the tegra-nvjpg driver, and on the first frame if libnvjpeg reports it
// did not use the engine) -- CPU decode is turbojpeg_decoder's job, selected
// explicitly in the config.
//
// We only keep the Y plane: luma rows are written straight into the caller's
// buffer and chroma rows land in a discard row.

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
// of stderr but visible at higher verbosity.
void OutputMessage(j_common_ptr cinfo) {
  char message[JMSG_LENGTH_MAX];
  (*cinfo->err->format_message)(cinfo, message);
  VLOG(1) << "libnvjpeg: " << message;
}

}  // namespace

struct NvJpegDecoderLib::Impl {
  struct jpeg_decompress_struct cinfo;
  DecodeErrorMgr err;

  // One row of DCT-padded scratch, for chroma rows and luma rows past the
  // image bottom.
  std::vector<uint8_t> discard_row;
  // Bounce buffer for one luma row group, only used when the image width is
  // not a multiple of the MCU width (libjpeg pads raw rows to DCT blocks).
  std::vector<uint8_t> bounce_rows;

  bool logged_first_decode = false;
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

bool NvJpegDecoderLib::DecodeToGray(const uint8_t *jpeg_data, size_t jpeg_size,
                                    uint8_t *gray_out, size_t max_out_size,
                                    Result *result) {
  jpeg_decompress_struct *cinfo = &impl_->cinfo;

  if (setjmp(impl_->err.setjmp_buffer)) {
    // libjpeg hit a fatal decode error and longjmp'd back here.
    jpeg_abort_decompress(cinfo);
    LOG(WARNING) << "JPEG decode failed: " << impl_->err.message;
    return false;
  }

  jpeg_mem_src(cinfo, const_cast<unsigned char *>(jpeg_data),
               static_cast<unsigned long>(jpeg_size));
  jpeg_read_header(cinfo, TRUE);

  const uint32_t width = cinfo->image_width;
  const uint32_t height = cinfo->image_height;
  if (static_cast<size_t>(width) * height > max_out_size) {
    LOG(WARNING) << "JPEG " << width << "x" << height
                 << " exceeds output buffer (" << max_out_size << " bytes)";
    jpeg_abort_decompress(cinfo);
    return false;
  }
  if (cinfo->progressive_mode) {
    LOG(WARNING) << "Progressive JPEG is not supported";
    jpeg_abort_decompress(cinfo);
    return false;
  }
  const bool grayscale_source = (cinfo->jpeg_color_space == JCS_GRAYSCALE);
  if (!grayscale_source && cinfo->jpeg_color_space != JCS_YCbCr) {
    LOG(WARNING) << "Unsupported JPEG color space "
                 << static_cast<int>(cinfo->jpeg_color_space);
    jpeg_abort_decompress(cinfo);
    return false;
  }

  cinfo->raw_data_out = TRUE;
  cinfo->out_color_space = grayscale_source ? JCS_GRAYSCALE : JCS_YCbCr;

  jpeg_start_decompress(cinfo);

  // Raw-data mode hands us one iMCU row group per call, DCT-padded on both
  // axes.  Y always carries the max sampling factor for the formats we
  // accept, so the group height equals the luma rows per group.
  const int lines_per_group = cinfo->max_v_samp_factor * DCTSIZE;
  const int y_rows_per_group = cinfo->comp_info[0].v_samp_factor * DCTSIZE;
  const uint32_t y_padded_width = cinfo->comp_info[0].width_in_blocks * DCTSIZE;
  if (y_rows_per_group != lines_per_group || y_rows_per_group > 4 * DCTSIZE) {
    LOG(WARNING) << "Unsupported subsampling (Y "
                 << cinfo->comp_info[0].h_samp_factor << "x"
                 << cinfo->comp_info[0].v_samp_factor << ", max_v "
                 << cinfo->max_v_samp_factor << ")";
    jpeg_abort_decompress(cinfo);
    return false;
  }

  size_t scratch_row_size = y_padded_width;
  for (int c = 1; c < cinfo->num_components; ++c) {
    scratch_row_size = std::max<size_t>(
        scratch_row_size,
        static_cast<size_t>(cinfo->comp_info[c].width_in_blocks) * DCTSIZE);
  }
  impl_->discard_row.resize(scratch_row_size);

  const bool bounce = (y_padded_width != width);
  if (bounce) {
    impl_->bounce_rows.resize(static_cast<size_t>(y_padded_width) *
                              y_rows_per_group);
  }

  JSAMPROW y_rows[4 * DCTSIZE];
  JSAMPROW cb_rows[4 * DCTSIZE];
  JSAMPROW cr_rows[4 * DCTSIZE];
  JSAMPARRAY planes[3] = {y_rows, cb_rows, cr_rows};
  for (int i = 0; i < 4 * DCTSIZE; ++i) {
    cb_rows[i] = impl_->discard_row.data();
    cr_rows[i] = impl_->discard_row.data();
  }

  while (cinfo->output_scanline < cinfo->output_height) {
    const uint32_t base = cinfo->output_scanline;
    for (int i = 0; i < y_rows_per_group; ++i) {
      const uint32_t row = base + i;
      if (bounce) {
        y_rows[i] =
            impl_->bounce_rows.data() + static_cast<size_t>(i) * y_padded_width;
      } else if (row < height) {
        y_rows[i] = gray_out + static_cast<size_t>(row) * width;
      } else {
        // DCT padding rows below the image bottom.
        y_rows[i] = impl_->discard_row.data();
      }
    }
    if (jpeg_read_raw_data(cinfo, planes, lines_per_group) == 0) {
      LOG(WARNING) << "jpeg_read_raw_data made no progress at row " << base;
      jpeg_abort_decompress(cinfo);
      return false;
    }
    if (bounce) {
      for (int i = 0; i < y_rows_per_group; ++i) {
        const uint32_t row = base + i;
        if (row < height) {
          memcpy(gray_out + static_cast<size_t>(row) * width,
                 impl_->bounce_rows.data() +
                     static_cast<size_t>(i) * y_padded_width,
                 width);
        }
      }
    }
  }

  // Read before finish -- finish/abort may reset library state.
  const bool hardware_used = cinfo->tegra_acceleration != 0;
  jpeg_finish_decompress(cinfo);

  // tegra_acceleration is libnvjpeg's own report of the path it took.  The
  // engine being bound is not enough -- refuse to masquerade as a hardware
  // decoder while burning CPU.
  if (!hardware_used) {
    LOG(FATAL) << "libnvjpeg decoded on the CPU even though an NVJPG engine "
                  "is bound -- refusing to run without hardware decode.  For "
                  "CPU decode, switch the config template to "
                  "turbojpeg_decoder instead.";
  }
  if (!impl_->logged_first_decode) {
    impl_->logged_first_decode = true;
    LOG(INFO) << "First JPEG decoded (" << width << "x" << height
              << ") on the NVJPG engine";
  }

  result->width = width;
  result->height = height;
  return true;
}

}  // namespace frc::vision
