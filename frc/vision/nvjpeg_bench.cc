// Standalone probe/benchmark for libnvjpeg (NVIDIA's TEGRA_ACCELERATE
// libjpeg-8b) on the Orin.  Decodes a JPEG file N times through the same
// raw-data-out call sequence NvJpegDecoderLib uses, reports whether the
// library engaged the NVJPG engine (cinfo.tegra_acceleration) and the
// per-frame decode time, and optionally writes the grayscale result as a
// PGM for pixel-level verification.
//
// Deliberately independent of NvJpegDecoderLib so it runs (and reports)
// even on systems where the engine is unavailable -- this is the tool to
// answer "would the hardware path work here?" without the decoder
// service's fail-loud policy in the way.
//
// Usage: nvjpeg_bench <file.jpg> [iterations] [out.pgm]

#include <setjmp.h>

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "absl/log/globals.h"
#include "absl/log/initialize.h"
#include "absl/log/log.h"
#include "absl/strings/str_format.h"

extern "C" {
#include <jpeglib.h>
}

namespace {

struct BenchErrorMgr {
  struct jpeg_error_mgr pub;
  jmp_buf setjmp_buffer;
  char message[JMSG_LENGTH_MAX];
};

void ErrorExit(j_common_ptr cinfo) {
  BenchErrorMgr *err = reinterpret_cast<BenchErrorMgr *>(cinfo->err);
  (*cinfo->err->format_message)(cinfo, err->message);
  longjmp(err->setjmp_buffer, 1);
}

void OutputMessage(j_common_ptr cinfo) {
  char message[JMSG_LENGTH_MAX];
  (*cinfo->err->format_message)(cinfo, message);
  LOG(WARNING) << "libnvjpeg: " << message;
}

}  // namespace

int main(int argc, char **argv) {
  absl::InitializeLog();
  absl::SetStderrThreshold(absl::LogSeverityAtLeast::kInfo);
  if (argc < 2) {
    LOG(ERROR) << "usage: " << argv[0] << " <file.jpg> [iterations] [out.pgm]";
    return 1;
  }
  const int iterations = argc > 2 ? atoi(argv[2]) : 20;

  FILE *f = fopen(argv[1], "rb");
  if (f == nullptr) {
    LOG(ERROR) << "open " << argv[1] << ": " << strerror(errno);
    return 1;
  }
  fseek(f, 0, SEEK_END);
  const long file_size = ftell(f);
  fseek(f, 0, SEEK_SET);
  std::vector<unsigned char> jpeg(file_size);
  if (fread(jpeg.data(), 1, file_size, f) != static_cast<size_t>(file_size)) {
    LOG(ERROR) << "short read of " << argv[1];
    return 1;
  }
  fclose(f);

  struct jpeg_decompress_struct cinfo;
  BenchErrorMgr jerr;
  memset(&cinfo, 0, sizeof(cinfo));
  memset(&jerr, 0, sizeof(jerr));
  cinfo.err = jpeg_std_error(&jerr.pub);
  jerr.pub.error_exit = ErrorExit;
  jerr.pub.output_message = OutputMessage;
  if (setjmp(jerr.setjmp_buffer)) {
    LOG(ERROR) << "libnvjpeg failed to initialize: " << jerr.message;
    return 1;
  }
  jpeg_create_decompress(&cinfo);

  std::vector<unsigned char> gray;
  std::vector<unsigned char> discard_row;
  std::vector<unsigned char> bounce_rows;
  double total_us = 0;
  double min_us = 1e12;
  uint32_t width = 0, height = 0;

  for (int iter = 0; iter < iterations; ++iter) {
    if (setjmp(jerr.setjmp_buffer)) {
      LOG(ERROR) << "decode failed on iteration " << iter << ": "
                 << jerr.message;
      return 1;
    }
    const auto start = std::chrono::steady_clock::now();

    jpeg_mem_src(&cinfo, jpeg.data(), jpeg.size());
    jpeg_read_header(&cinfo, TRUE);
    width = cinfo.image_width;
    height = cinfo.image_height;
    const bool grayscale_source = (cinfo.jpeg_color_space == JCS_GRAYSCALE);
    cinfo.raw_data_out = TRUE;
    cinfo.out_color_space = grayscale_source ? JCS_GRAYSCALE : JCS_YCbCr;
    jpeg_start_decompress(&cinfo);

    if (iter == 0) {
      LOG(INFO) << absl::StrFormat(
          "image: %ux%u, components=%d, Y samp=%dx%d, jpeg_color_space=%d",
          width, height, cinfo.num_components, cinfo.comp_info[0].h_samp_factor,
          cinfo.comp_info[0].v_samp_factor,
          static_cast<int>(cinfo.jpeg_color_space));
      LOG(INFO) << absl::StrFormat("tegra_acceleration=%d mjpeg_decode=%d",
                                   static_cast<int>(cinfo.tegra_acceleration),
                                   static_cast<int>(cinfo.mjpeg_decode));
      if (cinfo.jpegTegraMgr != nullptr) {
        LOG(INFO) << "jpegTegraMgr: buff[0]="
                  << static_cast<void *>(cinfo.jpegTegraMgr->buff[0])
                  << " pitch[0]=" << cinfo.jpegTegraMgr->pitch[0]
                  << " mcu_type=" << cinfo.jpegTegraMgr->mcu_type;
      } else {
        LOG(INFO) << "jpegTegraMgr: null";
      }
    }

    const int lines_per_group = cinfo.max_v_samp_factor * DCTSIZE;
    const int y_rows_per_group = cinfo.comp_info[0].v_samp_factor * DCTSIZE;
    const uint32_t y_padded_width =
        cinfo.comp_info[0].width_in_blocks * DCTSIZE;
    if (y_rows_per_group != lines_per_group || y_rows_per_group > 4 * DCTSIZE) {
      LOG(ERROR) << "unsupported subsampling";
      return 1;
    }
    size_t scratch = y_padded_width;
    for (int c = 1; c < cinfo.num_components; ++c) {
      const size_t w =
          static_cast<size_t>(cinfo.comp_info[c].width_in_blocks) * DCTSIZE;
      if (w > scratch) scratch = w;
    }
    discard_row.resize(scratch);
    gray.resize(static_cast<size_t>(width) * height);
    const bool bounce = (y_padded_width != width);
    if (bounce) {
      bounce_rows.resize(static_cast<size_t>(y_padded_width) *
                         y_rows_per_group);
    }

    JSAMPROW y_rows[4 * DCTSIZE];
    JSAMPROW cb_rows[4 * DCTSIZE];
    JSAMPROW cr_rows[4 * DCTSIZE];
    JSAMPARRAY planes[3] = {y_rows, cb_rows, cr_rows};
    for (int i = 0; i < 4 * DCTSIZE; ++i) {
      cb_rows[i] = discard_row.data();
      cr_rows[i] = discard_row.data();
    }

    while (cinfo.output_scanline < cinfo.output_height) {
      const uint32_t base = cinfo.output_scanline;
      for (int i = 0; i < y_rows_per_group; ++i) {
        const uint32_t row = base + i;
        if (bounce) {
          y_rows[i] =
              bounce_rows.data() + static_cast<size_t>(i) * y_padded_width;
        } else if (row < height) {
          y_rows[i] = gray.data() + static_cast<size_t>(row) * width;
        } else {
          y_rows[i] = discard_row.data();
        }
      }
      if (jpeg_read_raw_data(&cinfo, planes, lines_per_group) == 0) {
        LOG(ERROR) << "jpeg_read_raw_data made no progress at row " << base;
        return 1;
      }
      if (bounce) {
        for (int i = 0; i < y_rows_per_group; ++i) {
          const uint32_t row = base + i;
          if (row < height) {
            memcpy(gray.data() + static_cast<size_t>(row) * width,
                   bounce_rows.data() + static_cast<size_t>(i) * y_padded_width,
                   width);
          }
        }
      }
    }
    const bool hw = cinfo.tegra_acceleration != 0;
    jpeg_finish_decompress(&cinfo);

    const double us = std::chrono::duration<double, std::micro>(
                          std::chrono::steady_clock::now() - start)
                          .count();
    total_us += us;
    if (us < min_us) min_us = us;
    if (iter == 0) {
      LOG(INFO) << absl::StrFormat("first decode: %.1f us, hardware=%s", us,
                                   hw ? "YES" : "no");
    }
  }

  LOG(INFO) << absl::StrFormat("%d iterations: mean %.1f us, min %.1f us",
                               iterations, total_us / iterations, min_us);

  if (argc > 3) {
    FILE *out = fopen(argv[3], "wb");
    if (out == nullptr) {
      LOG(ERROR) << "open " << argv[3] << ": " << strerror(errno);
      return 1;
    }
    fprintf(out, "P5\n%u %u\n255\n", width, height);
    fwrite(gray.data(), 1, gray.size(), out);
    fclose(out);
    LOG(INFO) << "wrote " << argv[3];
  }

  jpeg_destroy_decompress(&cinfo);
  return 0;
}
