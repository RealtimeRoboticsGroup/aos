#include <arm_neon.h>

#include "absl/log/check.h"
#include "absl/log/log.h"

#include "aos/time/time.h"
#include "frc/orin/threshold.h"

namespace frc::apriltag {

typedef std::chrono::duration<double, std::milli> double_milli;

class NeonThreshold : public Threshold {
 public:
  NeonThreshold(size_t width, size_t height)
      : width_(width),
        height_(height),
        horizontal_filtered_min_max_image_(width_ / 8 * height_ / 8) {
    CHECK_EQ(width_ % 16, 0u);
    CHECK_EQ(height_ % 8, 0u);
  }
  // Create a full-size grayscale image from a color image on the provided
  // stream.
  void ToGreyscale(const uint8_t *color_image, uint8_t *gray_image,
                   CudaStream * /*stream*/) override;

  // Converts to grayscale, decimates, and thresholds an image on the provided
  // stream.
  void ThresholdAndDecimate(const uint8_t *color_image,
                            uint8_t *decimated_image,
                            uint8_t *thresholded_image,
                            apriltag_size_t min_white_black_diff,
                            CudaStream *stream) override;

  virtual ~NeonThreshold() = default;

 private:
  size_t width_;
  size_t height_;

  std::vector<uint16_t> horizontal_filtered_min_max_image_;
};

std::unique_ptr<Threshold> MakeNeonThreshold(vision::ImageFormat image_format,
                                             size_t width, size_t height) {
  switch (image_format) {
    case vision::ImageFormat::MONO8:
    case vision::ImageFormat::YUYV422:
      return std::make_unique<NeonThreshold>(width, height);

    case vision::ImageFormat::MONO16:
    case vision::ImageFormat::BGR8:
    case vision::ImageFormat::BGRA8:
      LOG(FATAL) << "Unsupported NEON image format: "
                 << vision::EnumNameImageFormat(image_format);
    default:
      LOG(FATAL) << "Unknown image format: "
                 << vision::EnumNameImageFormat(image_format);
  }
}

void NeonThreshold::ToGreyscale(const uint8_t *color_image, uint8_t *gray_image,
                                CudaStream * /*stream*/) {
  const aos::monotonic_clock::time_point start_time =
      aos::monotonic_clock::now();
  // Process in chunks of 8 bytes for optimal NEON usage
  for (size_t i = 0; i < height_ * width_; i += 64) {
    for (size_t j = 0; j < 64; j += 8) {
      // Process 16 pixels wide
      const uint8x16_t data = vld1q_u8(color_image + (i + j) * 2);
      const uint8x8_t result = vuzp1_u8(vget_low_u8(data), vget_high_u8(data));
      vst1_u8(gray_image + i + j, result);
    }
  }

  const aos::monotonic_clock::time_point end_time = aos::monotonic_clock::now();

  VLOG(1) << "Neon Greyscale took "
          << double_milli(end_time - start_time).count() << "ms";
}

void NeonThreshold::ThresholdAndDecimate(const uint8_t *color_image,
                                         uint8_t *decimated_image,
                                         uint8_t *thresholded_image,
                                         apriltag_size_t min_white_black_diff,
                                         CudaStream * /*stream*/) {
  const aos::monotonic_clock::time_point start_time =
      aos::monotonic_clock::now();
  uint16_t *min_max_image_data = horizontal_filtered_min_max_image_.data();

  // The underlying operation is pretty simple.
  // 1) Downscale by a factor of 2 in each direction with nearest neighbor
  //   (copying 1 out of each group of 4 pixels).
  // 2) For each resulting group of 4x4 pixels, compute the min and max for the
  //   block.
  // 3) For each block, look at the neighboring blocks (3x3 centered on the
  //   block in question), and compute the min/max for all 9 blocks.
  // 4) If the filtered max - min for a block is less than the min_white_black,
  //   return 127 for each pixel in the block.  Otherwise, return 0 for < (max +
  //   min) / 2, and 255 when >.

  // Unfortunately, the nieve algorithm is slow.  We can do better with some
  // good SIMD. Process 2 blocks at a time to align with SIMD instructions well.
  // This ends up with a 16 byte read, and a 8 byte write, which fits nicely.
  for (size_t h = 0; h < height_ / 8; h += 1) {
    // Horizontally filter the min and max as we go across the image.
    // Defer writing the filtered min and max out by 1 pair of blocks for
    // efficiency.
    uint8_t last_last_min_horizontal = 0xff;
    uint8_t last_min_horizontal = 0xff;

    uint8_t last_last_max_horizontal = 0x00;
    uint8_t last_max_horizontal = 0x00;

    size_t w = 0;
    for (; w < width_ / 8; w += 2) {
      // Process 16 pixels wide by 8 pixels high
      // Load every other row, since we are decimating by 2 vertically.
      const uint8x16_t data0 =
          vld1q_u8(color_image + (h * 8 + 0) * width_ + w * 8);
      const uint8x16_t data1 =
          vld1q_u8(color_image + (h * 8 + 2) * width_ + w * 8);
      const uint8x16_t data2 =
          vld1q_u8(color_image + (h * 8 + 4) * width_ + w * 8);
      const uint8x16_t data3 =
          vld1q_u8(color_image + (h * 8 + 6) * width_ + w * 8);

      // Now, start pulling every even byte out and saving it.  That gives us
      // every other pixel.
      const uint8x8_t result0 =
          vuzp1_u8(vget_low_u8(data0), vget_high_u8(data0));
      // Save the decimated pixels back out too.
      vst1_u8(decimated_image + (h * 8 + 0) * width_ / 4 + w * 4, result0);

      const uint8x8_t result1 =
          vuzp1_u8(vget_low_u8(data1), vget_high_u8(data1));
      vst1_u8(decimated_image + (h * 8 + 2) * width_ / 4 + w * 4, result1);

      // Now that we have 2 rows, do vertical max/min operations to compute
      // the per block min/max.
      uint8x8_t vmin = vmin_u8(result0, result1);
      uint8x8_t vmax = vmax_u8(result0, result1);

      // And continue to accumulate rows.
      const uint8x8_t result2 =
          vuzp1_u8(vget_low_u8(data2), vget_high_u8(data2));
      vmin = vmin_u8(vmin, result2);
      vmax = vmax_u8(vmax, result2);
      vst1_u8(decimated_image + (h * 8 + 4) * width_ / 4 + w * 4, result2);

      const uint8x8_t result3 =
          vuzp1_u8(vget_low_u8(data3), vget_high_u8(data3));
      vst1_u8(decimated_image + (h * 8 + 6) * width_ / 4 + w * 4, result3);
      vmin = vmin_u8(vmin, result3);
      vmax = vmax_u8(vmax, result3);

      // Finally, reduce the max/min to a single value for each of the 8x8
      // pixel blocks.
      //
      // TODO(austin): We should be able to do this with some nice SIMD and save
      // 0.05ms.  There are instructions to combine adjacent pairs using
      // min/max.  We'd need to process 8 blocks at a time to do that
      // efficiently.
      const uint8_t max0 =
          std::max(std::max(vmax[0], vmax[1]), std::max(vmax[2], vmax[3]));
      const uint8_t min0 =
          std::min(std::min(vmin[0], vmin[1]), std::min(vmin[2], vmin[3]));
      const uint8_t max1 =
          std::max(std::max(vmax[4], vmax[5]), std::max(vmax[6], vmax[7]));
      const uint8_t min1 =
          std::min(std::min(vmin[4], vmin[5]), std::min(vmin[6], vmin[7]));

      const uint8_t filtered_max0 =
          std::max(last_max_horizontal, std::max(max0, max1));
      const uint8_t filtered_min0 =
          std::min(last_min_horizontal, std::min(min0, min1));

      // Min/max need to be delayed by 1 iteration so we can write the filtered
      // ones out.
      //
      // And write the min/max out.
      if (w != 0) {
        const uint8_t filtered_last_min0 = std::min(
            last_last_min_horizontal, std::min(last_min_horizontal, min0));
        const uint8_t filtered_last_max0 = std::max(
            last_last_max_horizontal, std::max(last_max_horizontal, max0));

        *(min_max_image_data + h * width_ / 8 + w - 1) =
            static_cast<uint16_t>(filtered_last_min0) |
            (static_cast<uint16_t>(filtered_last_max0) << 8);
      }

      *(min_max_image_data + h * width_ / 8 + w) =
          static_cast<uint16_t>(filtered_min0) |
          (static_cast<uint16_t>(filtered_max0) << 8);

      last_last_max_horizontal = max0;
      last_max_horizontal = max1;
      last_last_min_horizontal = min0;
      last_min_horizontal = min1;
    }

    // Last one isn't written yet, do it manually.
    const uint8_t filtered_last_max0 =
        std::max(last_last_max_horizontal, last_max_horizontal);
    const uint8_t filtered_last_min0 =
        std::min(last_last_min_horizontal, last_min_horizontal);

    *(min_max_image_data + h * width_ / 8 + w - 1) =
        static_cast<uint16_t>(filtered_last_min0) |
        (static_cast<uint16_t>(filtered_last_max0) << 8);
  }

  // TODO(austin): I think if we ran the second pass 1 row behind the first
  // pass, things might be in the processor cache already, and go a lot faster.
  // Worth trying later if we need less CPU usage and more speed.

  const aos::monotonic_clock::time_point pass1_time =
      aos::monotonic_clock::now();

  // Now, we've got filtered horizontally min/max, and a decimated image in RAM.
  //
  // Process pairs of 8x8 blocks again so we end up with 8 byte loads nicely
  // again.
  for (size_t h = 0; h < height_ / 8; h += 1) {
    for (size_t w = 0; w < width_ / 8; w += 2) {
      // We are going to be working on 8 pixels at a time.
      // Load the extra 2 max/min values we need on lines above and below.
      const uint16_t prior_min_max0 =
          *(min_max_image_data + (h == 0 ? 0 : h - 1) * width_ / 8 + w);
      const uint16_t prior_min_max1 =
          *(min_max_image_data + (h == 0 ? 0 : h - 1) * width_ / 8 + w + 1);

      const uint16_t min_max0 = *(min_max_image_data + h * width_ / 8 + w);
      const uint16_t min_max1 = *(min_max_image_data + h * width_ / 8 + w + 1);

      const uint16_t next_min_max0 =
          *(min_max_image_data +
            ((h == height_ / 8 - 1) ? h : (h + 1)) * width_ / 8 + w);
      const uint16_t next_min_max1 =
          *(min_max_image_data +
            ((h == height_ / 8 - 1) ? h : (h + 1)) * width_ / 8 + w + 1);

      // Compute the min/max values to use for the threshold.
      const uint8_t min0 =
          std::min({static_cast<uint8_t>(prior_min_max0 & 0xff),
                    static_cast<uint8_t>(min_max0 & 0xff),
                    static_cast<uint8_t>(next_min_max0 & 0xff)});
      const uint8_t min1 =
          std::min({static_cast<uint8_t>(prior_min_max1 & 0xff),
                    static_cast<uint8_t>(min_max1 & 0xff),
                    static_cast<uint8_t>(next_min_max1 & 0xff)});
      const uint8_t max0 =
          std::max({static_cast<uint8_t>((prior_min_max0 >> 8) & 0xff),
                    static_cast<uint8_t>((min_max0 >> 8) & 0xff),
                    static_cast<uint8_t>((next_min_max0 >> 8) & 0xff)});
      const uint8_t max1 =
          std::max({static_cast<uint8_t>((prior_min_max1 >> 8) & 0xff),
                    static_cast<uint8_t>((min_max1 >> 8) & 0xff),
                    static_cast<uint8_t>((next_min_max1 >> 8) & 0xff)});

      // Load the 4 rows.
      const uint8x8_t data0 =
          vld1_u8(decimated_image + (h * 4 + 0) * width_ / 2 + w * 4);
      const uint8x8_t data1 =
          vld1_u8(decimated_image + (h * 4 + 1) * width_ / 2 + w * 4);
      const uint8x8_t data2 =
          vld1_u8(decimated_image + (h * 4 + 2) * width_ / 2 + w * 4);
      const uint8x8_t data3 =
          vld1_u8(decimated_image + (h * 4 + 3) * width_ / 2 + w * 4);

      // Now, put pixels from the same block into the same registers so we can
      // do the threshold all at once.
      uint8x8_t block00_u8;
      uint8x8_t block10_u8;
      if (max0 - min0 < min_white_black_diff) {
        // If the difference is too small, set them all to 127.
        block00_u8 = vdup_n_u8(127);
        block10_u8 = block00_u8;
      } else {
        const uint8x8_t thresh0 = vdup_n_u8(min0 + (max0 - min0) / 2);

        // Otherwise, now that we have pulled out the even uint32_t's (which is
        // a row of each block of pixels, giving us 2 rows from the block in 1
        // register), compare against the threshold and write that out.
        block00_u8 =
            vcgt_u8(vreinterpret_u8_u32(vuzp1_u32(vreinterpret_u32_u8(data0),
                                                  vreinterpret_u32_u8(data1))),
                    thresh0);
        block10_u8 =
            vcgt_u8(vreinterpret_u8_u32(vuzp1_u32(vreinterpret_u32_u8(data2),
                                                  vreinterpret_u32_u8(data3))),
                    thresh0);
      }

      // Do this for the second block.
      uint8x8_t block01_u8;
      uint8x8_t block11_u8;
      if (max1 - min1 < min_white_black_diff) {
        block01_u8 = vdup_n_u8(127);
        block11_u8 = block01_u8;
      } else {
        const uint8x8_t thresh1 = vdup_n_u8(min1 + (max1 - min1) / 2);

        block01_u8 =
            vcgt_u8(vreinterpret_u8_u32(vuzp2_u32(vreinterpret_u32_u8(data0),
                                                  vreinterpret_u32_u8(data1))),
                    thresh1);
        block11_u8 =
            vcgt_u8(vreinterpret_u8_u32(vuzp2_u32(vreinterpret_u32_u8(data2),
                                                  vreinterpret_u32_u8(data3))),
                    thresh1);
      }

      // Unzip things to get us back in memory order and push it out to RAM.
      vst1_u8(thresholded_image + (h * 4 + 0) * width_ / 2 + w * 4,
              vreinterpret_u8_u32(vuzp1_u32(vreinterpret_u32_u8(block00_u8),
                                            vreinterpret_u32_u8(block01_u8))));
      vst1_u8(thresholded_image + (h * 4 + 1) * width_ / 2 + w * 4,
              vreinterpret_u8_u32(vuzp2_u32(vreinterpret_u32_u8(block00_u8),
                                            vreinterpret_u32_u8(block01_u8))));

      vst1_u8(thresholded_image + (h * 4 + 2) * width_ / 2 + w * 4,
              vreinterpret_u8_u32(vuzp1_u32(vreinterpret_u32_u8(block10_u8),
                                            vreinterpret_u32_u8(block11_u8))));
      vst1_u8(thresholded_image + (h * 4 + 3) * width_ / 2 + w * 4,
              vreinterpret_u8_u32(vuzp2_u32(vreinterpret_u32_u8(block10_u8),
                                            vreinterpret_u32_u8(block11_u8))));
    }
  }

  const aos::monotonic_clock::time_point end_time = aos::monotonic_clock::now();

  VLOG(1) << "Neon After, took "
          << double_milli(pass1_time - start_time).count() << "ms for pass 1, "
          << double_milli(end_time - pass1_time).count() << "ms for pass 2, "
          << double_milli(end_time - start_time).count() << "ms overall";
}

}  // namespace frc::apriltag
