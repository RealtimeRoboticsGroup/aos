#include <arm_neon.h>

#include "absl/log/check.h"
#include "absl/log/log.h"

#include "aos/time/time.h"
#include "frc/orin/apriltag.h"
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
  LOG(INFO) << "Neon Greyscale Before";
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

  LOG(INFO) << "Neon Greyscale After, took "
            << double_milli(end_time - start_time).count() << "ms";
}

void NeonThreshold::ThresholdAndDecimate(
    const uint8_t *color_image, uint8_t *decimated_image,
    uint8_t * thresholded_image, apriltag_size_t min_white_black_diff,
    CudaStream * /*stream*/) {
  LOG(INFO) << "Neon Before";
  const aos::monotonic_clock::time_point start_time =
      aos::monotonic_clock::now();
  uint16_t *min_max_image_data = horizontal_filtered_min_max_image_.data();

  // Process in chunks of 8 bytes for optimal NEON usage
  for (size_t h = 0; h < height_ / 8; h += 1) {
    uint8_t last_last_max_horizontal = 0x00;
    uint8_t last_max_horizontal = 0x00;

    uint8_t last_last_min_horizontal = 0xff;
    uint8_t last_min_horizontal = 0xff;

    size_t w = 0;
    for (; w < width_ / 8; w += 2) {
      const bool print = (w == 0 && h == 540/4) && false;
      if (print) {
        LOG(INFO) << "Block " << h << ", " << w;
      }
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
      //if (print) {
        //LOG(INFO) << std::hex << "vmin=["
          //<< static_cast<int>(vmin[0]) << ", " << static_cast<int>(vmin[1]) << ", "
          //<< static_cast<int>(vmin[2]) << ", " << static_cast<int>(vmin[3]) << ", "
          //<< static_cast<int>(vmin[4]) << ", " << static_cast<int>(vmin[5]) << ", "
          //<< static_cast<int>(vmin[6]) << ", " << static_cast<int>(vmin[7]) << "]";
        //LOG(INFO) << std::hex << "vmax=["
          //<< static_cast<int>(vmax[0]) << ", " << static_cast<int>(vmax[1]) << ", "
          //<< static_cast<int>(vmax[2]) << ", " << static_cast<int>(vmax[3]) << ", "
          //<< static_cast<int>(vmax[4]) << ", " << static_cast<int>(vmax[5]) << ", "
          //<< static_cast<int>(vmax[6]) << ", " << static_cast<int>(vmax[7]) << "]";
      //}

      // And continue to accumulate rows.
      const uint8x8_t result2 =
          vuzp1_u8(vget_low_u8(data2), vget_high_u8(data2));
      vmin = vmin_u8(vmin, result2);
      vmax = vmax_u8(vmax, result2);
      vst1_u8(decimated_image + (h * 8 + 4) * width_ / 4 + w * 4, result2);
      //if (print) {
        //LOG(INFO) << std::hex << "vmin=["
          //<< static_cast<int>(vmin[0]) << ", " << static_cast<int>(vmin[1]) << ", "
          //<< static_cast<int>(vmin[2]) << ", " << static_cast<int>(vmin[3]) << ", "
          //<< static_cast<int>(vmin[4]) << ", " << static_cast<int>(vmin[5]) << ", "
          //<< static_cast<int>(vmin[6]) << ", " << static_cast<int>(vmin[7]) << "]";
        //LOG(INFO) << std::hex << "vmax=["
          //<< static_cast<int>(vmax[0]) << ", " << static_cast<int>(vmax[1]) << ", "
          //<< static_cast<int>(vmax[2]) << ", " << static_cast<int>(vmax[3]) << ", "
          //<< static_cast<int>(vmax[4]) << ", " << static_cast<int>(vmax[5]) << ", "
          //<< static_cast<int>(vmax[6]) << ", " << static_cast<int>(vmax[7]) << "]";
      //}

      const uint8x8_t result3 =
          vuzp1_u8(vget_low_u8(data3), vget_high_u8(data3));
      vst1_u8(decimated_image + (h * 8 + 6) * width_ / 4 + w * 4, result3);
      vmin = vmin_u8(vmin, result3);
      vmax = vmax_u8(vmax, result3);
      if (print && false) {
        LOG(INFO) << std::hex << "vmin=["
          << static_cast<int>(vmin[0]) << ", " << static_cast<int>(vmin[1]) << ", "
          << static_cast<int>(vmin[2]) << ", " << static_cast<int>(vmin[3]) << ", "
          << static_cast<int>(vmin[4]) << ", " << static_cast<int>(vmin[5]) << ", "
          << static_cast<int>(vmin[6]) << ", " << static_cast<int>(vmin[7]) << "]";
        LOG(INFO) << std::hex << "vmax=["
          << static_cast<int>(vmax[0]) << ", " << static_cast<int>(vmax[1]) << ", "
          << static_cast<int>(vmax[2]) << ", " << static_cast<int>(vmax[3]) << ", "
          << static_cast<int>(vmax[4]) << ", " << static_cast<int>(vmax[5]) << ", "
          << static_cast<int>(vmax[6]) << ", " << static_cast<int>(vmax[7]) << "]";
      }

      // Finally, reduce the max/min to a single value for each of the 8x8
      // pixel blocks.
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

      /*
      const uint8_t max0 = max[0];
      const uint8_t min0 = min[0];
      const uint8_t max1 = max[1];
      const uint8_t min1 = min[1];
      */

      if (print) {
        /*
        LOG(INFO) << std::hex << "result0=["
          << static_cast<int>(result0[0]) << ", " << static_cast<int>(result0[1]) << ", "
          << static_cast<int>(result0[2]) << ", " << static_cast<int>(result0[3]) << ", "
          << static_cast<int>(result0[4]) << ", " << static_cast<int>(result0[5]) << ", "
          << static_cast<int>(result0[6]) << ", " << static_cast<int>(result0[7]) << "]";
        LOG(INFO) << std::hex
                  << "result1=["
          << static_cast<int>(result1[0]) << ", " << static_cast<int>(result1[1]) << ", "
          << static_cast<int>(result1[2]) << ", " << static_cast<int>(result1[3]) << ", "
          << static_cast<int>(result1[4]) << ", " << static_cast<int>(result1[5]) << ", "
          << static_cast<int>(result1[6]) << ", " << static_cast<int>(result1[7]) << "]";
        LOG(INFO) << std::hex
                  << "result2=["
          << static_cast<int>(result2[0]) << ", " << static_cast<int>(result2[1]) << ", "
          << static_cast<int>(result2[2]) << ", " << static_cast<int>(result2[3]) << ", "
          << static_cast<int>(result2[4]) << ", " << static_cast<int>(result2[5]) << ", "
          << static_cast<int>(result2[6]) << ", " << static_cast<int>(result2[7]) << "]";
        LOG(INFO) << std::hex
                  << "result3=["
          << static_cast<int>(result3[0]) << ", " << static_cast<int>(result3[1]) << ", "
          << static_cast<int>(result3[2]) << ", " << static_cast<int>(result3[3]) << ", "
          << static_cast<int>(result3[4]) << ", " << static_cast<int>(result3[5]) << ", "
          << static_cast<int>(result3[6]) << ", " << static_cast<int>(result3[7]) << "];";
          */
        LOG(INFO) << std::hex << "min01: " << static_cast<int>(min0) << " "
                  << static_cast<int>(min1);
        LOG(INFO) << std::hex << "max01: " << static_cast<int>(max0) << " "
                  << static_cast<int>(max1);
      }

      // Min/max need to be delayed by 1 iteration.
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
        if (print) {
          if (w != 0) {
            LOG(INFO) << std::hex << "filtered min max 1: "
                      << static_cast<int>(filtered_last_min0) << " "
                      << static_cast<int>(filtered_last_max0) << " wrote0: "
                      << static_cast<int>(
                             static_cast<uint16_t>(filtered_last_min0) |
                             (static_cast<uint16_t>(filtered_last_max0) << 8));
          }
      }
      }

      *(min_max_image_data + h * width_ / 8 + w) =
          static_cast<uint16_t>(filtered_min0) |
          (static_cast<uint16_t>(filtered_max0) << 8);

      if (print) {
        LOG(INFO) << "Wrote " <<(h * width_ / 8 + w);
        LOG(INFO) << std::hex
                  << "filtered min max 0: " << static_cast<int>(filtered_min0)
                  << " " << static_cast<int>(filtered_max0) << " wrote0: "
                  << static_cast<int>(
                         static_cast<uint16_t>(filtered_min0) |
                         (static_cast<uint16_t>(filtered_max0) << 8));
      }

      last_last_max_horizontal = max0;
      last_max_horizontal = max1;
      last_last_min_horizontal = min0;
      last_min_horizontal = min1;
    }

    const uint8_t filtered_last_max0 =
        std::max(last_last_max_horizontal, last_max_horizontal);
    const uint8_t filtered_last_min0 =
        std::min(last_last_min_horizontal, last_min_horizontal);

    *(min_max_image_data + h * width_ / 8 + w - 1) =
        static_cast<uint16_t>(filtered_last_min0) |
        (static_cast<uint16_t>(filtered_last_max0) << 8);
  }

  const aos::monotonic_clock::time_point pass1_time = aos::monotonic_clock::now();

  // Process each 8x8 block of the image at a time for efficiency.
  for (size_t h = 0; h < height_ / 8; h += 1) {
    for (size_t w = 0; w < width_ / 8; w += 2) {
      const bool print = (w == 0 && h == 540/4) && false;
      if (print) {
        LOG(INFO) << "Block " << h << ", " << w << " out of " << height_ / 8
                  << ", " << width_ / 8;
      }
      // We are going to be working on 8 pixels at a time.
      // Load the 2 max/min values we need.
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

      const uint8x8_t data0 =
          vld1_u8(decimated_image + (h * 4 + 0) * width_ / 2 + w * 4);
      const uint8x8_t data1 =
          vld1_u8(decimated_image + (h * 4 + 1) * width_ / 2 + w * 4);
      const uint8x8_t data2 =
          vld1_u8(decimated_image + (h * 4 + 2) * width_ / 2 + w * 4);
      const uint8x8_t data3 =
          vld1_u8(decimated_image + (h * 4 + 3) * width_ / 2 + w * 4);

      if (print) {
        LOG(INFO) << "Read from " << (h * width_ / 8 + w);
        LOG(INFO) << "Read next from "
                  << (((h == height_ / 8 - 1) ? h : (h + 1)) * width_ / 8 + w);
        LOG(INFO) << std::hex
                  << "prior min max0: " << static_cast<int>(prior_min_max0)
                  << " min max0: " << static_cast<int>(min_max0)
                  << " next min max0: " << static_cast<int>(next_min_max0)
                  << " overall " << static_cast<int>(min0) << " "
                  << static_cast<int>(max0);
        LOG(INFO) << std::hex
                  << "prior min max1: " << static_cast<int>(prior_min_max1)
                  << " min max1: " << static_cast<int>(min_max1)
                  << " next min max1: " << static_cast<int>(next_min_max1)
                  << " overall " << static_cast<int>(min1) << " "
                  << static_cast<int>(max1);

        for (int i = 0; i < 4; i++) {
          for (int j = 0; j < 8; j++) {
            // LOG(INFO) << "px[" << i << ", " << j << "]: " << std::hex
            //<< static_cast<int>(*(decimated_image +
            //(h * 4 + i) * width_ / 2 +
            // w * 4 + j));
          }
        }

        /*LOG(INFO) << std::hex << "data0=["
          << static_cast<int>(data0[0]) << ", " << static_cast<int>(data0[1]) << ", "
          << static_cast<int>(data0[2]) << ", " << static_cast<int>(data0[3]) << ", "
          << static_cast<int>(data0[4]) << ", " << static_cast<int>(data0[5]) << ", "
          << static_cast<int>(data0[6]) << ", " << static_cast<int>(data0[7]) << "], "
                  << "data1=["
          << static_cast<int>(data1[0]) << ", " << static_cast<int>(data1[1]) << ", "
          << static_cast<int>(data1[2]) << ", " << static_cast<int>(data1[3]) << ", "
          << static_cast<int>(data1[4]) << ", " << static_cast<int>(data1[5]) << ", "
          << static_cast<int>(data1[6]) << ", " << static_cast<int>(data1[7]) << "], "
                  << "data2=["
          << static_cast<int>(data2[0]) << ", " << static_cast<int>(data2[1]) << ", "
          << static_cast<int>(data2[2]) << ", " << static_cast<int>(data2[3]) << ", "
          << static_cast<int>(data2[4]) << ", " << static_cast<int>(data2[5]) << ", "
          << static_cast<int>(data2[6]) << ", " << static_cast<int>(data2[7]) << "], "
                  << "data3=["
          << static_cast<int>(data3[0]) << ", " << static_cast<int>(data3[1]) << ", "
          << static_cast<int>(data3[2]) << ", " << static_cast<int>(data3[3]) << ", "
          << static_cast<int>(data3[4]) << ", " << static_cast<int>(data3[5]) << ", "
          << static_cast<int>(data3[6]) << ", " << static_cast<int>(data3[7]) << "];";

        LOG(INFO) << std::hex << "data0=[" << vreinterpret_u32_u8(data0)[0]
                  << ", " << vreinterpret_u32_u8(data0)[1] << "], "
                  << "data1=[" << vreinterpret_u32_u8(data1)[0] << ", "
                  << vreinterpret_u32_u8(data1)[1] << "], "
                  << "data2=[" << vreinterpret_u32_u8(data2)[0] << ", "
                  << vreinterpret_u32_u8(data2)[1] << "], "
                  << "data3=[" << vreinterpret_u32_u8(data3)[0] << ", "
                  << vreinterpret_u32_u8(data3)[1] << "]; ";
                  */

        LOG(INFO) << "Difference 0: " << std::hex << static_cast<int>(max0 - min0)
                  << " threshold0: "
                  << static_cast<int>(min0 + (max0 - min0) / 2);
        LOG(INFO) << "Difference 1: " << std::hex << static_cast<int>(max1 - min1)
                  << " threshold1: "
                  << static_cast<int>(min1 + (max1 - min1) / 2);

        uint8x8_t block00_u8 = vreinterpret_u8_u32(
            vuzp1_u32(vreinterpret_u32_u8(data0), vreinterpret_u32_u8(data1)));
        uint8x8_t block10_u8 = vreinterpret_u8_u32(
            vuzp1_u32(vreinterpret_u32_u8(data2), vreinterpret_u32_u8(data3)));

        LOG(INFO) << "block0";

        LOG(INFO) << std::hex
          << static_cast<int>(block00_u8[0]) << ", "
          << static_cast<int>(block00_u8[1]) << ", "
          << static_cast<int>(block00_u8[2]) << ", "
          << static_cast<int>(block00_u8[3]);
        LOG(INFO) << std::hex
          << static_cast<int>(block00_u8[4]) << ", "
          << static_cast<int>(block00_u8[5]) << ", "
          << static_cast<int>(block00_u8[6]) << ", "
          << static_cast<int>(block00_u8[7]);

        LOG(INFO) << std::hex
          << static_cast<int>(block10_u8[0]) << ", "
          << static_cast<int>(block10_u8[1]) << ", "
          << static_cast<int>(block10_u8[2]) << ", "
          << static_cast<int>(block10_u8[3]);
        LOG(INFO) << std::hex
          << static_cast<int>(block10_u8[4]) << ", "
          << static_cast<int>(block10_u8[5]) << ", "
          << static_cast<int>(block10_u8[6]) << ", "
          << static_cast<int>(block10_u8[7]);

        const uint8x8_t thresh0 = vdup_n_u8(min0 + (max0 - min0) / 2);

      if (max0 - min0 < min_white_black_diff) {
        // If the difference is too small, set them all to 127.
        block00_u8 = vdup_n_u8(127);
        block10_u8 = block00_u8;
      } else {
        block00_u8 = vcgt_u8(block00_u8, thresh0);
        block10_u8 = vcgt_u8(block10_u8, thresh0);
      }

        LOG(INFO) << std::hex
          << static_cast<int>(block00_u8[0]) << ", "
          << static_cast<int>(block00_u8[1]) << ", "
          << static_cast<int>(block00_u8[2]) << ", "
          << static_cast<int>(block00_u8[3]);
        LOG(INFO) << std::hex
          << static_cast<int>(block00_u8[4]) << ", "
          << static_cast<int>(block00_u8[5]) << ", "
          << static_cast<int>(block00_u8[6]) << ", "
          << static_cast<int>(block00_u8[7]);

        LOG(INFO) << std::hex
          << static_cast<int>(block10_u8[0]) << ", "
          << static_cast<int>(block10_u8[1]) << ", "
          << static_cast<int>(block10_u8[2]) << ", "
          << static_cast<int>(block10_u8[3]);
        LOG(INFO) << std::hex
          << static_cast<int>(block10_u8[4]) << ", "
          << static_cast<int>(block10_u8[5]) << ", "
          << static_cast<int>(block10_u8[6]) << ", "
          << static_cast<int>(block10_u8[7]);

        uint8x8_t block01_u8 = vreinterpret_u8_u32(
            vuzp2_u32(vreinterpret_u32_u8(data0), vreinterpret_u32_u8(data1)));
        uint8x8_t block11_u8 = vreinterpret_u8_u32(
            vuzp2_u32(vreinterpret_u32_u8(data2), vreinterpret_u32_u8(data3)));

        LOG(INFO) << "block1";

        LOG(INFO) << std::hex
          << static_cast<int>(block01_u8[0]) << ", "
          << static_cast<int>(block01_u8[1]) << ", "
          << static_cast<int>(block01_u8[2]) << ", "
          << static_cast<int>(block01_u8[3]);
        LOG(INFO) << std::hex
          << static_cast<int>(block01_u8[4]) << ", "
          << static_cast<int>(block01_u8[5]) << ", "
          << static_cast<int>(block01_u8[6]) << ", "
          << static_cast<int>(block01_u8[7]);

        LOG(INFO) << std::hex
          << static_cast<int>(block11_u8[0]) << ", "
          << static_cast<int>(block11_u8[1]) << ", "
          << static_cast<int>(block11_u8[2]) << ", "
          << static_cast<int>(block11_u8[3]);
        LOG(INFO) << std::hex
          << static_cast<int>(block11_u8[4]) << ", "
          << static_cast<int>(block11_u8[5]) << ", "
          << static_cast<int>(block11_u8[6]) << ", "
          << static_cast<int>(block11_u8[7]);

        const uint8x8_t thresh1 = vdup_n_u8(min1 + (max1 - min1) / 2);

        if (max1 - min1 < min_white_black_diff) {
          block01_u8 = vdup_n_u8(127);
          block11_u8 = block01_u8;
        } else {
          block01_u8 = vcgt_u8(block01_u8, thresh1);
          block11_u8 = vcgt_u8(block11_u8, thresh1);
        }

        LOG(INFO) << std::hex
          << static_cast<int>(block01_u8[0]) << ", "
          << static_cast<int>(block01_u8[1]) << ", "
          << static_cast<int>(block01_u8[2]) << ", "
          << static_cast<int>(block01_u8[3]);
        LOG(INFO) << std::hex
          << static_cast<int>(block01_u8[4]) << ", "
          << static_cast<int>(block01_u8[5]) << ", "
          << static_cast<int>(block01_u8[6]) << ", "
          << static_cast<int>(block01_u8[7]);

        LOG(INFO) << std::hex
          << static_cast<int>(block11_u8[0]) << ", "
          << static_cast<int>(block11_u8[1]) << ", "
          << static_cast<int>(block11_u8[2]) << ", "
          << static_cast<int>(block11_u8[3]);
        LOG(INFO) << std::hex
          << static_cast<int>(block11_u8[4]) << ", "
          << static_cast<int>(block11_u8[5]) << ", "
          << static_cast<int>(block11_u8[6]) << ", "
          << static_cast<int>(block11_u8[7]);
      }

      // Now, put pixels from the same block into the same registers.
      uint8x8_t block00_u8;
      uint8x8_t block10_u8;
      if (max0 - min0 < min_white_black_diff) {
        // If the difference is too small, set them all to 127.
        block00_u8 = vdup_n_u8(127);
        block10_u8 = block00_u8;
      } else {
        const uint8x8_t thresh0 = vdup_n_u8(min0 + (max0 - min0) / 2);

        // Otherwise, now that we have pulled out the even uint32_t's (which is
        // a row of each block of pixels), compare against the threshold and
        // write that out.
        block00_u8 =
            vcgt_u8(vreinterpret_u8_u32(vuzp1_u32(vreinterpret_u32_u8(data0),
                                                  vreinterpret_u32_u8(data1))),
                    thresh0);
        block10_u8 =
            vcgt_u8(vreinterpret_u8_u32(vuzp1_u32(vreinterpret_u32_u8(data2),
                                                  vreinterpret_u32_u8(data3))),
                    thresh0);
      }

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

      if (print) {
        const uint8x8_t data0 = vreinterpret_u8_u32(vuzp1_u32(
            vreinterpret_u32_u8(block00_u8), vreinterpret_u32_u8(block01_u8)));
        const uint8x8_t data1 = vreinterpret_u8_u32(vuzp2_u32(
            vreinterpret_u32_u8(block00_u8), vreinterpret_u32_u8(block01_u8)));

        const uint8x8_t data2 = vreinterpret_u8_u32(vuzp1_u32(
            vreinterpret_u32_u8(block10_u8), vreinterpret_u32_u8(block11_u8)));
        const uint8x8_t data3 = vreinterpret_u8_u32(vuzp2_u32(
            vreinterpret_u32_u8(block10_u8), vreinterpret_u32_u8(block11_u8)));

        LOG(INFO) << std::hex << "data0=["
          << static_cast<int>(data0[0]) << ", " << static_cast<int>(data0[1]) << ", "
          << static_cast<int>(data0[2]) << ", " << static_cast<int>(data0[3]) << ", "
          << static_cast<int>(data0[4]) << ", " << static_cast<int>(data0[5]) << ", "
          << static_cast<int>(data0[6]) << ", " << static_cast<int>(data0[7]) << "]";
        LOG(INFO) << std::hex
                  << "data1=["
          << static_cast<int>(data1[0]) << ", " << static_cast<int>(data1[1]) << ", "
          << static_cast<int>(data1[2]) << ", " << static_cast<int>(data1[3]) << ", "
          << static_cast<int>(data1[4]) << ", " << static_cast<int>(data1[5]) << ", "
          << static_cast<int>(data1[6]) << ", " << static_cast<int>(data1[7]) << "]";
        LOG(INFO) << std::hex
                  << "data2=["
          << static_cast<int>(data2[0]) << ", " << static_cast<int>(data2[1]) << ", "
          << static_cast<int>(data2[2]) << ", " << static_cast<int>(data2[3]) << ", "
          << static_cast<int>(data2[4]) << ", " << static_cast<int>(data2[5]) << ", "
          << static_cast<int>(data2[6]) << ", " << static_cast<int>(data2[7]) << "]";
        LOG(INFO) << std::hex
                  << "data3=["
          << static_cast<int>(data3[0]) << ", " << static_cast<int>(data3[1]) << ", "
          << static_cast<int>(data3[2]) << ", " << static_cast<int>(data3[3]) << ", "
          << static_cast<int>(data3[4]) << ", " << static_cast<int>(data3[5]) << ", "
          << static_cast<int>(data3[6]) << ", " << static_cast<int>(data3[7]) << "]";

        LOG(INFO) << "h: " << h << " w: " << w;
        LOG(INFO) << "Writing to address " << std::hex << (h * 4 + 0) * width_ / 2 + w * 4;
        LOG(INFO) << "Writing to address " << std::hex << (h * 4 + 1) * width_ / 2 + w * 4;
        LOG(INFO) << "Writing to address " << std::hex << (h * 4 + 2) * width_ / 2 + w * 4;
        LOG(INFO) << "Writing to address " << std::hex << (h * 4 + 3) * width_ / 2 + w * 4;
      }


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

  LOG(INFO) << "Neon After, took "
            << double_milli(pass1_time - start_time).count()
            << "ms for pass 1, " << double_milli(end_time - pass1_time).count()
            << "ms for pass 2, " << double_milli(end_time - start_time).count()
            << "ms overall";
}

}  // namespace frc::apriltag
