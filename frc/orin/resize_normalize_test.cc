#include "frc/orin/resize_normalize.h"

#include "absl/log/log.h"
#include "gtest/gtest.h"

#include "HalideBuffer.h"
#include "HalideRuntime.h"
#include "aos/flatbuffers.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/time/time.h"
#include "frc/vision/vision_generated.h"

namespace frc::apriltag::testing {

typedef std::chrono::duration<double, std::milli> double_milli;

class NormalizeResizeTest : public ::testing::Test {
 public:
  aos::FlatbufferVector<frc::vision::CameraImage> ReadImage(
      std::string_view path) {
    return aos::FileToFlatbuffer<frc::vision::CameraImage>("../" +
                                                           std::string(path));
  }
};

// Tests that the halide threshold matches a simple C++ implementation that I
// know is correct.
TEST_F(NormalizeResizeTest, HalideNoramlize) {
  auto image_fbs = ReadImage("coral_image_thriftycam_2025/file/image.bfbs");

  LOG(INFO) << "Image is: " << image_fbs.message().cols() << " x "
            << image_fbs.message().rows();

  const size_t width = image_fbs.message().cols();
  const size_t height = image_fbs.message().rows();

  constexpr size_t kNormalizedWidth = 512;
  constexpr size_t kNormalizedHeight = 416;
  std::vector<float> normalized_image(3 * kNormalizedWidth * kNormalizedHeight);

  {
    std::array<halide_dimension_t, 2> image_dimensions{{
        {
            /*.min =*/0,
            /*.extent =*/
            static_cast<int32_t>(image_fbs.message().cols()),
            /*.stride =*/1,
            /*.flags =*/0,
        },
        {
            /*.min = */ 0,
            /*.extent =*/
            static_cast<int32_t>(image_fbs.message().rows()),
            /*.stride =*/
            static_cast<int32_t>(image_fbs.message().cols()),
            /*.flags =*/0,
        },
    }};

    Halide::Runtime::Buffer<const uint8_t, 2> image(
        reinterpret_cast<const uint8_t *>(image_fbs.message().data()->data()),
        image_dimensions.size(), image_dimensions.data());

    std::array<halide_dimension_t, 3> normalized_dimensions{{
        {
            /*.min =*/0,
            /*.extent =*/
            static_cast<int32_t>(kNormalizedWidth),
            /*.stride =*/1,
            /*.flags =*/0,
        },
        {
            /*.min = */ 0,
            /*.extent =*/
            static_cast<int32_t>(kNormalizedHeight),
            /*.stride =*/
            static_cast<int32_t>(kNormalizedWidth),
            /*.flags =*/0,
        },
        {
            /*.min = */ 0,
            /*.extent =*/
            static_cast<int32_t>(3),
            /*.stride =*/
            static_cast<int32_t>(kNormalizedWidth * kNormalizedHeight),
            /*.flags =*/0,
        },
    }};

    Halide::Runtime::Buffer<float, 3> normalized(
        reinterpret_cast<float *>(normalized_image.data()),
        normalized_dimensions.size(), normalized_dimensions.data());

    aos::monotonic_clock::time_point start = aos::monotonic_clock::now();

    resize_normalize(image, normalized);

    LOG(INFO) << "Took: "
              << std::chrono::duration<double, std::milli>(
                     aos::monotonic_clock::now() - start)
                     .count()
              << "ms";
  }

  // args = "input_rows=1304 input_cols=1600 output_rows=416 output_cols=512
  // input_start_row=0 input_start_col=64",
  size_t input_start_row = 0;
  size_t input_start_col = 64;
  const uint8_t *image_data =
      reinterpret_cast<const uint8_t *>(image_fbs.message().data()->data());

  for (size_t c = 0; c < 3; ++c) {
    for (size_t i = input_start_row;
         i < input_start_row + kNormalizedHeight * 3; i += 3) {
      const size_t output_i = (i - input_start_row) / 3;
      ASSERT_LT(i, height);

      for (size_t j = input_start_col;
           j < input_start_col + kNormalizedWidth * 3; j += 3) {
        ASSERT_LT(j, width);
        const size_t output_j = (j - input_start_col) / 3;

        const float expected =
            (static_cast<float>(image_data[(i + 0) * width + j + 0]) +
             static_cast<float>(image_data[(i + 0) * width + j + 1]) +
             static_cast<float>(image_data[(i + 0) * width + j + 2]) +
             static_cast<float>(image_data[(i + 1) * width + j + 0]) +
             static_cast<float>(image_data[(i + 1) * width + j + 1]) +
             static_cast<float>(image_data[(i + 1) * width + j + 2]) +
             static_cast<float>(image_data[(i + 2) * width + j + 0]) +
             static_cast<float>(image_data[(i + 2) * width + j + 1]) +
             static_cast<float>(image_data[(i + 2) * width + j + 2])) /
            (9.0f * 255.0f);

        ASSERT_NEAR(expected,
                    normalized_image[c * kNormalizedWidth * kNormalizedHeight +
                                     kNormalizedWidth * output_i + output_j],
                    1e-4)
            << "i = " << i << ", j = " << j << ", output_i = " << output_i
            << ", output_j = " << output_j;

        ASSERT_GE(expected, 0.0f);
        ASSERT_LE(expected, 1.0f);
      }
    }
  }
}

}  // namespace frc::apriltag::testing
