#include "frc/orin/threshold.h"

#include "gtest/gtest.h"
#include "third_party/apriltag/apriltag.h"
#include "third_party/apriltag/common/unionfind.h"
#include "third_party/apriltag/tag16h5.h"
#include "third_party/apriltag/tag36h11.h"

#include "aos/flatbuffer_merge.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/time/time.h"
#include "frc/vision/vision_generated.h"

extern "C" {
image_u8_t *threshold(apriltag_detector_t *td, image_u8_t *im);
}

namespace frc::apriltag::testing {

typedef std::chrono::duration<double, std::milli> double_milli;

class ThresholdTest : public ::testing::Test {
 public:
  ThresholdTest()
      : tag_family_(tag16h5_create()),
        tag_detector_(MakeTagDetector(tag_family_)) {}

  aos::FlatbufferVector<frc::vision::CameraImage> ReadImage(
      std::string_view path) {
    return aos::FileToFlatbuffer<frc::vision::CameraImage>("../" +
                                                           std::string(path));
  }

  apriltag_detector_t *MakeTagDetector(apriltag_family_t *tag_family) {
    apriltag_detector_t *tag_detector = apriltag_detector_create();

    apriltag_detector_add_family_bits(tag_detector, tag_family, 1);

    tag_detector->nthreads = 6;
    tag_detector->wp = workerpool_create(tag_detector->nthreads);
    tag_detector->qtp.min_white_black_diff = 5;
    tag_detector->debug = false;

    return tag_detector;
  }

 protected:
  apriltag_family_t *tag_family_;
  apriltag_detector_t *tag_detector_;
};

// Checks that 2 images match.
void CheckImage(image_u8_t compare_im_one, image_u8_t compare_im_two,
                std::string_view label) {
  CHECK_EQ(compare_im_one.width, compare_im_two.width);
  CHECK_EQ(compare_im_one.height, compare_im_two.height);
  for (int j = 0; j < compare_im_one.height; ++j) {
    for (int i = 0; i < compare_im_one.width; ++i) {
      CHECK_EQ(compare_im_one.buf[j * compare_im_one.stride + i],
               compare_im_two.buf[j * compare_im_two.stride + i])
          << "First Image Value "
          << (int)compare_im_one.buf[j * compare_im_one.stride + i] << " "
          << "Second Image Value "
          << (int)compare_im_two.buf[j * compare_im_two.stride + i] << " "
          << "At " << i << ", " << j << " for " << label;
    }
  }
}


// Tests that the NEON threshold matches the CUDA one which we know is correct.
// Note: this only runs on armv8.
TEST_F(ThresholdTest, NeonThreshold) {
  auto image =
      ReadImage("orin_capture_24_04_side/file/orin_capture_24_04_side.bfbs");

  LOG(INFO) << "Image is: " << image.message().cols() << " x "
            << image.message().rows();

  const size_t width = image.message().cols();
  const size_t height = image.message().rows();
  const uint8_t *color_image = image.message().data()->data();

  std::unique_ptr<Threshold> cuda_threshold =
      MakeThreshold(vision::ImageFormat::YUYV422, width, height);

  std::unique_ptr<Threshold> neon_threshold =
      MakeNeonThreshold(vision::ImageFormat::YUYV422, width, height);

  GpuMemory<uint8_t> color_image_device(width * height * 2);
  GpuMemory<uint8_t> gray_image_device(width * height);
  GpuMemory<uint8_t> decimated_image_device(width * height / 4);
  GpuMemory<uint8_t> thresholded_image_device(width * height / 4);
  HostMemory<uint8_t> gray_image_host(width * height);
  HostMemory<uint8_t> decimated_image_host(width * height / 4);
  HostMemory<uint8_t> thresholded_image_host(width * height / 4);
  HostMemory<uint8_t> neon_gray_image_host(width * height);
  HostMemory<uint8_t> neon_decimated_image_host(width * height / 4);
  HostMemory<uint8_t> neon_thresholded_image_host(width * height / 4);

  // Run the CUDA version.  Don't worry about efficiency.
  apriltag_size_t min_white_black_diff = 5;
  {
    CudaStream stream;
    color_image_device.MemcpyFrom(color_image);
    cuda_threshold->ToGreyscale(color_image_device.get(), gray_image_device.get(),
                           &stream);
    cuda_threshold->ThresholdAndDecimate(
        color_image_device.get(), decimated_image_device.get(),
        thresholded_image_device.get(), min_white_black_diff, &stream);
    CudaEvent end;
    end.Record(&stream);
    end.Synchronize();
  }

  gray_image_device.MemcpyTo(&gray_image_host);
  decimated_image_device.MemcpyTo(&decimated_image_host);
  thresholded_image_device.MemcpyTo(&thresholded_image_host);

  // Now, run the neon version.
  neon_threshold->ToGreyscale(color_image, neon_gray_image_host.get(), nullptr);
  neon_threshold->ThresholdAndDecimate(
      neon_gray_image_host.get(), neon_decimated_image_host.get(),
      neon_thresholded_image_host.get(), min_white_black_diff, nullptr);

  for (size_t i = 0; i < height; ++i) {
    for (size_t j = 0; j < width; ++j) {
      ASSERT_EQ(gray_image_host.get()[i * width + j],
                neon_gray_image_host.get()[i * width + j])
          << "i = " << i << ", j = " << j;
    }
  }

  image_u8_t gray_im = {
      .width = static_cast<int32_t>(width),
      .height = static_cast<int32_t>(height),
      .stride = static_cast<int32_t>(width),
      .buf = gray_image_host.get(),
  };

  const aos::monotonic_clock::time_point start_time =
      aos::monotonic_clock::now();
  image_u8_t *quad_im = image_u8_decimate(&gray_im, 2);
  const aos::monotonic_clock::time_point decimated_time = aos::monotonic_clock::now();
  image_u8_t *thresholded_im = threshold(tag_detector_, quad_im);
  const aos::monotonic_clock::time_point end_time = aos::monotonic_clock::now();
  LOG(INFO) << "April robotics after, decimated took "
            << double_milli(decimated_time - start_time).count()
            << "ms, threshold took "
            << double_milli(end_time - decimated_time).count()
            << "ms overall took " << double_milli(end_time - start_time).count()
            << "ms";

  image_u8_t decimated_im = {
      .width = static_cast<int32_t>(width / 2),
      .height = static_cast<int32_t>(height / 2),
      .stride = static_cast<int32_t>(width / 2),
      .buf = decimated_image_host.get(),
  };

  image_u8_t neon_decimated_im = {
      .width = static_cast<int32_t>(width / 2),
      .height = static_cast<int32_t>(height / 2),
      .stride = static_cast<int32_t>(width / 2),
      .buf = neon_decimated_image_host.get(),
  };

  // Validate both decimated images against the aprilrobotics version.
  CheckImage(*quad_im, decimated_im, "Decimated Image");
  CheckImage(*quad_im, neon_decimated_im, "Neon Decimated Image");

  image_u8_t cuda_thresholded_im = {
      .width = static_cast<int32_t>(width / 2),
      .height = static_cast<int32_t>(height / 2),
      .stride = static_cast<int32_t>(width / 2),
      .buf = thresholded_image_host.get(),
  };

  image_u8_t neon_thresholded_im = {
      .width = static_cast<int32_t>(width / 2),
      .height = static_cast<int32_t>(height / 2),
      .stride = static_cast<int32_t>(width / 2),
      .buf = neon_thresholded_image_host.get(),
  };

  CheckImage(cuda_thresholded_im, neon_thresholded_im, "Neon Thresholded Image");

  image_u8_destroy(quad_im);
  image_u8_destroy(thresholded_im);
}

}  // namespace frc::apriltag::testing
