#include "frc/orin/threshold.h"

#include <stdint.h>

#include "frc/orin/cuda.h"

namespace frc::apriltag {
namespace {

// 1280 -> 2 * 128 * 5
// 720 -> 2 * 8 * 5 * 9
//
// 1456 -> 2 * 8 * 7 * 13
// 1088 -> 2 * 32 * 17
//
// 1600 -> 64 * 25
// 1304 -> 8 * 163
template <vision::ImageFormat IMAGE_FORMAT>
__device__ __forceinline__ uint8_t ToGray(const uint8_t *color_image) {
  if constexpr (IMAGE_FORMAT == vision::ImageFormat::MONO8) {
    return color_image[0];  // Grayscale input is already aliased to
                            // color device image
  } else if constexpr (IMAGE_FORMAT == vision::ImageFormat::MONO16) {
    return color_image[1];  // MSBits of MONO16 - does this also work
                            // on jetson or do we need a be/le split?
  } else if constexpr (IMAGE_FORMAT == vision::ImageFormat::YUYV422) {
    return color_image[0];  // YUY input
  } else if constexpr (IMAGE_FORMAT == vision::ImageFormat::BGR8) {
    return 0.114f * static_cast<float>(color_image[0]) +
           0.587f * static_cast<float>(color_image[1]) +
           0.299f * static_cast<float>(color_image[2]);  // BGR input
  } else if constexpr (IMAGE_FORMAT == vision::ImageFormat::BGRA8) {
    return 0.114f * static_cast<float>(color_image[0]) +
           0.587f * static_cast<float>(color_image[1]) +
           0.299f * static_cast<float>(
                        color_image[2]);  // BGRA input, skip alpha channel
  }
}

// Convert from input format to grayscale.
template <vision::ImageFormat IMAGE_FORMAT>
__global__ void InternalCudaToGreyscale(const uint8_t *color_image,
                                        uint8_t *gray_image,
                                        apriltag_size_t width,
                                        apriltag_size_t height) {
  constexpr apriltag_size_t kBytesPerPixel = BytesPerPixel(IMAGE_FORMAT);
  apriltag_size_t i = blockIdx.x * blockDim.x + threadIdx.x;
  while (i < width * height) {
    gray_image[i] = ToGray<IMAGE_FORMAT>(color_image + i * kBytesPerPixel);

    i += blockDim.x * gridDim.x;
  }
}

// Writes out the grayscale image and decimated image.
template <vision::ImageFormat IMAGE_FORMAT>
__global__ void InternalCudaToGreyscaleAndDecimateHalide(
    const uint8_t *color_image, uint8_t *decimated_image,
    const apriltag_size_t in_width, const apriltag_size_t in_height) {
  constexpr apriltag_size_t kBytesPerPixel = BytesPerPixel(IMAGE_FORMAT);
  const apriltag_size_t out_height = in_height / 2;
  const apriltag_size_t out_width = in_width / 2;
  apriltag_size_t out_i = blockIdx.x * blockDim.x + threadIdx.x;

  while (out_i < out_width * out_height) {
    const apriltag_size_t out_row = out_i / out_width;
    const apriltag_size_t out_col = out_i - out_width * out_row;

    const u_int32_t in_row = out_row * 2;
    const u_int32_t in_col = out_col * 2;

    const apriltag_size_t in_i = in_row * in_width + in_col;

    decimated_image[out_row * out_width + out_col] =
        ToGray<IMAGE_FORMAT>(color_image + in_i * kBytesPerPixel);
    out_i += blockDim.x * gridDim.x;
  }
  // TODO(austin): Figure out how to load contiguous memory reasonably
  // efficiently and max/min over it.

  // TODO(austin): Can we do the threshold here too?  That would be less memory
  // bandwidth consumed...
  //
  //   could do it by merging this code with InternalBlockMinMax, altering
  //   the input indexing so it grabs from the undecimated input image.  Add
  //   the grayscale converion code in there as well?
}

// Returns the min and max for a row of 4 pixels.
__forceinline__ __device__ uchar2 minmax(uchar4 row) {
  uint8_t min_val = std::min(std::min(row.x, row.y), std::min(row.z, row.w));
  uint8_t max_val = std::max(std::max(row.x, row.y), std::max(row.z, row.w));
  return make_uchar2(min_val, max_val);
}

// Returns the min and max for a set of min and maxes.
__forceinline__ __device__ uchar2 minmax(uchar2 val0, uchar2 val1) {
  return make_uchar2(std::min(val0.x, val1.x), std::max(val0.y, val1.y));
}

// Returns the pixel index of a pixel at the provided x and y location.
__forceinline__ __device__ apriltag_size_t XYToIndex(apriltag_size_t width,
                                                     apriltag_size_t x,
                                                     apriltag_size_t y) {
  return width * y + x;
}

// Computes the min and max pixel value for each block of 4 pixels.
__global__ void InternalBlockMinMax(const uint8_t *decimated_image,
                                    uchar2 *unfiltered_minmax_image,
                                    const apriltag_size_t width,
                                    const apriltag_size_t height) {
  uchar2 vals[4];
  const apriltag_size_t x = blockIdx.x * blockDim.x + threadIdx.x;
  const apriltag_size_t y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x >= width || y >= height) {
    return;
  }

  for (int i = 0; i < 4; ++i) {
    const uchar4 decimated_block = *reinterpret_cast<const uchar4 *>(
        decimated_image + XYToIndex(width * 4, x * 4, y * 4 + i));

    vals[i] = minmax(decimated_block);
  }

  unfiltered_minmax_image[XYToIndex(width, x, y)] =
      minmax(minmax(vals[0], vals[1]), minmax(vals[2], vals[3]));
}

// Filters the min/max for the surrounding block of 9 pixels centered on our
// location using min/max and writes the result back out.
__global__ void InternalBlockFilter(const uchar2 *unfiltered_minmax_image,
                                    uchar2 *minmax_image,
                                    const apriltag_size_t width,
                                    const apriltag_size_t height) {
  uchar2 result = make_uchar2(255, 0);

  const apriltag_size_t x = blockIdx.x * blockDim.x + threadIdx.x;
  const apriltag_size_t y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x >= width || y >= height) {
    return;
  }

  // Iterate through the 3x3 set of points centered on the point this image is
  // responsible for, and compute the overall min/max.
#pragma unroll
  for (int32_t i = -1; i <= 1; ++i) {
#pragma unroll
    for (int32_t j = -1; j <= 1; ++j) {
      const int32_t read_x = x + i;
      const int32_t read_y = y + j;

      if (read_x < 0 || read_x >= static_cast<int32_t>(width)) {
        continue;
      }
      if (read_y < 0 || read_y >= static_cast<int32_t>(height)) {
        continue;
      }

      result = minmax(
          result, unfiltered_minmax_image[XYToIndex(width, read_x, read_y)]);
    }
  }

  minmax_image[XYToIndex(width, x, y)] = result;
}

// Thresholds the image based on the filtered thresholds.
__global__ void InternalThreshold(const uint8_t *decimated_image,
                                  const uchar2 *minmax_image,
                                  uint8_t *thresholded_image,
                                  const apriltag_size_t width,
                                  const apriltag_size_t height,
                                  const apriltag_size_t min_white_black_diff) {
  apriltag_size_t i = blockIdx.x * blockDim.x + threadIdx.x;
  while (i < width * height) {
    const apriltag_size_t x = i % width;
    const apriltag_size_t y = i / width;

    const uchar2 minmax_val = minmax_image[x / 4 + (y / 4) * width / 4];

    uint8_t result;
    if (minmax_val.y - minmax_val.x < min_white_black_diff) {
      result = 127;
    } else {
      uint8_t thresh = minmax_val.x + (minmax_val.y - minmax_val.x) / 2;
      if (decimated_image[i] > thresh) {
        result = 255;
      } else {
        result = 0;
      }
    }

    thresholded_image[i] = result;
    i += blockDim.x * gridDim.x;
  }
}

template <vision::ImageFormat IMAGE_FORMAT>
class TypedThreshold : public Threshold {
 public:
  // Create a full-size grayscale image from a color image on the provided
  // stream.
  void CudaToGreyscale(const uint8_t *color_image, uint8_t *gray_image,
                       apriltag_size_t width, apriltag_size_t height,
                       CudaStream *stream) override;

  // Converts to grayscale, decimates, and thresholds an image on the provided
  // stream.
  void CudaThresholdAndDecimate(
      const uint8_t *color_image, uint8_t *decimated_image,
      uint8_t *unfiltered_minmax_image, uint8_t *minmax_image,
      uint8_t *thresholded_image, apriltag_size_t width, apriltag_size_t height,
      apriltag_size_t min_white_black_diff, CudaStream *stream) override;

  virtual ~TypedThreshold() = default;
};

template <vision::ImageFormat IMAGE_FORMAT>
void TypedThreshold<IMAGE_FORMAT>::CudaToGreyscale(const uint8_t *color_image,
                                                   uint8_t *gray_image,
                                                   apriltag_size_t width,
                                                   apriltag_size_t height,
                                                   CudaStream *stream) {
  constexpr size_t kThreads = 256;
  {
    // Step one, convert to gray and decimate.
    size_t kBlocks = (width * height + kThreads - 1) / kThreads / 4;
    InternalCudaToGreyscale<IMAGE_FORMAT>
        <<<kBlocks, kThreads, 0, stream->get()>>>(color_image, gray_image,
                                                  width, height);
    MaybeCheckAndSynchronize();
  }
}

template <vision::ImageFormat IMAGE_FORMAT>
void TypedThreshold<IMAGE_FORMAT>::CudaThresholdAndDecimate(
    const uint8_t *color_image, uint8_t *decimated_image,
    uint8_t *unfiltered_minmax_image, uint8_t *minmax_image,
    uint8_t *thresholded_image, apriltag_size_t width, apriltag_size_t height,
    apriltag_size_t min_white_black_diff, CudaStream *stream) {
  CHECK((width % 8) == 0);
  CHECK((height % 8) == 0);
  constexpr size_t kThreads = 256;
  const apriltag_size_t decimated_width = width / 2;
  const apriltag_size_t decimated_height = height / 2;

  {
    // Step one, convert to gray and decimate.
    const size_t kBlocks =
        (decimated_width * decimated_height + kThreads - 1) / kThreads / 4;
    InternalCudaToGreyscaleAndDecimateHalide<IMAGE_FORMAT>
        <<<kBlocks, kThreads, 0, stream->get()>>>(color_image, decimated_image,
                                                  width, height);
    MaybeCheckAndSynchronize();
  }

  {
    // Step 2, compute a min/max for each block of 4x4 (16) pixels.
    const dim3 threads(16, 16, 1);
    const dim3 blocks((decimated_width / 4 + 15) / 16,
                      (decimated_height / 4 + 15) / 16, 1);

    InternalBlockMinMax<<<blocks, threads, 0, stream->get()>>>(
        decimated_image, reinterpret_cast<uchar2 *>(unfiltered_minmax_image),
        decimated_width / 4, decimated_height / 4);
    MaybeCheckAndSynchronize();

    // Step 3, Blur those min/max's a further +- 1 block in each direction using
    // min/max.
    InternalBlockFilter<<<blocks, threads, 0, stream->get()>>>(
        reinterpret_cast<uchar2 *>(unfiltered_minmax_image),
        reinterpret_cast<uchar2 *>(minmax_image), decimated_width / 4,
        decimated_height / 4);
    MaybeCheckAndSynchronize();
  }

  {
    // Now, write out 127 if the min/max are too close to each other, or 0/255
    // if the pixels are above or below the average of the min/max.
    const apriltag_size_t kBlocks =
        (width * height / 4 + kThreads - 1) / kThreads / 4;
    InternalThreshold<<<kBlocks, kThreads, 0, stream->get()>>>(
        decimated_image, reinterpret_cast<uchar2 *>(minmax_image),
        thresholded_image, decimated_width, decimated_height,
        min_white_black_diff);
    MaybeCheckAndSynchronize();
  }
}

}  // namespace

std::unique_ptr<Threshold> MakeThreshold(vision::ImageFormat image_format) {
  switch (image_format) {
    case vision::ImageFormat::MONO8:
      return std::make_unique<TypedThreshold<vision::ImageFormat::MONO8>>();
    case vision::ImageFormat::MONO16:
      return std::make_unique<TypedThreshold<vision::ImageFormat::MONO16>>();
    case vision::ImageFormat::YUYV422:
      return std::make_unique<TypedThreshold<vision::ImageFormat::YUYV422>>();
    case vision::ImageFormat::BGR8:
      return std::make_unique<TypedThreshold<vision::ImageFormat::BGR8>>();
    case vision::ImageFormat::BGRA8:
      return std::make_unique<TypedThreshold<vision::ImageFormat::BGRA8>>();
    default:
      LOG(FATAL) << "Unknown image format: "
                 << vision::EnumNameImageFormat(image_format);
  }
}

}  // namespace frc::apriltag
