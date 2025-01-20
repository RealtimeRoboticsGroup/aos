#ifndef FRC_ORIN_GPU_IMAGE_H_
#define FRC_ORIN_GPU_IMAGE_H_

template <typename T>
struct GpuImage {
  typedef T type;
  T *data;
  size_t rows;
  size_t cols;
  // Step is in elements, not bytes.
  size_t step;
};

#endif  // FRC_ORIN_GPU_IMAGE_H_
