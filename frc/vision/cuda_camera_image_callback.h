#ifndef FRC_VISION_CUDA_CAMERA_IMAGE_CALLBACK_H_
#define FRC_VISION_CUDA_CAMERA_IMAGE_CALLBACK_H_

#include "aos/events/shm_event_loop.h"
#include "frc/vision/charuco_lib.h"

namespace frc::vision {

// Constructs a CameraImageCallback where the image will be in CPU/GPU unified
// memory when the callback is called.
class CudaCameraImageCallback : public CameraImageCallback {
 public:
  // `max_age` is the age to start dropping frames at
  CudaCameraImageCallback(
      aos::EventLoop *event_loop, std::string_view channel,
      std::function<void(const CameraImage &image,
                         aos::monotonic_clock::time_point)> &&handle_image_fn,
      aos::monotonic_clock::duration max_age = std::chrono::milliseconds(100));

  ~CudaCameraImageCallback();

  // Pins the watcher's shared memory to CPU/GPU unified memory.
  void PinMemory(aos::ShmEventLoop *shm_event_loop);

 private:
  const aos::Channel *channel_;

  std::vector<absl::Span<char>> unified_memory_;
};

}  // namespace frc::vision

#endif  // FRC_VISION_CUDA_CAMERA_IMAGE_CALLBACK_H_
