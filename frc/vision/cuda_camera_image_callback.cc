#include "frc/vision/cuda_camera_image_callback.h"

#include "cuda_runtime.h"
#include "frc/orin/cuda.h"

namespace frc::vision {

CudaCameraImageCallback::CudaCameraImageCallback(
    aos::EventLoop *event_loop, std::string_view channel,
    std::function<void(const CameraImage &image,
                       aos::monotonic_clock::time_point)> &&handle_image_fn,
    aos::monotonic_clock::duration max_age)
    : CameraImageCallback(event_loop, channel, std::move(handle_image_fn),
                          max_age),
      channel_(event_loop->GetChannel<CameraImage>(channel)) {}

CudaCameraImageCallback::~CudaCameraImageCallback() {
  for (absl::Span<char> memory : unified_memory_) {
    CHECK_CUDA(cudaHostUnregister(memory.data()));
  }
}

void CudaCameraImageCallback::PinMemory(aos::ShmEventLoop *shm_event_loop) {
  shm_event_loop->SetWatcherUseWritableMemory(channel_, true);

  CHECK(channel_->read_method() == aos::ReadMethod::PIN)
      << ": Channel must be pinned to pin to CPU/GPU unified memory: "
      << aos::FlatbufferToJson(channel_);
  absl::Span<char> memory = shm_event_loop->GetWatcherSharedMemory(channel_);

  auto result =
      cudaHostRegister(memory.data(), memory.size(), cudaHostRegisterDefault);
  if (result != cudaErrorHostMemoryAlreadyRegistered) {
    CHECK_CUDA(result) << "Failed to pin unified memory at address "
                       << static_cast<const void *>(memory.data()) << ", "
                       << memory.size() << "bytes";
  }

  // We should be able to get the device pointer if unified memory registration
  // worked.
  uint8_t *device_ptr = nullptr;
  CHECK_CUDA(cudaHostGetDevicePointer(&device_ptr,
                                      static_cast<void *>(memory.data()), 0));
  CHECK_NE(device_ptr, nullptr);

  unified_memory_.push_back(memory);
}

}  // namespace frc::vision
