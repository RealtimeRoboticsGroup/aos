#include "aos/flatbuffers.h"

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

namespace aos {

uint8_t *FixedAllocatorBase::allocate(size_t allocated_size) {
  if (is_allocated_) {
    ABSL_LOG(FATAL)
        << "Can't allocate more memory with a fixed size allocator.  "
           "Increase the memory reserved.";
  }
  ABSL_CHECK_LE(allocated_size, size());

  is_allocated_ = true;
  return data();
}

uint8_t *FixedAllocatorBase::reallocate_downward(uint8_t *, size_t, size_t,
                                                 size_t, size_t) {
  ABSL_LOG(FATAL) << "Can't allocate more memory with a fixed size allocator.  "
                     "Increase the memory reserved.";
  return nullptr;
}

}  // namespace aos
