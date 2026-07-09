#include "aos/ipc_lib/shm_mapping.h"

#include <atomic>

namespace aos::ipc_lib {

void PageFaultDataWrite(char *data, size_t size, const long page_size) {
  const size_t pages = (size + page_size - 1) / page_size;
  for (size_t i = 0; i < pages; ++i) {
    char zero = 0;
    std::atomic_ref<char>(data[i * page_size])
        .compare_exchange_strong(zero, 0, std::memory_order_relaxed,
                                 std::memory_order_relaxed);
  }
}

void PageFaultDataRead(const char *data, size_t size, const long page_size) {
  const size_t pages = (size + page_size - 1) / page_size;
  for (size_t i = 0; i < pages; ++i) {
    static_cast<void>(
        std::atomic_ref<char>(const_cast<char &>(data[i * page_size]))
            .load(std::memory_order_relaxed));
  }
}

}  // namespace aos::ipc_lib
