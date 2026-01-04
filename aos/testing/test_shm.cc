#include "aos/testing/test_shm.h"

#include <stddef.h>
#include <sys/mman.h>

namespace aos::testing {

#if !defined(MAP_ANONYMOUS) && defined(MAP_ANON)
#define MAP_ANONYMOUS MAP_ANON
#endif

SharedMemoryBlock::SharedMemoryBlock(size_t size) : size_(size) {
  addr_ = mmap(nullptr, size_, PROT_READ | PROT_WRITE,
               MAP_SHARED | MAP_ANONYMOUS, -1, 0);

  if (addr_ == MAP_FAILED) {
    throw std::system_error(errno, std::generic_category(), "mmap failed");
  }
}

// Corrected Destructor: Now actually frees the memory
SharedMemoryBlock::~SharedMemoryBlock() {
  if (addr_ != nullptr && addr_ != MAP_FAILED) {
    munmap(addr_, size_);
  }
}

SharedMemoryBlock &SharedMemoryBlock::operator=(
    SharedMemoryBlock &&other) noexcept {
  if (this != &other) {
    if (addr_ != nullptr) munmap(addr_, size_);
    addr_ = other.addr_;
    size_ = other.size_;
    other.addr_ = nullptr;
    other.size_ = 0;
  }
  return *this;
}

}  // namespace aos::testing
