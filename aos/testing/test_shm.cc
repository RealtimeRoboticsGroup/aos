#include "aos/testing/test_shm.h"

#include <stddef.h>
#ifdef _WIN32
#include <windows.h>
#else
#include <sys/mman.h>
#endif

#include "absl/log/absl_check.h"

namespace aos::testing {

// OSX and Linux have different names for the same thing.
#if !defined(MAP_ANONYMOUS) && defined(MAP_ANON)
#define MAP_ANONYMOUS MAP_ANON
#endif

SharedMemoryBlock::SharedMemoryBlock(size_t size) : size_(size) {
#ifdef _WIN32
  addr_ = VirtualAlloc(NULL, size_, MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE);
  ABSL_PCHECK(addr_ != NULL);
#else
  addr_ = mmap(nullptr, size_, PROT_READ | PROT_WRITE,
               MAP_SHARED | MAP_ANONYMOUS, -1, 0);

  ABSL_PCHECK(addr_ != MAP_FAILED);
#endif
}

SharedMemoryBlock::~SharedMemoryBlock() {
  if (addr_ != nullptr) {
#ifdef _WIN32
    VirtualFree(addr_, 0, MEM_RELEASE);
#else
    if (addr_ != MAP_FAILED) {
      munmap(addr_, size_);
    }
#endif
  }
}

}  // namespace aos::testing
