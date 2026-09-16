#include <sys/mman.h>
#include <sys/syscall.h>
#include <unistd.h>

#include "absl/log/absl_check.h"

#include "aos/ipc_lib/process_local_queue_internal.h"

namespace aos::ipc_lib::internal {

namespace {

// Not every glibc AOS builds against wraps memfd_create(2) -- the roboRIO's
// predates it -- but every kernel AOS runs on has the syscall, so go straight
// to that.  The flag value is the kernel's; the header defining it is as new
// as the wrapper.
constexpr unsigned int kMfdCloexec = 0x0001U;

int CreateMemfd() {
  return static_cast<int>(syscall(SYS_memfd_create, "aos-queue", kMfdCloexec));
}

}  // namespace

// A memfd is exactly this: anonymous shared memory with a descriptor, so it
// can be mapped twice, and no name in any filesystem, so there is nothing to
// unlink.  The descriptor is closed as soon as both views exist; the mappings
// keep the memory alive.
ProcessLocalMapping MapProcessLocal(size_t size) {
  const int fd = CreateMemfd();
  ABSL_PCHECK(fd != -1) << ": memfd_create failed";
  ABSL_PCHECK(ftruncate(fd, size) == 0);

  void *writable =
      mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  ABSL_PCHECK(writable != MAP_FAILED);
  void *readonly = mmap(nullptr, size, PROT_READ, MAP_SHARED, fd, 0);
  ABSL_PCHECK(readonly != MAP_FAILED);
  ABSL_PCHECK(close(fd) == 0);

  return ProcessLocalMapping{writable, readonly, size};
}

void UnmapProcessLocal(const ProcessLocalMapping &mapping) {
  ABSL_PCHECK(munmap(mapping.writable, mapping.size) == 0);
  ABSL_PCHECK(munmap(const_cast<void *>(mapping.readonly), mapping.size) == 0);
}

}  // namespace aos::ipc_lib::internal
