#include <mach/mach.h>
#include <mach/mach_vm.h>
#include <sys/mman.h>
#include <unistd.h>

#include "absl/log/absl_check.h"

#include "aos/ipc_lib/process_local_queue_internal.h"

namespace aos::ipc_lib::internal {

// macOS has no memfd.  POSIX shm would need a name, if only for the instant
// between shm_open() and shm_unlink().  Mach can do it with no name at all:
// map anonymous memory, then ask the kernel for a second view of the same
// pages and take write permission away from that one.
//
// Both views are inherited across fork(), as both mappings of the memfd are
// on Linux.  A fork()ed child (a death test, say) reads through the
// read-only view, and without it the child faults instead of running.
ProcessLocalMapping MapProcessLocal(size_t size) {
  // The Mach calls work in whole pages.  mmap rounds up internally; this
  // rounds the same way so the remapped view covers exactly the first one.
  const size_t page_size = static_cast<size_t>(sysconf(_SC_PAGESIZE));
  const size_t rounded = (size + page_size - 1) / page_size * page_size;

  void *writable = mmap(nullptr, rounded, PROT_READ | PROT_WRITE,
                        MAP_SHARED | MAP_ANON, -1, 0);
  ABSL_PCHECK(writable != MAP_FAILED);

  mach_vm_address_t readonly = 0;
  vm_prot_t current_protection = VM_PROT_NONE;
  vm_prot_t max_protection = VM_PROT_NONE;
  const kern_return_t remap_result = mach_vm_remap(
      mach_task_self(), &readonly, rounded, /*mask=*/0, VM_FLAGS_ANYWHERE,
      mach_task_self(), reinterpret_cast<mach_vm_address_t>(writable),
      /*copy=*/FALSE, &current_protection, &max_protection, VM_INHERIT_SHARE);
  ABSL_CHECK_EQ(remap_result, KERN_SUCCESS)
      << ": mach_vm_remap failed: " << mach_error_string(remap_result);

  const kern_return_t protect_result = mach_vm_protect(
      mach_task_self(), readonly, rounded, /*set_maximum=*/FALSE, VM_PROT_READ);
  ABSL_CHECK_EQ(protect_result, KERN_SUCCESS)
      << ": mach_vm_protect failed: " << mach_error_string(protect_result);

  return ProcessLocalMapping{writable, reinterpret_cast<const void *>(readonly),
                             rounded};
}

void UnmapProcessLocal(const ProcessLocalMapping &mapping) {
  ABSL_PCHECK(munmap(mapping.writable, mapping.size) == 0);
  ABSL_PCHECK(munmap(const_cast<void *>(mapping.readonly), mapping.size) == 0);
}

}  // namespace aos::ipc_lib::internal
