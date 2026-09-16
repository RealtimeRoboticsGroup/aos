#ifndef AOS_IPC_LIB_PROCESS_LOCAL_QUEUE_INTERNAL_H_
#define AOS_IPC_LIB_PROCESS_LOCAL_QUEUE_INTERNAL_H_

#include <stddef.h>

namespace aos::ipc_lib::internal {

// One block of anonymous memory, mapped twice.
struct ProcessLocalMapping {
  void *writable;
  const void *readonly;
  size_t size;
};

// Maps `size` bytes of zeroed, page-aligned memory into this process twice:
// once read/write and once read-only, both views of the same pages.  The
// memory has no name and no file behind it, so nothing outside the process
// can find it and the OS reclaims it when the process exits.  Defined per OS:
// memfd on Linux, a Mach remap on macOS, an unnamed section on Windows.
ProcessLocalMapping MapProcessLocal(size_t size);

// Unmaps both views.
void UnmapProcessLocal(const ProcessLocalMapping &mapping);

}  // namespace aos::ipc_lib::internal

#endif  // AOS_IPC_LIB_PROCESS_LOCAL_QUEUE_INTERNAL_H_
