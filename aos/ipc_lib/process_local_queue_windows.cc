#include <windows.h>

#include "absl/log/absl_check.h"

#include "aos/ipc_lib/aos_sync.h"
#include "aos/ipc_lib/process_local_queue_internal.h"

namespace aos::ipc_lib::internal {

// An unnamed page-file-backed section: documented to start out zeroed, sized
// at creation, and gone when its last handle and view go away.  This is the
// section shm_mapping_windows.cc turns down for shared memory because its
// contents die with the process -- which is exactly what is wanted here.
// The handle is closed once both views exist; the views keep the section
// alive.
ProcessLocalMapping MapProcessLocal(size_t size) {
  const ULARGE_INTEGER section_size{.QuadPart = size};
  HANDLE section =
      CreateFileMappingW(INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE,
                         section_size.HighPart, section_size.LowPart, nullptr);
  ABSL_CHECK(section != nullptr)
      << ": CreateFileMappingW failed: " << GetLastError();

  void *writable =
      MapViewOfFile(section, FILE_MAP_READ | FILE_MAP_WRITE, 0, 0, 0);
  ABSL_CHECK(writable != nullptr)
      << ": MapViewOfFile(writable) failed: " << GetLastError();
  const void *readonly = MapViewOfFile(section, FILE_MAP_READ, 0, 0, 0);
  ABSL_CHECK(readonly != nullptr)
      << ": MapViewOfFile(readonly) failed: " << GetLastError();
  ABSL_CHECK(CloseHandle(section))
      << ": CloseHandle failed: " << GetLastError();

  return ProcessLocalMapping{writable, readonly, size};
}

void UnmapProcessLocal(const ProcessLocalMapping &mapping) {
  ABSL_CHECK(UnmapViewOfFile(mapping.writable))
      << ": UnmapViewOfFile failed: " << GetLastError();
  ABSL_CHECK(UnmapViewOfFile(mapping.readonly))
      << ": UnmapViewOfFile failed: " << GetLastError();
  NoteMappingUnmapped();
}

}  // namespace aos::ipc_lib::internal
