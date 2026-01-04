#ifndef AOS_REALTIME_INTERNAL_H_
#define AOS_REALTIME_INTERNAL_H_

namespace aos {

// Internal functions to access the thread-local is_realtime flag.
// This abstraction is needed because on macOS, thread_local initialization
// can recursively call malloc/calloc in ways that interfere with our malloc hooks.
bool GetIsRealtime();
void SetIsRealtime(bool realtime);

// Exposed to platform specific files so they can set it to false on failure.
extern bool has_malloc_hook;

}  // namespace aos

#endif  // AOS_REALTIME_INTERNAL_H_
