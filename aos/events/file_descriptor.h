#ifndef AOS_EVENTS_FILE_DESCRIPTOR_H_
#define AOS_EVENTS_FILE_DESCRIPTOR_H_

namespace aos {

// The platform's handle for something an event loop can wait on.  Split out
// of aio.h so that types used *by* Aio -- Pipe, for one -- can name it
// without depending on Aio itself.
#if defined(_WIN32)
using FileDescriptor = void *;
inline constexpr FileDescriptor kInvalidFd = nullptr;
#else
using FileDescriptor = int;
inline constexpr FileDescriptor kInvalidFd = -1;
#endif

}  // namespace aos

#endif  // AOS_EVENTS_FILE_DESCRIPTOR_H_
