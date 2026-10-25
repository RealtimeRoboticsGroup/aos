#ifndef AOS_EVENTS_FILE_DESCRIPTOR_H_
#define AOS_EVENTS_FILE_DESCRIPTOR_H_

#include <cstdint>

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

// What an event loop can wait on a FileDescriptor for, in the encoding
// Aio::OnEvents() reports and Aio::SetEvents() takes on every platform.  The
// values are the low four epoll bits.
inline constexpr uint32_t kReadableEvent = 0x01;
inline constexpr uint32_t kPriorityEvent = 0x02;
inline constexpr uint32_t kWritableEvent = 0x04;
inline constexpr uint32_t kErrorEvent = 0x08;

// Everything a read handler is woken for.
inline constexpr uint32_t kReadableEvents = kReadableEvent | kPriorityEvent;

}  // namespace aos

#endif  // AOS_EVENTS_FILE_DESCRIPTOR_H_
