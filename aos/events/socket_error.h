#ifndef AOS_EVENTS_SOCKET_ERROR_H_
#define AOS_EVENTS_SOCKET_ERROR_H_

#include <sys/socket.h>
#include <sys/stat.h>

#include <string>

#include "aos/libc/aos_strerror.h"

namespace aos::internal {

// Whether fd is a socket.
inline bool IsSocket(int fd) {
  struct stat st;
  if (fstat(fd, &st) == -1) {
    return false;
  }
  return static_cast<bool>(S_ISSOCK(st.st_mode));
}

// The socket detail EPoll has always added to an unhandled error event.
// Shared so that CHECK reads the same from EPoll and every Aio backend.
// Empty for anything that is not a socket, or a socket with no pending error.
inline std::string GetSocketErrorStr(int fd) {
  std::string error_str;
  if (IsSocket(fd)) {
    int error = 0;
    socklen_t errlen = sizeof(error);
    if (getsockopt(fd, SOL_SOCKET, SO_ERROR, (void *)&error, &errlen) == 0) {
      if (error) {
        error_str = "Socket error: " + std::string(aos_strerror(error));
      }
    }
  }
  return error_str;
}

}  // namespace aos::internal

#endif  // AOS_EVENTS_SOCKET_ERROR_H_
