#include "aos/libc/aos_strerror.h"

#include <cassert>
#include <cstdio>
#include <cstring>

#ifdef _WIN32

const char *aos_strerror(int error) {
  thread_local char buffer[128];
  if (strerror_s(buffer, sizeof(buffer), error) != 0) {
    snprintf(buffer, sizeof(buffer), "Unknown error %d", error);
  } else if (strcmp(buffer, "Unknown error") == 0) {
    snprintf(buffer, sizeof(buffer), "Unknown error %d", error);
  }
  return buffer;
}

#else

// This code uses an overloaded function to handle the result from either
// version of strerror_r correctly without needing a way to get the choice out
// of the compiler/glibc/whatever explicitly.

namespace {

const size_t kBufferSize = 128;

// Handle the result from the GNU version of strerror_r. It never fails, so
// that's pretty easy...
__attribute__((unused)) char *aos_strerror_handle_result(int /*error*/,
                                                         char *ret,
                                                         char * /*buffer*/) {
  return ret;
}

// Handle the result from the POSIX version of strerror_r.
__attribute__((unused)) char *aos_strerror_handle_result(int error, int ret,
                                                         char *buffer) {
  if (ret != 0) {
#ifndef NDEBUG
    // assert doesn't use the return value when building optimized.
    const int r =
#endif
#ifdef __APPLE__
        snprintf(buffer, kBufferSize, "Unknown error: %d", error);
#else
    snprintf(buffer, kBufferSize, "Unknown error %d", error);
#endif
    assert(r > 0);
  }
  return buffer;
}

}  // namespace

const char *aos_strerror(int error) {
  thread_local char buffer[kBufferSize];

  // Call the overload for whichever version we're using.
  return aos_strerror_handle_result(
      error, strerror_r(error, buffer, sizeof(buffer)), buffer);
}

#endif  // _WIN32
