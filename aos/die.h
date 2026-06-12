#ifndef AOS_DIE_H_
#define AOS_DIE_H_

#include <cstdarg>

#include "aos/libc/aos_strerror.h"
#include "aos/macros.h"

namespace aos {

// Terminates the task/process and logs a message (without using the logging
// framework). Designed for use in code that can't use the logging framework
// (code that can should LOG(FATAL), which calls this).
[[noreturn]] void Die(const char *format, ...) AOS_PRINTF_FORMAT(1, 2);
[[noreturn]] void VDie(const char *format, va_list args)
    AOS_PRINTF_FORMAT(1, 0);

// The same as Die except appends " because of %d (%s)" (formatted with errno
// and aos_strerror(errno)) to the message.
#define PDie(format, ...)                                          \
  do {                                                             \
    const int error = errno;                                       \
    ::aos::Die(format " because of %d (%s)", ##__VA_ARGS__, error, \
               aos_strerror(error));                               \
  } while (false)

// The same as Die except appends " because of %d (%s)" (formatted with error
// and aos_strerror(error)) to the message.
// PCHECK is to PDie as PRCHECK is to PRDie
//
// Example:
// const int ret = pthread_mutex_lock(whatever);
// if (ret != 0) PRDie(ret, "pthread_mutex_lock(%p) failed", whatever);
#define PRDie(error, format, ...)                                  \
  do {                                                             \
    ::aos::Die(format " because of %d (%s)", ##__VA_ARGS__, error, \
               aos_strerror(error));                               \
  } while (false)

// Turns on (or off) "test mode", where (V)Die doesn't write out files and
// doesn't print to stdout.
// Test mode defaults to false.
void SetDieTestMode(bool test_mode);

}  // namespace aos

#endif  // AOS_DIE_H_
