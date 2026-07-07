#include "aos/libc/aos_strsignal.h"

#ifdef __linux__
#include <features.h>
#endif
#include <stdio.h>
#include <string.h>

#include <csignal>

#include "absl/log/check.h"
#include "absl/log/log.h"

const char *aos_strsignal(int signal) {
  thread_local char buffer[512];

#ifdef _WIN32
  switch (signal) {
    case SIGINT:
      return "Interrupt";
    case SIGILL:
      return "Illegal instruction";
    case SIGFPE:
      return "Floating-point exception";
    case SIGSEGV:
      return "Segmentation violation";
    case SIGTERM:
      return "Software termination signal from kill";
    case SIGABRT:
      return "Aborted";
    default:
      CHECK_GT(snprintf(buffer, sizeof(buffer), "Unknown signal %d", signal),
               0);
      return buffer;
  }
#else

#if defined(SIGRTMIN) && defined(SIGRTMAX)
  if (signal >= SIGRTMIN && signal <= SIGRTMAX) {
    CHECK_GT(snprintf(buffer, sizeof(buffer), "Real-time signal %d",
                      signal - SIGRTMIN),
             0);
    return buffer;
  }
#endif

// sys_strsignal depricated in glibc2.32
#ifdef __GLIBC__
#if __GLIBC_PREREQ(2, 32)
  if (signal > 0 && signal < NSIG && sigdescr_np(signal) != nullptr) {
    return sigdescr_np(signal);
  }
#else
  if (signal > 0 && signal < NSIG && sys_siglist[signal] != nullptr) {
    return sys_siglist[signal];
  }
#endif
// If not using GLIBC assume we can use sys_siglist
#else
  if (signal > 0 && signal < NSIG && sys_siglist[signal] != nullptr) {
    return sys_siglist[signal];
  }
#endif

  CHECK_GT(snprintf(buffer, sizeof(buffer), "Unknown signal %d", signal), 0);
  return buffer;
#endif  // _WIN32
}
