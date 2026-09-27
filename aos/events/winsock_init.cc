#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include "aos/events/winsock_init.h"

#include <winsock2.h>

#include "absl/log/absl_check.h"

namespace aos {

void EnsureWinsockInitialized() {
  // Function-local static: thread-safe initialization, run once, and never
  // torn down.  See the header for why there is no matching WSACleanup().
  static const bool initialized = []() {
    WSADATA wsa_data;
    const int ret = WSAStartup(MAKEWORD(2, 2), &wsa_data);
    ABSL_CHECK_EQ(ret, 0) << ": WSAStartup failed";
    return true;
  }();
  (void)initialized;
}

}  // namespace aos
