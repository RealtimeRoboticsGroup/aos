#include "aos/scoped/scoped_fd.h"

#ifndef _WIN32
#include <unistd.h>
#else
#include <io.h>
#endif

#include <ostream>

#include "absl/log/absl_log.h"

namespace aos {

void ScopedFD::Close() {
  if (fd_ != -1) {
#ifndef _WIN32
    if (close(fd_) == -1) {
#else
    if (_close(fd_) == -1) {
#endif
      ABSL_PLOG(WARNING) << "close(" << fd_ << ") failed";
    }
  }
}

}  // namespace aos
