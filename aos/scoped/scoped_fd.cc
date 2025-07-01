#include "aos/scoped/scoped_fd.h"

#include <unistd.h>

#include <ostream>

#include "absl/log/absl_log.h"

namespace aos {

void ScopedFD::Close() {
  if (fd_ != -1) {
    if (close(fd_) == -1) {
      ABSL_PLOG(WARNING) << "close(" << fd_ << ") failed";
    }
  }
}

}  // namespace aos
