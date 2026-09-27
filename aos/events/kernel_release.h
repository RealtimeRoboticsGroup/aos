#ifndef AOS_EVENTS_KERNEL_RELEASE_H_
#define AOS_EVENTS_KERNEL_RELEASE_H_

#include <string_view>

namespace aos::internal {

// Whether a kernel, named by its uname(2) release string ("5.10.120-tegra"),
// has the RWF_NOWAIT bug readv(2) documents under BUGS: "Linux 5.9 and Linux
// 5.10 have a bug where preadv2() with the RWF_NOWAIT flag may return 0 even
// when not at end of file."  A spurious 0 reads as end of file, so on those
// kernels the readiness backends do not use RWF_NOWAIT at all.  A release
// that does not parse is not one of them.
inline bool KernelHasSpuriousNowaitZero(std::string_view release) {
  const auto parse = [&release](int *value) {
    if (release.empty() || release.front() < '0' || release.front() > '9') {
      return false;
    }
    *value = 0;
    while (!release.empty() && release.front() >= '0' &&
           release.front() <= '9') {
      *value = *value * 10 + (release.front() - '0');
      release.remove_prefix(1);
    }
    return true;
  };
  int major = 0;
  int minor = 0;
  if (!parse(&major) || release.empty() || release.front() != '.') {
    return false;
  }
  release.remove_prefix(1);
  if (!parse(&minor)) {
    return false;
  }
  return major == 5 && (minor == 9 || minor == 10);
}

}  // namespace aos::internal

#endif  // AOS_EVENTS_KERNEL_RELEASE_H_
