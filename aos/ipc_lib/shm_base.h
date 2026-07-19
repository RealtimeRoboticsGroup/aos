#ifndef AOS_IPC_LIB_SHM_BASE_H_
#define AOS_IPC_LIB_SHM_BASE_H_

#include <string>
#include <string_view>

#include "absl/flags/declare.h"

ABSL_DECLARE_FLAG(std::string, shm_base);

namespace aos {
// Returns --shm_base.  On Windows it dies unless that is absolute, since a
// path that is not means a different directory to a process started from
// another drive or directory.
std::string ShmBase();
}  // namespace aos

namespace aos::testing {
void SetShmBase(const std::string_view base);
}

#endif  // AOS_IPC_LIB_SHM_BASE_H_
