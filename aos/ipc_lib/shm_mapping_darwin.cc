#include "aos/ipc_lib/shm_mapping.h"

#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <thread>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/util/file.h"

namespace aos::ipc_lib {

long SystemPageSize() { return sysconf(_SC_PAGESIZE); }

namespace {

void *MapShm(std::string_view path, size_t size,
             std::filesystem::perms permissions, bool writable) {
  bool use_posix_shm = path.starts_with("/dev/shm/");

  int fd = -1;
  bool created = false;
  std::string path_str(path);

  if (use_posix_shm) {
    // Format a unique name: "/aos_<hash>"
    uint64_t hash_val = std::hash<std::string_view>{}(path);
    char name_buf[32];
    std::snprintf(name_buf, sizeof(name_buf), "/aos_%016llx",
                  static_cast<unsigned long long>(hash_val));
    std::string posix_shm_name(name_buf);

    if (writable) {
      // The same protocol as shm_mapping_linux.cc: create it exclusively, and
      // fall back to opening the existing object if we lose that race.  Only
      // the process that won sizes it; everyone else waits below.
      //
      // The distinction is what keeps the two calls from being one: a POSIX
      // shared memory object's size may only be set once, so a loser that
      // sized it too -- which it would, having found it still empty -- would
      // fail outright rather than harmlessly re-truncating to the same size.
      fd = shm_open(posix_shm_name.c_str(), O_RDWR | O_CREAT | O_EXCL,
                    static_cast<mode_t>(permissions));
      if (fd != -1) {
        created = true;
      } else if (errno == EEXIST) {
        fd = shm_open(posix_shm_name.c_str(), O_RDWR,
                      static_cast<mode_t>(permissions));
      }
    } else {
      // No O_CREAT, matching shm_mapping_linux.cc: a read-only mapping
      // attaches to an object some writer has already created, and says so
      // with ENOENT rather than waiting below for a writer that may never
      // come.
      fd = shm_open(posix_shm_name.c_str(), O_RDONLY,
                    static_cast<mode_t>(permissions));
    }
    ABSL_PCHECK(fd != -1) << ": shm_open " << posix_shm_name << " failed";
  } else {
    util::MkdirP(path_str, permissions);
    if (writable) {
      // As above, and as shm_mapping_linux.cc.
      fd = open(path_str.c_str(), O_RDWR | O_CREAT | O_EXCL,
                static_cast<mode_t>(permissions));
      if (fd != -1) {
        created = true;
      } else if (errno == EEXIST) {
        fd = open(path_str.c_str(), O_RDWR, static_cast<mode_t>(permissions));
      }
    } else {
      fd = open(path_str.c_str(), O_RDONLY, static_cast<mode_t>(permissions));
    }
    ABSL_PCHECK(fd != -1) << ": opening " << path_str << " failed";
  }

  struct stat st;
  ABSL_PCHECK(fstat(fd, &st) == 0);
  if (created) {
    // We created the (empty) object, so we're the one who gives it its real
    // size.
    ABSL_PCHECK(ftruncate(fd, size) == 0);
    ABSL_PCHECK(fstat(fd, &st) == 0);
  } else {
    // We opened something someone else created.  Creating it and giving it
    // its size aren't atomic, so we can land in the window in between, while
    // it's still zero length.  Wait for the creator to finish sizing it.
    // This applies to readers as well as to writers that lost the create
    // race.
    while (st.st_size == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      ABSL_VLOG(1) << path_str << " is zero size, waiting";
      ABSL_PCHECK(fstat(fd, &st) == 0);
    }
  }
  ABSL_CHECK_EQ(static_cast<size_t>(st.st_size), size)
      << ": Size of " << path_str
      << " doesn't match expected size of backing queue file.";

  void *data = mmap(NULL, size, writable ? (PROT_READ | PROT_WRITE) : PROT_READ,
                    MAP_SHARED, fd, 0);
  ABSL_PCHECK(data != MAP_FAILED);
  ABSL_PCHECK(close(fd) == 0);

  const long page_size = SystemPageSize();
  if (writable) {
    PageFaultDataWrite(static_cast<char *>(data), size, page_size);
  } else {
    PageFaultDataRead(static_cast<const char *>(data), size, page_size);
  }

  return data;
}

}  // namespace

WritableShmMapping::WritableShmMapping(std::string_view path, size_t size,
                                       std::filesystem::perms permissions)
    : size_(size) {
  data_ = MapShm(path, size, permissions, /*writable=*/true);
}

WritableShmMapping::~WritableShmMapping() {
  ABSL_PCHECK(munmap(data_, size_) == 0);
}

ReadOnlyShmMapping::ReadOnlyShmMapping(std::string_view path, size_t size,
                                       std::filesystem::perms permissions)
    : size_(size) {
  data_ = MapShm(path, size, permissions, /*writable=*/false);
}

ReadOnlyShmMapping::~ReadOnlyShmMapping() {
  ABSL_PCHECK(munmap(const_cast<void *>(data_), size_) == 0);
}

}  // namespace aos::ipc_lib
