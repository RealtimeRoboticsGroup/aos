#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <thread>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/ipc_lib/shm_mapping.h"
#include "aos/util/file.h"

namespace aos::ipc_lib {

namespace {

void *MapShm(std::string_view path, size_t size,
             std::filesystem::perms permissions, bool writable) {
  std::string path_str(path);
  util::MkdirP(path_str, permissions);

  int fd = -1;
  if (writable) {
    fd = open(path_str.c_str(), O_RDWR | O_CREAT | O_EXCL,
              static_cast<mode_t>(permissions));
    if (fd == -1 && errno == EEXIST) {
      fd = open(path_str.c_str(), O_RDWR, static_cast<mode_t>(permissions));
    }
  } else {
    fd = open(path_str.c_str(), O_RDONLY, static_cast<mode_t>(permissions));
  }
  ABSL_PCHECK(fd != -1) << ": opening " << path_str << " failed";

  struct stat st;
  ABSL_PCHECK(fstat(fd, &st) == 0);
  if (writable && st.st_size == 0) {
    ABSL_PCHECK(ftruncate(fd, size) == 0);
    ABSL_PCHECK(fstat(fd, &st) == 0);
  } else {
    while (st.st_size == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
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

  const long page_size = sysconf(_SC_PAGESIZE);
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
