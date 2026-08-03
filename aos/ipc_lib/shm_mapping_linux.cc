#include "aos/ipc_lib/shm_mapping.h"

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

#include "aos/util/file.h"

namespace aos::ipc_lib {

long SystemPageSize() { return sysconf(_SC_PAGESIZE); }

namespace {

void *MapShm(std::string_view path, size_t size,
             std::filesystem::perms permissions, bool writable) {
  std::string path_str(path);
  util::MkdirP(path_str, permissions);

  int fd = -1;
  bool created = false;
  if (writable) {
    // Two cases: either the file doesn't exist yet and we create it, or it
    // already exists.  Try to create it exclusively first; if that races with
    // another creator (EEXIST) fall back to opening the existing file.  Once
    // created, the file is never deleted.  Only the process whose exclusive
    // create succeeded is responsible for sizing the file; everyone else waits
    // for it (see below).
    fd = open(path_str.c_str(), O_RDWR | O_CREAT | O_EXCL | O_CLOEXEC,
              static_cast<mode_t>(permissions));
    if (fd != -1) {
      created = true;
    } else if (errno == EEXIST) {
      fd = open(path_str.c_str(), O_RDWR | O_CLOEXEC,
                static_cast<mode_t>(permissions));
    }
  } else {
    fd = open(path_str.c_str(), O_RDONLY | O_CLOEXEC,
              static_cast<mode_t>(permissions));
  }
  ABSL_PCHECK(fd != -1) << ": opening " << path_str << " failed";

  struct stat st;
  ABSL_PCHECK(fstat(fd, &st) == 0);
  if (created) {
    // We created the (empty) file, so we're the one who gives it its real size.
    ABSL_PCHECK(ftruncate(fd, size) == 0);
    ABSL_PCHECK(fstat(fd, &st) == 0);
  } else {
    // We opened a file someone else created.  Creating the file (open) and
    // giving it its size (ftruncate) aren't atomic, so we can open it in the
    // window in between, while it's still zero length.  Wait for the creating
    // process to finish sizing it before we check the size and map it.  This
    // applies to readers as well as to writers that lost the create race.
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

  // Pre-fault the pages so a later (possibly realtime) access never takes a
  // fault.  A write-fault installs a pagetable entry that is both writable and
  // readable, so faulting the writable mapping for writing also covers reads
  // through it -- there's no need to additionally read-fault it.  The read-only
  // mapping is PROT_READ (writing to it would fault), so it can only be
  // read-faulted.
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
