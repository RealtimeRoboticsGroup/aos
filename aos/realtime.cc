#include "aos/realtime.h"

#include <dirent.h>
#include <malloc.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "absl/base/internal/raw_logging.h"
#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/sanitizers.h"

ABSL_FLAG(
    bool, die_on_malloc, true,
    "If true, die when the application allocates memory in a RT section.");
ABSL_FLAG(bool, skip_realtime_scheduler, false,
          "If true, skip changing the scheduler.  Pretend that we changed "
          "the scheduler instead.");
ABSL_FLAG(bool, skip_locking_memory, false,
          "If true, skip locking memory.  Pretend that we did it instead.");

namespace FLAG__namespace_do_not_use_directly_use_DECLARE_double_instead {
extern double FLAGS_tcmalloc_release_rate __attribute__((weak));
}
using FLAG__namespace_do_not_use_directly_use_DECLARE_double_instead::
    FLAGS_tcmalloc_release_rate;

#include "aos/realtime_internal.h"

namespace aos {

void SetSoftRLimit(int resource, RlimT soft, SetLimitForRoot set_for_root,
                   std::string_view help_string,
                   AllowSoftLimitDecrease allow_decrease) {
  bool am_root = getuid() == 0;
  if (set_for_root == SetLimitForRoot::kYes || !am_root) {
    struct rlimit64 rlim;
    ABSL_PCHECK(getrlimit64(resource, &rlim) == 0)
        << ": getting limit for " << resource;

    if (allow_decrease == AllowSoftLimitDecrease::kYes) {
      rlim.rlim_cur = soft;
    } else {
      rlim.rlim_cur = std::max(rlim.rlim_cur, soft);
    }
    rlim.rlim_max = ::std::max(rlim.rlim_max, soft);

    ABSL_PCHECK(setrlimit64(resource, &rlim) == 0)
        << ": changing limit for " << resource << " to " << rlim.rlim_cur
        << " with max of " << rlim.rlim_max << " (" << help_string << ")";
  }
}

void LockAllMemory() {
  CheckNotRealtime();
  // Allow locking as much as we want into RAM.
  SetSoftRLimit(RLIMIT_MEMLOCK, RLIM_INFINITY, SetLimitForRoot::kNo,
                "use --skip_locking_memory to not lock memory.");

  ABSL_PCHECK(mlockall(MCL_CURRENT | MCL_FUTURE) == 0)
      << ": Failed to lock memory, use --skip_locking_memory to bypass this.  "
         "Bypassing will impact RT performance.";

#if !defined(AOS_SANITIZE_ADDRESS) && !defined(AOS_SANITIZE_MEMORY)
  // Don't give freed memory back to the OS.
  ABSL_CHECK_EQ(1, mallopt(M_TRIM_THRESHOLD, -1));
  // Don't use mmap for large malloc chunks.
  ABSL_CHECK_EQ(1, mallopt(M_MMAP_MAX, 0));
#endif

  // TODO(austin): new tcmalloc does this differently...
  if (&FLAGS_tcmalloc_release_rate) {
    // Tell tcmalloc not to return memory.
    FLAGS_tcmalloc_release_rate = 0.0;
  }

  // Forces the memory pages for all the stack space that we're ever going to
  // use to be loaded into memory (so it can be locked there).
  uint8_t data[4096 * 8];
  // Not 0 because linux might optimize that to a 0-filled page.
  memset(data, 1, sizeof(data));
  __asm__ __volatile__("" ::"m"(data));

  static const size_t kHeapPreallocSize = 512 * 1024;
  char *const heap_data = static_cast<char *>(malloc(kHeapPreallocSize));
  memset(heap_data, 1, kHeapPreallocSize);
  __asm__ __volatile__("" ::"m"(heap_data));
  free(heap_data);
}

void InitRT() {
  if (absl::GetFlag(FLAGS_skip_locking_memory)) {
    ABSL_LOG(WARNING) << "Ignoring request to lock all memory due to "
                         "--skip_locking_memory.";
    return;
  }

  CheckNotRealtime();
  LockAllMemory();

  if (absl::GetFlag(FLAGS_skip_realtime_scheduler)) {
    return;
  }
  // Only let rt processes run for 3 seconds straight.
  SetSoftRLimit(
      RLIMIT_RTTIME, 3000000, SetLimitForRoot::kYes,
      ", use --skip_realtime_scheduler to stay non-rt and bypass this "
      "warning.");

  // Allow rt processes up to priority 40.
  SetSoftRLimit(
      RLIMIT_RTPRIO, 40, SetLimitForRoot::kNo,
      ", use --skip_realtime_scheduler to stay non-rt and bypass this "
      "warning.");
}

void WriteCoreDumps() {
  // Do create core files of unlimited size.
  SetSoftRLimit(RLIMIT_CORE, RLIM_INFINITY, SetLimitForRoot::kYes, "");
}

void ExpandStackSize() {
  SetSoftRLimit(RLIMIT_STACK, 1000000, SetLimitForRoot::kYes, "",
                AllowSoftLimitDecrease::kNo);
}

// Bool to track if malloc hooks have failed to be configured.
// Exposed to platform specific files so they can set it to false on failure.
bool has_malloc_hook = true;

bool MarkRealtime(bool realtime) {
  if (realtime) {
    // For some applications (generally tools built for the host in Bazel), we
    // don't have malloc hooks available, but we also don't go realtime.  Delay
    // complaining in that case until we try to go RT and it matters.
#if !(defined(AOS_SANITIZE_ADDRESS) || defined(AOS_SANITIZE_MEMORY) || \
      defined(AOS_SANITIZE_THREAD))
    ABSL_CHECK(has_malloc_hook)
        << ": Failed to register required malloc hooks before going realtime.  "
           "Disable --die_on_malloc to continue.";
#endif
  }
  const bool prior = GetIsRealtime();
  SetIsRealtime(realtime);
  return prior;
}

bool IsDieOnMallocEnabled() {
  return absl::GetFlag(FLAGS_die_on_malloc) && GetIsRealtime() &&
         has_malloc_hook;
}

void CheckRealtime() { ABSL_CHECK(GetIsRealtime()); }

void CheckNotRealtime() { ABSL_CHECK(!GetIsRealtime()); }

ScopedRealtimeRestorer::ScopedRealtimeRestorer() : prior_(GetIsRealtime()) {}

void NewHook(const void *ptr, size_t size) {
  if (GetIsRealtime()) {
    SetIsRealtime(false);
    ABSL_RAW_LOG(FATAL, "Malloced %p -> %zu bytes", ptr, size);
  }
}

void DeleteHook(const void *ptr) {
  // It is legal to call free(nullptr) unconditionally and assume that it won't
  // do anything.  Eigen does this.  So, if we are RT, ignore any of these
  // calls.
  if (GetIsRealtime() && ptr != nullptr) {
    SetIsRealtime(false);
    ABSL_RAW_LOG(FATAL, "Delete Hook %p", ptr);
  }
}

void FatalUnsetRealtimePriority() {
  int saved_errno = errno;
  // Drop our priority first.  We are about to do lots of work to undo
  // everything, don't get overly clever.
  UnsetCurrentThreadRealtimePriority();

  SetIsRealtime(false);

  // Put all sub-tasks back to non-rt priority too.
  DIR *dirp = opendir("/proc/self/task");
  if (dirp) {
    struct dirent *directory_entry;
    while ((directory_entry = readdir(dirp)) != NULL) {
      int thread_id = std::atoi(directory_entry->d_name);

      // ignore . and .. which are zeroes for some reason
      if (thread_id != 0) {
        struct sched_param param;
        param.sched_priority = 0;
        sched_setscheduler(thread_id, SCHED_OTHER, &param);
      }
    }
    closedir(dirp);
  }
  errno = saved_errno;
}

}  // namespace aos
