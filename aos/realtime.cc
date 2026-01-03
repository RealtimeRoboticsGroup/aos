#include "aos/realtime.h"

#include <dirent.h>
#ifdef __linux__
#include <malloc.h>
#include <sys/prctl.h>
#endif
#include <sched.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/types.h>
#include <unistd.h>

#if defined(__APPLE__)
#include <mach/mach.h>
#include <mach/thread_policy.h>
#include <malloc/malloc.h>
#include <pthread.h>
#endif

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
#include "aos/uuid.h"

ABSL_FLAG(
    bool, die_on_malloc, true,
    "If true, die when the application allocates memory in a RT section.");
ABSL_FLAG(bool, skip_realtime_scheduler, false,
          "If true, skip changing the scheduler.  Pretend that we changed "
          "the scheduler instead.");
ABSL_FLAG(bool, skip_locking_memory, false,
          "If true, skip locking memory.  Pretend that we did it instead.");

extern "C" {
typedef void (*MallocHook_NewHook)(const void *ptr, size_t size);
int MallocHook_AddNewHook(MallocHook_NewHook hook) __attribute__((weak));
int MallocHook_RemoveNewHook(MallocHook_NewHook hook) __attribute__((weak));

typedef void (*MallocHook_DeleteHook)(const void *ptr);
int MallocHook_AddDeleteHook(MallocHook_DeleteHook hook) __attribute__((weak));
int MallocHook_RemoveDeleteHook(MallocHook_DeleteHook hook)
    __attribute__((weak));

// Declare tc_malloc weak so we can check if it exists.
void *tc_malloc(size_t size) __attribute__((weak));

void *__libc_malloc(size_t size);
void __libc_free(void *ptr);
void *__libc_realloc(void *ptr, size_t size);
void *__libc_calloc(size_t n, size_t elem_size);
}  // extern "C"

namespace FLAG__namespace_do_not_use_directly_use_DECLARE_double_instead {
extern double FLAGS_tcmalloc_release_rate __attribute__((weak));
}
using FLAG__namespace_do_not_use_directly_use_DECLARE_double_instead::
    FLAGS_tcmalloc_release_rate;

namespace aos {

CpuSet::CpuSet() {
#ifdef __linux__
  CPU_ZERO(&set_);
#elif defined(__APPLE__)
#else
#error "Only linux and apple (Mac OS X) are supported"
#endif
}

void CpuSet::Set(int cpu) {
#ifdef __linux__
  CPU_SET(cpu, &set_);
#elif defined(__APPLE__)
  if (cpu >= 0 && cpu < static_cast<int>(set_.size())) {
    set_.set(cpu);
  }
#else
#error "Only linux and apple (Mac OS X) are supported"
#endif
}

void CpuSet::Clear(int cpu) {
#ifdef __linux__
  CPU_CLR(cpu, &set_);
#elif defined(__APPLE__)
  if (cpu >= 0 && cpu < static_cast<int>(set_.size())) {
    set_.reset(cpu);
  }
#else
#error "Only linux and apple (Mac OS X) are supported"
#endif
}

void CpuSet::Clear() {
#ifdef __linux__
  CPU_ZERO(&set_);
#elif defined(__APPLE__)
  set_.reset();
#else
#error "Only linux and apple (Mac OS X) are supported"
#endif
}

bool CpuSet::IsSet(int cpu) const {
#ifdef __linux__
  return CPU_ISSET(cpu, &set_);
#elif defined(__APPLE__)
  if (cpu >= 0 && cpu < static_cast<int>(set_.size())) {
    return set_.test(cpu);
  }
  return false;
#else
#error "Only linux and apple (Mac OS X) are supported"
#endif
}

bool CpuSet::Empty() const {
#ifdef __linux__
  return CPU_COUNT(&set_) == 0;
#elif defined(__APPLE__)
  return set_.none();
#else
#error "Only linux and apple (Mac OS X) are supported"
#endif
}

bool CpuSet::operator==(const CpuSet &other) const {
#ifdef __linux__
  return CPU_EQUAL(&set_, &other.set_);
#elif defined(__APPLE__)
  return set_ == other.set_;
#else
#error "Only linux and apple (Mac OS X) are supported"
#endif
}

bool CpuSet::operator!=(const CpuSet &other) const {
#ifdef __linux__
  return !CPU_EQUAL(&set_, &other.set_);
#elif defined(__APPLE__)
  return set_ != other.set_;
#else
#error "Only linux and apple (Mac OS X) are supported"
#endif
}

namespace logging::internal {

// Implemented in aos/logging/context.cc.
void ReloadThreadName() __attribute__((weak));

}  // namespace logging::internal

namespace {

enum class SetLimitForRoot { kYes, kNo };

enum class AllowSoftLimitDecrease { kYes, kNo };

#ifdef __linux__
using RlimT = rlim64_t;
#else
using RlimT = rlim_t;
#endif

void SetSoftRLimit(
    int resource, RlimT soft, SetLimitForRoot set_for_root,
    std::string_view help_string,
    AllowSoftLimitDecrease allow_decrease = AllowSoftLimitDecrease::kYes) {
  bool am_root = getuid() == 0;
  if (set_for_root == SetLimitForRoot::kYes || !am_root) {
#ifdef __linux__
    struct rlimit64 rlim;
    ABSL_PCHECK(getrlimit64(resource, &rlim) == 0)
        << ": getting limit for " << resource;
#else
    struct rlimit rlim;
    ABSL_PCHECK(getrlimit(resource, &rlim) == 0)
        << ": getting limit for " << resource;
#endif

    if (allow_decrease == AllowSoftLimitDecrease::kYes) {
      rlim.rlim_cur = soft;
    } else {
      rlim.rlim_cur = std::max(rlim.rlim_cur, soft);
    }
    rlim.rlim_max = ::std::max(rlim.rlim_max, soft);

#ifdef __linux__
    ABSL_PCHECK(setrlimit64(resource, &rlim) == 0)
        << ": changing limit for " << resource << " to " << rlim.rlim_cur
        << " with max of " << rlim.rlim_max << " (" << help_string << ")";
#else
    ABSL_PCHECK(setrlimit(resource, &rlim) == 0)
        << ": changing limit for " << resource << " to " << rlim.rlim_cur
        << " with max of " << rlim.rlim_max << " (" << help_string << ")";
#endif
  }
}

}  // namespace

void LockAllMemory() {
  CheckNotRealtime();
  // Allow locking as much as we want into RAM.
  SetSoftRLimit(RLIMIT_MEMLOCK, RLIM_INFINITY, SetLimitForRoot::kNo,
                "use --skip_locking_memory to not lock memory.");

#if defined(__linux__) || defined(__APPLE__)
  ABSL_PCHECK(mlockall(MCL_CURRENT | MCL_FUTURE) == 0)
      << ": Failed to lock memory, use --skip_locking_memory to bypass this.  "
         "Bypassing will impact RT performance.";
#else
#error "Only linux and apple (Mac OS X) are supported"
#endif

#if !defined(AOS_SANITIZE_ADDRESS) && !defined(AOS_SANITIZE_MEMORY)
#ifdef __linux__
  // Don't give freed memory back to the OS.
  ABSL_CHECK_EQ(1, mallopt(M_TRIM_THRESHOLD, -1));
  // Don't use mmap for large malloc chunks.
  ABSL_CHECK_EQ(1, mallopt(M_MMAP_MAX, 0));
#endif
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
#ifdef __linux__
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
#endif
}

void UnsetCurrentThreadRealtimePriority() {
  struct sched_param param;
  param.sched_priority = 0;
#ifdef __linux__
  ABSL_PCHECK(sched_setscheduler(0, SCHED_OTHER, &param) == 0);
#elif defined(__APPLE__)
  ABSL_LOG(WARNING) << "No RT scheduler on OSX, ignoring";
#else
#error "Only linux and apple (Mac OS X) are supported"
#endif
  MarkRealtime(false);
}

namespace {
#if defined(__APPLE__)
pthread_key_t kCpuSetKey;
pthread_once_t kCpuSetKeyOnce = PTHREAD_ONCE_INIT;

void FreeCpuSet(void *p) { delete static_cast<CpuSet *>(p); }

void CreateCpuSetKey() {
  ABSL_PCHECK(pthread_key_create(&kCpuSetKey, FreeCpuSet) == 0);
}
#endif
}  // namespace

void SetCurrentThreadAffinity(const CpuSet &cpuset) {
#ifdef __linux__
  ABSL_PCHECK(sched_setaffinity(0, sizeof(cpu_set_t), cpuset.native_handle()) ==
              0)
      << cpuset;
#elif defined(__APPLE__)
  pthread_once(&kCpuSetKeyOnce, CreateCpuSetKey);
  CpuSet *current_affinity =
      static_cast<CpuSet *>(pthread_getspecific(kCpuSetKey));
  if (current_affinity == nullptr) {
    current_affinity = new CpuSet();
    ABSL_PCHECK(pthread_setspecific(kCpuSetKey, current_affinity) == 0);
  }
  *current_affinity = cpuset;

  if (cpuset.Empty() || cpuset == DefaultAffinity()) {
    thread_affinity_policy_data_t policy = {THREAD_AFFINITY_TAG_NULL};
    ABSL_CHECK_EQ(thread_policy_set(pthread_mach_thread_np(pthread_self()),
                                    THREAD_AFFINITY_POLICY,
                                    (thread_policy_t)&policy,
                                    THREAD_AFFINITY_POLICY_COUNT),
                  KERN_SUCCESS);
  } else {
    integer_t tag = 0;
    // We want to map the cpuset to an affinity tag.  The kernel doesn't give us
    // enough control to do this perfectly, but we can do a decent job by hashing
    // the cpuset.
    for (size_t i = 0; i < CpuSet::kSize; ++i) {
      if (cpuset.IsSet(i)) {
        tag = (tag << 1) ^ i;
      }
    }

    thread_affinity_policy_data_t policy = {tag};

    ABSL_CHECK_EQ(thread_policy_set(pthread_mach_thread_np(pthread_self()),
                                    THREAD_AFFINITY_POLICY,
                                    (thread_policy_t)&policy,
                                    THREAD_AFFINITY_POLICY_COUNT),
                  KERN_SUCCESS);
  }
#else
#error "Only linux and apple (Mac OS X) are supported"
#endif
}

void SetCurrentThreadName(const std::string_view name) {
  ABSL_CHECK_LE(name.size(), 16u) << ": thread name '" << name << "' too long";
  ABSL_VLOG(1) << "This thread is changing to '" << name << "'";
  std::string string_name(name);
#ifdef __linux__
  ABSL_PCHECK(prctl(PR_SET_NAME, string_name.c_str()) == 0)
      << ": changing name to " << string_name;
#elif defined(__APPLE__)
  pthread_setname_np(string_name.c_str());
#endif
  if (&logging::internal::ReloadThreadName != nullptr) {
    logging::internal::ReloadThreadName();
  }
}

CpuSet GetCurrentThreadAffinity() {
  CpuSet result;
#ifdef __linux__
  ABSL_PCHECK(sched_getaffinity(0, sizeof(cpu_set_t), result.native_handle()) ==
              0);
#elif defined(__APPLE__)
  // There is no way to query the affinity back from the kernel, so just return
  // what we were last set to.
  pthread_once(&kCpuSetKeyOnce, CreateCpuSetKey);
  CpuSet *current_affinity =
      static_cast<CpuSet *>(pthread_getspecific(kCpuSetKey));
  if (current_affinity == nullptr) {
    result = DefaultAffinity();
  } else {
    result = *current_affinity;
  }
#else
#error "Only linux and apple (Mac OS X) are supported"
#endif
  return result;
}

void SetCurrentThreadRealtimePriority(int priority, int scheduling_policy) {
  // Ensure that we won't get expensive reads of /dev/random when the realtime
  // scheduler is running.
  UUID::Random();

  if (absl::GetFlag(FLAGS_skip_realtime_scheduler)) {
    ABSL_LOG(WARNING)
        << "Ignoring request to switch to the RT scheduler due to "
           "--skip_realtime_scheduler.";
    return;
  }
#ifdef __linux__
  // Make sure we will only be allowed to run for 3 seconds straight.
  SetSoftRLimit(
      RLIMIT_RTTIME, 3000000, SetLimitForRoot::kYes,
      ", use --skip_realtime_scheduler to stay non-rt and bypass this "
      "warning.");

  // Raise our soft rlimit if necessary.
  SetSoftRLimit(
      RLIMIT_RTPRIO, priority, SetLimitForRoot::kNo,
      ", use --skip_realtime_scheduler to stay non-rt and bypass this "
      "warning.",
      AllowSoftLimitDecrease::kNo);
#endif

  ABSL_CHECK(scheduling_policy == SCHED_FIFO || scheduling_policy == SCHED_RR)
      << "Specified non-realtime scheduling policy with realtime priority";
  ABSL_CHECK(priority > 0 && priority < 100)
      << "Realtime priority must fall within [1,99]";
  struct sched_param param;
  param.sched_priority = priority;
#if defined(__APPLE__)
  ABSL_LOG(INFO) << "RT priority not implemented on OSX, pretending to be RT";
#endif
  MarkRealtime(true);
#ifdef __linux__
  ABSL_PCHECK(sched_setscheduler(0, scheduling_policy, &param) == 0)
      << ": changing to realtime scheduler ("
      << (scheduling_policy == SCHED_FIFO ? "SCHED_FIFO" : "SCHED_RR")
      << ") with priority " << priority
      << ", if you want to bypass this check for testing, use "
         "--skip_realtime_scheduler";
#elif defined(__APPLE__)
#else
#error "Only linux and apple (Mac OS X) are supported"
#endif
}

int GetCurrentThreadRealtimePriority() {
#ifdef __linux__
  struct sched_param result;
  ABSL_PCHECK(sched_getparam(0, &result) == 0)
      << ": Failed to retrieve the Realtime Priority";
  return result.sched_priority;
#elif defined(__APPLE__)
  struct sched_param param;
  int policy;
  ABSL_PCHECK(pthread_getschedparam(pthread_self(), &policy, &param) == 0);
  return param.sched_priority;
#else
#error "Only linux and apple (Mac OS X) are supported"
#endif
}

int GetCurrentThreadSchedulingPolicy() {
#ifdef __linux__
  int scheduling_policy = sched_getscheduler(0);
  ABSL_PCHECK(scheduling_policy >= 0)
      << ": Failed to retrieve the scheduling policy";
  return scheduling_policy;
#elif defined(__APPLE__)
  struct sched_param param;
  int policy;
  ABSL_PCHECK(pthread_getschedparam(pthread_self(), &policy, &param) == 0);
  return policy;
#else
#error "Only linux and apple (Mac OS X) are supported"
#endif
}

void WriteCoreDumps() {
  // Do create core files of unlimited size.
  SetSoftRLimit(RLIMIT_CORE, RLIM_INFINITY, SetLimitForRoot::kYes, "");
}

void ExpandStackSize() {
  SetSoftRLimit(RLIMIT_STACK, 1000000, SetLimitForRoot::kYes, "",
                AllowSoftLimitDecrease::kNo);
}

namespace {
// Bool to track if malloc hooks have failed to be configured.
bool has_malloc_hook = true;
thread_local bool is_realtime = false;
}  // namespace

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
  const bool prior = is_realtime;
  is_realtime = realtime;
  return prior;
}

bool IsDieOnMallocEnabled() {
  return absl::GetFlag(FLAGS_die_on_malloc) && aos::is_realtime &&
         has_malloc_hook;
}

void CheckRealtime() { ABSL_CHECK(is_realtime); }

void CheckNotRealtime() { ABSL_CHECK(!is_realtime); }

ScopedRealtimeRestorer::ScopedRealtimeRestorer() : prior_(is_realtime) {}

void NewHook(const void *ptr, size_t size) {
  if (is_realtime) {
    is_realtime = false;
    ABSL_RAW_LOG(FATAL, "Malloced %p -> %zu bytes", ptr, size);
  }
}

void DeleteHook(const void *ptr) {
  // It is legal to call free(nullptr) unconditionally and assume that it won't
  // do anything.  Eigen does this.  So, if we are RT, ignore any of these
  // calls.
  if (is_realtime && ptr != nullptr) {
    is_realtime = false;
    ABSL_RAW_LOG(FATAL, "Delete Hook %p", ptr);
  }
}

extern "C" {

// malloc hooks for libc. Tcmalloc will replace everything it finds (malloc,
// __libc_malloc, etc.), so we need its specific hook above as well.
void *aos_malloc_hook(size_t size) {
  if (absl::GetFlag(FLAGS_die_on_malloc) && aos::is_realtime) {
    aos::is_realtime = false;
    ABSL_RAW_LOG(FATAL,
                 "Malloced %zu bytes: This error usually happens when a user "
                 "does something that is not realtime. Either change the "
                 "implementation to be realtime compatible or disable the "
                 "die_on_malloc flag to run without this constraint.",
                 size);
    return nullptr;
  } else {
    return __libc_malloc(size);
  }
}

void aos_free_hook(void *ptr) {
  if (absl::GetFlag(FLAGS_die_on_malloc) && aos::is_realtime &&
      ptr != nullptr) {
    aos::is_realtime = false;
    ABSL_RAW_LOG(FATAL, "Deleted %p", ptr);
  } else {
    __libc_free(ptr);
  }
}

void *aos_realloc_hook(void *ptr, size_t size) {
  if (absl::GetFlag(FLAGS_die_on_malloc) && aos::is_realtime) {
    aos::is_realtime = false;
    ABSL_RAW_LOG(FATAL, "Malloced %p -> %zu bytes", ptr, size);
    return nullptr;
  } else {
    return __libc_realloc(ptr, size);
  }
}

void *aos_calloc_hook(size_t n, size_t elem_size) {
  if (absl::GetFlag(FLAGS_die_on_malloc) && aos::is_realtime) {
    aos::is_realtime = false;
    ABSL_RAW_LOG(FATAL, "Malloced %zu * %zu bytes", n, elem_size);
    return nullptr;
  } else {
    return __libc_calloc(n, elem_size);
  }
}

#ifdef __linux__
void *malloc(size_t size) __attribute__((weak, alias("aos_malloc_hook")));
void free(void *ptr) __attribute__((weak, alias("aos_free_hook")));
void *realloc(void *ptr, size_t size)
    __attribute__((weak, alias("aos_realloc_hook")));
void *calloc(size_t n, size_t elem_size)
    __attribute__((weak, alias("aos_calloc_hook")));
#endif
}

void FatalUnsetRealtimePriority() {
  int saved_errno = errno;
  // Drop our priority first.  We are about to do lots of work to undo
  // everything, don't get overly clever.
  struct sched_param param;
  param.sched_priority = 0;
#ifdef __linux__
  sched_setscheduler(0, SCHED_OTHER, &param);
#elif defined(__APPLE__)
#else
#error "Only linux and apple (Mac OS X) are supported"
#endif

  is_realtime = false;

  // Put all sub-tasks back to non-rt priority too.
#ifdef __linux__
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
#elif defined(__APPLE__)
#else
#error "Only linux and apple (Mac OS X) are supported"
#endif
  errno = saved_errno;
}

#ifdef __linux__
void RegisterMallocHook() {
  if (absl::GetFlag(FLAGS_die_on_malloc)) {
    // tcmalloc redefines __libc_malloc, so use this as a feature test.
    if (&__libc_malloc == &tc_malloc) {
      if (ABSL_VLOG_IS_ON(1)) {
        ABSL_RAW_LOG(INFO, "Hooking tcmalloc for die_on_malloc");
      }
      if (&MallocHook_AddNewHook != nullptr) {
        ABSL_CHECK(MallocHook_AddNewHook(&NewHook));
      } else {
        has_malloc_hook = false;
      }
      if (&MallocHook_AddDeleteHook != nullptr) {
        ABSL_CHECK(MallocHook_AddDeleteHook(&DeleteHook));
      } else {
        has_malloc_hook = false;
      }
    } else {
      if (ABSL_VLOG_IS_ON(1)) {
        ABSL_RAW_LOG(INFO, "Replacing glibc malloc");
      }
      if (&malloc != &aos_malloc_hook) {
        has_malloc_hook = false;
      }
      if (&free != &aos_free_hook) {
        has_malloc_hook = false;
      }
    }
  }
}
#elif defined(__APPLE__)

typedef struct {
  void *(*malloc)(struct _malloc_zone_t *zone, size_t size);
  void *(*calloc)(struct _malloc_zone_t *zone, size_t num_items, size_t size);
  void *(*valloc)(struct _malloc_zone_t *zone, size_t size);
  void (*free)(struct _malloc_zone_t *zone, void *ptr);
  void *(*realloc)(struct _malloc_zone_t *zone, void *ptr, size_t size);
  void (*destroy)(struct _malloc_zone_t *zone);
  unsigned (*batch_malloc)(struct _malloc_zone_t *zone, size_t size,
                           void **results, unsigned num_requested);
  void (*batch_free)(struct _malloc_zone_t *zone, void **to_be_freed,
                     unsigned num_to_be_freed);
  struct _malloc_introspection_t *introspect;
  unsigned version;
  void *(*memalign)(struct _malloc_zone_t *zone, size_t alignment, size_t size);
  void (*free_definite_size)(struct _malloc_zone_t *zone, void *ptr,
                             size_t size);
  size_t (*pressure_relief)(struct _malloc_zone_t *zone, size_t goal);
  boolean_t (*claimed_address)(struct _malloc_zone_t *zone, void *ptr);
} OriginalFunctions;

static OriginalFunctions original_functions;

static void *rt_malloc(struct _malloc_zone_t *zone, size_t size) {
  if (aos::is_realtime && absl::GetFlag(FLAGS_die_on_malloc)) {
    aos::is_realtime = false;
    ABSL_RAW_LOG(FATAL, "Malloced %zu bytes", size);
  }

  return original_functions.malloc(zone, size);
}

static void *rt_calloc(struct _malloc_zone_t *zone, size_t num_items,
                       size_t size) {
  if (aos::is_realtime && absl::GetFlag(FLAGS_die_on_malloc)) {
    aos::is_realtime = false;
    ABSL_RAW_LOG(FATAL, "Malloced %zu * %zu bytes", num_items, size);
  }

  return original_functions.calloc(zone, num_items, size);
}

static void *rt_valloc(struct _malloc_zone_t *zone, size_t size) {
  if (aos::is_realtime && absl::GetFlag(FLAGS_die_on_malloc)) {
    aos::is_realtime = false;
    ABSL_RAW_LOG(FATAL, "Malloced %zu bytes", size);
  }

  return original_functions.valloc(zone, size);
}

static void rt_free(struct _malloc_zone_t *zone, void *ptr) {
  if (aos::is_realtime && absl::GetFlag(FLAGS_die_on_malloc) &&
      ptr != nullptr) {
    aos::is_realtime = false;
    ABSL_RAW_LOG(FATAL, "Deleted %p", ptr);
  }

  original_functions.free(zone, ptr);
}

static void *rt_realloc(struct _malloc_zone_t *zone, void *ptr, size_t size) {
  if (aos::is_realtime && absl::GetFlag(FLAGS_die_on_malloc)) {
    aos::is_realtime = false;
    ABSL_RAW_LOG(FATAL, "Malloced %p -> %zu bytes", ptr, size);
  }

  return original_functions.realloc(zone, ptr, size);
}

static void rt_free_definite_size(struct _malloc_zone_t *zone, void *ptr,
                                  size_t size) {
  if (aos::is_realtime && absl::GetFlag(FLAGS_die_on_malloc) &&
      ptr != nullptr) {
    aos::is_realtime = false;
    ABSL_RAW_LOG(FATAL, "Deleted %p", ptr);
  }

  original_functions.free_definite_size(zone, ptr, size);
}



static unsigned rt_batch_malloc(struct _malloc_zone_t *zone, size_t size,
                                void **results, unsigned num_requested) {
  // Don't support batch malloc in RT mode for now, or just check the flag once.
  // Batch malloc is rare.
  if (aos::is_realtime && absl::GetFlag(FLAGS_die_on_malloc)) {
    aos::is_realtime = false;
    ABSL_RAW_LOG(FATAL, "Batch Malloced %u * %zu bytes", num_requested, size);
  }
  return original_functions.batch_malloc(zone, size, results, num_requested);
}

static void rt_batch_free(struct _malloc_zone_t *zone, void **to_be_freed,
                          unsigned num_to_be_freed) {
  if (aos::is_realtime && absl::GetFlag(FLAGS_die_on_malloc) &&
      num_to_be_freed > 0) {
    aos::is_realtime = false;
    ABSL_RAW_LOG(FATAL, "Batch Deleted %u items", num_to_be_freed);
  }
  original_functions.batch_free(zone, to_be_freed, num_to_be_freed);
}

static void *rt_memalign(struct _malloc_zone_t *zone, size_t alignment,
                         size_t size) {
  if (aos::is_realtime && absl::GetFlag(FLAGS_die_on_malloc)) {
    aos::is_realtime = false;
    ABSL_RAW_LOG(FATAL, "Memaligned %zu bytes", size);
  }
  // memalign is not in OriginalFunctions struct I added previously?
  // Checking Chunk 0 from previous turn...
  // `void *(*memalign)(struct _malloc_zone_t *zone, size_t alignment, size_t size);`
  // Yes it is.
  return original_functions.memalign(zone, alignment, size);
}

// Drops rt_pressure_relief, rt_destroy, rt_size since we don't need to intercept them.

void RegisterMallocHook() {
  if (absl::GetFlag(FLAGS_die_on_malloc)) {
    if (ABSL_VLOG_IS_ON(1)) {
      ABSL_RAW_LOG(INFO, "Hooking malloc zone for die_on_malloc");
    }

    // We want to make sure we don't accidentally recurse and explode if this
    // function triggers a malloc, so stash is_realtime and clear it.
    bool old_is_realtime = MarkRealtime(false);

    malloc_zone_t *default_zone = malloc_default_zone();

    // Store original functions.
    // Copy the relevant pointers. If any are null, our rt_* wrappers might crash if called unconditionally,
    // but the system malloc usually checks before calling specific optional ones generally?
    // Actually our wrappers call original_functions.foo unconditionally.
    // Safe to copy nulls.
    original_functions.malloc = default_zone->malloc;
    original_functions.calloc = default_zone->calloc;
    original_functions.valloc = default_zone->valloc;
    original_functions.free = default_zone->free;
    original_functions.realloc = default_zone->realloc;
    original_functions.destroy = default_zone->destroy;
    original_functions.batch_malloc = default_zone->batch_malloc;
    original_functions.batch_free = default_zone->batch_free;
    original_functions.memalign = default_zone->memalign;
    original_functions.free_definite_size = default_zone->free_definite_size;
    original_functions.pressure_relief = default_zone->pressure_relief;
    
    // Unprotect the default zone so we can write to it.
    vm_address_t zone_address = (vm_address_t)default_zone;
    vm_protect(mach_task_self(), zone_address, sizeof(malloc_zone_t), 0,
               VM_PROT_READ | VM_PROT_WRITE | VM_PROT_COPY);

    // Patch function pointers in place.
    if (original_functions.malloc) default_zone->malloc = rt_malloc;
    if (original_functions.calloc) default_zone->calloc = rt_calloc;
    if (original_functions.valloc) default_zone->valloc = rt_valloc;
    if (original_functions.free) default_zone->free = rt_free;
    if (original_functions.realloc) default_zone->realloc = rt_realloc;
    if (original_functions.batch_malloc) default_zone->batch_malloc = rt_batch_malloc;
    if (original_functions.batch_free) default_zone->batch_free = rt_batch_free;
    if (original_functions.memalign) default_zone->memalign = rt_memalign;
    if (original_functions.free_definite_size)
      default_zone->free_definite_size = rt_free_definite_size;
    
    // Re-protect the default zone.
    vm_protect(mach_task_self(), zone_address, sizeof(malloc_zone_t), 0,
               VM_PROT_READ);

    // Restore is_realtime.
    MarkRealtime(old_is_realtime);
  }
}
#else
void RegisterMallocHook() {
  // If we are on a platform that doesn't support malloc hooks, we can't die on
  // malloc.
#error "Only linux and apple (Mac OS X) are supported"
}
#endif

}  // namespace aos
