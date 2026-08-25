#include <errno.h>
#include <malloc.h>
#include <pwd.h>
#include <unistd.h>

#if defined(AOS_SANITIZE_MEMORY)
#include <sanitizer/msan_interface.h>
#endif
#include <sched.h>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <sys/time.h>

#include <optional>
#include <string>

#include "absl/flags/declare.h"
#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/realtime.h"
#include "aos/realtime_internal.h"
#include "aos/uuid.h"

ABSL_DECLARE_FLAG(bool, die_on_malloc);
ABSL_DECLARE_FLAG(bool, skip_realtime_scheduler);

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

namespace logging::internal {

// Implemented in aos/logging/context.cc.
void ReloadThreadName() __attribute__((weak));

}  // namespace logging::internal

extern "C" char *program_invocation_short_name;

namespace aos {

extern bool has_malloc_hook;

void NewHook(const void *ptr, size_t size);
void DeleteHook(const void *ptr);

namespace {
thread_local bool is_realtime = false;
}

bool GetIsRealtime() { return is_realtime; }
void SetIsRealtime(bool realtime) { is_realtime = realtime; }

extern "C" {

// malloc hooks for libc. Tcmalloc will replace everything it finds (malloc,
// __libc_malloc, etc.), so we need its specific hook above as well.
void *aos_malloc_hook(size_t size) {
  if (absl::GetFlag(FLAGS_die_on_malloc) && GetIsRealtime()) {
    SetIsRealtime(false);
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
  if (absl::GetFlag(FLAGS_die_on_malloc) && GetIsRealtime() && ptr != nullptr) {
    SetIsRealtime(false);
    ABSL_RAW_LOG(FATAL, "Deleted %p", ptr);
  } else {
    __libc_free(ptr);
  }
}

void *aos_realloc_hook(void *ptr, size_t size) {
  if (absl::GetFlag(FLAGS_die_on_malloc) && GetIsRealtime()) {
    SetIsRealtime(false);
    ABSL_RAW_LOG(FATAL, "Malloced %p -> %zu bytes", ptr, size);
    return nullptr;
  } else {
    return __libc_realloc(ptr, size);
  }
}

void *aos_calloc_hook(size_t n, size_t elem_size) {
  if (absl::GetFlag(FLAGS_die_on_malloc) && GetIsRealtime()) {
    SetIsRealtime(false);
    ABSL_RAW_LOG(FATAL, "Malloced %zu * %zu bytes", n, elem_size);
    return nullptr;
  } else {
    return __libc_calloc(n, elem_size);
  }
}

void *malloc(size_t size) __attribute__((weak, alias("aos_malloc_hook")));
void free(void *ptr) __attribute__((weak, alias("aos_free_hook")));
void *realloc(void *ptr, size_t size)
    __attribute__((weak, alias("aos_realloc_hook")));
void *calloc(size_t n, size_t elem_size)
    __attribute__((weak, alias("aos_calloc_hook")));
}

CpuSet::CpuSet() { CPU_ZERO(&set_); }

void CpuSet::Set(int cpu) { CPU_SET(cpu, &set_); }

void CpuSet::Clear(int cpu) { CPU_CLR(cpu, &set_); }

void CpuSet::Clear() { CPU_ZERO(&set_); }

bool CpuSet::IsSet(int cpu) const { return CPU_ISSET(cpu, &set_); }

bool CpuSet::Empty() const { return CPU_COUNT(&set_) == 0; }

bool CpuSet::operator==(const CpuSet &other) const {
  return CPU_EQUAL(&set_, &other.set_);
}

bool CpuSet::operator!=(const CpuSet &other) const {
  return !CPU_EQUAL(&set_, &other.set_);
}

void UnsetCurrentThreadRealtimePriority() {
  struct sched_param param;
  param.sched_priority = 0;
  ABSL_PCHECK(sched_setscheduler(0, SCHED_OTHER, &param) == 0);
  MarkRealtime(false);
}

void SetCurrentThreadAffinity(const CpuSet &cpuset) {
  ABSL_PCHECK(sched_setaffinity(0, sizeof(cpu_set_t), cpuset.native_handle()) ==
              0)
      << cpuset;
}

void SetCurrentThreadName(const std::string_view name) {
  ABSL_CHECK_LE(name.size(), 16u) << ": thread name '" << name << "' too long";
  ABSL_VLOG(1) << "This thread is changing to '" << name << "'";
  std::string string_name(name);
  ABSL_PCHECK(prctl(PR_SET_NAME, string_name.c_str()) == 0)
      << ": changing name to " << string_name;
  if (&logging::internal::ReloadThreadName != nullptr) {
    logging::internal::ReloadThreadName();
  }
}

CpuSet GetCurrentThreadAffinity() {
  CpuSet result;
  ABSL_PCHECK(sched_getaffinity(0, sizeof(cpu_set_t), result.native_handle()) ==
              0);
  return result;
}

void SetCurrentThreadRealtimePriority(int priority, int scheduling_policy,
                                      RealtimePolicy realtime_policy) {
  // Ensure that we won't get expensive reads of /dev/random when the realtime
  // scheduler is running.
  UUID::Random();

  if (absl::GetFlag(FLAGS_skip_realtime_scheduler)) {
    ABSL_LOG(WARNING)
        << "Ignoring request to switch to the RT scheduler due to "
           "--skip_realtime_scheduler.";
    return;
  }
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

  ABSL_CHECK(scheduling_policy == SCHED_FIFO || scheduling_policy == SCHED_RR)
      << "Specified non-realtime scheduling policy with realtime priority";
  ABSL_CHECK(priority > 0 && priority < 100)
      << "Realtime priority must fall within [1,99]";
  struct sched_param param;
  param.sched_priority = priority;
  if (realtime_policy != RealtimePolicy::NO_MODE) {
    MarkRealtime(true);
  }
  ABSL_PCHECK(sched_setscheduler(0, scheduling_policy, &param) == 0)
      << ": changing to realtime scheduler ("
      << (scheduling_policy == SCHED_FIFO ? "SCHED_FIFO" : "SCHED_RR")
      << ") with priority " << priority
      << ", if you want to bypass this check for testing, use "
         "--skip_realtime_scheduler";
}

int GetCurrentThreadRealtimePriority() {
  struct sched_param result;
  ABSL_PCHECK(sched_getparam(0, &result) == 0)
      << ": Failed to retrieve the Realtime Priority";
  return result.sched_priority;
}

int GetCurrentThreadSchedulingPolicy() {
  int scheduling_policy = sched_getscheduler(0);
  ABSL_PCHECK(scheduling_policy >= 0)
      << ": Failed to retrieve the scheduling policy";
  return scheduling_policy;
}

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

std::string GetThreadName() {
  char thread_name_array[17];
  if (prctl(PR_GET_NAME, thread_name_array) != 0) {
    ABSL_PLOG(FATAL) << "prctl(PR_GET_NAME, " << thread_name_array
                     << ") failed";
  }
#if defined(AOS_SANITIZE_MEMORY)
  // msan doesn't understand PR_GET_NAME, so help it along.
  __msan_unpoison(thread_name_array, sizeof(thread_name_array));
#endif
  thread_name_array[sizeof(thread_name_array) - 1] = '\0';
  return std::string(thread_name_array);
}

pid_t GetProcessId() { return getpid(); }

std::chrono::nanoseconds GetCurrentThreadCpuTime() {
  struct timespec ts;
  ABSL_PCHECK(clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts) == 0);
  return std::chrono::seconds(ts.tv_sec) + std::chrono::nanoseconds(ts.tv_nsec);
}

uid_t GetUserId() {
  uid_t ruid, euid, suid;
  ABSL_PCHECK(getresuid(&ruid, &euid, &suid) == 0);

  // Pick the UID that peers can use to signal us.  A signal is deliverable
  // when the sender's real or effective UID matches our real or *saved* UID,
  // so whatever we return has to be one of our ruid/suid.
  //
  // If the effective and saved UIDs are equal, use them, even when that
  // differs from the real UID.  This lets a process keep a real UID of 0 (to
  // have permissions to perform system-level changes) while still being able
  // to communicate with processes running unprivileged as a distinct user.
  //
  // If they differ, we've changed our euid away from the saved one, and it may
  // be neither our ruid nor our suid.  Fall back to the ruid, which always is
  // one of them.
  if (euid == suid) {
    return euid;
  } else {
    return ruid;
  }
}

std::optional<std::string> GetUsername(uid_t uid) {
  struct passwd const *pw = getpwuid(uid);
  if (pw == nullptr) {
    return std::nullopt;
  } else {
    return pw->pw_name;
  }
}

int SetCurrentThreadRealtimePriorityLowLevel(int priority,
                                             int scheduling_policy) {
  struct sched_param param;
  param.sched_priority = priority;
  return sched_setscheduler(0, scheduling_policy, &param);
}

}  // namespace aos
