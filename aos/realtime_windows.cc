// <windows.h> must be included before <detours.h> to prevent compilation
// errors, as <detours.h> relies on types defined in <windows.h>.
// clang-format off
#include <windows.h>
#include <detours.h>
// clang-format on

#include "absl/flags/declare.h"
#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/realtime.h"
#include "aos/realtime_internal.h"
#include "aos/uuid.h"

ABSL_DECLARE_FLAG(bool, die_on_malloc);
ABSL_DECLARE_FLAG(bool, skip_realtime_scheduler);

namespace logging::internal {
void ReloadThreadName() {}
}  // namespace logging::internal

namespace aos {

namespace {
// Here, we are trying to emulate the Linux kernel thread + priority model.
// Windows doesn't really have any of this.  Scheduler + priority + affinity is
// per-thread, so be per-thread on each of these.
thread_local int fake_rt_priority = 0;
thread_local int fake_rt_policy = SCHED_FIFO;
thread_local bool is_realtime = false;
thread_local CpuSet thread_affinity = DefaultAffinity();
}  // namespace

// CpuSet methods are in realtime_fake_cpuset.cc (shared with Darwin).

bool GetIsRealtime() { return is_realtime; }

void SetIsRealtime(bool realtime) { is_realtime = realtime; }

void UnsetCurrentThreadRealtimePriority() {
  MarkRealtime(false);
  fake_rt_priority = 0;
  fake_rt_policy = SCHED_OTHER;
  if (ABSL_VLOG_IS_ON(1)) {
    ABSL_LOG(WARNING) << "No RT scheduler on Windows, ignoring";
  }
}

void SetCurrentThreadAffinity(const CpuSet &cpuset) {
  thread_affinity = cpuset;
}

void SetCurrentThreadName(const std::string_view name) {
  // Not implemented on Windows yet.
  if (ABSL_VLOG_IS_ON(1)) {
    ABSL_LOG(WARNING)
        << "SetCurrentThreadName not implemented on Windows (name: '" << name
        << "')";
  }
}

CpuSet GetCurrentThreadAffinity() { return thread_affinity; }

void SetCurrentThreadRealtimePriority(int priority, int scheduling_policy,
                                      RealtimePolicy realtime_policy) {
  UUID::Random();

  if (absl::GetFlag(FLAGS_skip_realtime_scheduler)) {
    ABSL_LOG(WARNING) << "Ignoring request to switch to the RT scheduler due "
                         "to --skip_realtime_scheduler.";
    return;
  }

  ABSL_CHECK(scheduling_policy == SCHED_FIFO || scheduling_policy == SCHED_RR)
      << "Specified non-realtime scheduling policy with realtime priority";
  ABSL_CHECK(priority > 0 && priority < 100)
      << "Realtime priority must fall within [1,99]";

  ABSL_VLOG(1) << "RT priority not implemented on Windows, pretending to be RT";

  fake_rt_priority = priority;
  fake_rt_policy = scheduling_policy;

  if (realtime_policy != RealtimePolicy::NO_MODE) {
    MarkRealtime(true);
  }
}

int GetCurrentThreadRealtimePriority() { return fake_rt_priority; }

int GetCurrentThreadSchedulingPolicy() { return fake_rt_policy; }

namespace {

static void *(__cdecl *original_malloc)(size_t) = nullptr;
static void(__cdecl *original_free)(void *) = nullptr;
static void *(__cdecl *original_realloc)(void *, size_t) = nullptr;
static void *(__cdecl *original_calloc)(size_t, size_t) = nullptr;

void *AosMalloc(size_t size) {
  if (aos::GetIsRealtime() && absl::GetFlag(FLAGS_die_on_malloc)) {
    aos::SetIsRealtime(false);
    ABSL_RAW_LOG(FATAL, "RAW: Malloced");
  }
  return original_malloc(size);
}

void AosFree(void *ptr) { original_free(ptr); }

void *AosRealloc(void *ptr, size_t size) {
  if (aos::GetIsRealtime() && absl::GetFlag(FLAGS_die_on_malloc)) {
    aos::SetIsRealtime(false);
    ABSL_RAW_LOG(FATAL, "RAW: Malloced");
  }
  return original_realloc(ptr, size);
}

void *AosCalloc(size_t num, size_t size) {
  if (aos::GetIsRealtime() && absl::GetFlag(FLAGS_die_on_malloc)) {
    aos::SetIsRealtime(false);
    ABSL_RAW_LOG(FATAL, "RAW: Malloced");
  }
  return original_calloc(num, size);
}

}  // namespace

void RegisterMallocHook() {
  static bool registered = false;
  if (registered) return;
  registered = true;

  HMODULE h_ucrt = GetModuleHandleA("ucrtbase.dll");
  if (!h_ucrt) {
    h_ucrt = GetModuleHandleA("ucrtbased.dll");
  }
  ABSL_CHECK(h_ucrt != nullptr)
      << ": Failed to find ucrtbase.dll or ucrtbased.dll";

  original_malloc = reinterpret_cast<decltype(original_malloc)>(
      GetProcAddress(h_ucrt, "malloc"));
  ABSL_CHECK(original_malloc != nullptr)
      << ": Failed to find malloc in ucrtbase.dll";
  original_free =
      reinterpret_cast<decltype(original_free)>(GetProcAddress(h_ucrt, "free"));
  ABSL_CHECK(original_free != nullptr)
      << ": Failed to find free in ucrtbase.dll";
  original_realloc = reinterpret_cast<decltype(original_realloc)>(
      GetProcAddress(h_ucrt, "realloc"));
  ABSL_CHECK(original_realloc != nullptr)
      << ": Failed to find realloc in ucrtbase.dll";
  original_calloc = reinterpret_cast<decltype(original_calloc)>(
      GetProcAddress(h_ucrt, "calloc"));
  ABSL_CHECK(original_calloc != nullptr)
      << ": Failed to find calloc in ucrtbase.dll";

  LONG begin_err = DetourTransactionBegin();
  ABSL_CHECK_EQ(begin_err, NO_ERROR)
      << ": DetourTransactionBegin failed: " << begin_err;

  LONG update_err = DetourUpdateThread(GetCurrentThread());
  ABSL_CHECK_EQ(update_err, NO_ERROR)
      << ": DetourUpdateThread failed: " << update_err;

  LONG attach_malloc = DetourAttach(reinterpret_cast<void **>(&original_malloc),
                                    reinterpret_cast<void *>(AosMalloc));
  ABSL_CHECK_EQ(attach_malloc, NO_ERROR)
      << ": DetourAttach malloc failed: " << attach_malloc;

  LONG attach_free = DetourAttach(reinterpret_cast<void **>(&original_free),
                                  reinterpret_cast<void *>(AosFree));
  ABSL_CHECK_EQ(attach_free, NO_ERROR)
      << ": DetourAttach free failed: " << attach_free;

  LONG attach_realloc =
      DetourAttach(reinterpret_cast<void **>(&original_realloc),
                   reinterpret_cast<void *>(AosRealloc));
  ABSL_CHECK_EQ(attach_realloc, NO_ERROR)
      << ": DetourAttach realloc failed: " << attach_realloc;

  LONG attach_calloc = DetourAttach(reinterpret_cast<void **>(&original_calloc),
                                    reinterpret_cast<void *>(AosCalloc));
  ABSL_CHECK_EQ(attach_calloc, NO_ERROR)
      << ": DetourAttach calloc failed: " << attach_calloc;

  LONG commit_err = DetourTransactionCommit();
  ABSL_CHECK_EQ(commit_err, NO_ERROR)
      << ": DetourTransactionCommit failed: " << commit_err;
}

std::string GetProgramName() {
  char exe_path[MAX_PATH];
  GetModuleFileNameA(nullptr, exe_path, MAX_PATH);
  const char *last_slash = strrchr(exe_path, '\\');
  return std::string(last_slash ? (last_slash + 1) : exe_path);
}

std::string GetThreadName() { return ""; }

pid_t GetProcessId() { return GetCurrentProcessId(); }
pid_t GetThreadId() { return static_cast<pid_t>(GetCurrentThreadId()); }

uid_t GetUserId() { return 0; }

std::string GetUsername(uid_t /*uid*/) { return "unknown"; }

int SetCurrentThreadRealtimePriorityLowLevel(int /*priority*/,
                                             int /*scheduling_policy*/) {
  return 0;
}

}  // namespace aos
