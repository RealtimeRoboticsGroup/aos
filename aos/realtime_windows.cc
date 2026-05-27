#include <windows.h>

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
int fake_rt_priority = 0;
int fake_rt_policy = SCHED_FIFO;
bool is_realtime = false;
CpuSet thread_affinity;
}  // namespace

// CpuSet methods are in realtime_fake_cpuset.cc (shared with Darwin).

bool GetIsRealtime() { return is_realtime; }

void SetIsRealtime(bool realtime) { is_realtime = realtime; }

void UnsetCurrentThreadRealtimePriority() {
  MarkRealtime(false);
  fake_rt_priority = 0;
  fake_rt_policy = SCHED_FIFO;
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

void RegisterMallocHook() {
  if (absl::GetFlag(FLAGS_die_on_malloc)) {
    ABSL_LOG(WARNING) << "Malloc hooks not implemented on Windows";
  }
}

std::string GetProgramName() {
  char exe_path[MAX_PATH];
  GetModuleFileNameA(nullptr, exe_path, MAX_PATH);
  const char *last_slash = strrchr(exe_path, '\\');
  return std::string(last_slash ? (last_slash + 1) : exe_path);
}

std::string GetThreadName() { return ""; }

int32_t GetProcessId() { return GetCurrentProcessId(); }

}  // namespace aos
