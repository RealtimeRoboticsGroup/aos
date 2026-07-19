#ifndef AOS_REALTIME_H_
#define AOS_REALTIME_H_

#ifndef _WIN32
#include <sched.h>
#include <sys/types.h>
#ifdef __linux__
#include <sys/syscall.h>
#endif
#include <unistd.h>
#else
#define SCHED_OTHER 0
#define SCHED_FIFO 1
#define SCHED_RR 2
using pid_t = int;
#endif

#include <cstring>
#include <ostream>
#include <span>
#include <string_view>

#if defined(__APPLE__) || defined(_WIN32)
#include <bitset>
#endif

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "absl/log/absl_vlog_is_on.h"
#include "absl/strings/str_format.h"

namespace aos {

class CpuSet {
 public:
#ifdef __linux__
  static constexpr size_t kSize = CPU_SETSIZE;
#elif defined(__APPLE__) || defined(_WIN32)
  static constexpr size_t kSize = 1024;
#else
#error "Only linux, apple (Mac OS X), and Windows are supported"
#endif

  CpuSet();

  void Set(int cpu);
  void Clear(int cpu);
  void Clear();
  bool IsSet(int cpu) const;
  bool Empty() const;

#ifdef __linux__
  cpu_set_t *native_handle() { return &set_; }
  const cpu_set_t *native_handle() const { return &set_; }
#endif

  bool operator==(const CpuSet &other) const;
  bool operator!=(const CpuSet &other) const;

  template <typename Sink>
  friend void AbslStringify(Sink &sink, const CpuSet &cpuset) {
    sink.Append("{CPUs ");
    bool first_found = false;
    for (int i = 0; i < static_cast<int>(kSize); ++i) {
      if (cpuset.IsSet(i)) {
        if (first_found) {
          sink.Append(", ");
        }
        absl::Format(&sink, "%d", i);
        first_found = true;
      }
    }
    sink.Append("}");
  }

 private:
#ifdef __linux__
  cpu_set_t set_;
#elif defined(__APPLE__) || defined(_WIN32)
  std::bitset<kSize> set_;
#else
#error "Only linux, apple (Mac OS X), and Windows are supported"
#endif
};

// Locks everything into memory and sets the limits.  This plus InitNRT are
// everything you need to do before SetCurrentThreadRealtimePriority will make
// your thread RT.  Called as part of ShmEventLoop::Run()
#ifndef _WIN32
void InitRT();

// Sets up this process to write core dump files.
// This is called by Init*, but it's here for other files that want this
// behavior without calling Init*.
void WriteCoreDumps();

void LockAllMemory();

void ExpandStackSize();
#endif

// Sets the name of the current thread.
// This will displayed by `top -H`, dump_rtprio, and show up in logs.
// name can have a maximum of 16 characters.
void SetCurrentThreadName(const std::string_view name);

// Creates a CpuSet from a list of CPUs.
inline CpuSet MakeCpusetFromCpus(std::span<const int> cpus) {
  CpuSet result;
  for (int cpu : cpus) {
    result.Set(cpu);
  }
  return result;
}

inline CpuSet MakeCpusetFromCpus(std::initializer_list<int> cpus) {
  return MakeCpusetFromCpus(std::span<const int>(cpus.begin(), cpus.end()));
}

// Returns the affinity representing all the CPUs.
inline CpuSet DefaultAffinity() {
  CpuSet result;
  for (int i = 0; i < static_cast<int>(CpuSet::kSize); ++i) {
    result.Set(i);
  }
  return result;
}

// Returns the current thread's CPU affinity.
CpuSet GetCurrentThreadAffinity();

// Sets the current thread's scheduling affinity.
void SetCurrentThreadAffinity(const CpuSet &cpuset);

// Returns the ID of the current process.
pid_t GetProcessId();

// Returns the ID of the current thread.
#ifdef __linux__
inline pid_t GetThreadId() { return syscall(SYS_gettid); }
#else
pid_t GetThreadId();
#endif

// Returns the name of the current process/program.
std::string GetProgramName();

// Returns the name of the current thread. If the OS does not support
// thread names (e.g., Windows by default), returns an empty string.
std::string GetThreadName();

// Everything below here needs AOS to be initialized before it will work
// properly.

enum class RealtimePolicy {
  // Change to "real-time-mode" and prohibit malloc.
  REALTIME_MODE_DENY_MALLOC,
  // Don't change to "real-time-mode", and allow malloc.
  NO_MODE,

  // It could be useful to support a REALTIME_MODE_WITH_MALLOC in the future.
  // That will require decoupling the malloc hooks from SetIsRealtime and
  // friends.
};

// Sets the current thread's realtime priority.
// Takes in an integer argument for the realtime priority value between [1,99],
// and an optional integer for the scheduling_policy as defined in
// `include/linux/sched.h` (otherwise defaults to SCHED_FIFO).
void SetCurrentThreadRealtimePriority(
    int priority, int scheduling_policy = SCHED_FIFO,
    RealtimePolicy realtime_policy = RealtimePolicy::REALTIME_MODE_DENY_MALLOC);

// Returns the current thread's realtime priority.
int GetCurrentThreadRealtimePriority();

// Returns the current thread's scheduling policy.
int GetCurrentThreadSchedulingPolicy();

// Unsets all threads realtime priority in preparation for exploding.
extern "C" void aos_FatalUnsetRealtimePriority();

// Sets the current thread back down to non-realtime priority.
void UnsetCurrentThreadRealtimePriority();

// Registers our hooks which crash on RT malloc.
void RegisterMallocHook();

// CHECKs that we are (or are not) running on the RT scheduler.  Useful for
// enforcing that operations which are or are not bounded shouldn't be run. This
// works both in simulation and when running against the real target.
void CheckRealtime();
void CheckNotRealtime();

// Marks that we are or are not running on the realtime scheduler.  Returns the
// previous state.
//
// Note: this shouldn't be used directly.  The event loop primitives should be
// used instead.
bool MarkRealtime(bool realtime);

// Returns true if we are running on the realtime scheduler and the malloc hooks
// are active. If this returns true, no memory allocations or frees are allowed.
bool IsDieOnMallocEnabled();

// Class which restores the current RT state when destructed.
class ScopedRealtimeRestorer {
 public:
  ScopedRealtimeRestorer();
  ~ScopedRealtimeRestorer() { MarkRealtime(prior_); }

 private:
  const bool prior_;
};

// Class which marks us as on the RT scheduler until it goes out of scope.
// Note: this shouldn't be needed for most applications.
class ScopedRealtime {
 public:
  ScopedRealtime() : prior_(MarkRealtime(true)) {}
  ~ScopedRealtime() {
    ABSL_CHECK(MarkRealtime(prior_)) << ": Priority was modified";
  }

 private:
  const bool prior_;
};

// Class which marks us as not on the RT scheduler until it goes out of scope.
// Note: this shouldn't be needed for most applications.
class ScopedNotRealtime {
 public:
  ScopedNotRealtime() : prior_(MarkRealtime(false)) {}
  ~ScopedNotRealtime() {
    ABSL_CHECK(!MarkRealtime(prior_)) << ": Priority was modified";
  }

 private:
  const bool prior_;
};

}  // namespace aos

#endif  // AOS_REALTIME_H_
