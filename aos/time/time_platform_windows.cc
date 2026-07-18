#include "aos/time/time.h"

#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0602  // Windows 8
#endif
#ifndef NTDDI_VERSION
#define NTDDI_VERSION NTDDI_WIN8
#endif

#include <profileapi.h>
#include <synchapi.h>
#include <windows.h>

#include <chrono>
#include <cstdint>
#include <thread>

#include "absl/log/absl_check.h"
#include "absl/numeric/int128.h"

namespace aos {
namespace {

// Some Windows SDK versions gate the prototype on higher NTDDI guards.
extern "C" __declspec(dllimport) void WINAPI
GetSystemTimePreciseAsFileTime(LPFILETIME file_time);

static uint64_t FileTimeToUnixNanos(const FILETIME &file_time) {
  const ULONGLONG ticks =
      (static_cast<ULONGLONG>(file_time.dwHighDateTime) << 32) |
      file_time.dwLowDateTime;
  constexpr ULONGLONG kUnixEpochDiff100ns = 11644473600ULL * 10000000ULL;
  const ULONGLONG unix_ticks = ticks - kUnixEpochDiff100ns;
  return unix_ticks * 100ULL;
}

static int64_t WindowsPerformanceFrequency() {
  static const int64_t frequency = [] {
    LARGE_INTEGER freq;
    // QueryPerformance{Counter,Frequency} report failures via their return
    // value, not errno, so ABSL_CHECK (rather than ABSL_PCHECK) is
    // appropriate here.
    ABSL_CHECK(QueryPerformanceFrequency(&freq) != 0);
    ABSL_CHECK(freq.QuadPart != 0);
    return freq.QuadPart;
  }();
  return frequency;
}

// Number of nanoseconds in a second, pulled out to a constant so the 2
// QueryPerformanceCounter->nanoseconds conversions below are easy to compare
// and keep in sync.
constexpr int64_t kNanosPerSecond = 1000000000;

uint64_t QueryPerformanceCounterNanos(int64_t frequency) {
  LARGE_INTEGER counter;
  ABSL_CHECK(QueryPerformanceCounter(&counter) != 0);
  const absl::int128 nanos =
      (absl::int128(counter.QuadPart) * kNanosPerSecond) / frequency;
  return static_cast<uint64_t>(nanos);
}

}  // namespace

monotonic_clock::time_point monotonic_clock::now() noexcept {
  const int64_t frequency = WindowsPerformanceFrequency();
  uint64_t current_nanos = QueryPerformanceCounterNanos(frequency);

  // The lockless queue, among other things, wants to see time go forwards each
  // time we read it.  On Windows, the QueryPerformanceCounter resolution is
  // typically low enough (often 100 ns) that duplicate values are common under
  // rapid polling.  Spin until it changes to make this true.
  static thread_local uint64_t last_nanos = 0;
  while (current_nanos <= last_nanos) {
    current_nanos = QueryPerformanceCounterNanos(frequency);
  }
  last_nanos = current_nanos;

  return time_point(std::chrono::nanoseconds(current_nanos));
}

realtime_clock::time_point realtime_clock::now() noexcept {
  FILETIME file_time;
  GetSystemTimePreciseAsFileTime(&file_time);
  return realtime_clock::time_point(
      std::chrono::nanoseconds(FileTimeToUnixNanos(file_time)));
}

namespace this_thread {

void sleep_until(aos::monotonic_clock::time_point end_time) {
  // Windows doesn't have a sleep_until, so fake it.
  auto now = aos::monotonic_clock::now();
  if (end_time <= now) {
    return;
  }
  std::this_thread::sleep_for(end_time - now);
}

}  // namespace this_thread

}  // namespace aos
