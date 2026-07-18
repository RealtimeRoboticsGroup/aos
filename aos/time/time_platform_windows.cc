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
    ABSL_PCHECK(QueryPerformanceFrequency(&freq) != 0);
    ABSL_PCHECK(freq.QuadPart != 0);
    return freq.QuadPart;
  }();
  return frequency;
}

}  // namespace

monotonic_clock::time_point monotonic_clock::now() noexcept {
  LARGE_INTEGER counter;
  ABSL_PCHECK(QueryPerformanceCounter(&counter) != 0);

  const int64_t frequency = WindowsPerformanceFrequency();
  absl::int128 nanos = absl::int128(counter.QuadPart) * 1000000000;
  nanos /= frequency;
  uint64_t current_nanos = static_cast<uint64_t>(nanos);

  // The lockless queue, among other things, wants to see time go forwards each
  // time we read it.  On Windows, the QueryPerformanceCounter resolution is
  // typically low enough (often 100 ns) that duplicate values are common under
  // rapid polling.  Spin until it changes to make this true.
  static thread_local uint64_t last_nanos = 0;
  if (current_nanos <= last_nanos) {
    do {
      ABSL_PCHECK(QueryPerformanceCounter(&counter) != 0);
      nanos = absl::int128(counter.QuadPart) * 1000000000;
      nanos /= frequency;
      current_nanos = static_cast<uint64_t>(nanos);
    } while (current_nanos <= last_nanos);
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
