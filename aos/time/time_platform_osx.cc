#include <mach/mach.h>
#include <mach/mach_time.h>

#include <chrono>
#include <cstdint>

#include "absl/log/absl_check.h"
#include "absl/numeric/int128.h"

#include "aos/time/time.h"

namespace aos {
namespace {

mach_timebase_info_data_t LoadMachTimebaseInfo() {
  mach_timebase_info_data_t info;
  ABSL_CHECK_EQ(mach_timebase_info(&info), KERN_SUCCESS);
  return info;
}

const mach_timebase_info_data_t &MachTimebaseInfo() {
  static const mach_timebase_info_data_t timebase_info = LoadMachTimebaseInfo();
  return timebase_info;
}

}  // namespace

monotonic_clock::time_point monotonic_clock::now() noexcept {
  const mach_timebase_info_data_t &timebase_info = MachTimebaseInfo();
  uint64_t current_nanos = static_cast<uint64_t>(
      (absl::uint128(mach_absolute_time()) * timebase_info.numer) /
      timebase_info.denom);
  return time_point(std::chrono::nanoseconds(current_nanos));
}

realtime_clock::time_point realtime_clock::now() noexcept {
  auto now = std::chrono::system_clock::now().time_since_epoch();
  return realtime_clock::time_point(
      std::chrono::duration_cast<std::chrono::nanoseconds>(now));
}

namespace this_thread {

void sleep_until(aos::monotonic_clock::time_point end_time) {
  const mach_timebase_info_data_t &timebase_info = MachTimebaseInfo();

  uint64_t end_nanos =
      std::chrono::nanoseconds(end_time.time_since_epoch()).count();

  uint64_t end_ticks = static_cast<uint64_t>(
      (absl::uint128(end_nanos) * timebase_info.denom) / timebase_info.numer);

  kern_return_t result;
  do {
    result = mach_wait_until(end_ticks);
    if (result == KERN_SUCCESS) break;
    ABSL_PCHECK(result == KERN_ABORTED)
        << ": mach_wait_until(" << end_ticks
        << ") failed with unexpected error: " << result;
  } while (result == KERN_ABORTED);
}

}  // namespace this_thread

}  // namespace aos
