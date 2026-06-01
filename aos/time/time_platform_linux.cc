#include <time.h>

#include <chrono>
#include <cstdint>

#include "absl/log/absl_check.h"

#include "aos/time/time.h"

namespace aos {

monotonic_clock::time_point monotonic_clock::now() noexcept {
  struct timespec current_time;
  ABSL_PCHECK(clock_gettime(CLOCK_MONOTONIC, &current_time) == 0)
      << "clock_gettime(" << static_cast<uintmax_t>(CLOCK_MONOTONIC) << ", "
      << &current_time << ") failed";

  return time_point(std::chrono::seconds(current_time.tv_sec) +
                    std::chrono::nanoseconds(current_time.tv_nsec));
}

realtime_clock::time_point realtime_clock::now() noexcept {
  struct timespec current_time;
  ABSL_PCHECK(clock_gettime(CLOCK_REALTIME, &current_time) == 0)
      << "clock_gettime(" << static_cast<uintmax_t>(CLOCK_REALTIME) << ", "
      << &current_time << ") failed";

  return realtime_clock::time_point(
      std::chrono::seconds(current_time.tv_sec) +
      std::chrono::nanoseconds(current_time.tv_nsec));
}

namespace this_thread {

void sleep_until(::aos::monotonic_clock::time_point end_time) {
  struct timespec end_time_timespec;
  ::std::chrono::seconds sec =
      ::std::chrono::duration_cast<::std::chrono::seconds>(
          end_time.time_since_epoch());
  ::std::chrono::nanoseconds nsec =
      ::std::chrono::duration_cast<::std::chrono::nanoseconds>(
          end_time.time_since_epoch() - sec);
  end_time_timespec.tv_sec = sec.count();
  end_time_timespec.tv_nsec = nsec.count();
  int returnval;
  do {
    returnval = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                                &end_time_timespec, nullptr);
    ABSL_PCHECK(returnval == 0 || returnval == EINTR)
        << ": clock_nanosleep(" << static_cast<uintmax_t>(CLOCK_MONOTONIC)
        << ", TIMER_ABSTIME, " << &end_time_timespec << ", nullptr) failed";
  } while (returnval != 0);
}

}  // namespace this_thread

}  // namespace aos
