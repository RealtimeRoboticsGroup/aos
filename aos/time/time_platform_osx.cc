#include <mach/mach.h>
#include <mach/mach_time.h>

#include <atomic>
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

// Why CLOCK_UPTIME_RAW, and not CLOCK_MONOTONIC or CLOCK_MONOTONIC_RAW?
//
// Two independent reasons: the timebase has to be Mach absolute time, and the
// resolution has to be fine enough that spinning for a distinct value is cheap.
//
// 1) Timebase.  Everything on Darwin that we hand an *absolute* deadline to
//    consumes Mach absolute ticks, not POSIX clock nanoseconds:
//      - sleep_until() below, via mach_wait_until().
//      - The kqueue timers in //aos/events:aio_darwin and
//        //aos/events:epoll_darwin, which arm EVFILT_TIMER with
//        NOTE_MACHTIME | NOTE_ABSOLUTE.
//    All of those take a monotonic_clock::time_point and convert it with
//    nothing but the mach_timebase_info ratio (see ToMachTicks() in
//    aio_darwin.cc).  That conversion is only correct if now() is *already*
//    expressed in the Mach timebase, so what we need is a clock that is
//    contractually mach_absolute_time().  Of the three candidates, only
//    CLOCK_UPTIME_RAW is; from clock_gettime(3) on Darwin:
//
//      CLOCK_UPTIME_RAW     "...does not increment while the system is asleep.
//                            The returned value is identical to the result of
//                            mach_absolute_time() after the appropriate
//                            mach_timebase conversion is applied."
//      CLOCK_MONOTONIC      "...will continue to increment while the system is
//                            asleep."
//      CLOCK_MONOTONIC_RAW  "...like CLOCK_MONOTONIC.  However, this clock is
//                            unaffected by frequency or time adjustments.  It
//                            should not be compared to other system time
//                            sources."
//
//    So CLOCK_MONOTONIC is disqualified outright: counting through sleep puts
//    it on a different epoch than Mach time.  Measured on the machine below it
//    already runs 2.3 *seconds* ahead, growing with every suspend, and every
//    kqueue deadline would fire that far early with a drift no test would
//    catch.  CLOCK_MONOTONIC_RAW happens to track Mach time today, but the
//    documentation explicitly disclaims being comparable to other time sources,
//    so depending on it would be depending on a coincidence.
//
// 2) Resolution.  now() below spins until the clock reports a new value, so a
//    coarsely quantized clock is paid for directly in the spin loop.
//    CLOCK_MONOTONIC is quantized to 1us on Darwin -- ~24x coarser than the
//    hardware tick -- and is also the slowest of the three to read.
//
// Measured on an M-series Mac (Darwin 25.5.0, 24MHz timebase, 41.66ns tick).
// "duplicates" is the fraction of back-to-back reads that returned an
// unchanged value, i.e. how often a spinning now() would have to go around
// again:
//
//   clock                  granularity   duplicates   read cost
//   CLOCK_UPTIME_RAW              41 ns        80.4%      8.0 ns
//   CLOCK_MONOTONIC_RAW           41 ns        80.4%      8.0 ns
//   CLOCK_MONOTONIC            1,000 ns        98.7%     12.2 ns
//
//   aos::monotonic_clock          41 ns         0.0%     41.7 ns
//     (CLOCK_UPTIME_RAW + the spin loop below; by construction it never
//      repeats, and the 41.7ns cost is one full tick, which is the point.)
//
// To regenerate all of the above, run //aos/time:time_test.  The granularity
// and duplicate columns come from TimeTest.MonotonicClockGranularity, the read
// costs from TimeTest.ClockBenchmark, and the timebase requirement in (1) is
// asserted by TimeTest.MonotonicClockUsesMachTimebase, which also prints the
// current CLOCK_MONOTONIC-vs-Mach gap.
monotonic_clock::time_point monotonic_clock::now() noexcept {
  uint64_t current_nanos = clock_gettime_nsec_np(CLOCK_UPTIME_RAW);

  // The lockless queue, among other things, wants to see time go forwards each
  // time we read it. That isn't a crazy request.  On OSX, the timer resolution
  // is 41.66ns (24 mhz), and it takes about 10 ns to read the clock.  Spin
  // until it changes to make this true.
  static thread_local uint64_t last_nanos = 0;
  while (current_nanos <= last_nanos) {
    current_nanos = clock_gettime_nsec_np(CLOCK_UPTIME_RAW);
  }
  last_nanos = current_nanos;

  return time_point(std::chrono::nanoseconds(current_nanos));
}

realtime_clock::time_point realtime_clock::now() noexcept {
  return realtime_clock::time_point(
      std::chrono::nanoseconds(clock_gettime_nsec_np(CLOCK_REALTIME)));
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
