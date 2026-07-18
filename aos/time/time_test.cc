#include "aos/time/time.h"

#if !defined(_WIN32)
#include <sys/time.h>
#endif

#include <algorithm>
#include <atomic>
#include <limits>
#include <memory>
#include <string_view>
#include <thread>
#include <vector>

#include "absl/log/log.h"
#include "absl/numeric/int128.h"
#include "absl/strings/str_format.h"
#include "gtest/gtest.h"

namespace aos::time::testing {

namespace chrono = std::chrono;

TEST(TimeTest, FromRate) { EXPECT_EQ(chrono::milliseconds(10), FromRate(100)); }

// Test the monotonic_clock and sleep_until functions.
TEST(TimeTest, MonotonicClockSleepAndNow) {
  monotonic_clock::time_point start = monotonic_clock::now();
  const auto kSleepTime = chrono::milliseconds(500);
  ::std::this_thread::sleep_until(start + kSleepTime);
  monotonic_clock::time_point end = monotonic_clock::now();
  EXPECT_GE(end - start, kSleepTime);
  EXPECT_LT(end - start, kSleepTime + chrono::milliseconds(100));
}

TEST(TimeTest, MonotonicClockSleepUntilPastTime) {
  // sleep_until should return promptly when the requested time is in the past.
  bool success = false;
  chrono::nanoseconds worst_case = chrono::nanoseconds::zero();
  for (int attempt = 0; attempt < 5 && !success; ++attempt) {
    monotonic_clock::time_point start = monotonic_clock::now();
    ::std::this_thread::sleep_until(start - chrono::milliseconds(200));
    monotonic_clock::time_point end = monotonic_clock::now();
    auto elapsed = end - start;
    worst_case = std::max(worst_case, elapsed);
    success = elapsed < chrono::milliseconds(5);
  }
  EXPECT_TRUE(success)
      << "sleep_until took "
      << chrono::duration_cast<chrono::milliseconds>(worst_case).count()
      << "ms at best";
}

TEST(TimeTest, StdChronoSleepFor) {
  const auto kSleepTime = chrono::milliseconds(200);
  bool success = false;
  chrono::nanoseconds last = chrono::nanoseconds::zero();
  for (int attempt = 0; attempt < 5 && !success; ++attempt) {
    const auto start = chrono::steady_clock::now();
    ::std::this_thread::sleep_for(kSleepTime);
    const auto end = chrono::steady_clock::now();
    last = end - start;
    success =
        last >= kSleepTime && last < kSleepTime + chrono::milliseconds(100);
  }
  EXPECT_TRUE(success)
      << "sleep_for duration "
      << chrono::duration_cast<chrono::milliseconds>(last).count() << "ms";
}

TEST(TimeTest, StdChronoSleepUntil) {
  const auto kSleepTime = chrono::milliseconds(300);
  bool success = false;
  chrono::nanoseconds last = chrono::nanoseconds::zero();
  for (int attempt = 0; attempt < 5 && !success; ++attempt) {
    const auto start = chrono::steady_clock::now();
    ::std::this_thread::sleep_until(start + kSleepTime);
    const auto end = chrono::steady_clock::now();
    last = end - start;
    success =
        last >= kSleepTime && last < kSleepTime + chrono::milliseconds(100);
  }
  EXPECT_TRUE(success)
      << "sleep_until duration "
      << chrono::duration_cast<chrono::milliseconds>(last).count() << "ms";
}

// Test to_timespec for a duration.
TEST(TimeTest, DurationToTimespec) {
  struct timespec pos_time = to_timespec(chrono::milliseconds(56262));
  EXPECT_EQ(pos_time.tv_sec, 56);
  EXPECT_EQ(pos_time.tv_nsec, 262000000);

  struct timespec neg_time = to_timespec(chrono::milliseconds(-56262));
  EXPECT_EQ(neg_time.tv_sec, -56);
  EXPECT_EQ(neg_time.tv_nsec, -262000000);
}

// Test to_timespec for a time_point.
TEST(TimeTest, TimePointToTimespec) {
  struct timespec pos_time =
      to_timespec(::aos::monotonic_clock::epoch() + chrono::seconds(1432423));
  EXPECT_EQ(pos_time.tv_sec, 1432423);
  EXPECT_EQ(pos_time.tv_nsec, 0);

  struct timespec neg_time =
      to_timespec(::aos::monotonic_clock::epoch() - chrono::seconds(1432423));
  EXPECT_EQ(neg_time.tv_sec, -1432423);
  EXPECT_EQ(neg_time.tv_nsec, 0);
}

// Tests from_timeval.
TEST(TimeTest, TimevalToTimePoint) {
  struct timeval pos_time;
  pos_time.tv_sec = 1432423;
  pos_time.tv_usec = 0;
  EXPECT_EQ(::aos::monotonic_clock::epoch() + chrono::seconds(1432423),
            from_timeval(pos_time));

  struct timeval neg_time;
  neg_time.tv_sec = -1432423;
  neg_time.tv_usec = 0;
  EXPECT_EQ(::aos::monotonic_clock::epoch() - chrono::seconds(1432423),
            from_timeval(neg_time));
}

// Test that << works with numbers with leading 0's.
TEST(TimeTest, OperatorStream) {
  const monotonic_clock::time_point t = monotonic_clock::epoch() +
                                        chrono::seconds(1432423) +
                                        chrono::milliseconds(15);

  // And confirm that the stream's settings are restored by adding a random
  // number afterwords.
  std::stringstream s;
  s << t << " and number " << 123;

  EXPECT_EQ(s.str(), "1432423.015000000sec and number 123");
}

// Test that << works with negative numbers.
TEST(TimeTest, OperatorStreamNegative) {
  {
    const monotonic_clock::time_point t = monotonic_clock::epoch() -
                                          chrono::seconds(14) +
                                          chrono::milliseconds(915);

    std::stringstream s;
    s << t;

    EXPECT_EQ(s.str(), "-13.085000000sec");
    EXPECT_EQ(monotonic_clock::FromString(s.str()).value(), t);
  }
  {
    const monotonic_clock::time_point t =
        monotonic_clock::epoch() - chrono::nanoseconds(1);

    std::stringstream s;
    s << t;

    EXPECT_EQ(s.str(), "-0.000000001sec");
    EXPECT_EQ(monotonic_clock::FromString(s.str()).value(), t);
  }
  {
    const monotonic_clock::time_point t =
        monotonic_clock::epoch() - chrono::seconds(1) - chrono::nanoseconds(1);

    std::stringstream s;
    s << t;

    EXPECT_EQ(s.str(), "-1.000000001sec");
    EXPECT_EQ(monotonic_clock::FromString(s.str()).value(), t);
  }
  {
    const monotonic_clock::time_point t =
        monotonic_clock::epoch() - chrono::seconds(2) - chrono::nanoseconds(1);

    std::stringstream s;
    s << t;

    EXPECT_EQ(s.str(), "-2.000000001sec");
    EXPECT_EQ(monotonic_clock::FromString(s.str()).value(), t);
  }
}

// Test that << works with min_time.
TEST(TimeTest, OperatorStreamMinTime) {
  const monotonic_clock::time_point t = monotonic_clock::min_time;

  std::stringstream s;
  s << t;

  EXPECT_EQ(s.str(), "-9223372036.854775808sec");
  EXPECT_EQ(monotonic_clock::FromString(s.str()).value(), t);
}

// Test that << works with the epoch on the realtime clock.
TEST(TimeTest, OperatorStreamRealtimeEpoch) {
  const realtime_clock::time_point t = realtime_clock::epoch();

  std::stringstream s;
  s << t;

  EXPECT_EQ(s.str(), "1970-01-01_00-00-00.000000000");
  EXPECT_EQ(realtime_clock::FromString(s.str()).value(), t);
}

// Test that << works with positive time on the realtime clock.
TEST(TimeTest, OperatorStreamRealtimePositive) {
  const realtime_clock::time_point t =
      realtime_clock::epoch() + std::chrono::hours(5 * 24) +
      std::chrono::seconds(11) + std::chrono::milliseconds(5);

  std::stringstream s;
  s << t;

  EXPECT_EQ(s.str(), "1970-01-06_00-00-11.005000000");
  EXPECT_EQ(realtime_clock::FromString(s.str()).value(), t);
}

// Test that << works with negative time on the realtime clock.
TEST(TimeTest, OperatorStreamRealtimeNegative) {
  {
    const realtime_clock::time_point t =
        realtime_clock::epoch() - std::chrono::nanoseconds(1);

    std::stringstream s;
    s << t;

    EXPECT_EQ(s.str(), "1969-12-31_23-59-59.999999999");
    EXPECT_EQ(realtime_clock::FromString(s.str()).value(), t);
  }
  {
    const realtime_clock::time_point t =
        realtime_clock::epoch() - std::chrono::nanoseconds(999999999);

    std::stringstream s;
    s << t;

    EXPECT_EQ(s.str(), "1969-12-31_23-59-59.000000001");
    EXPECT_EQ(realtime_clock::FromString(s.str()).value(), t);
  }
  {
    const realtime_clock::time_point t = realtime_clock::epoch() -
                                         std::chrono::seconds(1) -
                                         std::chrono::nanoseconds(999999999);

    std::stringstream s;
    s << t;

    EXPECT_EQ(s.str(), "1969-12-31_23-59-58.000000001");
    EXPECT_EQ(realtime_clock::FromString(s.str()).value(), t);
  }
  {
    const realtime_clock::time_point t =
        realtime_clock::epoch() - std::chrono::hours(5 * 24) +
        std::chrono::seconds(11) - std::chrono::milliseconds(5);

    std::stringstream s;
    s << t;

    EXPECT_EQ(s.str(), "1969-12-27_00-00-10.995000000");
    EXPECT_EQ(realtime_clock::FromString(s.str()).value(), t);
  }

  {
    const realtime_clock::time_point t = realtime_clock::min_time;

    std::stringstream s;
    s << t;

    // min_time happens to be unrepresentable because of rounding and signed
    // integer overflow.
    EXPECT_EQ(s.str(), "(unrepresentable realtime -9223372036854775808)");
  }

#ifdef __linux__
  {
    const realtime_clock::time_point t =
        realtime_clock::min_time + std::chrono::nanoseconds(999999999);

    std::stringstream s;
    s << t;

    EXPECT_EQ(s.str(), "1677-09-21_00-12-44.145224191");
    EXPECT_EQ(realtime_clock::FromString(s.str()).value(), t);
  }
#endif
}

// Test that ToString works for monotonic and realtime time points.
TEST(TimeTest, ToStringTimePoints) {
  EXPECT_EQ(ToString(realtime_clock::epoch() + std::chrono::hours(5 * 24) +
                     std::chrono::seconds(11) + std::chrono::milliseconds(5)),
            "1970-01-06_00-00-11.005000000");
  EXPECT_EQ(ToString(monotonic_clock::min_time), "-9223372036.854775808sec");
}

}  // namespace aos::time::testing

#ifdef __APPLE__
#include <mach/mach_time.h>
#endif
#include <benchmark/benchmark.h>

static void BM_MonotonicClockNow(benchmark::State &state) {
  for (auto _ : state) {
    benchmark::DoNotOptimize(aos::monotonic_clock::now());
  }
}
BENCHMARK(BM_MonotonicClockNow);

static void BM_RealtimeClockNow(benchmark::State &state) {
  for (auto _ : state) {
    benchmark::DoNotOptimize(aos::realtime_clock::now());
  }
}
BENCHMARK(BM_RealtimeClockNow);

#ifdef __APPLE__
static void BM_MachAbsoluteTime(benchmark::State &state) {
  for (auto _ : state) {
    benchmark::DoNotOptimize(mach_absolute_time());
  }
}
BENCHMARK(BM_MachAbsoluteTime);

// The three candidate sources for monotonic_clock::now() on Darwin.  The read
// costs quoted in the comment above monotonic_clock::now() in
// time_platform_osx.cc come from these.
static void BM_ClockGetTimeNsecNpUptimeRaw(benchmark::State &state) {
  for (auto _ : state) {
    benchmark::DoNotOptimize(clock_gettime_nsec_np(CLOCK_UPTIME_RAW));
  }
}
BENCHMARK(BM_ClockGetTimeNsecNpUptimeRaw);

static void BM_ClockGetTimeNsecNpMonotonicRaw(benchmark::State &state) {
  for (auto _ : state) {
    benchmark::DoNotOptimize(clock_gettime_nsec_np(CLOCK_MONOTONIC_RAW));
  }
}
BENCHMARK(BM_ClockGetTimeNsecNpMonotonicRaw);

static void BM_ClockGetTimeNsecNpMonotonic(benchmark::State &state) {
  for (auto _ : state) {
    benchmark::DoNotOptimize(clock_gettime_nsec_np(CLOCK_MONOTONIC));
  }
}
BENCHMARK(BM_ClockGetTimeNsecNpMonotonic);
#endif

#if !defined(_WIN32)
static void BM_ClockGetTimeMonotonic(benchmark::State &state) {
  for (auto _ : state) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    benchmark::DoNotOptimize(ts);
  }
}
BENCHMARK(BM_ClockGetTimeMonotonic);
#endif

// Benchmark of various clock APIs using the Google Benchmark library.
TEST(TimeTest, ClockBenchmark) {
  int argc = 1;
  char *argv[] = {const_cast<char *>("time_test")};
  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
}

// Concurrency test for monotonic_clock::now()
TEST(TimeTest, MonotonicClockConcurrency) {
  const int kNumThreads = 8;
  const int kIterations = 100000;
  std::atomic<uint64_t> max_time{0};
  std::atomic<bool> error{false};

  std::vector<std::thread> threads;
  for (int i = 0; i < kNumThreads; ++i) {
    threads.emplace_back([&]() {
      uint64_t local_last = 0;
      for (int j = 0; j < kIterations; ++j) {
        uint64_t now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                           aos::monotonic_clock::now().time_since_epoch())
                           .count();
        if (now <= local_last) {
          error.store(true);
        }
        local_last = now;

        uint64_t current_max = max_time.load(std::memory_order_relaxed);
        while (now > current_max) {
          if (max_time.compare_exchange_weak(current_max, now,
                                             std::memory_order_relaxed,
                                             std::memory_order_relaxed)) {
            break;
          }
        }
      }
    });
  }
  for (auto &t : threads) {
    t.join();
  }

  EXPECT_FALSE(error.load())
      << "Time was observed going backward or remaining equal in a thread!";
}

namespace {

struct GranularityResult {
  // Smallest non-zero step the clock was seen to take, i.e. its effective
  // granularity.
  uint64_t min_non_zero_diff_ns = std::numeric_limits<uint64_t>::max();
  // How many back-to-back reads returned the same value (or went backwards).
  uint64_t duplicate_reads = 0;
  uint64_t total_reads = 0;

  double duplicate_fraction() const {
    return total_reads == 0 ? 0.0
                            : static_cast<double>(duplicate_reads) /
                                  static_cast<double>(total_reads);
  }
};

// Reads `read_nanos` in a tight loop and reports both how finely it ticks and
// how often two adjacent reads land on the same value.  The duplicate rate is
// what a spinning now() implementation pays for: every duplicate is another
// trip around the spin loop.
template <typename Fn>
GranularityResult MeasureGranularity(Fn &&read_nanos, int iterations) {
  GranularityResult result;
  result.total_reads = iterations;
  uint64_t last = read_nanos();
  for (int i = 0; i < iterations; ++i) {
    const uint64_t current = read_nanos();
    if (current > last) {
      result.min_non_zero_diff_ns =
          std::min(result.min_non_zero_diff_ns, current - last);
    } else {
      ++result.duplicate_reads;
    }
    last = current;
  }
  return result;
}

void LogGranularity(std::string_view name, const GranularityResult &result) {
  LOG(INFO) << absl::StrFormat(
      "%-24s granularity = %6d ns   duplicates = %5.1f%%", name,
      result.min_non_zero_diff_ns, 100.0 * result.duplicate_fraction());
}

uint64_t AosMonotonicNanos() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             aos::monotonic_clock::now().time_since_epoch())
      .count();
}

uint64_t AosRealtimeNanos() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             aos::realtime_clock::now().time_since_epoch())
      .count();
}

}  // namespace

// Measures the effective granularity (min non-zero delta) and duplicate-read
// rate of each clock we could plausibly build monotonic_clock::now() on.
//
// On Darwin this is what justifies picking CLOCK_UPTIME_RAW over
// CLOCK_MONOTONIC: CLOCK_MONOTONIC is quantized to 1us, so it duplicates on the
// overwhelming majority of reads and would make now()'s spin loop far more
// expensive.  See the comment above monotonic_clock::now() in
// time_platform_osx.cc.
TEST(TimeTest, MonotonicClockGranularity) {
#ifdef __APPLE__
  mach_timebase_info_data_t info;
  ASSERT_EQ(mach_timebase_info(&info), KERN_SUCCESS);
  LOG(INFO) << "Mach Timebase Numerator:   " << info.numer;
  LOG(INFO) << "Mach Timebase Denominator: " << info.denom;
  LOG(INFO) << "Tick duration (numer/denom): "
            << static_cast<double>(info.numer) / info.denom << " ns";
#endif

  const int kIterations = 1000000;

  const GranularityResult monotonic =
      MeasureGranularity(AosMonotonicNanos, kIterations);
  LogGranularity("aos::monotonic_clock", monotonic);
  EXPECT_GT(monotonic.min_non_zero_diff_ns, 0U);
  // now() spins until the value changes, so it must never report a duplicate.
  EXPECT_EQ(monotonic.duplicate_reads, 0U);

  const GranularityResult realtime =
      MeasureGranularity(AosRealtimeNanos, kIterations);
  LogGranularity("aos::realtime_clock", realtime);
  EXPECT_GT(realtime.min_non_zero_diff_ns, 0U);

#ifdef __APPLE__
  // The underlying source now() is built on.
  const GranularityResult uptime_raw = MeasureGranularity(
      [] { return clock_gettime_nsec_np(CLOCK_UPTIME_RAW); }, kIterations);
  LogGranularity("CLOCK_UPTIME_RAW", uptime_raw);
  EXPECT_GT(uptime_raw.min_non_zero_diff_ns, 0U);

  // The two alternatives we rejected.
  const GranularityResult monotonic_raw = MeasureGranularity(
      [] { return clock_gettime_nsec_np(CLOCK_MONOTONIC_RAW); }, kIterations);
  LogGranularity("CLOCK_MONOTONIC_RAW", monotonic_raw);

  const GranularityResult posix_monotonic = MeasureGranularity(
      [] { return clock_gettime_nsec_np(CLOCK_MONOTONIC); }, kIterations);
  LogGranularity("CLOCK_MONOTONIC", posix_monotonic);

  // CLOCK_MONOTONIC is quantized far more coarsely than the hardware tick that
  // CLOCK_UPTIME_RAW exposes, which is half of why we don't use it.
  EXPECT_GT(posix_monotonic.min_non_zero_diff_ns,
            uptime_raw.min_non_zero_diff_ns)
      << "CLOCK_MONOTONIC is no longer coarser than CLOCK_UPTIME_RAW; the "
         "resolution half of the rationale in time_platform_osx.cc may be "
         "stale.";
#endif
}

#ifdef __APPLE__
// aos::monotonic_clock::now() has to stay in the Mach absolute timebase.  The
// Darwin timer paths (ToMachTicks() in //aos/events:aio_darwin and
// //aos/events:epoll_darwin) convert our time_points into Mach ticks using
// nothing but the mach_timebase_info ratio and hand them to EVFILT_TIMER with
// NOTE_MACHTIME | NOTE_ABSOLUTE, and sleep_until() does the same for
// mach_wait_until().  If now() ever moves to a clock on a different epoch,
// every one of those deadlines silently skews.
//
// This is the other half of the rationale in time_platform_osx.cc.
TEST(TimeTest, MonotonicClockUsesMachTimebase) {
  mach_timebase_info_data_t info;
  ASSERT_EQ(mach_timebase_info(&info), KERN_SUCCESS);

  const auto to_nanos = [&info](uint64_t ticks) {
    return static_cast<uint64_t>((absl::uint128(ticks) * info.numer) /
                                 info.denom);
  };

  // Bracket a now() call between two mach_absolute_time() reads.  If the two
  // share a timebase, now() has to land inside the bracket.  Preemption only
  // ever widens the bracket, so this can't flake.
  bool bracketed = false;
  uint64_t worst_skew_ns = 0;
  for (int i = 0; i < 1000 && !bracketed; ++i) {
    const uint64_t before_ns = to_nanos(mach_absolute_time());
    const uint64_t now_ns = AosMonotonicNanos();
    const uint64_t after_ns = to_nanos(mach_absolute_time());

    if (now_ns >= before_ns && now_ns <= after_ns) {
      bracketed = true;
    } else {
      worst_skew_ns =
          now_ns < before_ns ? before_ns - now_ns : now_ns - after_ns;
    }
  }
  EXPECT_TRUE(bracketed)
      << "aos::monotonic_clock::now() is not in the Mach absolute timebase "
         "(off by at least "
      << worst_skew_ns
      << " ns).  ToMachTicks() in aio_darwin.cc/epoll_darwin.cc and "
         "sleep_until()'s mach_wait_until() all assume it is.";

  // And show what going to CLOCK_MONOTONIC would have cost: it keeps counting
  // while the machine is asleep, so it runs ahead of Mach time by the total
  // accumulated sleep.  This is informational -- on a machine that has never
  // slept the gap is legitimately ~0.
  const uint64_t mach_ns = to_nanos(mach_absolute_time());
  const uint64_t posix_monotonic_ns = clock_gettime_nsec_np(CLOCK_MONOTONIC);
  LOG(INFO) << "CLOCK_MONOTONIC runs ahead of Mach absolute time by "
            << (posix_monotonic_ns > mach_ns ? posix_monotonic_ns - mach_ns : 0)
            << " ns (the machine's accumulated sleep time).  That is the error "
               "we would inject into every kqueue timer deadline by switching "
               "now() to it.";
}
#endif
