#include "aos/time/time.h"

#include <ctype.h>
#include <errno.h>
#if !defined(_WIN32)
#include <sys/time.h>
#endif  // !_WIN32

#include <algorithm>
#include <chrono>
#include <compare>
#include <cstdint>
#include <ctime>
#include <iomanip>
#include <limits>
#include <ratio>
#include <sstream>
#include <thread>

#include "absl/log/absl_check.h"
#include "absl/strings/numbers.h"

namespace chrono = ::std::chrono;

namespace {

void PrintToStream(std::ostream &stream, chrono::nanoseconds duration) {
  chrono::seconds seconds = chrono::duration_cast<chrono::seconds>(duration);
  if (duration < chrono::nanoseconds(0)) {
    stream << "-" << -seconds.count() << "." << std::setfill('0')
           << std::setw(9)
           << chrono::duration_cast<chrono::nanoseconds>(seconds - duration)
                  .count()
           << "sec";
  } else {
    stream << seconds.count() << "." << std::setfill('0') << std::setw(9)
           << chrono::duration_cast<chrono::nanoseconds>(duration - seconds)
                  .count()
           << "sec";
  }
}

}  // namespace

namespace aos {

std::ostream &operator<<(std::ostream &stream,
                         const aos::monotonic_clock::time_point &now) {
  PrintToStream(stream, now.time_since_epoch());
  return stream;
}

std::string ToString(const aos::monotonic_clock::time_point &now) {
  std::ostringstream stream;
  stream << now;
  return stream.str();
}

std::string ToString(const aos::realtime_clock::time_point &now) {
  std::ostringstream stream;
  stream << now;
  return stream.str();
}

#if !AOS_OS_NONE
std::optional<monotonic_clock::time_point> monotonic_clock::FromString(
    const std::string_view now) {
  // This should undo the operator << above.
  if (now.size() < 14) {
    return std::nullopt;
  }

  if (now.substr(now.size() - 3, now.size()) != "sec") {
    return std::nullopt;
  }

  if (now[now.size() - 13] != '.') {
    return std::nullopt;
  }

  bool negative = now[0] == '-';

  std::string_view sec(
      now.substr(negative ? 1 : 0, now.size() - (negative ? 14 : 13)));
  std::string_view nsec(now.substr(now.size() - 12, 9));

  if (!std::all_of(sec.begin(), sec.end(), ::isdigit) ||
      !std::all_of(nsec.begin(), nsec.end(), ::isdigit)) {
    return std::nullopt;
  }

  std::chrono::seconds::rep seconds_data;
  if (!absl::SimpleAtoi(sec, &seconds_data)) {
    return std::nullopt;
  }

  std::chrono::nanoseconds::rep nanoseconds_data;
  if (!absl::SimpleAtoi(nsec, &nanoseconds_data)) {
    return std::nullopt;
  }

  return monotonic_clock::time_point(
      std::chrono::seconds((negative ? -1 : 1) * seconds_data) +
      std::chrono::nanoseconds((negative ? -1 : 1) * nanoseconds_data));
}

std::optional<realtime_clock::time_point> realtime_clock::FromString(
    const std::string_view now) {
  // This should undo the operator << above.
  // Supporting format: 1970-01-06_00-00-11.005000000
  constexpr int kFormatLength = 29;

  if (now.size() < kFormatLength) {
    return std::nullopt;
  }

  if (now[kFormatLength - 10] != '.') {
    return std::nullopt;
  }

  std::string_view nsec(now.substr(kFormatLength - 9, 9));

  if (!std::all_of(nsec.begin(), nsec.end(), ::isdigit)) {
    return std::nullopt;
  }

  std::tm tm = {};
  std::istringstream ss(std::string(now.substr(0, kFormatLength - 10)));
  ss >> std::get_time(&tm, "%Y-%m-%d_%H-%M-%S");
  if (ss.fail()) {
    return std::nullopt;
  }

  std::chrono::year_month_day ymd(
      std::chrono::year{tm.tm_year + 1900},
      std::chrono::month{static_cast<unsigned>(tm.tm_mon + 1)},
      std::chrono::day{static_cast<unsigned>(tm.tm_mday)});
  if (!ymd.ok()) {
    return std::nullopt;
  }

  const std::chrono::sys_days days(ymd);
  const auto tod = std::chrono::hours{tm.tm_hour} +
                   std::chrono::minutes{tm.tm_min} +
                   std::chrono::seconds{tm.tm_sec};
  const auto sys_time = days + tod;

  std::chrono::nanoseconds::rep nanoseconds_data;
  if (!absl::SimpleAtoi(nsec, &nanoseconds_data)) {
    return std::nullopt;
  }

  return realtime_clock::time_point(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          sys_time.time_since_epoch()) +
      std::chrono::nanoseconds(nanoseconds_data));
}
#endif

std::ostream &operator<<(std::ostream &stream,
                         const aos::realtime_clock::time_point &now) {
  // Guard against overflowing when converting to nanoseconds.
  constexpr int64_t kScale = 1'000'000'000;
  const auto seconds =
      std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());
  const int64_t seconds_count = seconds.count();
  const bool overflow =
      (seconds_count > std::numeric_limits<int64_t>::max() / kScale) ||
      (seconds_count < std::numeric_limits<int64_t>::min() / kScale);
  if (overflow) {
    stream << "(unrepresentable realtime " << now.time_since_epoch().count()
           << ")";
    return stream;
  }

  if (now == realtime_clock::min_time) {
    stream << "(unrepresentable realtime " << now.time_since_epoch().count()
           << ")";
    return stream;
  }

  const int64_t total_ns = now.time_since_epoch().count();
  // Avoid undefined behavior.
  constexpr int64_t kNanosecondsPerDay =
      std::chrono::days::period::num * std::chrono::nanoseconds::period::den;
  int64_t days_count = total_ns / kNanosecondsPerDay;
  int64_t remainder_ns = total_ns % kNanosecondsPerDay;
  if (remainder_ns < 0) {
    remainder_ns += kNanosecondsPerDay;
    days_count -= 1;
  }

  const std::chrono::sys_days days{std::chrono::days{days_count}};
  const std::chrono::year_month_day ymd(days);

  if (!ymd.ok()) {
    stream << "(unrepresentable realtime " << now.time_since_epoch().count()
           << ")";
    return stream;
  }

  const std::chrono::hh_mm_ss<std::chrono::nanoseconds> hms{
      std::chrono::nanoseconds(remainder_ns)};

  stream << std::setfill('0') << std::setw(4) << static_cast<int>(ymd.year())
         << '-' << std::setw(2) << static_cast<unsigned>(ymd.month()) << '-'
         << std::setw(2) << static_cast<unsigned>(ymd.day()) << '_'
         << std::setw(2) << hms.hours().count() << '-' << std::setw(2)
         << hms.minutes().count() << '-' << std::setw(2)
         << hms.seconds().count() << '.' << std::setw(9)
         << std::chrono::duration_cast<std::chrono::nanoseconds>(
                hms.subseconds())
                .count();
  return stream;
}

namespace time {

struct timespec to_timespec(const ::aos::monotonic_clock::duration duration) {
  struct timespec time_timespec;
  ::std::chrono::seconds sec =
      ::std::chrono::duration_cast<::std::chrono::seconds>(duration);
  ::std::chrono::nanoseconds nsec =
      ::std::chrono::duration_cast<::std::chrono::nanoseconds>(duration - sec);
  time_timespec.tv_sec = sec.count();
  time_timespec.tv_nsec = nsec.count();
  return time_timespec;
}

struct timespec to_timespec(const ::aos::monotonic_clock::time_point time) {
  return to_timespec(time.time_since_epoch());
}

::aos::monotonic_clock::time_point from_timeval(struct timeval t) {
  return monotonic_clock::epoch() + std::chrono::seconds(t.tv_sec) +
         std::chrono::microseconds(t.tv_usec);
}

}  // namespace time

constexpr monotonic_clock::time_point monotonic_clock::min_time;
constexpr monotonic_clock::time_point monotonic_clock::max_time;
constexpr realtime_clock::time_point realtime_clock::min_time;
constexpr realtime_clock::time_point realtime_clock::max_time;

}  // namespace aos
