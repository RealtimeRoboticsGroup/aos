#include "aos/libc/aos_strsignal.h"

#include <string.h>

#include <csignal>
#include <string>
#include <thread>

#include "gtest/gtest.h"

#include "aos/sanitizers.h"

namespace aos::libc::testing {

// Tries a couple of easy ones.
TEST(StrsignalTest, Basic) {
  EXPECT_STREQ("Hangup", aos_strsignal(SIGHUP));
  EXPECT_STREQ("Broken pipe", aos_strsignal(SIGPIPE));
#ifdef SIGRTMIN
  EXPECT_STREQ("Real-time signal 2", aos_strsignal(SIGRTMIN + 2));
#endif
  EXPECT_STREQ("Unknown signal 155", aos_strsignal(155));
}

// macOS strsignal appends ": <number>" to some descriptions. Strip it.
std::string NormalizeStrsignal(int signal) {
  const char *result = strsignal(signal);
  if (result == nullptr) return "";
#ifdef __APPLE__
  const char *colon = strrchr(result, ':');
  if (colon != nullptr && colon[1] == ' ') {
    // Check that everything after ": " is a digit
    const char *num = colon + 2;
    if (*num != '\0') {
      bool all_digits = true;
      for (const char *p = num; *p != '\0'; ++p) {
        if (!std::isdigit(static_cast<unsigned char>(*p))) {
          all_digits = false;
          break;
        }
      }
      if (all_digits) {
        std::string prefix(result, colon - result);
        // macOS "Unknown signal: N" needs to become "Unknown signal N"
        // to match aos_strsignal.
        if (prefix == "Unknown signal") {
          return "Unknown signal " + std::string(num);
        }
        return prefix;
      }
    }
  }
#endif
  return result;
}

class SignalNameTester {
 public:
  void operator()() {
#ifdef SIGRTMAX
    for (int i = 0; i < SIGRTMAX + 5; ++i) {
#else
    for (int i = 0; i < NSIG; ++i) {
#endif
      EXPECT_EQ(NormalizeStrsignal(i), aos_strsignal(i));
    }
  }
};

// msan doesn't seem to like strsignal().
#if !defined(AOS_SANITIZE_MEMORY)
// Tests that all the signals give the same result as strsignal(3).
TEST(StrsignalTest, All) {
  // Sigh, strsignal allocates a buffer that uses pthread local storage.  This
  // interacts poorly with asan.  Spawning a thread causes the storage to get
  // cleaned up before asan checks.
  SignalNameTester t;
#if defined(AOS_SANITIZER_thread)
  // tsan doesn't like this usage of ::std::thread. It looks like
  // <https://gcc.gnu.org/bugzilla/show_bug.cgi?id=57507>.
  t();
#else
  ::std::thread thread(::std::ref(t));
  thread.join();
#endif
}
#endif

}  // namespace aos::libc::testing
