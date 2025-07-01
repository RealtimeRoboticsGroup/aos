#include <signal.h>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "gtest/gtest.h"

#include "aos/realtime.h"

namespace aos::testing {

namespace {

// A global function pointer purely for testing purposes when creating stack
// traces.
void (*g_function)() = nullptr;

class StacktraceTest : public ::testing::Test {
 public:
  StacktraceTest() {
    // Make sure that tests cannot accidentally use another test's helper
    // function.
    g_function = nullptr;
  }
};

using StacktraceDeathTest = StacktraceTest;

}  // namespace

// Create some (non-inlined) functions to create a predictable stack trace for
// the tests below.
__attribute__((optnone)) void function1() {
  if (g_function != nullptr) {
    g_function();
  }
}
__attribute__((optnone)) void function2() { function1(); }
__attribute__((optnone)) void function3() { function2(); }
__attribute__((optnone)) void function4() { function3(); }

// Tests that we get a useful stacktrace on a `LOG(FATAL)`. Also makes sure that
// we don't get a duplicate stack trace in the SIGABRT handler.
TEST(StacktraceDeathTest, StackTraceOnCrash) {
  g_function = [] { LOG(FATAL) << "Triggering death!"; };

  EXPECT_DEATH(
      { function4(); },
      ::testing::MatchesRegex(
          R"(F.... ..:..:......... .* stacktrace_test.cc:.*] Triggering death!
\*\*\* Check failure stack trace: \*\*\*
    @ .*  absl::.*::log_internal::LogMessage::SendToLog\(\)
    @ .*  absl::.*::log_internal::LogMessage::Flush\(\)
    @ .*  absl::.*::log_internal::LogMessageFatal::~LogMessageFatal\(\)
    @ .*  aos::testing::StacktraceDeathTest_StackTraceOnCrash_Test::TestBody\(\)::.*invoke\(\)
    @ .*  aos::testing::function1\(\)
    @ .*  aos::testing::function2\(\)
    @ .*  aos::testing::function3\(\)
    @ .*  aos::testing::function4\(\)
    @ .*  aos::testing::StacktraceDeathTest_StackTraceOnCrash_Test::TestBody\(\)
.*
\*\*\* SIGABRT received at time=[[:digit:]]+ on cpu [[:digit:]]+ \*\*\*
)"));
}

// Tests that we get a useful stacktrace on a segfault.
// TODO(philipp.schrader): Enable when we get stack unwinding in signal handlers
// working.
TEST(StacktraceDeathTest, DISABLED_StackTraceOnSegfault) {
  g_function = [] { CHECK_EQ(raise(SIGSEGV), 0); };

  EXPECT_DEATH(
      { function4(); },
      ::testing::MatchesRegex(
          R"(\*\*\* SIGSEGV received at time=[[:digit:]]+ on cpu [[:digit:]]+ \*\*\*
(libunwind: unsupported .eh_frame_hdr version: [[:digit:]]+ at [[:xdigit:]]+
)?PC: @ .*  .*
    @ .*  absl::.*::AbslFailureSignalHandler\(\)
    @ .*  .*
    @ .*  aos::testing::StacktraceDeathTest_StackTraceOnSegfault_Test::TestBody\(\)::.*invoke\(\)
    @ .*  aos::testing::function1\(\)
    @ .*  aos::testing::function2\(\)
    @ .*  aos::testing::function3\(\)
    @ .*  aos::testing::function4\(\)
    @ .*  aos::testing::StacktraceDeathTest_StackTraceOnSegfault_Test::TestBody\(\)
.*
)"));
}

// Tests that we get a useful stacktrace on a malloc.
// TODO(philipp.schrader): Enable when we get stack unwinding in signal handlers
// working.
TEST(StacktraceDeathTest, DISABLED_StackTraceOnMalloc) {
  g_function = [] {
    ScopedRealtime rt;
    volatile int *a = new int[3];
    *a = 5;
    EXPECT_EQ(*a, 5);
  };

  EXPECT_DEATH(
      { function4(); },
      ::testing::MatchesRegex(
          R"(\[realtime\.cc : [[:digit:]]+\] RAW: Malloced [[:digit:]]+ bytes
\*\*\* SIGABRT received at time=[[:digit:]]+ on cpu [[:digit:]]+ \*\*\*
(libunwind: unsupported .eh_frame_hdr version: [[:digit:]]+ at [[:xdigit:]]+
)?PC: @ .*  .*
    @ .*  absl::.*::AbslFailureSignalHandler\(\)
    @ .*  .*
    @ .*  aos::testing::StacktraceDeathTest_StackTraceOnMalloc_Test::TestBody\(\)::.*invoke\(\)
    @ .*  aos::testing::function1\(\)
    @ .*  aos::testing::function2\(\)
    @ .*  aos::testing::function3\(\)
    @ .*  aos::testing::function4\(\)
    @ .*  aos::testing::StacktraceDeathTest_StackTraceOnMalloc_Test::TestBody\(\)
.*
)"));
}

}  // namespace aos::testing
