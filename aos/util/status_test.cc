#include "aos/util/status.h"

#include <filesystem>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "aos/macros.h"
#include "aos/realtime.h"
#include "aos/testing/path.h"

namespace aos::testing {
class ErrorTest : public ::testing::Test {
 protected:
  ErrorTest() {}
};

// Tests that we can construct an errored status in realtime code.
TEST_F(ErrorTest, RealtimeError) {
  std::optional<Error> error;
  {
    aos::ScopedRealtime realtime;
    error = Error::MakeError("Hello, World!");
  }
  const int line = __LINE__ - 2;
  ASSERT_TRUE(error.has_value());
  EXPECT_NE(0, error->code());
  EXPECT_EQ(std::string("Hello, World!"), error->message());
  ASSERT_TRUE(error->source_location().has_value());
  EXPECT_EQ(
      std::string("status_test.cc"),
      std::filesystem::path(error->source_location()->file_name()).filename());
  EXPECT_EQ(
      std::string("virtual void "
                  "aos::testing::ErrorTest_RealtimeError_Test::TestBody()"),
      error->source_location()->function_name());
  EXPECT_EQ(line, error->source_location()->line());
  EXPECT_LT(1, error->source_location()->column());
  EXPECT_THAT(
      error->ToString(),
      ::testing::HasSubstr(absl::StrFormat(
          "status_test.cc:%d in virtual void "
          "aos::testing::ErrorTest_RealtimeError_Test::TestBody(): Errored "
          "with code of 1 and message: Hello, World!",
          line)));
}

// Tests that the ResultExitCode() function will correctly transform a Result<>
// object into an exit code suitable for exiting a program.
TEST_F(ErrorTest, ExitCode) {
  static_assert(0 == static_cast<int>(Error::StatusCode::kOk));
  EXPECT_EQ(static_cast<int>(Error::StatusCode::kOk),
            ResultExitCode(Result<void>{}));
  EXPECT_EQ(static_cast<int>(Error::StatusCode::kError),
            ResultExitCode(Error::MakeUnexpectedError("")));
}

// Malloc hooks don't work with asan/msan.
#if !__has_feature(address_sanitizer) && !__has_feature(memory_sanitizer)
// Tests that we do indeed malloc (and catch it) on an extra-long error message
// (this is mostly intended to ensure that the test setup is working correctly).
TEST(ErrorDeathTest, BlowsUpOnRealtimeAllocation) {
  std::string message(" ", Error::kStaticMessageLength + 1);
  EXPECT_DEATH(
      {
        aos::ScopedRealtime realtime;
        aos::CheckRealtime();
        Error foo = Error::MakeError(message);
      },
      "Malloced");
}

#endif

// Tests that we can use arbitrarily-sized string literals for error messages.
TEST_F(ErrorTest, StringLiteralError) {
  std::optional<Error> error;
  const char *message =
      "Hellllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllll"
      "llllllllllllllloooooooooooooooooooooooooooooooooooooooooooo, "
      "World!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
      "!!!!!!!!!!!!!!";
  ASSERT_LT(Error::kStaticMessageLength, strlen(message));
  {
    aos::ScopedRealtime realtime;
    error = Error::MakeStringLiteralError(message);
  }
  ASSERT_TRUE(error.has_value());
  EXPECT_EQ(message, error->message());
  ASSERT_TRUE(error->source_location().has_value());
  EXPECT_EQ(
      std::string("status_test.cc"),
      std::filesystem::path(error->source_location()->file_name()).filename());
}

// Tests that the CheckExpected() call works as intended.
TEST(ErrorDeathTest, CheckExpected) {
  tl::expected<int, Error> expected;
  expected.emplace(118);
  EXPECT_EQ(118, CheckExpected(expected))
      << "Should have gotten out the emplaced value on no error.";
  expected = Error::MakeUnexpectedError("Hello, World!");
  EXPECT_DEATH(CheckExpected(expected), "Hello, World!")
      << "An error message including the error string should have been printed "
         "on death.";
  EXPECT_DEATH(CheckExpected<void>(Error::MakeUnexpectedError("void expected")),
               "void expected")
      << "A void expected should work with CheckExpected().";
}

// Test struct that cannot be copied but which can be moved, to be used to
// ensure that the various Result macros do not induce extra copies.
struct DisallowCopy {
  DisallowCopy() {}
  DISALLOW_COPY_AND_ASSIGN(DisallowCopy);
  DisallowCopy(DisallowCopy &&) = default;
  DisallowCopy &operator=(DisallowCopy &&) = default;
};

TEST_F(ErrorTest, ReturnResultIfErrorNoExtraCopies) {
  Result<DisallowCopy> test_value = {};
  bool executed = false;
  const Result<void> result = [&test_value, &executed]() -> Result<void> {
    AOS_RETURN_IF_ERROR(test_value);
    executed = true;
    // next, confirm that we do actually return early on an unexpected.
    AOS_RETURN_IF_ERROR(
        Result<void>(Error::MakeUnexpectedError("Hello, World!")));
    return {};
  }();
  EXPECT_FALSE(result.has_value());
  EXPECT_TRUE(executed);
}

// Validates that the AOS_RETURN_IF_ERROR() macro can handle a temporary
// expression. When run under sanitizers this should also help to validate if
// the lifetime of any temporaries in AOS_RETURN_IF_ERROR are handled
// incorrectly.
TEST_F(ErrorTest, ReturnResultHandlesLifetime) {
  const Result<void> result = []() -> Result<void> {
    AOS_RETURN_IF_ERROR(
        Result<void>(Error::MakeUnexpectedError("Hello, World!")));
    return {};
  }();
  EXPECT_FALSE(result.has_value());
}

// Validates that we evaluate the expression passed to
// AOS_RETURN_IF_ERROR exactly one.
TEST_F(ErrorTest, ReturnResultEvaluatesOnce) {
  int counter = 0;
  const Result<void> result = [&counter]() -> Result<void> {
    AOS_RETURN_IF_ERROR([&counter]() -> Result<void> {
      counter++;
      return {};
    }());
    return {};
  }();
  EXPECT_TRUE(result.has_value());
  EXPECT_EQ(1, counter) << "The expression passed to AOS_RETURN_IF_ERROR "
                           "should have been evaluated exactly once.";
}

TEST_F(ErrorTest, DeclareVariableNoExtraCopies) {
  Result<DisallowCopy> test_value = {};
  bool executed = false;
  const Result<void> result = [&test_value, &executed]() -> Result<void> {
    AOS_DECLARE_OR_RETURN_IF_ERROR(expected, test_value);
    (void)expected;
    executed = true;
    // next, confirm that we do actually return early on an unexpected.
    AOS_DECLARE_OR_RETURN_IF_ERROR(
        never_reached,
        Result<DisallowCopy>(Error::MakeUnexpectedError("Hello, World!")));
    (void)never_reached;
    return {};
  }();
  EXPECT_FALSE(result.has_value());
  EXPECT_TRUE(executed);
}

// Validates that the AOS_DECLARE_OR_RETURN_IF_ERROR() macro can handle
// a temporary expression. When run under sanitizers this should also help to
// validate if the lifetime of any temporaries in
// AOS_DECLARE_OR_RETURN_IF_ERROR are handled incorrectly.
TEST_F(ErrorTest, DeclareVariableLifetime) {
  const Result<void> result = []() -> Result<void> {
    AOS_DECLARE_OR_RETURN_IF_ERROR(
        tmp, Result<int>(Error::MakeUnexpectedError("Hello, World!")));
    (void)tmp;
    return {};
  }();
  EXPECT_FALSE(result.has_value());
}

// Validates that we evaluate the expression passed to
// AOS_DECLARE_OR_RETURN_IF_ERROR exactly one.
TEST_F(ErrorTest, DeclareVariableEvaluatesOnce) {
  int counter = 0;
  const Result<void> result = [&counter]() -> Result<void> {
    AOS_DECLARE_OR_RETURN_IF_ERROR(tmp, [&counter]() -> Result<int> {
      counter++;
      return {};
    }());
    (void)tmp;
    return {};
  }();
  EXPECT_TRUE(result.has_value());
  EXPECT_EQ(1, counter) << "The expression passed to AOS_RETURN_IF_ERROR "
                           "should have been evaluated exactly once.";
}

TEST_F(ErrorTest, InitializeVariableNoExtraCopies) {
  bool executed = false;
  const Result<void> result = [&executed]() -> Result<void> {
    DisallowCopy tmp;
    AOS_GET_VALUE_OR_RETURN_ERROR(tmp, Result<DisallowCopy>{});
    executed = true;
    DisallowCopy tmp2;
    // next, confirm that we do actually return early on an unexpected.
    AOS_GET_VALUE_OR_RETURN_ERROR(
        tmp2,
        Result<DisallowCopy>(Error::MakeUnexpectedError("Hello, World!")));
    return {};
  }();
  EXPECT_FALSE(result.has_value());
  EXPECT_TRUE(executed);
}

// Validates that the AOS_GET_VALUE_OR_RETURN_ERROR() macro can
// handle a temporary expression. When run under sanitizers this should also
// help to validate if the lifetime of any temporaries in
// AOS_GET_VALUE_OR_RETURN_ERROR are handled incorrectly.
TEST_F(ErrorTest, InitializeVariableLifetime) {
  const Result<void> result = []() -> Result<void> {
    DisallowCopy tmp;
    AOS_GET_VALUE_OR_RETURN_ERROR(
        tmp, Result<DisallowCopy>(Error::MakeUnexpectedError("Hello, World!")));
    return {};
  }();
  EXPECT_FALSE(result.has_value());
}

// Validates that we evaluate the expression passed to
// AOS_GET_VALUE_OR_RETURN_ERROR exactly one.
TEST_F(ErrorTest, InitializeVariableEvaluatesOnce) {
  int counter = 0;
  const Result<void> result = [&counter]() -> Result<void> {
    int tmp;
    AOS_GET_VALUE_OR_RETURN_ERROR(tmp, [&counter]() -> Result<int> {
      counter++;
      return counter;
    }());
    EXPECT_EQ(tmp, counter);
    return {};
  }();
  EXPECT_TRUE(result.has_value());
  EXPECT_EQ(1, counter) << "The expression passed to AOS_RETURN_IF_ERROR "
                           "should have been evaluated exactly once.";
}
}  // namespace aos::testing
