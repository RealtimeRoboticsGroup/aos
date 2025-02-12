#ifndef AOS_UTIL_STATUS_H_
#define AOS_UTIL_STATUS_H_
#include <optional>
#include <source_location>
#include <string_view>

#include "absl/log/log.h"
#include "absl/strings/str_format.h"
#include "tl/expected.hpp"

#include "aos/containers/inlined_vector.h"

namespace aos {
// The Error class provides a means by which errors can be readily returned
// from methods. It will typically be wrapped by an std::expected<> to
// accommodate a return value or the Error.
//
// The Error class is similar to the absl::Status or std::error_code classes,
// in that it consists of an integer error code of some sort (where 0 implicitly
// would indicate "ok", although we assume that if there is no error then
// you will be using an expected<> to return void or your actual return type)
// and a string error message of some sort. The main additions of this
// class are:
// 1. Adding a first-class exposure of an std::source_location to make exposure
//    of the sources of errors easier.
// 2. Providing an interface that allows for Error implementations that expose
//    messages without malloc'ing (not possible with absl::Status, although it
//    is possible with std::error_code).
// 3. Making it relatively easy to quickly return a simple error & message
//    (specifying a custom error with std::error_code is possible but requires
//    jumping through hoops and managing some global state).
// 4. Does not support an "okay" state, to make it clear that the user is
//    supposed to use a wrapper that will itself indicate okay.
//
// The goal of this class is that it should be easy to convert from existing
// error types (absl::Status, std::error_code) to this type.
//
// Users should typically use the Result<T> convenience method when returning
// Errors from methods. In the case where the method would normally return void,
// use Result<void>. Result<> is just a wrapper for tl::expected; when our
// compilers upgrade to support std::expected this should ease the transition,
// in addition to just providing a convenience wrapper to encourage a standard
// pattern of use.
class Error {
 public:
  // In order to allow simple error messages without memory allocation, we
  // reserve a small amount of stack space for error messages. This constant
  // specifies the length of these strings.
  static constexpr size_t kStaticMessageLength = 128;

  // Attaches human-readable status enums to integer codes---the specific
  // numeric codes are used as exit codes when terminating execution of the
  // program.
  // Note: While 0 will always indicate success and non-zero values will always
  // indicate failures we may attempt to further expand the set of non-zero exit
  // codes in the future and may decide to reuse 1 for a more specific error
  // code at the time (although it is reasonably likely that it will be kept as
  // a catch-all general error).
  enum class StatusCode : int {
    kOk = 0,
    kError = 1,
  };

  // Wraps an Error with an unexpected<> so that a Result<> may be constructed
  // from the Error.
  static tl::unexpected<Error> MakeUnexpected(const Error &error) {
    return tl::unexpected<Error>(error);
  }

  // Constructs an Error, copying the provided message. If the message is
  // shorter than kStaticMessageLength, then the message will be stored entirely
  // on the stack; longer messages will require dynamic memory allocation.
  // The default source_location will correspond to the call-site of the
  // Error::Error() method. This should only be overridden by wrappers that
  // want to present a fancier interface to users.
  static Error MakeError(
      std::string_view message,
      std::source_location source_location = std::source_location::current()) {
    return Error(StatusCode::kError, message, std::move(source_location));
  }
  static tl::unexpected<Error> MakeUnexpectedError(
      std::string_view message,
      std::source_location source_location = std::source_location::current()) {
    return MakeUnexpected(MakeError(message, std::move(source_location)));
  }

  // Constructs an error, retaining the provided pointer to a null-terminated
  // error message. It is assumed that the message pointer will stay valid
  // ~indefinitely. This is generally only appropriate to use with string
  // literals (e.g., Error::StringLiteralError("Hello, World!")).
  // The default source_location will correspond to the call-site of the
  // Error::Error() method. This should only be overridden by wrappers that
  // want to present a fancier interface to users.
  static Error MakeStringLiteralError(
      const char *message,
      std::source_location source_location = std::source_location::current()) {
    return Error(StatusCode::kError, message, std::move(source_location));
  }
  static tl::unexpected<Error> MakeUnexpectedStringLiteralError(
      const char *message,
      std::source_location source_location = std::source_location::current()) {
    return MakeUnexpected(
        MakeStringLiteralError(message, std::move(source_location)));
  }

  Error(Error &&other);
  Error &operator=(Error &&other);
  Error(const Error &other);

  // Returns a numeric value for the status code. Zero will always indicate
  // success; non-zero values will always indicate an error.
  [[nodiscard]] int code() const { return static_cast<int>(code_); }
  // Returns a view of the error message.
  [[nodiscard]] std::string_view message() const { return message_; }
  // Returns the source_location attached to the current Error. If the
  // source_location was never set, will return nullopt. The source_location
  // will typically be left unset for successful ("ok") statuses.
  [[nodiscard]] const std::optional<std::source_location> &source_location()
      const {
    return source_location_;
  }

  std::string ToString() const;

 private:
  Error(StatusCode code, std::string_view message,
        std::optional<std::source_location> source_location);
  Error(StatusCode code, const char *message,
        std::optional<std::source_location> source_location);

  StatusCode code_;
  aos::InlinedVector<char, kStaticMessageLength> owned_message_;
  std::string_view message_;
  std::optional<std::source_location> source_location_;
};

// Unfortunately, [[nodiscard]] does not work on using/typedef declarations, but
// if we change the type at all (e.g., creating a class that inherits from
// tl::expected) then converting between expected's and Results becomes a lot
// messier. In lieu of [[nodiscard]] being specified here, it is strongly
// advised the functions returning Result<>'s---especially those returning
// Result<void>---be marked [[nodiscard]].
template <typename T>
using Result = tl::expected<T, Error>;

// Dies fatally if the provided expected does not include the value T, printing
// out an error message that includes the Error on the way out.
// Returns the stored value on success.
template <typename T>
T CheckExpected(const Result<T> &expected) {
  if (expected.has_value()) {
    return expected.value();
  }
  LOG(FATAL) << expected.error().ToString();
}

template <>
void CheckExpected<void>(const Result<void> &expected);

int ResultExitCode(const Result<void> &expected);

inline std::ostream &operator<<(std::ostream &stream, const Error &error) {
  stream << error.ToString();
  return stream;
}

template <typename T>
std::ostream &operator<<(std::ostream &stream, const Result<T> &result) {
  if (result.has_value()) {
    stream << result.value();
  } else {
    stream << "<Error: " << result.error() << ">";
  }
  return stream;
}

inline std::ostream &operator<<(std::ostream &stream,
                                const Result<void> &result) {
  if (result.has_value()) {
    stream << "<void>";
  } else {
    stream << "<Error: " << result.error() << ">";
  }
  return stream;
}

// Takes an expression that evalutes to a Result<> and returns the error if
// there is one.
#define AOS_RETURN_IF_ERROR(result)                                           \
  {                                                                           \
    /* Ensure that we only evalute result once. (reference lifetime extension \
     * should prevent lifetime issues here). */                               \
    const auto &tmp = (result);                                               \
    if (!tmp.has_value()) {                                                   \
      return ::aos::Error::MakeUnexpected(tmp.error());                       \
    }                                                                         \
  }

// If expression evalutes to a type of Result<T> then a variable named variable
// of type const T& will be declared that is initialized to refer to the result
// of evaluating the expression. If expression evaluates to an error state,
// returns the error prior to initializing the variable.
#define AOS_DECLARE_OR_RETURN_IF_ERROR(variable, expression)                \
  /* Ensure that we only evalute result once. (reference lifetime extension \
   * should prevent lifetime issues here). */                               \
  const auto &variable##__tmp = (expression);                               \
  if (!variable##__tmp.has_value()) {                                       \
    return ::aos::Error::MakeUnexpected(variable##__tmp.error());           \
  }                                                                         \
  const auto &variable = variable##__tmp.value();

namespace internal {
// The below is a cutesy way to prevent
// AOS_GET_VALUE_OR_RETURN_ERROR() from being called with an lvalue
// expression. We do this because it may not be obvious to users whether
// AOS_GET_VALUE_OR_RETURN_ERROR() would attempt to copy the
// provided lvalue or move from it, and unlike
// AOS_DECLARE_OR_RETURN_IF_ERROR where we can declare the result
// variable as a reference in this case the result variable is generally going
// to own the returned value, and so must either copy or move from the input.
// When the input expression is a temporary this is easy---we can just always
// move, avoiding extra copies without anyone noticing. If the input is an
// lvalue (e.g., some other variable being used to store a Result<>), then
// choosing one or the other creates a footgun. There may be cleaner ways to
// enforce that the input expression be a temporary, but this wrapper
// conveniently fails to compile when passed an lvalue reference. If we did want
// to make AOS_GET_VALUE_OR_RETURN_ERROR support moving lvalue
// references into it then we could define a template <typename T> T
// &ForwardExpression(T &lvalue) { return lvalue; } overload and the below macro
// would work fine If, alternatively, we wanted to copy the input lvalue's, we
// could define a template <typename T> T ForwardExpression(T &lvalue) { return
// lvalue; } to achieve that result.
template <typename T>
T ForwardExpression(T &&rvalue) {
  return std::move(rvalue);
}
}  // namespace internal
// Performs a similar function to AOS_DECLARE_OR_RETURN_IF_ERROR,
// except that this assumes that variable is already declared and so just sets
// it to the result of the expression, if it is an expected value. Does not
// support taking an lvalue as the expression, as it is not intuitive whether
// this would result in a copy or a move of the provided lvalue, and most
// use-cases will take an rvalue of some sort.
#define AOS_GET_VALUE_OR_RETURN_ERROR(variable, expression)                  \
  /* Ensure that we only evalute result once. Note that we use decltype()    \
   * rather than auto to preserve any reference type returned by             \
   * ForwardExpression() (not currently relevant, but may become relevant if \
   * we support lvalues in (expression). */                                  \
  decltype(::aos::internal::ForwardExpression(expression)) variable##__tmp = \
      ::aos::internal::ForwardExpression(expression);                        \
  if (!variable##__tmp.has_value()) {                                        \
    return ::aos::Error::MakeUnexpected(variable##__tmp.error());            \
  }                                                                          \
  variable = std::move(variable##__tmp.value());
}  // namespace aos
#endif  // AOS_UTIL_STATUS_H_
