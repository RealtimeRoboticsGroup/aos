#include "aos/init_stack_unwinder.h"

#include <algorithm>

#include "absl/debugging/stacktrace.h"
#include <libunwind.h>

namespace aos {
namespace {

// Taken from abseil-cpp's `absl/debugging/internal/stacktrace_generic-inl.inc`:
//
// > Sometimes, we can try to get a stack trace from within a stack
// > trace, because we don't block signals inside this code (which would be too
// > expensive: the two extra system calls per stack trace do matter here).
// > That can cause a self-deadlock.
// > Protect against such reentrant call by failing to get a stack trace.
//
// > We use __thread here because the code here is extremely low level -- it is
// > called while collecting stack traces from within malloc and mmap, and thus
// > can not call anything which might call malloc or mmap itself.
__thread int recursive = 0;

// This is a libunwind implementation of absl's stack unwinding helper. The
// implementation is based on the following pieces:
// * Abseil's `absl/debugging/internal/stacktrace_generic-inl.inc`, and
// * libunwind's example code from
// https://www.nongnu.org/libunwind/man/libunwind(3).html
int UnwindTheStack(void **result, int *sizes, int max_depth, int skip_count,
                   [[maybe_unused]] const void *ucp, int *min_dropped_frames) {
  // Don't print the stack trace while printing the stack trace.
  if (recursive) {
    return 0;
  }
  ++recursive;

  int result_count = 0;
  unw_cursor_t cursor;
  unw_context_t context;
  unw_word_t ip;

  // Initialize the cursor to the current frame for local unwinding
  unw_getcontext(&context);
  unw_init_local(&cursor, &context);

  // Actually unwind the stack now.
  while (unw_step(&cursor) > 0) {
    if (skip_count > 0) {
      // We're being asked to skip this frame. Do that.
      --skip_count;
      continue;
    }

    if (result_count < max_depth) {
      // The caller has room for information on this frame. Provide it.
      unw_get_reg(&cursor, UNW_REG_IP, &ip);
      result[result_count] = reinterpret_cast<void *>(ip);
    }
    ++result_count;
  }

  if (min_dropped_frames != nullptr) {
    // Tell the caller how many frames we could not report on.
    *min_dropped_frames = std::max(0, result_count - max_depth);
  }

  if (sizes != nullptr) {
    // No implementation for finding out the stack frame sizes yet.
    memset(sizes, 0, sizeof(*sizes) * static_cast<size_t>(result_count));
  }

  --recursive;

  return result_count;
}

}  // namespace

void InitStackUnwinder() { absl::SetStackUnwinder(&UnwindTheStack); }

}  // namespace aos
