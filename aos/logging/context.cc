#include "aos/logging/context.h"

#include "aos/realtime.h"

#ifndef _GNU_SOURCE
#define _GNU_SOURCE /* See feature_test_macros(7) */
#endif

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <limits>
#include <optional>
#include <ostream>
#include <string>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/macros.h"
#include "aos/sanitizers.h"
#include "aos/util/application_name.h"

#if defined(AOS_SANITIZE_MEMORY)
#include <sanitizer/msan_interface.h>
#endif

namespace aos::logging::internal {
namespace {

// TODO(brians): Differentiate between threads with the same name in the same
// process.

::std::string GetMyName() {
  std::string process_name = aos::GetProgramName();
  std::string thread_name = aos::GetThreadName();

  if (thread_name.empty() ||
      strncmp(thread_name.c_str(), process_name.c_str(),
              ::std::min(thread_name.length(), process_name.length())) == 0) {
    // This thread doesn't have an actual name or it's the same as the process.
    return process_name;
  }

  return process_name + '.' + thread_name;
}

thread_local std::optional<Context> my_context;

// True if we're going to delete the current Context object ASAP. The
// reason for doing this instead of just deleting them is that tsan (at least)
// doesn't like it when pthread_atfork handlers do complicated stuff and it's
// not a great idea anyways.
thread_local bool delete_current_context(false);

}  // namespace

Context::Context() : sequence(0) {}

// Used in aos/linux_code/init.cc when a thread's name is changed.
void ReloadThreadName() {
  if (my_context.has_value()) {
    my_context->ClearName();
  }
}

void Context::ClearName() { name_size = std::numeric_limits<size_t>::max(); }

std::string_view Context::MyName() {
  if (name_size == std::numeric_limits<size_t>::max()) {
    ::std::string my_name = GetMyName();
    ABSL_CHECK_LE(my_name.size() + 1, sizeof(Context::name))
        << ": process/thread name '" << my_name << "' is too long";
    strcpy(name, my_name.c_str());
    name_size = my_name.size();
  }

  return std::string_view(&name[0], name_size);
}

Context *Context::Get() {
  if (AOS_UNLIKELY(delete_current_context)) {
    my_context.reset();
    delete_current_context = false;
  }
  if (AOS_UNLIKELY(!my_context.has_value())) {
    my_context.emplace();
    my_context->ClearName();
    my_context->source = aos::GetProcessId();
  }
  return &*my_context;
}

void Context::Delete() { delete_current_context = true; }

void Context::DeleteNow() {
  my_context.reset();
  delete_current_context = false;
}

}  // namespace aos::logging::internal
