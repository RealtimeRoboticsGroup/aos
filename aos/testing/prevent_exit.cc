#include "aos/testing/prevent_exit.h"

#ifndef _WIN32
#include <unistd.h>
#endif

#include <cstdlib>

#include "absl/log/check.h"
#include "absl/log/log.h"

namespace aos::testing {
namespace {

void TerminateExitHandler() { _exit(EXIT_SUCCESS); }

}  // namespace

void PreventExit() { CHECK_EQ(atexit(TerminateExitHandler), 0); }

}  // namespace aos::testing
