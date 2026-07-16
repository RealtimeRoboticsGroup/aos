#include "aos/util/application_name.h"

#include <errno.h>

#include <string>

namespace aos {

std::string GetProgramName() { return ::program_invocation_short_name; }

}  // namespace aos
