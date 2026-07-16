#include "aos/util/application_name.h"

#include <stdlib.h>

#include <string>

namespace aos {

std::string GetProgramName() { return getprogname(); }

}  // namespace aos
