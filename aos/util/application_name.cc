#include "aos/util/application_name.h"

#include <string>

#include "absl/flags/flag.h"

ABSL_FLAG(std::string, application_name, aos::GetProgramName(),
          "The application name");
