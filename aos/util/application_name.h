#ifndef AOS_UTIL_APPLICATION_NAME_H_
#define AOS_UTIL_APPLICATION_NAME_H_

#include <string>

#include "absl/flags/declare.h"

ABSL_DECLARE_FLAG(std::string, application_name);

namespace aos {

// Returns the name of the current process/program.
std::string GetProgramName();

}  // namespace aos

#endif  // AOS_UTIL_APPLICATION_NAME_H_
