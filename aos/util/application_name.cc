#include "aos/util/application_name.h"

#include <string_view>

#include "absl/flags/flag.h"

namespace {

const char *Filename(const char *path) {
  const std::string_view path_string_view = path;
  auto last_slash_pos = path_string_view.find_last_of("/");

  return last_slash_pos == std::string_view::npos ? path
                                                  : path + last_slash_pos + 1;
}

}  // namespace

ABSL_FLAG(std::string, application_name, Filename(program_invocation_name),
          "The application name");
