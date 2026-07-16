#include "aos/util/application_name.h"

#include <windows.h>

#include <string>
#include <vector>

namespace aos {

std::string GetProgramName() {
  // GetModuleFileNameA returns how much it wrote, or the size of the buffer if
  // the path didn't fit.  In that second case what's in the buffer is a
  // truncated path -- and on older Windows it isn't even terminated -- so grow
  // and ask again rather than reading whatever is there.  A zero means it
  // failed outright and left the buffer alone.
  std::vector<char> exe_path(MAX_PATH);
  while (true) {
    const DWORD length = GetModuleFileNameA(
        nullptr, exe_path.data(), static_cast<DWORD>(exe_path.size()));
    if (length == 0) {
      return "";
    }
    if (length < exe_path.size()) {
      exe_path.resize(length);
      break;
    }
    exe_path.resize(exe_path.size() * 2);
  }

  const std::string path(exe_path.begin(), exe_path.end());
  const size_t last_slash = path.find_last_of('\\');
  return last_slash == std::string::npos ? path : path.substr(last_slash + 1);
}

}  // namespace aos
