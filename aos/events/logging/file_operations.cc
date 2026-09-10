#include "aos/events/logging/file_operations.h"

#include <algorithm>
#include <ostream>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "absl/strings/match.h"

namespace aos::logger::internal {

bool IsValidFilename(std::string_view filename) {
  return absl::EndsWith(filename, ".bfbs") ||
         absl::EndsWith(filename, ".bfbs.xz") ||
         absl::EndsWith(filename, ".bfbs.sz");
}

void LocalFileOperations::FindLogs(std::vector<File> *files) {
  auto MaybeAddFile = [&files](std::string_view filename, size_t size) {
    if (!IsValidFilename(filename)) {
      ABSL_VLOG(1) << "Ignoring " << filename << " with invalid extension.";
    } else {
      ABSL_VLOG(1) << "Found log " << filename;
      files->emplace_back(File{
          .name = std::string(filename),
          .size = size,
      });
    }
  };
  if (std::filesystem::is_directory(filename_)) {
    ABSL_VLOG(1) << "Searching in " << filename_;
    for (const auto &file :
         std::filesystem::recursive_directory_iterator(filename_)) {
      if (!file.is_regular_file()) {
        ABSL_VLOG(1) << file << " is not file.";
        continue;
      }
      // generic_string() so the reported names use '/' whichever separator the
      // caller's path and the iterator happened to use.  Callers match these
      // against their own base names, so the two have to agree.
      MaybeAddFile(file.path().generic_string(), file.file_size());
    }
  } else {
    MaybeAddFile(filename_.generic_string(),
                 std::filesystem::file_size(filename_));
  }
}

}  // namespace aos::logger::internal
