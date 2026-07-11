#include "aos/events/logging/file_operations.h"

#include <algorithm>
#include <ostream>

#include "absl/log/check.h"
#include "absl/log/log.h"
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
      VLOG(1) << "Ignoring " << filename << " with invalid extension.";
    } else {
      VLOG(1) << "Found log " << filename;
      files->emplace_back(File{
          .name = std::string(filename),
          .size = size,
      });
    }
  };
  std::filesystem::path base_path(filename_);
  if (std::filesystem::is_directory(base_path)) {
    VLOG(1) << "Searching in " << base_path;
    for (const auto &file :
         std::filesystem::recursive_directory_iterator(base_path)) {
      if (!file.is_regular_file()) {
        VLOG(1) << file << " is not file.";
        continue;
      }
      MaybeAddFile(file.path().generic_string(), file.file_size());
    }
  } else {
    MaybeAddFile(base_path.generic_string(),
                 std::filesystem::file_size(base_path));
  }
}

}  // namespace aos::logger::internal
