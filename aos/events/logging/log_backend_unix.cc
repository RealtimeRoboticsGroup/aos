#include <fcntl.h>
#include <unistd.h>

#include <filesystem>
#include <string>

#include "absl/flags/declare.h"
#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"

#include "aos/events/logging/log_backend.h"
#include "aos/util/file.h"

ABSL_DECLARE_FLAG(bool, sync);

namespace aos::logger {

WriteCode FileHandler::OpenForWrite() {
  iovec_.reserve(10);
  if (!aos::util::MkdirPIfSpace(filename_, std::filesystem::perms::all,
                                absl::GetFlag(FLAGS_sync))) {
    return WriteCode::kOutOfSpace;
  } else {
    fd_ = open(filename_.c_str(), O_RDWR | O_CLOEXEC | O_CREAT | O_EXCL, 0774);
    if (fd_ == -1 && errno == ENOSPC) {
      return WriteCode::kOutOfSpace;
    } else {
      PCHECK(fd_ != -1) << ": Failed to open " << filename_ << " for writing";
      VLOG(1) << "Opened " << filename_ << " for writing";
    }

    flags_ = fcntl(fd_, F_GETFL, 0);
    PCHECK(flags_ >= 0) << ": Failed to get flags for " << filename_;

    EnableDirect();

    CHECK(std::filesystem::exists(filename_));

    return WriteCode::kOk;
  }
}

WriteCode FileHandler::Close() {
  if (!is_open()) {
    return WriteCode::kOk;
  }
  bool ran_out_of_space = false;

  if (absl::GetFlag(FLAGS_sync)) {
    PlatformSync();
  }

  if (close(fd_) == -1) {
    if (errno == ENOSPC) {
      ran_out_of_space = true;
    } else {
      PLOG(ERROR) << "Closing log file failed";
    }
  }
  if (absl::GetFlag(FLAGS_sync)) {
    aos::util::SyncDirectory(std::filesystem::path(filename_).parent_path());
  }
  fd_ = -1;
  VLOG(1) << "Closed " << filename_;
  return ran_out_of_space ? WriteCode::kOutOfSpace : WriteCode::kOk;
}

bool RenamableFileBackend::RenameLogBase(std::string_view new_base_name) {
  auto paths = ValidateAndSplitRenamePaths(new_base_name);
  if (!paths) {
    return true;
  }
  const auto &[current_directory, new_directory] = *paths;

  const bool dir_exists = std::filesystem::exists(current_directory);

  if (dir_exists) {
    const int result = rename(current_directory.c_str(), new_directory.c_str());
    if (result != 0) {
      PLOG(ERROR) << "Unable to rename " << current_directory << " to "
                  << new_directory;
      return false;
    }

    SyncParentDirectories(current_directory, new_directory);
  } else {
    // Handle if directory was already renamed.
    if (!std::filesystem::exists(new_directory)) {
      LOG(ERROR) << "Old directory " << current_directory
                 << " missing and new directory " << new_directory
                 << " not present.";
      return false;
    }
  }
  old_base_name_ = base_name_;
  base_name_ = std::string(new_base_name);
  separator_ = base_name_.back() == '/' ? "" : "_";

  return true;
}

std::unique_ptr<RenamableFileBackend> MakeRenamableFileBackend(
    std::string_view base_name, bool supports_odirect) {
  return std::make_unique<RenamableFileBackend>(base_name, supports_odirect);
}

}  // namespace aos::logger
