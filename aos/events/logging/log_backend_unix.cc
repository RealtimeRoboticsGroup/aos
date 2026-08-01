#include "aos/events/logging/log_backend.h"

#include <fcntl.h>
#include <unistd.h>

#include <filesystem>
#include <string>

#include "absl/flags/declare.h"
#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"

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
    // Same reasoning as the failing close() below: a sync that ran out of
    // space leaves the data non-durable, so it can't come back as a clean
    // close.
    if (PlatformSync() == WriteCode::kOutOfSpace) {
      ran_out_of_space = true;
    }
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

  const bool dir_exists = DirectoryExists(current_directory);

  if (dir_exists) {
    const int result = rename(current_directory.c_str(), new_directory.c_str());
    if (result != 0) {
      // Out of space is the one condition the logger is built to ride out, so
      // it stays a reported failure: the base name is left alone and logging
      // carries on in the old directory until the writes themselves start
      // coming back kOutOfSpace and wind it down.
      //
      // Nothing else rename(2) can fail with here fixes itself.  They all say
      // this deployment is wrong rather than unlucky -- EXDEV for a new base
      // name on a different mount, EACCES or EROFS for a parent we can't write,
      // EEXIST or ENOTEMPTY for a target already sitting there -- and the
      // caller has no way to act on any of them (set_base_name() drops this
      // return value on the floor).
      PCHECK(errno == ENOSPC) << ": Unable to rename " << current_directory
                              << " to " << new_directory;
      PLOG(ERROR) << "Ran out of space renaming " << current_directory << " to "
                  << new_directory << "; logging continues at the old path";
      return false;
    }

    SyncParentDirectories(current_directory, new_directory);
  } else {
    // Neither name is on disk, so the directory being logged into has gone out
    // from under us -- this is not the "somebody already renamed it" case,
    // which is new_directory being present.  There is nothing left to rename
    // and nothing to keep logging into.
    CHECK(DirectoryExists(new_directory))
        << ": Old directory " << current_directory
        << " missing and new directory " << new_directory << " not present.";
  }
  old_base_name_ = base_name_;
  base_name_ = std::string(new_base_name);
  separator_ = BaseNameSeparator(base_name_);

  return true;
}

std::unique_ptr<RenamableFileBackend> MakeRenamableFileBackend(
    std::string_view base_name, bool supports_odirect) {
  return std::make_unique<RenamableFileBackend>(base_name, supports_odirect);
}

}  // namespace aos::logger
