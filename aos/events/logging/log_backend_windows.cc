#include <fcntl.h>
#include <io.h>
#include <sys/stat.h>

#include <chrono>
#include <numeric>
#include <thread>

#include "absl/flags/declare.h"
#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/strings/str_cat.h"

#include "aos/events/logging/log_backend.h"
#include "aos/util/file.h"

ABSL_DECLARE_FLAG(bool, sync);

namespace aos::logger {

WriteCode FileHandler::OpenForWrite() {
  if (!aos::util::MkdirPIfSpace(filename_, std::filesystem::perms::all,
                                absl::GetFlag(FLAGS_sync))) {
    return WriteCode::kOutOfSpace;
  } else {
    // On Windows, Google Test death tests are implemented by re-spawning the
    // binary and re-running the test setup from scratch. Because the parent
    // process still holds the log file open, the child's attempt to clean up
    // and delete the file via unlink() fails due to Windows file locks. Omit
    // O_EXCL on Windows so that the child process can successfully
    // open/overwrite the file instead of failing with EEXIST (File exists).
    fd_ = open(filename_.c_str(), O_RDWR | O_CREAT | O_BINARY, 0774);
    if (fd_ == -1 && errno == ENOSPC) {
      return WriteCode::kOutOfSpace;
    } else {
      PCHECK(fd_ != -1) << ": Failed to open " << filename_ << " for writing";
      VLOG(1) << "Opened " << filename_ << " for writing";
    }

    flags_ = 0;

    EnableDirect();

    CHECK(std::filesystem::exists(filename_));

    return WriteCode::kOk;
  }
}

WriteResult FileHandler::DoWrite(
    const absl::Span<const absl::Span<const uint8_t>> &queue) {
  // On Windows, we implement a simple, standard-compliant sequential write loop
  // instead of POSIX writev or high-performance Linux O_DIRECT/alignment
  // splitting. This bypasses alignment constraints and complex O_DIRECT logic
  // (which are Linux-only features) and avoids compatibility shims. We write in
  // chunks up to 2GB to avoid potential 32-bit overflow issues in UCRT
  // _write().
  size_t total_bytes_written = 0;
  for (const auto &item : queue) {
    if (item.empty()) continue;
    size_t bytes_to_write = item.size();
    const uint8_t *data_ptr = item.data();
    while (bytes_to_write > 0) {
      unsigned int chunk = static_cast<unsigned int>(
          std::min<size_t>(bytes_to_write, 0x7FFFFFFF));
      int written = _write(fd_, data_ptr, chunk);
      if (written == -1) {
        if (errno == ENOSPC) {
          return {
              .code = WriteCode::kOutOfSpace,
              .messages_written = queue.size(),
              .bytes_written = total_bytes_written,
          };
        }
        PLOG(FATAL) << "Write failed for " << filename_;
      }
      total_bytes_written += written;
      total_write_bytes_ += written;
      bytes_to_write -= written;
      data_ptr += written;
    }
  }
  return {
      .code = WriteCode::kOk,
      .messages_written = queue.size(),
      .bytes_written = total_bytes_written,
  };
}

namespace {
std::mutex &GetBackendsMutex() {
  static std::mutex mutex;
  return mutex;
}

std::vector<WindowsRenamableFileBackend *> &GetActiveBackends() {
  static std::vector<WindowsRenamableFileBackend *> backends;
  return backends;
}
}  // namespace

WindowsRenamableFileBackend::WindowsRenamableFileBackend(
    std::string_view base_name, bool supports_odirect)
    : RenamableFileBackend(base_name, supports_odirect) {
  std::unique_lock lock(GetBackendsMutex());
  GetActiveBackends().push_back(this);
}

WindowsRenamableFileBackend::~WindowsRenamableFileBackend() {
  std::unique_lock lock(GetBackendsMutex());
  std::erase(GetActiveBackends(), this);
}

WindowsRenamableFileBackend::WindowsRenamableFileHandler::
    WindowsRenamableFileHandler(WindowsRenamableFileBackend *owner,
                                std::string filename, bool supports_odirect,
                                size_t memory_buffer_size)
    : BufferedFileHandler(std::move(filename), supports_odirect,
                          memory_buffer_size),
      owner_(owner) {
  owner_->RegisterHandler(this);
}

WindowsRenamableFileBackend::WindowsRenamableFileHandler::
    ~WindowsRenamableFileHandler() {
  owner_->UnregisterHandler(this);
}

void WindowsRenamableFileBackend::RegisterHandler(
    WindowsRenamableFileHandler *handler) {
  std::unique_lock lock(handlers_mutex_);
  active_handlers_.push_back(handler);
}

void WindowsRenamableFileBackend::UnregisterHandler(
    WindowsRenamableFileHandler *handler) {
  std::unique_lock lock(handlers_mutex_);
  std::erase(active_handlers_, handler);
}

std::unique_ptr<LogSink> WindowsRenamableFileBackend::RequestFile(
    const std::string_view id, const size_t memory_buffer_size) {
  const std::string filename =
      absl::StrCat(base_name_, separator_, id, temp_suffix_);
  return std::make_unique<WindowsRenamableFileHandler>(
      this, filename, supports_odirect_, memory_buffer_size);
}

bool WindowsRenamableFileBackend::RenameLogBase(
    std::string_view new_base_name) {
  auto paths = ValidateAndSplitRenamePaths(new_base_name);
  if (!paths) {
    return true;
  }
  const auto &[current_directory, new_directory] = *paths;

  // Windows file locking strictly prevents renaming a directory if any files
  // inside it are currently open. In a multi-node test environment, multiple
  // backend instances might have files open in the same directory.
  // We close all active handlers across ALL backends.
  {
    std::unique_lock lock(GetBackendsMutex());
    for (auto *backend : GetActiveBackends()) {
      std::unique_lock h_lock(backend->handlers_mutex_);
      for (auto *handler : backend->active_handlers_) {
        if (handler->is_open()) {
          handler->BufferedFileHandler::Close();
        }
      }
    }
  }

  const bool dir_exists = std::filesystem::exists(current_directory);
  if (dir_exists) {
    int result = -1;
    // Windows file locks may take a short time to release after file handles
    // are closed. Try renaming multiple times with a small delay.
    for (int retry = 0; retry < 20; ++retry) {
      result = rename(current_directory.c_str(), new_directory.c_str());
      if (result == 0) {
        break;
      }
      if (errno != EACCES && errno != EEXIST) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (result != 0) {
      PLOG(ERROR) << "Unable to rename " << current_directory << " to "
                  << new_directory;
      // Reopen the files at the original path if the rename failed.
      {
        std::unique_lock lock(GetBackendsMutex());
        for (auto *backend : GetActiveBackends()) {
          std::unique_lock h_lock(backend->handlers_mutex_);
          for (auto *handler : backend->active_handlers_) {
            handler->ReopenAndSeek();
          }
        }
      }
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

  // Update base name and reopen active handlers for all affected backends.
  {
    std::unique_lock lock(GetBackendsMutex());
    for (auto *backend : GetActiveBackends()) {
      std::unique_lock h_lock(backend->handlers_mutex_);
      if (backend->base_name_.starts_with(current_directory)) {
        backend->old_base_name_ = backend->base_name_;
        backend->base_name_.replace(0, current_directory.length(),
                                    new_directory);
        backend->separator_ = backend->base_name_.back() == '/' ? "" : "_";
      }
      for (auto *handler : backend->active_handlers_) {
        if (handler->name().starts_with(current_directory)) {
          handler->UpdateFilename(current_directory, new_directory);
        }
        handler->ReopenAndSeek();
      }
    }
  }

  return true;
}

bool RenamableFileBackend::RenameLogBase(std::string_view /*new_base_name*/) {
  LOG(FATAL) << "RenameLogBase should be called on WindowsRenamableFileBackend "
                "on Windows";
  return false;
}

WriteCode WindowsRenamableFileBackend::WindowsRenamableFileHandler::Close() {
  if (!is_open()) {
    return WriteCode::kOk;
  }

  // If the base directory has been renamed, update the filename to point to the
  // new location.
  if (owner_->was_renamed()) {
    UpdateFilenameBase(owner_->old_base_name_, owner_->base_name_);
  }

  // Continue with the standard close logic.
  if (BufferedFileHandler::Close() == WriteCode::kOutOfSpace) {
    return WriteCode::kOutOfSpace;
  }
  if (owner_->RenameFileAfterClose(filename()) == WriteCode::kOutOfSpace) {
    return WriteCode::kOutOfSpace;
  }
  return WriteCode::kOk;
}

void WindowsRenamableFileBackend::WindowsRenamableFileHandler::ReopenAndSeek() {
  OpenForWrite();
  if (fd_ != -1) {
    _lseeki64(fd_, 0, SEEK_END);
  }
}

WriteCode FileHandler::Close() {
  if (!is_open()) {
    return WriteCode::kOk;
  }
  bool ran_out_of_space = false;

  if (absl::GetFlag(FLAGS_sync)) {
    _commit(fd_);
  }

  if (_close(fd_) == -1) {
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

void FileHandler::EnableDirect() {}
void FileHandler::DisableDirect() {}

std::pair<WriteCode, size_t> FileHandler::WriteV(bool) {
  LOG(FATAL) << "WriteV not implemented on Windows";
  return {WriteCode::kOk, 0};
}

std::unique_ptr<RenamableFileBackend> MakeRenamableFileBackend(
    std::string_view base_name, bool supports_odirect) {
  return std::make_unique<WindowsRenamableFileBackend>(base_name,
                                                       supports_odirect);
}

}  // namespace aos::logger
