#include <fcntl.h>
#include <io.h>
#include <sys/stat.h>

#include <chrono>
#include <numeric>
#include <thread>
#include <vector>

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
    // Match the Linux backend: use O_EXCL so OpenForWrite() always starts a
    // fresh file and fails loudly (via the PCHECK below) rather than silently
    // overwriting or appending to an existing one.  In normal operation log
    // names are unique, so an EEXIST here means a name collision or a bug, and
    // we'd rather crash than clobber a log.  (Appending to an existing file
    // after a directory rename is ReopenAndSeek(), which opens its own
    // descriptor below.)
    //
    // Note: if a Google Test death test ever needs to reopen a log the parent
    // still holds open (the child re-spawns the binary on Windows), it will
    // fail loudly here; add an explicit test-only overwrite override at that
    // point rather than relaxing this in normal operation.
    //
    // O_NOINHERIT is the O_CLOEXEC of the Unix backend: without it any process
    // spawned while this file is open (a death test re-execing the binary, as
    // above) inherits the descriptor and holds the log file open after we close
    // ours.  Windows will not rename a directory that still contains an open
    // file, so such a leaked handle burns every EACCES retry in
    // RenameLogBase() and fails the rotation.
    fd_ = open(filename_.c_str(),
               O_RDWR | O_CREAT | O_EXCL | O_BINARY | O_NOINHERIT, 0774);
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
  const auto start = aos::monotonic_clock::now();
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
          WriteStatistics()->UpdateDiskStats(
              aos::monotonic_clock::now() - start, total_bytes_written);
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
  // Stop the clock before flushing, like the Unix backend does, so the disk
  // stats keep measuring the writes themselves rather than the sync behind
  // them.
  const auto end = aos::monotonic_clock::now();

  if (absl::GetFlag(FLAGS_sync) && total_bytes_written > 0) {
    // Without this, --sync only reaches the disk in Close(), so a crash loses
    // everything written since the last close rather than the bounded amount
    // the Unix backend loses.
    //
    // Windows has no ranged writeback primitive -- no sync_file_range -- so the
    // Linux pipeline (queue writeback for this chunk, block on the previous
    // one, then drop its pages) has nothing to build on.  _commit() is
    // FlushFileBuffers(): the whole file, synchronously.  That makes this the
    // macOS F_FULLFSYNC path rather than the Linux one -- a blocking full flush
    // per write -- which costs throughput, but --sync is the flag that asks for
    // durability over throughput.
    if (_commit(fd_) == -1) {
      if (errno == ENOSPC) {
        // The data isn't durable, so report it the way a failed write is
        // reported.  Coming back kOk here would tell the logger this batch is
        // safely on disk when it isn't.
        WriteStatistics()->UpdateDiskStats(end - start, total_bytes_written);
        return {
            .code = WriteCode::kOutOfSpace,
            .messages_written = queue.size(),
            .bytes_written = total_bytes_written,
        };
      }
      PLOG(ERROR) << "Failed to _commit " << filename_;
    }
  }

  // written_aligned_ stays at zero: it counts bytes written through the
  // O_DIRECT aligned path, which this backend doesn't have.  last_synced_bytes_
  // likewise: it tracks how far the ranged writeback above got, and _commit()
  // has no range to track.
  WriteStatistics()->UpdateDiskStats(end - start, total_bytes_written);
  return {
      .code = WriteCode::kOk,
      .messages_written = queue.size(),
      .bytes_written = total_bytes_written,
  };
}

WindowsRenamableFileBackend::WindowsRenamableFileBackend(
    std::string_view base_name, bool supports_odirect)
    : RenamableFileBackend(base_name, supports_odirect) {}

WindowsRenamableFileBackend::~WindowsRenamableFileBackend() {
  // Every sink RequestFile() handed out has to be destroyed before its backend.
  // That isn't a Windows rule -- the shared RenamableFileHandler::Close()
  // reaches through owner_ just the same -- and MultiNodeLogNamer satisfies it
  // structurally, declaring log_backend_ ahead of the maps that own the
  // writers, so the writers are destroyed first.
  //
  // Catch a violation of it here, where the invariant actually breaks, rather
  // than letting a handler unregister itself through a destroyed
  // handlers_mutex_ afterwards.  A handler leaves active_handlers_ when it is
  // destroyed, so anything still in here has outlived us.
  std::unique_lock lock(handlers_mutex_);
  CHECK(active_handlers_.empty())
      << ": " << active_handlers_.size()
      << " log sink(s) outlived the backend that created them";
}

WindowsRenamableFileBackend::WindowsRenamableFileHandler::
    WindowsRenamableFileHandler(WindowsRenamableFileBackend *owner,
                                std::string filename, bool supports_odirect,
                                size_t memory_buffer_size)
    : BufferedFileHandler(std::move(filename), supports_odirect,
                          memory_buffer_size),
      owner_(owner) {
  // Registration is RequestFile()'s job, not ours -- see RegisterHandlerLocked.
}

WindowsRenamableFileBackend::WindowsRenamableFileHandler::
    ~WindowsRenamableFileHandler() {
  // owner_ outlives us -- see ~WindowsRenamableFileBackend, which CHECKs it.
  owner_->CloseAndUnregisterHandler(this);
}

void WindowsRenamableFileBackend::RegisterHandlerLocked(
    WindowsRenamableFileHandler *handler) {
  active_handlers_.push_back(handler);
}

void WindowsRenamableFileBackend::CloseAndUnregisterHandler(
    WindowsRenamableFileHandler *handler) {
  std::unique_lock lock(handlers_mutex_);

  // Close while still registered, and under the lock.  Our descriptor isn't
  // closed until ~BufferedFileHandler, which runs after the destructor body
  // that called us -- so dropping out of active_handlers_ first would leave an
  // open file no concurrent RenameLogBase() could find to close, and Windows
  // will not rename a directory that still has one inside it.  Closing outside
  // the lock instead would just race that rename's own close or reopen of this
  // same pointer.
  //
  // Qualified so this matches what the base destructor would have done; going
  // through the override would additionally rename the temp file, which
  // destruction has never done.
  handler->BufferedFileHandler::Close();
  std::erase(active_handlers_, handler);
}

std::unique_ptr<LogSink> WindowsRenamableFileBackend::RequestFile(
    const std::string_view id, const size_t memory_buffer_size) {
  // Hold the lock across naming *and* registering.  base_name_ is what a rename
  // mutates, so reading it outside the lock both races that mutation and lets a
  // rename slip in afterwards -- leaving us to register a handler pointing into
  // the directory that just moved, and to recreate it there.
  std::unique_lock lock(handlers_mutex_);
  const std::string filename =
      absl::StrCat(base_name_, separator_, id, temp_suffix_);
  auto handler = std::make_unique<WindowsRenamableFileHandler>(
      this, filename, supports_odirect_, memory_buffer_size);
  RegisterHandlerLocked(handler.get());
  return handler;
}

bool WindowsRenamableFileBackend::RenameLogBase(
    std::string_view new_base_name) {
  auto paths = ValidateAndSplitRenamePaths(new_base_name);
  if (!paths) {
    return true;
  }
  const auto &[current_directory, new_directory] = *paths;

  // Hold handlers_mutex_ across the whole rename.  reopen_handlers below holds
  // raw pointers into active_handlers_, and a handler destroyed while the lock
  // was dropped (the retry loop can sleep for a while) would leave a dangling
  // pointer behind for us to reopen.  Renames happen once per rotation, so
  // blocking handler creation and destruction for the duration is cheap.
  std::unique_lock lock(handlers_mutex_);

  // Work out what we're going to do *before* closing anything, so we die here
  // rather than after tearing the handlers down.  Neither name being on disk
  // means the directory being logged into has gone out from under us -- not the
  // "somebody already renamed it" case, which is new_directory being present --
  // and there is nothing left to rename or to keep logging into.  Closing first
  // would additionally leave every handler shut with no file to reopen against
  // (see ReopenAndSeek) and nothing draining its buffer.
  const bool dir_exists = DirectoryExists(current_directory);
  CHECK(dir_exists || DirectoryExists(new_directory))
      << ": Old directory " << current_directory
      << " missing and new directory " << new_directory << " not present.";

  // Windows file locking strictly prevents renaming a directory if any files
  // inside it are currently open. All of this backend's handlers live under
  // current_directory (that's where base_name_ points), so closing this
  // backend's own active_handlers_ is sufficient; no other backend can have
  // files open in this directory.
  //
  // Remember exactly which handlers we closed so we can reopen just those.
  // Reopening a handler that wasn't open would spuriously create its (empty)
  // log file, and worse, leave a BufferedFileHandler in an
  // opened-behind-its-own-back state so its real OpenForWrite() later fails.
  std::vector<WindowsRenamableFileHandler *> reopen_handlers;
  bool flush_ran_out_of_space = false;
  for (auto *handler : active_handlers_) {
    if (handler->is_open()) {
      // Closing flushes whatever the writing thread still had buffered.
      if (handler->BufferedFileHandler::Close() == WriteCode::kOutOfSpace) {
        flush_ran_out_of_space = true;
      }
      reopen_handlers.push_back(handler);
    }
  }

  // The disk filled up while flushing, so this log already has a hole in it and
  // nothing we write from here on would be trustworthy.  Stop instead of trying
  // to carry on: leave the handlers closed so whatever reached disk stays
  // intact, and retire them so later writes report kOutOfSpace and the logger
  // winds down through its usual out-of-space path rather than writing to a
  // closed descriptor.
  //
  // Don't rename either.  Renaming is how a log gets promoted to its final
  // name, and there's nothing to gain by promoting a truncated one.  The rename
  // is also one-shot (ValidateAndSplitRenamePaths CHECKs old_base_name_ is
  // empty), so spending it here would turn a later, legitimate rename into a
  // crash.  Nothing moved, so report exactly that.
  if (flush_ran_out_of_space) {
    LOG(ERROR) << "Ran out of space flushing buffered log data; not renaming "
               << current_directory << " to " << new_directory
               << ".  Logging to it stops here.";
    for (auto *handler : reopen_handlers) {
      handler->StopForOutOfSpace();
    }
    return false;
  }

  if (dir_exists) {
    int result = -1;
    int rename_errno = 0;
    // Windows file locks may take a short time to release after file handles
    // are closed. Try renaming multiple times with a small delay.
    for (int retry = 0; retry < 20; ++retry) {
      result = rename(current_directory.c_str(), new_directory.c_str());
      if (result == 0) {
        break;
      }
      // Captured because sleep_for() below is free to clobber errno, and this
      // is the errno everything after the loop reports on.
      rename_errno = errno;
      if (rename_errno != EACCES && rename_errno != EEXIST) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (result != 0) {
      errno = rename_errno;
      // EACCES and EEXIST are what the retry loop is for, and outlasting it
      // doesn't make them permanent: something outside this process can hold a
      // handle in the directory for longer than we wait -- a virus scanner
      // reading the log we just closed is the usual one -- and it will let go.
      // Logging survives that, since we reopen at the old path just below and
      // nothing has consumed the one-shot rename, so the caller can try again.
      // Out of space is recoverable in the usual way too: writes wind the
      // logger down through kOutOfSpace.
      //
      // Anything else left the loop on its first pass and is a deployment that
      // is wrong rather than unlucky -- a new base name on another volume, an
      // unwritable parent -- with no caller able to act on it either, since
      // set_base_name() drops this return value on the floor.
      PCHECK(rename_errno == EACCES || rename_errno == EEXIST ||
             rename_errno == ENOSPC)
          << ": Unable to rename " << current_directory << " to "
          << new_directory;
      PLOG(ERROR) << "Unable to rename " << current_directory << " to "
                  << new_directory << "; logging continues at the old path";
      // Reopen the files we closed at the original path if the rename failed.
      for (auto *handler : reopen_handlers) {
        handler->ReopenAndSeek();
      }
      return false;
    }

    SyncParentDirectories(current_directory, new_directory);
  }

  // Update base name and reopen handlers.  Every handler's path moved with the
  // directory, so update all of their filenames; but only reopen the ones that
  // were actually open before the rename -- the rest stay closed and will open
  // fresh at the new path on their next OpenForWrite().
  old_base_name_ = base_name_;
  base_name_.replace(0, current_directory.length(), new_directory);
  separator_ = base_name_.back() == '/' ? "" : "_";
  for (auto *handler : active_handlers_) {
    handler->UpdateFilename(current_directory, new_directory);
  }
  for (auto *handler : reopen_handlers) {
    handler->ReopenAndSeek();
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
  // Reopen the existing (already-written) file to continue appending after a
  // rename.  We must NOT go through OpenForWrite() here: it starts a fresh file
  // (O_EXCL/O_TRUNC), which would discard everything logged so far.
  //
  // No O_CREAT: this file was just renamed into place (or never moved, on the
  // rename-failure recovery path), so it must already exist -- if it doesn't,
  // that's a bug and we want to fail loudly rather than silently create an
  // empty stand-in and lose the append point.
  //
  // O_NOINHERIT for the same reason as OpenForWrite(): an inherited descriptor
  // outlives our close and blocks the next directory rename.
  fd_ = open(filename_.c_str(), O_RDWR | O_BINARY | O_NOINHERIT);
  PCHECK(fd_ != -1) << ": Failed to reopen " << filename_ << " after rename";
  flags_ = 0;
  _lseeki64(fd_, 0, SEEK_END);

  // Close() stopped the disk-writing thread.  Restart it now that the
  // descriptor is valid again, otherwise a buffered handler would keep
  // accepting writes into a buffer nothing ever drains -- silently retaining
  // the data until the buffer fills, and then blocking the writer forever.
  StartWriterThread();
}

WriteCode FileHandler::Close() {
  if (!is_open()) {
    return WriteCode::kOk;
  }
  bool ran_out_of_space = false;

  if (absl::GetFlag(FLAGS_sync)) {
    // Force everything out so we know it hit disk.  A failure here means the
    // data isn't durable, so it has to be reported the same way a failing
    // _close() is -- otherwise running out of space while flushing would look
    // like a clean close.
    if (_commit(fd_) == -1) {
      if (errno == ENOSPC) {
        ran_out_of_space = true;
      } else {
        PLOG(ERROR) << "Failed to _commit " << filename_;
      }
    }
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
