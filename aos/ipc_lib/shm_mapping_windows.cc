#include "aos/ipc_lib/shm_mapping.h"

#include <windows.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <thread>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/util/file.h"

namespace aos::ipc_lib {

long SystemPageSize() {
  SYSTEM_INFO sys_info;
  GetSystemInfo(&sys_info);
  return sys_info.dwPageSize;
}

// Why a real file, rather than the more obvious page-file-backed section
// (CreateFileMapping() with INVALID_HANDLE_VALUE and a name)?  Lifetime.  A
// named section is refcounted, and its contents are gone the moment the last
// handle to it closes, with no way to opt out of that.  Shared memory here has
// to survive every process that mapped it exiting, since the next process to
// attach expects to find the queue as it was left (see shm_mapping.h).
//
// That is the only thing wrong with sections, and it is decisive.  They would
// otherwise fit better than a file does: they are documented to start out
// zero-initialized, they are sized atomically at creation -- which would retire
// both the "wait for the creator to size it" handshake below and the ordering
// hazard it exists to cover -- and they never reach storage at all.
// FILE_ATTRIBUTE_TEMPORARY below buys back most of that last point.
WritableShmMapping::WritableShmMapping(std::string_view path, size_t size,
                                       std::filesystem::perms permissions)
    : size_(size) {
  const long kSystemPageSize = SystemPageSize();

  std::string path_str(path);
  util::MkdirP(path_str, permissions);

  // FILE_ATTRIBUTE_TEMPORARY tells the cache manager to hold the contents in
  // memory and avoid writing them back to storage as long as there is cache to
  // hold them -- as close as Windows gets to Linux backing these with tmpfs.
  // It is only a writeback hint: the file still exists, and its contents still
  // outlive every process that mapped it, which the queue protocol depends on.
  // Deliberately not FILE_FLAG_DELETE_ON_CLOSE, which would take the queue with
  // it as soon as the last mapping went away.  Attributes apply only when this
  // call creates the file; an existing one keeps whatever it already has.
  HANDLE hFile = CreateFileA(path_str.c_str(), GENERIC_READ | GENERIC_WRITE,
                             FILE_SHARE_READ | FILE_SHARE_WRITE, NULL,
                             OPEN_ALWAYS, FILE_ATTRIBUTE_TEMPORARY, NULL);
  // Read this before anything below can overwrite it.  It means two different
  // things depending on whether we got a handle: the reason we failed, or --
  // see the branches below -- which of the two things a success did.
  const DWORD create_error = GetLastError();
  ABSL_PCHECK(hFile != INVALID_HANDLE_VALUE)
      << "CreateFileA(" << path_str << ") failed with " << create_error;

  // We have a handle, so the file is open either way; OPEN_ALWAYS reports
  // which case this is only through the last-error code.  It documents exactly
  // two values here: zero if it created the file, and ERROR_ALREADY_EXISTS if
  // it opened one that was already there.  Anything else is outside the
  // documented contract, so refuse to guess rather than fall into a branch:
  // guessing "created" would re-size a file whose real creator is still
  // filling in, and guessing "existed" would wait forever on a file nobody is
  // ever going to size.
  if (create_error == 0) {
    ABSL_VLOG(1) << "Created " << path_str;
    // Size the file by writing its last byte rather than with SetEndOfFile().
    // SetEndOfFile() leaves the bytes between the old and new end of file "not
    // defined", while extending a file with a write documents the intervening
    // bytes as zero initialized -- and everything mapping this file expects a
    // freshly created one to read as zeros.  (In practice NTFS reads zeros past
    // the valid data length either way; only SetFileValidData(), which requires
    // SE_MANAGE_VOLUME_NAME and which we never call, exposes uninitialized disk
    // contents.  Writing the last byte doesn't depend on that being true.)
    ABSL_CHECK_GT(size_, size_t{0}) << ": zero-sized mapping of " << path_str;
    LARGE_INTEGER li;
    li.QuadPart = static_cast<LONGLONG>(size_) - 1;
    ABSL_PCHECK(SetFilePointerEx(hFile, li, NULL, FILE_BEGIN))
        << "SetFilePointerEx failed";
    const char zero = 0;
    DWORD written = 0;
    ABSL_PCHECK(WriteFile(hFile, &zero, 1, &written, NULL))
        << "WriteFile failed";
    ABSL_CHECK_EQ(written, DWORD{1}) << ": short write sizing " << path_str;
  } else if (create_error == ERROR_ALREADY_EXISTS) {
    ABSL_VLOG(1) << path_str << " already created.";
    LARGE_INTEGER liSize;
    ABSL_PCHECK(GetFileSizeEx(hFile, &liSize)) << "GetFileSizeEx failed";
    while (liSize.QuadPart == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      ABSL_PCHECK(GetFileSizeEx(hFile, &liSize)) << "GetFileSizeEx failed";
    }
    ABSL_CHECK_EQ(static_cast<size_t>(liSize.QuadPart), size_)
        << ": Size of " << path_str
        << " doesn't match expected size of backing queue file.  Did the "
           "queue definition change?";
  } else {
    ABSL_PLOG(FATAL) << "CreateFileA(" << path_str
                     << ") opened the file but set an undocumented last-error "
                        "code: "
                     << create_error;
  }

  HANDLE hMapping = CreateFileMappingA(hFile, NULL, PAGE_READWRITE, 0, 0, NULL);
  ABSL_PCHECK(hMapping != NULL)
      << ": Unable to create file mapping for " << path_str;

  data_ = MapViewOfFile(hMapping, FILE_MAP_WRITE | FILE_MAP_READ, 0, 0, 0);
  ABSL_PCHECK(data_ != NULL) << "MapViewOfFile failed";

  CloseHandle(hMapping);
  CloseHandle(hFile);

  PageFaultDataWrite(static_cast<char *>(data_), size_, kSystemPageSize);
}

WritableShmMapping::~WritableShmMapping() {
  ABSL_PCHECK(UnmapViewOfFile(data_));
}

ReadOnlyShmMapping::ReadOnlyShmMapping(std::string_view path, size_t size,
                                       std::filesystem::perms /*permissions*/)
    : size_(size) {
  const long kSystemPageSize = SystemPageSize();

  std::string path_str(path);

  HANDLE hFile = CreateFileA(path_str.c_str(), GENERIC_READ,
                             FILE_SHARE_READ | FILE_SHARE_WRITE, NULL,
                             OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
  ABSL_PCHECK(hFile != INVALID_HANDLE_VALUE)
      << "Unable to open file " << path_str;

  LARGE_INTEGER liSize;
  ABSL_PCHECK(GetFileSizeEx(hFile, &liSize)) << "GetFileSizeEx failed";
  while (liSize.QuadPart == 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ABSL_PCHECK(GetFileSizeEx(hFile, &liSize)) << "GetFileSizeEx failed";
  }
  ABSL_CHECK_EQ(static_cast<size_t>(liSize.QuadPart), size_)
      << ": Size of " << path_str
      << " doesn't match expected size of backing queue file.";

  HANDLE hMapping = CreateFileMappingA(hFile, NULL, PAGE_READONLY, 0, 0, NULL);
  ABSL_PCHECK(hMapping != NULL)
      << ": Unable to create file mapping for " << path_str;

  data_ = MapViewOfFile(hMapping, FILE_MAP_READ, 0, 0, 0);
  ABSL_PCHECK(data_ != NULL) << "MapViewOfFile failed";

  CloseHandle(hMapping);
  CloseHandle(hFile);

  PageFaultDataRead(static_cast<const char *>(data_), size_, kSystemPageSize);
}

ReadOnlyShmMapping::~ReadOnlyShmMapping() {
  ABSL_PCHECK(UnmapViewOfFile(const_cast<void *>(data_)));
}

}  // namespace aos::ipc_lib
