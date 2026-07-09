#include <windows.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <thread>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/ipc_lib/shm_mapping.h"
#include "aos/util/file.h"

namespace aos::ipc_lib {

WritableShmMapping::WritableShmMapping(std::string_view path, size_t size,
                                       std::filesystem::perms permissions)
    : size_(size) {
  SYSTEM_INFO sys_info;
  GetSystemInfo(&sys_info);
  const long kSystemPageSize = sys_info.dwPageSize;

  std::string path_str(path);
  util::MkdirP(path_str, permissions);

  HANDLE hFile = CreateFileA(path_str.c_str(), GENERIC_READ | GENERIC_WRITE,
                             FILE_SHARE_READ | FILE_SHARE_WRITE, NULL,
                             OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
  ABSL_PCHECK(hFile != INVALID_HANDLE_VALUE)
      << "Unable to open file " << path_str;

  if (GetLastError() != ERROR_ALREADY_EXISTS) {
    ABSL_VLOG(1) << "Created " << path_str;
    LARGE_INTEGER li;
    li.QuadPart = size_;
    ABSL_PCHECK(SetFilePointerEx(hFile, li, NULL, FILE_BEGIN))
        << "SetFilePointerEx failed";
    ABSL_PCHECK(SetEndOfFile(hFile)) << "SetEndOfFile failed";
  } else {
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
  SYSTEM_INFO sys_info;
  GetSystemInfo(&sys_info);
  const long kSystemPageSize = sys_info.dwPageSize;

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
