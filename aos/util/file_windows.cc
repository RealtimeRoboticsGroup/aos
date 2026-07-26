#include "aos/util/file.h"

#include <errno.h>
#include <fcntl.h>
#include <io.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <windows.h>

#include <algorithm>
#include <iterator>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

using ssize_t = std::ptrdiff_t;
using mode_t = int;

namespace aos::util {

std::optional<std::string> MaybeReadFileToString(
    const std::string_view filename) {
  std::string r;
  ScopedFD fd(_open(::std::string(filename).c_str(), _O_RDONLY | _O_BINARY));
  if (fd.get() == -1) {
    ABSL_PLOG(ERROR) << "Failed to open " << filename;
    return std::nullopt;
  }
  while (true) {
    char buffer[1024];
    const ssize_t result = _read(fd.get(), buffer, sizeof(buffer));
    if (result < 0) {
      ABSL_PLOG(ERROR) << "Failed to read from " << filename;
      return std::nullopt;
    }
    if (result == 0) {
      break;
    }
    r.append(buffer, result);
  }
  return r;
}

std::vector<uint8_t> ReadFileToVecOrDie(const std::string_view filename) {
  std::vector<uint8_t> r;
  ScopedFD fd(_open(::std::string(filename).c_str(), _O_RDONLY | _O_BINARY));
  ABSL_PCHECK(fd.get() != -1) << ": opening " << filename;
  while (true) {
    uint8_t buffer[1024];
    const ssize_t result = _read(fd.get(), buffer, sizeof(buffer));
    ABSL_PCHECK(result >= 0) << ": reading from " << filename;
    if (result == 0) {
      break;
    }
    std::copy(buffer, buffer + result, std::back_inserter(r));
  }
  return r;
}

void SyncDirectory(const std::filesystem::path &path) {
  // Windows does not support syncing directory handles (flushing a directory
  // handle via FlushFileBuffers is not supported and returns an error).
  // The alternative of flushing the entire drive/volume via FlushFileBuffers
  // on a volume handle (e.g., \\.\C:) is not viable here because it requires
  // administrative privileges and incurs system-wide performance overhead.
  // In the end, this is trying to preserve data against power loss, and we
  // aren't going to use Windows for things where that matters.
  (void)path;
}

std::shared_ptr<absl::Span<uint8_t>> MMapFile(const std::string &path,
                                              FileOptions options) {
  HANDLE hFile = CreateFileA(path.c_str(),
                             options == FileOptions::kReadable
                                 ? GENERIC_READ
                                 : (GENERIC_READ | GENERIC_WRITE),
                             FILE_SHARE_READ | FILE_SHARE_WRITE, NULL,
                             OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
  ABSL_PCHECK(hFile != INVALID_HANDLE_VALUE) << "Unable to open file " << path;

  LARGE_INTEGER liSize;
  ABSL_PCHECK(GetFileSizeEx(hFile, &liSize))
      << ": Unable to get file size of " << path;
  size_t size = static_cast<size_t>(liSize.QuadPart);

  HANDLE hMapping = CreateFileMappingA(
      hFile, NULL,
      options == FileOptions::kReadable ? PAGE_READONLY : PAGE_READWRITE, 0, 0,
      NULL);
  ABSL_PCHECK(hMapping != NULL)
      << ": Unable to create file mapping for " << path;

  uint8_t *start = reinterpret_cast<uint8_t *>(MapViewOfFile(
      hMapping,
      options == FileOptions::kReadable ? FILE_MAP_READ : FILE_MAP_WRITE, 0, 0,
      0));
  ABSL_CHECK(start != NULL) << ": Unable to open mapping to file " << path;

  CloseHandle(hMapping);
  CloseHandle(hFile);

  return std::shared_ptr<absl::Span<uint8_t>>(
      new absl::Span<uint8_t>(start, size), [](absl::Span<uint8_t> *span) {
        ABSL_PCHECK(FlushViewOfFile(span->data(), span->size()))
            << ": Failed to flush data before unmapping.";
        ABSL_PCHECK(UnmapViewOfFile(span->data()));
        delete span;
      });
}

FileReader::FileReader(std::string_view filename,
                       FileReaderErrorType error_type)
    : file_(_open(::std::string(filename).c_str(), _O_RDONLY | _O_BINARY)) {
  if (!is_open()) {
    ABSL_PLOG_IF(FATAL, error_type == FileReaderErrorType::kFatal)
        << ": opening " << filename;
    ABSL_PLOG(ERROR) << "opening " << filename;
  }
}

std::optional<absl::Span<char>> FileReader::ReadContents(
    absl::Span<char> buffer) {
  ABSL_CHECK(file_.get() != -1) << ": Bad file descriptor";
  ABSL_PCHECK(0 == _lseek(file_.get(), 0, SEEK_SET));
  const ssize_t result = _read(file_.get(), buffer.data(), buffer.size());
  ABSL_PCHECK(result >= 0);
  return absl::Span<char>{buffer.data(), static_cast<size_t>(result)};
}

FileWriter::FileWriter(std::string_view filename,
                       std::filesystem::perms permissions)
    : file_(_open(::std::string(filename).c_str(),
                  _O_WRONLY | _O_CREAT | _O_TRUNC | _O_BINARY,
                  static_cast<mode_t>(permissions))) {
  ABSL_PCHECK(file_.get() != -1) << ": opening " << filename;
}

FileWriter::WriteResult FileWriter::WriteBytes(
    absl::Span<const uint8_t> bytes) {
  ABSL_CHECK(file_.get() != -1) << ": Bad file descriptor";
  size_t size_written = 0;
  while (size_written != bytes.size()) {
    const ssize_t result = _write(file_.get(), bytes.data() + size_written,
                                  bytes.size() - size_written);
    if (result < 0) {
      return {size_written, static_cast<int>(result)};
    }
    if (result == 0) {
      return {size_written, static_cast<int>(result)};
    }
    size_written += result;
  }
  return {size_written, static_cast<int>(size_written)};
}

std::optional<std::filesystem::path> GetExecutablePath() {
  char proc_self_exec_buffer[MAX_PATH + 1];
  DWORD s = GetModuleFileNameA(NULL, proc_self_exec_buffer, MAX_PATH);
  if (s > 0) {
    return std::filesystem::path(std::string_view(proc_self_exec_buffer, s));
  } else {
    return std::nullopt;
  }
}

}  // namespace aos::util
