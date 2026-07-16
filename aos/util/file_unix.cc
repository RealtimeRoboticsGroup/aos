#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#ifdef __APPLE__
#include <limits.h>
#include <mach-o/dyld.h>
#endif

#include <algorithm>
#include <iterator>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/util/file.h"

namespace aos::util {

std::optional<std::string> MaybeReadFileToString(
    const std::string_view filename) {
  std::string r;
  ScopedFD fd(open(::std::string(filename).c_str(), O_RDONLY));
  if (fd.get() == -1) {
    ABSL_PLOG(ERROR) << "Failed to open " << filename;
    return std::nullopt;
  }
  while (true) {
    char buffer[1024];
    const ssize_t result = read(fd.get(), buffer, sizeof(buffer));
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
  ScopedFD fd(open(::std::string(filename).c_str(), O_RDONLY));
  ABSL_PCHECK(fd.get() != -1) << ": opening " << filename;
  while (true) {
    uint8_t buffer[1024];
    const ssize_t result = read(fd.get(), buffer, sizeof(buffer));
    ABSL_PCHECK(result >= 0) << ": reading from " << filename;
    if (result == 0) {
      break;
    }
    std::copy(buffer, buffer + result, std::back_inserter(r));
  }
  return r;
}

void SyncDirectory(const std::filesystem::path &path) {
  const int dir_fd = open(path.c_str(), O_DIRECTORY);
  ABSL_PCHECK(dir_fd != -1) << "Failed to open directory " << path;
  ABSL_PCHECK(fsync(dir_fd) != -1) << "Failed to fsync directory " << path;
  ABSL_PCHECK(close(dir_fd) != -1) << "Failed to close directory " << path;
}

std::shared_ptr<absl::Span<uint8_t>> MMapFile(const std::string &path,
                                              FileOptions options) {
  int fd =
      open(path.c_str(), options == FileOptions::kReadable ? O_RDONLY : O_RDWR);
  ABSL_PCHECK(fd != -1) << "Unable to open file " << path;
  struct stat sb;
  ABSL_PCHECK(fstat(fd, &sb) != -1) << ": Unable to get file size of " << path;
  uint8_t *start = reinterpret_cast<uint8_t *>(mmap(
      NULL, sb.st_size,
      options == FileOptions::kReadable ? PROT_READ : (PROT_READ | PROT_WRITE),
      MAP_SHARED, fd, 0));
  ABSL_CHECK(start != MAP_FAILED)
      << ": Unable to open mapping to file " << path;
  std::shared_ptr<absl::Span<uint8_t>> span =
      std::shared_ptr<absl::Span<uint8_t>>(
          new absl::Span<uint8_t>(start, sb.st_size),
          [](absl::Span<uint8_t> *span) {
            ABSL_PCHECK(msync(span->data(), span->size(), MS_SYNC) == 0)
                << ": Failed to flush data before unmapping.";
            ABSL_PCHECK(munmap(span->data(), span->size()) != -1);
            delete span;
          });
  close(fd);
  return span;
}

FileReader::FileReader(std::string_view filename,
                       FileReaderErrorType error_type)
    : file_(open(::std::string(filename).c_str(), O_RDONLY)) {
  if (!is_open()) {
    ABSL_PLOG_IF(FATAL, error_type == FileReaderErrorType::kFatal)
        << ": opening " << filename;
    ABSL_PLOG(ERROR) << "opening " << filename;
  }
}

std::optional<absl::Span<char>> FileReader::ReadContents(
    absl::Span<char> buffer) {
  ABSL_CHECK(file_.get() != -1) << ": Bad file descriptor";
  ABSL_PCHECK(0 == lseek(file_.get(), 0, SEEK_SET));
  const ssize_t result = read(file_.get(), buffer.data(), buffer.size());
  if (result < 0) {
#ifdef __linux__
    // Read timeout for an i2c request returns this.
    if (errno == EREMOTEIO) {
      return std::nullopt;
    }
#endif
  }

  ABSL_PCHECK(result >= 0);
  return absl::Span<char>{buffer.data(), static_cast<size_t>(result)};
}

FileWriter::FileWriter(std::string_view filename,
                       std::filesystem::perms permissions)
    : file_(open(::std::string(filename).c_str(), O_WRONLY | O_CREAT | O_TRUNC,
                 static_cast<mode_t>(permissions))) {
  ABSL_PCHECK(file_.get() != -1) << ": opening " << filename;
}

FileWriter::WriteResult FileWriter::WriteBytes(
    absl::Span<const uint8_t> bytes) {
  ABSL_CHECK(file_.get() != -1) << ": Bad file descriptor";
  size_t size_written = 0;
  while (size_written != bytes.size()) {
    const ssize_t result = write(file_.get(), bytes.data() + size_written,
                                 bytes.size() - size_written);
    if (result < 0) {
      return {size_written, static_cast<int>(result)};
    }
    // Not really supposed to happen unless writing zero bytes without an error.
    // See, e.g.,
    // https://stackoverflow.com/questions/2176443/is-a-return-value-of-0-from-write2-in-c-an-error
    if (result == 0) {
      return {size_written, static_cast<int>(result)};
    }
    size_written += result;
  }
  return {size_written, static_cast<int>(size_written)};
}

std::optional<std::filesystem::path> GetExecutablePath() {
#ifdef __linux__
  char proc_self_exec_buffer[1024 + 1];
  std::memset(proc_self_exec_buffer, 0, sizeof(proc_self_exec_buffer));
  ssize_t s = readlink("/proc/self/exe", proc_self_exec_buffer, 1024);
  if (s > 0) {
    return std::filesystem::path(std::string_view(proc_self_exec_buffer, s));
  }
#elif defined(__APPLE__)
  char buf[PATH_MAX + 1];
  uint32_t size = sizeof(buf);
  if (_NSGetExecutablePath(buf, &size) == 0) {
    return std::filesystem::canonical(buf);
  }
#endif
  return std::nullopt;
}

}  // namespace aos::util
