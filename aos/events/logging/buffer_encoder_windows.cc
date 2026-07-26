#include "aos/events/logging/buffer_encoder.h"

#include <fcntl.h>
#include <io.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "absl/log/check.h"
#include "absl/log/log.h"

namespace aos::logger {

// Note: we are punting on unicode log paths.  _open() interprets a narrow path
// in the process's active code page rather than as UTF-8, so a path with
// non-ASCII characters fails the PCHECK below.  Supporting them means carrying
// UTF-16 (via _wopen, or std::filesystem::path) through the logging code
// generally, not just converting at this one call site, so it isn't worth doing
// piecemeal here.
DummyDecoder::DummyDecoder(std::string_view filename)
    : filename_(filename),
      fd_(_open(filename_.c_str(), _O_RDONLY | _O_NOINHERIT | _O_BINARY)) {
  PCHECK(fd_ != -1) << ": Failed to open " << filename;
}

DummyDecoder::~DummyDecoder() {
  int status = _close(fd_);
  if (status != 0) {
    PLOG(ERROR) << "DummyDecoder: Failed to close file";
  }
}

size_t DummyDecoder::Read(uint8_t *begin, uint8_t *end) {
  if (end_of_file_) {
    return 0;
  }
  const std::ptrdiff_t count =
      _read(fd_, begin, static_cast<unsigned int>(end - begin));
  PCHECK(count >= 0) << ": Failed to read from file";
  if (count == 0) {
    end_of_file_ = true;
  }
  return static_cast<size_t>(count);
}

}  // namespace aos::logger
