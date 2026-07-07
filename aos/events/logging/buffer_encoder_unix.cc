#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "absl/log/check.h"
#include "absl/log/log.h"

#include "aos/events/logging/buffer_encoder.h"

namespace aos::logger {

DummyDecoder::DummyDecoder(std::string_view filename)
    : filename_(filename), fd_(open(filename_.c_str(), O_RDONLY | O_CLOEXEC)) {
  PCHECK(fd_ != -1) << ": Failed to open " << filename;
}

DummyDecoder::~DummyDecoder() {
  int status = close(fd_);
  if (status != 0) {
    PLOG(ERROR) << "DummyDecoder: Failed to close file";
  }
}

size_t DummyDecoder::Read(uint8_t *begin, uint8_t *end) {
  if (end_of_file_) {
    return 0;
  }
  const ssize_t count = read(fd_, begin, end - begin);
  PCHECK(count >= 0) << ": Failed to read from file";
  if (count == 0) {
    end_of_file_ = true;
  }
  return static_cast<size_t>(count);
}

}  // namespace aos::logger
