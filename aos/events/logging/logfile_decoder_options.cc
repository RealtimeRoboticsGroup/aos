#include "aos/events/logging/logfile_decoder_options.h"

#include "aos/events/logging/snappy_encoder.h"
#include "aos/sanitizers.h"

#if defined(__x86_64__)
#define ENABLE_S3 1
#elif defined(__aarch64__)
#define ENABLE_S3 0
#else
#define ENABLE_S3 0
#endif

#include "aos/events/logging/lzma_encoder.h"

#if ENABLE_S3
#include "aos/events/logging/s3_fetcher.h"
#include "aos/events/logging/s3_file_operations.h"
#endif

namespace aos::logger::internal {
std::unique_ptr<DataDecoder> ResolveDecoder(std::string_view filename,
                                            bool quiet) {
  static constexpr std::string_view kS3 = "s3:";

  std::unique_ptr<DataDecoder> decoder;

  if (filename.substr(0, kS3.size()) == kS3) {
#if ENABLE_S3
    decoder = std::make_unique<S3Fetcher>(filename);
#else
    LOG(FATAL) << "Reading files from S3 not supported on this platform";
#endif
  } else {
    decoder = std::make_unique<DummyDecoder>(filename);
  }

  static constexpr std::string_view kXz = ".xz";
  static constexpr std::string_view kSnappy = SnappyDecoder::kExtension;
  if (filename.substr(filename.size() - kXz.size()) == kXz) {
    decoder = std::make_unique<ThreadedLzmaDecoder>(std::move(decoder), quiet);
  } else if (filename.substr(filename.size() - kSnappy.size()) == kSnappy) {
    decoder = std::make_unique<SnappyDecoder>(std::move(decoder));
  }
  return decoder;
}

std::unique_ptr<internal::FileOperations> MakeFileOperations(
    std::string_view filename) {
  static constexpr std::string_view kS3 = "s3:";
  if (filename.substr(0, kS3.size()) == kS3) {
#if ENABLE_S3
    return std::make_unique<internal::S3FileOperations>(filename);
#else
    LOG(FATAL) << "Reading files from S3 not supported on this platform";
#endif
  }
  if (filename.find("://") != filename.npos) {
    LOG(FATAL) << "This looks like a URL of an unknown type: " << filename;
  }
  return std::make_unique<internal::LocalFileOperations>(filename);
}
}  // namespace aos::logger::internal
