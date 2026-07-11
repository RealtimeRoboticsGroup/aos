#ifndef AOS_EVENTS_LOGGING_FILE_OPERATIONS_H_
#define AOS_EVENTS_LOGGING_FILE_OPERATIONS_H_

#include <stddef.h>

#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace aos::logger::internal {

// Predicate to include or exclude file to be considered as a log file.
bool IsValidFilename(std::string_view filename);

// Abstraction that supports listing of the logs on file system and S3. It is
// associated with either a single file or directory that contains log files.
class FileOperations {
 public:
  struct File {
    std::string name;
    size_t size;  // bytes.
  };

  virtual ~FileOperations() = default;

  virtual bool Exists() = 0;
  virtual void FindLogs(std::vector<File> *files) = 0;
};

// Implements FileOperations with standard filesystem APIs. These work on files
// local to the machine they're running on.
//
// Takes a std::filesystem::path rather than a string: unlike the FileOperations
// interface, which also covers S3 (where "s3://bucket/key" is a URL and not a
// path at all), everything this one does is a path operation, and holding the
// parsed form keeps callers from round-tripping through a string.
class LocalFileOperations final : public FileOperations {
 public:
  explicit LocalFileOperations(std::filesystem::path filename)
      : filename_(std::move(filename)) {}

  bool Exists() override { return std::filesystem::exists(filename_); }

  void FindLogs(std::vector<File> *files) override;

 private:
  std::filesystem::path filename_;
};

}  // namespace aos::logger::internal

#endif  // AOS_EVENTS_LOGGING_FILE_OPERATIONS_H_
