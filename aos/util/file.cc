#include "aos/util/file.h"

#include <array>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "flatbuffers/util.h"

namespace aos::util {

std::string ReadFileToStringOrDie(const std::string_view filename) {
  std::optional<std::string> r = MaybeReadFileToString(filename);
  ABSL_PCHECK(r.has_value()) << "Failed to read " << filename << " to string";
  return r.value();
}

bool MaybeWriteStringToFile(std::string_view filename,
                            std::string_view contents,
                            std::filesystem::perms permissions) {
  FileWriter writer(filename, permissions);
  auto result = writer.WriteBytes(
      {reinterpret_cast<const uint8_t *>(contents.data()), contents.size()});
  return result.bytes_written == contents.size();
}

void WriteStringToFileOrDie(const std::string_view filename,
                            const std::string_view contents,
                            std::filesystem::perms permissions) {
  FileWriter writer(filename, permissions);
  writer.WriteBytesOrDie(
      {reinterpret_cast<const uint8_t *>(contents.data()), contents.size()});
}

bool MkdirPIfSpace(std::string_view path, std::filesystem::perms mode,
                   bool sync) {
  std::filesystem::path p(path);
  std::filesystem::path folder = p.parent_path();
  if (folder.empty()) {
    return true;
  }

  std::error_code ec;
  if (std::filesystem::exists(folder, ec)) {
    return true;
  }

  if (folder.has_parent_path()) {
    if (!MkdirPIfSpace(folder.string(), mode, sync)) {
      return false;
    }
  }

  if (std::filesystem::create_directory(folder, ec)) {
    ABSL_VLOG(1) << "Created " << folder;
    std::filesystem::permissions(folder,
                                 static_cast<std::filesystem::perms>(mode), ec);
    if (ec) {
      ABSL_LOG(FATAL) << "Error setting permissions on " << folder << ": "
                      << ec.message();
    }
  } else {
    if (ec) {
      if (ec == std::errc::no_space_on_device) {
        ABSL_VLOG(2) << "Out of space";
        return false;
      }
      ABSL_LOG(FATAL) << "Error creating " << folder << ": " << ec.message();
    }
  }

  if (sync) {
    // Sync the newly created directory.
    SyncDirectory(folder);

    // Also sync the parent directory to ensure the directory entry is written
    // to disk.
    if (folder.has_parent_path()) {
      SyncDirectory(folder.parent_path());
    }
  }
  return true;
}

bool PathExists(std::string_view path) { return std::filesystem::exists(path); }

void UnlinkRecursive(std::string_view path) {
  std::error_code ec;
  std::filesystem::remove_all(path, ec);
  if (ec) {
    ABSL_LOG(WARNING) << "Failed to remove " << path << ": " << ec.message();
  }
}

// absl::SimpleAtoi doesn't interpret a leading 0x as hex, which we need here.
// Instead, we use the flatbufers API, which unfortunately relies on NUL
// termination.
std::optional<int32_t> FileReader::ReadInt32() {
  // Maximum characters for a 32-bit integer, +1 for the NUL.
  // Hex is the same size with the leading 0x.
  std::array<char, 11> buffer;
  int32_t result;
  const std::optional<absl::Span<char>> string_span =
      ReadContents(absl::Span<char>(buffer.data(), buffer.size())
                       .subspan(0, buffer.size() - 1));
  if (!string_span.has_value()) {
    return std::nullopt;
  }

  // Verify we found the newline.
  ABSL_CHECK_EQ(buffer[string_span->size() - 1], '\n');
  // Truncate the newline.
  buffer[string_span->size() - 1] = '\0';
  ABSL_CHECK(flatbuffers::StringToNumber(buffer.data(), &result))
      << ": Error parsing string to integer: "
      << std::string_view(string_span->data(), string_span->size());

  return result;
}

FileWriter::WriteResult FileWriter::WriteBytes(std::string_view bytes) {
  return WriteBytes(absl::Span<const uint8_t>{
      reinterpret_cast<const uint8_t *>(bytes.data()), bytes.size()});
}

void FileWriter::WriteBytesOrDie(std::string_view bytes) {
  WriteBytesOrDie(absl::Span<const uint8_t>{
      reinterpret_cast<const uint8_t *>(bytes.data()), bytes.size()});
}

void FileWriter::WriteBytesOrDie(absl::Span<const uint8_t> bytes) {
  ABSL_PCHECK(bytes.size() == WriteBytes(bytes).bytes_written)
      << ": Failed to write " << bytes.size() << " bytes.";
}

}  // namespace aos::util
