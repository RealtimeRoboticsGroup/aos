#ifndef AOS_IPC_LIB_SHM_MAPPING_H_
#define AOS_IPC_LIB_SHM_MAPPING_H_

#include <stddef.h>
#include <stdint.h>

#include <filesystem>
#include <string_view>

namespace aos::ipc_lib {

void PageFaultDataWrite(char *data, size_t size, const long page_size);
void PageFaultDataRead(const char *data, size_t size, const long page_size);

class WritableShmMapping {
 public:
  WritableShmMapping(std::string_view path, size_t size,
                     std::filesystem::perms permissions);
  ~WritableShmMapping();

  WritableShmMapping(const WritableShmMapping &) = delete;
  WritableShmMapping &operator=(const WritableShmMapping &) = delete;
  WritableShmMapping(WritableShmMapping &&other) = delete;
  WritableShmMapping &operator=(WritableShmMapping &&other) = delete;

  void *data() const { return data_; }
  size_t size() const { return size_; }

 private:
  const size_t size_;
  void *data_ = nullptr;
};

class ReadOnlyShmMapping {
 public:
  ReadOnlyShmMapping(std::string_view path, size_t size,
                     std::filesystem::perms permissions);
  ~ReadOnlyShmMapping();

  ReadOnlyShmMapping(const ReadOnlyShmMapping &) = delete;
  ReadOnlyShmMapping &operator=(const ReadOnlyShmMapping &) = delete;
  ReadOnlyShmMapping(ReadOnlyShmMapping &&other) = delete;
  ReadOnlyShmMapping &operator=(ReadOnlyShmMapping &&other) = delete;

  const void *data() const { return data_; }
  size_t size() const { return size_; }

 private:
  const size_t size_;
  const void *data_ = nullptr;
};

}  // namespace aos::ipc_lib

#endif  // AOS_IPC_LIB_SHM_MAPPING_H_
