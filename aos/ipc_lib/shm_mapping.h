#ifndef AOS_IPC_LIB_SHM_MAPPING_H_
#define AOS_IPC_LIB_SHM_MAPPING_H_

#include <stddef.h>
#include <stdint.h>

#include <filesystem>
#include <string_view>

namespace aos::ipc_lib {

// Returns the size of a memory page in bytes.  Defined per-OS so callers don't
// need their own #ifdefs to keep the OS-specific bits contained.
long SystemPageSize();

// Touches one byte in each `page_size`-byte page of the `size` bytes starting
// at `data`, forcing the kernel to populate the pagetable entries up front so
// that later accesses (in particular from a realtime thread) never take a page
// fault. The Write variant faults the pages in for writing, the Read variant
// for reading.
void PageFaultDataWrite(char *data, size_t size, const long page_size);
void PageFaultDataRead(const char *data, size_t size, const long page_size);

// RAII wrappers around a shared-memory-backed file mapped into this process.
//
// Lifetime is part of the contract, not an implementation detail: the contents
// outlive every process that maps them.  A queue that everyone has stopped
// using keeps its state, and the next process to attach finds it rather than a
// blank slate -- see InitializeLocklessQueueMemory(), which only sets a queue
// up when it finds one that hasn't been initialized yet.  Every backend has to
// preserve that, which rules out the OS primitives that free their contents
// once the last mapping goes away (see shm_mapping_windows.cc for the one we
// turned down over this).
//
// WritableShmMapping creates the file at `path` if it doesn't exist (racing
// safely with other creators), sizes it to `size`, and maps it read/write.
// Pre-faults every page so realtime code can touch the memory without faulting,
// and unmap on destruction.
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

// ReadOnlyShmMapping maps an existing (already-sized) file read-only, waiting
// if necessary for the creating process to finish sizing it.  Pre-faults
// every page so realtime code can touch the memory without faulting, and unmap
// on destruction.
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
