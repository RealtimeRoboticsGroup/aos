#include "aos/ipc_lib/shm_mapping.h"

#include <filesystem>
#include <string>

#ifdef _WIN32
// clang-format off
#include <windows.h>
#include <psapi.h>
// clang-format on
#else
#include <sys/resource.h>
#include <unistd.h>
#endif

#ifdef __linux__
#include <fcntl.h>
#include <sys/mman.h>

#include <cstdint>
#endif

#include "gtest/gtest.h"

#include "aos/testing/tmpdir.h"

namespace aos::ipc_lib::testing {

class ShmMappingTest : public ::testing::Test {
 protected:
  void SetUp() override {
    test_dir_ = aos::testing::TestTmpDir() + "/shm_mapping_test";
    std::filesystem::remove_all(test_dir_);
  }

  void TearDown() override { std::filesystem::remove_all(test_dir_); }

  std::string test_dir_;
};

using ShmMappingDeathTest = ShmMappingTest;

namespace {

// Number of minor (soft) page faults this process has taken so far.  A fault
// on a write to an already-resident page means the page's PTE wasn't writable
// yet -- i.e. PageFaultDataWrite didn't do its job.
long MinorPageFaults() {
#ifdef _WIN32
  // Windows doesn't split minor/major the way POSIX does; PageFaultCount
  // covers both soft faults (e.g. first touch of a mapped page, which is what
  // we care about here) and hard ones.  K32GetProcessMemoryInfo is the
  // kernel32-exported alias for GetProcessMemoryInfo, so this doesn't need an
  // extra import library.
  PROCESS_MEMORY_COUNTERS counters = {};
  counters.cb = sizeof(counters);
  EXPECT_TRUE(K32GetProcessMemoryInfo(GetCurrentProcess(), &counters,
                                      sizeof(counters)));
  return static_cast<long>(counters.PageFaultCount);
#else
  // getrusage()/ru_minflt is portable POSIX/BSD, so this works on Linux and
  // macOS.
  struct rusage usage = {};
  EXPECT_EQ(getrusage(RUSAGE_SELF, &usage), 0);
  return usage.ru_minflt;
#endif
}

}  // namespace

#ifdef __linux__
namespace {

// Returns true if the page containing `addr` is currently present in this
// process's page tables, per /proc/self/pagemap.  Bit 63 (the "present" flag)
// is readable without privileges even though the physical frame number is
// zeroed out for unprivileged readers, so this works in a normal test
// environment.  Linux-only: macOS has no equivalent of /proc/self/pagemap.
bool PageIsResident(const void *addr) {
  const long page_size = sysconf(_SC_PAGESIZE);
  const uintptr_t vaddr = reinterpret_cast<uintptr_t>(addr);
  const off_t offset = static_cast<off_t>(vaddr / page_size) * sizeof(uint64_t);

  const int fd = open("/proc/self/pagemap", O_RDONLY);
  EXPECT_GE(fd, 0);
  if (fd < 0) {
    return false;
  }
  uint64_t entry = 0;
  const ssize_t ret = pread(fd, &entry, sizeof(entry), offset);
  close(fd);
  EXPECT_EQ(ret, static_cast<ssize_t>(sizeof(entry)));
  // Bit 63 is set when the page is present in RAM.
  return (entry & (uint64_t{1} << 63)) != 0;
}

}  // namespace
#endif  // __linux__

TEST_F(ShmMappingTest, BasicReadWrite) {
  const std::string shm_path = test_dir_ + "/test_queue";
  const size_t size = 4096;

  // 1. Create a writable mapping.
  WritableShmMapping write_mapping(shm_path, size,
                                   static_cast<std::filesystem::perms>(0777));
  ASSERT_NE(write_mapping.data(), nullptr);
  EXPECT_EQ(write_mapping.size(), size);

  // Write some data.
  char *write_ptr = static_cast<char *>(write_mapping.data());
  write_ptr[0] = 'H';
  write_ptr[1] = 'e';
  write_ptr[2] = 'l';
  write_ptr[3] = 'l';
  write_ptr[4] = 'o';

  // 2. Open a read-only mapping on the same path.
  ReadOnlyShmMapping read_mapping(shm_path, size,
                                  static_cast<std::filesystem::perms>(0777));
  ASSERT_NE(read_mapping.data(), nullptr);
  EXPECT_EQ(read_mapping.size(), size);

  // Read the data and verify.
  const char *read_ptr = static_cast<const char *>(read_mapping.data());
  EXPECT_EQ(read_ptr[0], 'H');
  EXPECT_EQ(read_ptr[1], 'e');
  EXPECT_EQ(read_ptr[2], 'l');
  EXPECT_EQ(read_ptr[3], 'l');
  EXPECT_EQ(read_ptr[4], 'o');
}

TEST_F(ShmMappingDeathTest, ReadOnlyIsActuallyReadOnly) {
  const std::string shm_path = test_dir_ + "/test_queue_ro";
  const size_t size = 4096;

  // Create file and writable mapping first.
  WritableShmMapping write_mapping(shm_path, size,
                                   static_cast<std::filesystem::perms>(0777));
  ASSERT_NE(write_mapping.data(), nullptr);

  // Open the read-only mapping.
  ReadOnlyShmMapping read_mapping(shm_path, size,
                                  static_cast<std::filesystem::perms>(0777));
  ASSERT_NE(read_mapping.data(), nullptr);

  // Attempting to write to read-only pointer should crash.
  EXPECT_DEATH(
      {
        char *ro_ptr =
            const_cast<char *>(static_cast<const char *>(read_mapping.data()));
        ro_ptr[0] = 'X';
      },
      "");
}

TEST_F(ShmMappingTest, PageFaultDataWriteDoesNotCorrupt) {
  const std::string shm_path = test_dir_ + "/test_corruption";
  std::filesystem::remove(shm_path);

  const long page_size = SystemPageSize();

  // Make the file size at least 2 pages.
  const size_t size = page_size * 2;

  // 1. Create a writable mapping and write some non-zero data at page
  // boundaries.
  {
    WritableShmMapping write_mapping(shm_path, size,
                                     static_cast<std::filesystem::perms>(0777));
    ASSERT_NE(write_mapping.data(), nullptr);

    char *write_ptr = static_cast<char *>(write_mapping.data());
    // Write the same non-zero byte to the start of multiple pages.
    // If the PageFaultDataWrite bug were present, loading page 0's value
    // would set 'zero' to 'A'. Then, evaluating page 1 would see 'A' as
    // expected, matching the page 1 value, and write 0 back to it.
    write_ptr[0] = 'A';
    write_ptr[page_size] = 'A';
  }

  // 2. Open the mapping again.  This is exactly the scenario that exposes the
  // fault-in bug: PageFaultDataWrite runs over pages that already hold
  // nonzero data.
  {
    WritableShmMapping write_mapping(shm_path, size,
                                     static_cast<std::filesystem::perms>(0777));
    ASSERT_NE(write_mapping.data(), nullptr);

    char *write_ptr = static_cast<char *>(write_mapping.data());
    EXPECT_EQ(write_ptr[0], 'A');
    EXPECT_EQ(write_ptr[page_size], 'A');

    // Beyond not corrupting the data, PageFaultDataWrite must also leave both
    // pages with a writable PTE.  A CAS that only stores when the byte is
    // already 0 (the original bug) satisfies the corruption check above just
    // fine, but on architectures whose compare-exchange lowers to a
    // load-linked/store-conditional pair (LL/SC) -- baseline ARM64 among them
    // -- the compiler branches around the store-conditional when the
    // comparison fails, so a nonzero page never takes a write access at all;
    // it keeps whatever (possibly read-only) PTE it already had.
    //
    // Whether this build can actually observe that here depends on which
    // atomic instruction the compiler chose:
    //   - x86 always hides it: a locked cmpxchg triggers a bus RFO regardless
    //     of the comparison's outcome, so the page always ends up writable.
    //     This repo's Windows target is x86_64-only today (no Windows ARM64
    //     platform is defined), so this check is expected to always pass
    //     there -- it's run for consistency/future-proofing, not because it
    //     can currently catch a regression on that platform.
    //   - Apple Silicon defaults to it also being hidden: clang's default
    //     target for arm64-apple-macos uses the ARMv8.1 LSE `cas`/`casb`
    //     instruction (Apple Silicon always has it), which -- like x86's
    //     locked op -- appears to require write permission on the page to
    //     execute at all, comparison outcome notwithstanding, unlike LL/SC
    //     where the store is a separate, skippable instruction.
    //   - To force the LL/SC shape (and so actually exercise this check) on
    //     an Apple Silicon Mac, build with a pre-LSE CPU baseline, e.g.
    //     `bazel test //aos/ipc_lib:shm_mapping_test
    //     --platforms=//tools/platforms:darwin_arm64
    //     --copt=-mcpu=cortex-a53`.  That's real ARM64 machine code executing
    //     on real hardware -- LL/SC is always available even on chips that
    //     also support LSE -- so it faithfully reproduces the bug shape a
    //     generic/baseline ARM64 Linux build hits by default.
    //
    // A subsequent real write should take zero minor faults either way; this
    // check is a no-op on configurations where the bug is architecturally
    // masked, and a real regression test everywhere else.
    const long before = MinorPageFaults();
    write_ptr[0] = 'B';
    write_ptr[page_size] = 'B';
    EXPECT_EQ(MinorPageFaults() - before, 0)
        << "PageFaultDataWrite left a nonzero page without a writable PTE";
  }
}

#ifdef __linux__
// Confirms that PageFaultDataWrite actually faults in the pages it touches --
// the whole point of pre-faulting.  A fresh anonymous mapping is demand-paged,
// so none of its pages are resident until something touches them.
TEST(PageFaultTest, WriteFaultsPages) {
  const long page_size = sysconf(_SC_PAGESIZE);
  const size_t kPages = 8;
  const size_t size = kPages * page_size;

  void *data = mmap(nullptr, size, PROT_READ | PROT_WRITE,
                    MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
  ASSERT_NE(data, MAP_FAILED);

  // Nothing should be resident before we fault.  (Reading pagemap doesn't touch
  // the target pages, so this check doesn't perturb the result.)
  for (size_t i = 0; i < kPages; ++i) {
    EXPECT_FALSE(PageIsResident(static_cast<char *>(data) + i * page_size))
        << "page " << i << " should not be resident before faulting";
  }

  PageFaultDataWrite(static_cast<char *>(data), size, page_size);

  // Every page should be resident afterwards.
  for (size_t i = 0; i < kPages; ++i) {
    EXPECT_TRUE(PageIsResident(static_cast<char *>(data) + i * page_size))
        << "page " << i << " should be resident after PageFaultDataWrite";
  }

  ASSERT_EQ(munmap(data, size), 0);
}

// Same, for the read-faulting path.
TEST(PageFaultTest, ReadFaultsPages) {
  const long page_size = sysconf(_SC_PAGESIZE);
  const size_t kPages = 8;
  const size_t size = kPages * page_size;

  void *data = mmap(nullptr, size, PROT_READ | PROT_WRITE,
                    MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
  ASSERT_NE(data, MAP_FAILED);

  for (size_t i = 0; i < kPages; ++i) {
    EXPECT_FALSE(PageIsResident(static_cast<char *>(data) + i * page_size))
        << "page " << i << " should not be resident before faulting";
  }

  PageFaultDataRead(static_cast<const char *>(data), size, page_size);

  for (size_t i = 0; i < kPages; ++i) {
    EXPECT_TRUE(PageIsResident(static_cast<char *>(data) + i * page_size))
        << "page " << i << " should be resident after PageFaultDataRead";
  }

  ASSERT_EQ(munmap(data, size), 0);
}
#endif  // __linux__

}  // namespace aos::ipc_lib::testing
