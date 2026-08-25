// Tests for the Windows-specific behavior of aos_sync: shared-memory
// aos_mutexes are backed by named kernel mutexes (with kernel-enforced
// owner-death detection), while futexes and condition variables are
// process-local and explicitly refuse to work in file-backed shared memory.
//
// Everything here maps a real file with CreateFileMapping so the primitives
// classify as shared memory.  aos::testing::SharedMemoryBlock deliberately
// does NOT do that (it VirtualAllocs private memory), which is why the rest of
// the sync tests exercise the process-local paths.

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include "aos/ipc_lib/aos_sync.h"

#include <windows.h>

#include <thread>

#include "gtest/gtest.h"

#include "aos/testing/tmpdir.h"

namespace aos::testing {

namespace {

// A block of memory in a file-backed mapping, so that futexes and mutexes
// placed in it classify as shared rather than process-private.
class FileBackedMemory {
 public:
  static constexpr size_t kSize = 4096;

  FileBackedMemory() {
    const std::string path = TestTmpDir() + "\\aos_sync_windows_test_shm";
    file_ = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE,
                        FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, CREATE_ALWAYS,
                        FILE_ATTRIBUTE_NORMAL, NULL);
    EXPECT_NE(file_, INVALID_HANDLE_VALUE) << GetLastError();
    // CreateFileMapping extends the file to the mapping size with zero fill,
    // which doubles as the memset(0) initialization aos_mutex requires.
    mapping_ = CreateFileMappingA(file_, NULL, PAGE_READWRITE, 0, kSize, NULL);
    EXPECT_NE(mapping_, nullptr) << GetLastError();
    data_ = MapViewOfFile(mapping_, FILE_MAP_WRITE | FILE_MAP_READ, 0, 0, 0);
    EXPECT_NE(data_, nullptr) << GetLastError();
  }

  ~FileBackedMemory() {
    UnmapViewOfFile(data_);
    CloseHandle(mapping_);
    CloseHandle(file_);
  }

  aos_mutex *mutex() { return static_cast<aos_mutex *>(data_); }
  aos_futex *futex() {
    // Well clear of the mutex at the front.
    return reinterpret_cast<aos_futex *>(static_cast<char *>(data_) + 1024);
  }
  aos_condition *condition() {
    return reinterpret_cast<aos_condition *>(static_cast<char *>(data_) + 2048);
  }

 private:
  HANDLE file_ = INVALID_HANDLE_VALUE;
  HANDLE mapping_ = nullptr;
  void *data_ = nullptr;
};

class AosSyncWindowsTest : public ::testing::Test {
 protected:
  FileBackedMemory memory_;
};

// Tests that a shared-memory mutex locks and unlocks from a single thread,
// repeatedly.
TEST_F(AosSyncWindowsTest, SharedMutexLockUnlock) {
  aos_mutex *const m = memory_.mutex();
  for (int i = 0; i < 3; ++i) {
    ASSERT_EQ(0, mutex_grab(m));
    EXPECT_TRUE(mutex_islocked(m));
    mutex_unlock(m);
    EXPECT_FALSE(mutex_islocked(m));
  }
}

// Tests that trylock succeeds on an unlocked shared mutex and fails on a
// contended one, without blocking.
TEST_F(AosSyncWindowsTest, SharedMutexTrylock) {
  aos_mutex *const m = memory_.mutex();

  ASSERT_EQ(0, mutex_trylock(m));
  mutex_unlock(m);

  // Hold the lock on another thread and confirm trylock loses.
  std::atomic<bool> release{false};
  std::atomic<bool> locked{false};
  std::thread holder([&]() {
    ASSERT_EQ(0, mutex_grab(m));
    locked = true;
    while (!release) {
      std::this_thread::yield();
    }
    mutex_unlock(m);
  });
  while (!locked) {
    std::this_thread::yield();
  }
  EXPECT_EQ(4, mutex_trylock(m));
  release = true;
  holder.join();

  ASSERT_EQ(0, mutex_trylock(m));
  mutex_unlock(m);
}

// Tests that a blocking lock on a contended shared mutex waits for the holder
// and then acquires cleanly.
TEST_F(AosSyncWindowsTest, SharedMutexContention) {
  aos_mutex *const m = memory_.mutex();

  std::atomic<bool> locked{false};
  std::thread holder([&]() {
    ASSERT_EQ(0, mutex_grab(m));
    locked = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    mutex_unlock(m);
  });
  while (!locked) {
    std::this_thread::yield();
  }
  EXPECT_EQ(0, mutex_grab(m));
  mutex_unlock(m);
  holder.join();
}

// Tests that a thread exiting while holding a shared mutex results in the
// next locker being told the owner died, and that the notification is
// consumed by that lock.
//
// Two mechanisms deliver this and the test intentionally accepts their
// combination: the robust-list walk in the exiting thread's thread_local
// teardown marks FUTEX_OWNER_DIED in the word, and the kernel abandons the
// named mutex backing it (which is what would fire alone if the thread were
// killed hard enough to skip TLS teardown).
TEST_F(AosSyncWindowsTest, SharedMutexOwnerDied) {
  aos_mutex *const m = memory_.mutex();

  std::thread dier([&]() { ASSERT_EQ(0, mutex_grab(m)); });
  dier.join();

  // The owner died, so we are told exactly once.
  EXPECT_EQ(1, mutex_grab(m));
  mutex_unlock(m);

  EXPECT_EQ(0, mutex_grab(m));
  mutex_unlock(m);
}

// Tests the same owner-death path through trylock.
TEST_F(AosSyncWindowsTest, SharedMutexOwnerDiedTrylock) {
  aos_mutex *const m = memory_.mutex();

  std::thread dier([&]() { ASSERT_EQ(0, mutex_grab(m)); });
  dier.join();

  EXPECT_EQ(1, mutex_trylock(m));
  mutex_unlock(m);

  EXPECT_EQ(0, mutex_trylock(m));
  mutex_unlock(m);
}

// Tests that futexes explicitly refuse to work in file-backed shared memory:
// they are process-local (WaitOnAddress) on Windows, so allowing this would
// silently fail to wake cross-process waiters.
TEST_F(AosSyncWindowsTest, SharedFutexDies) {
  EXPECT_DEATH(
      { (void)futex_set(memory_.futex()); },
      "do not support cross-process use on Windows");
}

// Tests the same for condition variables.
TEST_F(AosSyncWindowsTest, SharedConditionDies) {
  // condition_signal only touches the condition, not the mutex, on Windows.
  aos_mutex private_mutex{};
  EXPECT_DEATH(
      { condition_signal(memory_.condition(), &private_mutex); },
      "do not support cross-process use on Windows");
}

}  // namespace
}  // namespace aos::testing
