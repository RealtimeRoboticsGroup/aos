#include "aos/ipc_lib/event.h"

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#include "gtest/gtest.h"

#include "aos/testing/test_logging.h"
#include "aos/testing/tmpdir.h"
#include "aos/time/time.h"
#include "aos/util/file.h"

namespace aos::testing {

namespace chrono = ::std::chrono;
namespace this_thread = ::std::this_thread;

class EventTest : public ::testing::Test {
 public:
  Event test_event_;

 protected:
  void SetUp() override { ::aos::testing::EnableTestLogging(); }
};

// Makes sure that basic operations with no blocking or anything work.
TEST_F(EventTest, Basic) {
  EXPECT_FALSE(test_event_.Clear());
  EXPECT_FALSE(test_event_.Clear());

  test_event_.Set();
  test_event_.Wait();
  EXPECT_TRUE(test_event_.Clear());
  EXPECT_FALSE(test_event_.Clear());
}

// Tests that tsan understands that events establish a happens-before
// relationship.
TEST_F(EventTest, ThreadSanitizer) {
  for (int i = 0; i < 3000; ++i) {
    int variable = 0;
    test_event_.Clear();
    ::std::thread thread([this, &variable]() {
      test_event_.Wait();
      --variable;
    });
    ++variable;
    test_event_.Set();
    thread.join();
    EXPECT_EQ(0, variable);
  }
}

// Tests that an event blocks correctly.
TEST_F(EventTest, Blocks) {
  monotonic_clock::time_point start_time, finish_time;
  // Without this, it sometimes manages to fail under tsan.
  Event started;
  ::std::thread thread([this, &start_time, &finish_time, &started]() {
    start_time = monotonic_clock::now();
    started.Set();
    test_event_.Wait();
    finish_time = monotonic_clock::now();
  });
  static constexpr auto kWaitTime = chrono::milliseconds(50);
  started.Wait();
  this_thread::sleep_for(kWaitTime);
  test_event_.Set();
  thread.join();
  EXPECT_GE(finish_time - start_time, kWaitTime);
}

TEST_F(EventTest, WaitTimeout) {
  EXPECT_FALSE(test_event_.WaitTimeout(chrono::milliseconds(50)));

  monotonic_clock::time_point start_time, finish_time;
  // Without this, it sometimes manages to fail under tsan.
  Event started;
  ::std::thread thread([this, &start_time, &finish_time, &started]() {
    start_time = monotonic_clock::now();
    started.Set();
    EXPECT_TRUE(test_event_.WaitTimeout(chrono::milliseconds(500)));
    finish_time = monotonic_clock::now();
  });
  constexpr auto kWaitTime = chrono::milliseconds(50);
  started.Wait();
  this_thread::sleep_for(kWaitTime);
  test_event_.Set();
  thread.join();
  EXPECT_GE(finish_time - start_time, kWaitTime);
}

#ifdef _WIN32
#include <windows.h>
#else
#include <time.h>
#endif

namespace {
// Returns the CPU time used by the current thread.
chrono::nanoseconds GetCurrentThreadCpuTime() {
#ifdef _WIN32
  FILETIME creation_time, exit_time, kernel_time, user_time;
  if (GetThreadTimes(GetCurrentThread(), &creation_time, &exit_time,
                     &kernel_time, &user_time)) {
    ULARGE_INTEGER kernel, user;
    kernel.LowPart = kernel_time.dwLowDateTime;
    kernel.HighPart = kernel_time.dwHighDateTime;
    user.LowPart = user_time.dwLowDateTime;
    user.HighPart = user_time.dwHighDateTime;
    // FileTime is in 100-nanosecond intervals.
    return chrono::nanoseconds((kernel.QuadPart + user.QuadPart) * 100);
  }
  return chrono::nanoseconds(0);
#else
  struct timespec ts;
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts) == 0) {
    return chrono::seconds(ts.tv_sec) + chrono::nanoseconds(ts.tv_nsec);
  }
  return chrono::nanoseconds(0);
#endif
}
}  // namespace

// Verifies that Event::Wait actually blocks in the kernel and does not
// busy-poll.
TEST_F(EventTest, WaitBlocksNoBusyPoll) {
  const chrono::nanoseconds start_cpu = GetCurrentThreadCpuTime();

  ::std::thread thread([this]() {
    ::std::this_thread::sleep_for(chrono::milliseconds(50));
    test_event_.Set();
  });

  test_event_.Wait();
  thread.join();

  const chrono::nanoseconds end_cpu = GetCurrentThreadCpuTime();
  // It should block in the kernel.  If it busy-polls, it would consume nearly
  // 50ms of CPU time.  Make sure it used less than 10ms.
  EXPECT_LT(end_cpu - start_cpu, chrono::milliseconds(10));
}

#ifdef _WIN32
struct WinShm {
  HANDLE hMap = NULL;
  HANDLE hFile = INVALID_HANDLE_VALUE;
  std::string file_path;
  void *ptr = nullptr;
  void Create(const char *name, size_t size) {
    std::string temp_dir = aos::testing::TestTmpDir();
    std::string safe_name(name);
    for (char &c : safe_name) {
      if (c == '\\' || c == '/') {
        c = '_';
      }
    }
    std::string path = temp_dir + "\\" + safe_name;
    file_path = path;

    hFile = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE,
                        FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, CREATE_ALWAYS,
                        FILE_ATTRIBUTE_NORMAL, NULL);
    if (hFile == INVALID_HANDLE_VALUE) {
      ptr = nullptr;
      return;
    }
    LARGE_INTEGER li;
    li.QuadPart = size;
    SetFilePointerEx(hFile, li, NULL, FILE_BEGIN);
    SetEndOfFile(hFile);

    hMap = CreateFileMappingA(hFile, NULL, PAGE_READWRITE, 0, 0, name);
    if (hMap == NULL) {
      CloseHandle(hFile);
      hFile = INVALID_HANDLE_VALUE;
      ptr = nullptr;
      return;
    }
    ptr = MapViewOfFile(hMap, FILE_MAP_ALL_ACCESS, 0, 0, size);
  }
  void Open(const char *name, size_t size) {
    hMap = OpenFileMappingA(FILE_MAP_ALL_ACCESS, FALSE, name);
    if (hMap == NULL) {
      ptr = nullptr;
      return;
    }
    ptr = MapViewOfFile(hMap, FILE_MAP_ALL_ACCESS, 0, 0, size);
  }
  void Destroy() {
    if (ptr != nullptr) {
      UnmapViewOfFile(ptr);
      ptr = nullptr;
    }
    if (hMap != NULL) {
      CloseHandle(hMap);
      hMap = NULL;
    }
    if (hFile != INVALID_HANDLE_VALUE) {
      CloseHandle(hFile);
      hFile = INVALID_HANDLE_VALUE;
      DeleteFileA(file_path.c_str());
    }
  }
};
#else
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
struct UnixShm {
  int fd;
  void *ptr;
  void Create(const char *name, size_t size) {
    shm_unlink(name);
    fd = shm_open(name, O_CREAT | O_RDWR, 0666);
    if (fd == -1) {
      ptr = nullptr;
      return;
    }
    if (ftruncate(fd, size) == -1) {
      ptr = nullptr;
      return;
    }
    ptr = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (ptr == MAP_FAILED) {
      ptr = nullptr;
    }
  }
  void Open(const char *name, size_t size) {
    fd = shm_open(name, O_RDWR, 0666);
    if (fd == -1) {
      ptr = nullptr;
      return;
    }
    ptr = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (ptr == MAP_FAILED) {
      ptr = nullptr;
    }
  }
  void Destroy() {
    if (ptr != nullptr) {
      munmap(ptr, sizeof(Event) * 2);
    }
    if (fd != -1) {
      close(fd);
    }
  }
};
#endif

#ifdef _WIN32
void SpawnChild(const std::string &exe_path, const std::string &shm_name) {
  STARTUPINFOA si;
  PROCESS_INFORMATION pi;
  ZeroMemory(&si, sizeof(si));
  si.cb = sizeof(si);
  ZeroMemory(&pi, sizeof(pi));

  std::string cmd =
      "\"" + exe_path + "\" --gtest_filter=EventTest.CrossProcessChild";

  // Set environment variables for the child process.
  // We can use SetEnvironmentVariableA for the current process, which
  // will be inherited by the child process.
  SetEnvironmentVariableA("EVENT_TEST_ROLE", "child");
  SetEnvironmentVariableA("EVENT_TEST_SHM_NAME", shm_name.c_str());

  BOOL success = CreateProcessA(NULL, const_cast<char *>(cmd.c_str()), NULL,
                                NULL, TRUE, 0, NULL, NULL, &si, &pi);
  ASSERT_TRUE(success) << "CreateProcessA failed: " << GetLastError();

  // Wait for the child process to exit.
  WaitForSingleObject(pi.hProcess, INFINITE);

  DWORD exit_code = 0;
  GetExitCodeProcess(pi.hProcess, &exit_code);
  EXPECT_EQ(0, exit_code);

  CloseHandle(pi.hProcess);
  CloseHandle(pi.hThread);

  // Clean up environment variables.
  SetEnvironmentVariableA("EVENT_TEST_ROLE", NULL);
  SetEnvironmentVariableA("EVENT_TEST_SHM_NAME", NULL);
}
#else
#include <sys/wait.h>
void SpawnChild(const std::string &exe_path, const std::string &shm_name) {
  pid_t pid = fork();
  if (pid == 0) {
    // This executes in the child process.
    setenv("EVENT_TEST_ROLE", "child", 1);
    setenv("EVENT_TEST_SHM_NAME", shm_name.c_str(), 1);
    char *args[] = {
        const_cast<char *>(exe_path.c_str()),
        const_cast<char *>("--gtest_filter=EventTest.CrossProcessChild"),
        nullptr};
    execv(exe_path.c_str(), args);
    std::exit(1);
  }
  // This executes in the parent process.
  int status;
  waitpid(pid, &status, 0);
  EXPECT_TRUE(WIFEXITED(status));
  EXPECT_EQ(0, WEXITSTATUS(status));
}
#endif

struct SharedData {
  Event event_parent_to_child;
  Event event_child_to_parent;
};

TEST_F(EventTest, CrossProcessParent) {
  // Only run parent logic if we are not the child process.
  const char *role = std::getenv("EVENT_TEST_ROLE");
  if (role != nullptr && std::strcmp(role, "child") == 0) {
    return;
  }

  const std::string shm_name =
#ifdef _WIN32
      "Local\\aos_event_test_shm_" + std::to_string(GetCurrentProcessId());
#else
      "/aos_event_test_shm_" + std::to_string(getpid());
#endif

  size_t shm_size = sizeof(SharedData);

#ifdef _WIN32
  WinShm shm;
#else
  UnixShm shm;
#endif

  shm.Create(shm_name.c_str(), shm_size);
  ASSERT_NE(nullptr, shm.ptr);

  SharedData *shared = static_cast<SharedData *>(shm.ptr);
  std::memset(shared, 0, shm_size);

  // Parent sets up a background thread to wait for the child's ready signal,
  // and then sets the parent_to_child event.
  std::thread parent_waiter([shared]() {
    shared->event_child_to_parent.Wait();
    shared->event_parent_to_child.Set();
  });

  std::string exe_path = aos::util::GetExecutablePath().value().string();
  SpawnChild(exe_path, shm_name);

  parent_waiter.join();
  shm.Destroy();
#ifndef _WIN32
  shm_unlink(shm_name.c_str());
#endif
}

TEST_F(EventTest, CrossProcessChild) {
  // Only run if we are the spawned child process.
  const char *role = std::getenv("EVENT_TEST_ROLE");
  if (role == nullptr || std::strcmp(role, "child") != 0) {
    return;
  }

  const char *shm_name = std::getenv("EVENT_TEST_SHM_NAME");
  ASSERT_NE(nullptr, shm_name);

  size_t shm_size = sizeof(SharedData);

#ifdef _WIN32
  WinShm shm;
#else
  UnixShm shm;
#endif

  shm.Open(shm_name, shm_size);
  ASSERT_NE(nullptr, shm.ptr);

  SharedData *shared = static_cast<SharedData *>(shm.ptr);

  // Child signals that it is ready, and then waits for the parent's event.
  shared->event_child_to_parent.Set();
  shared->event_parent_to_child.Wait();

  shm.Destroy();
}

}  // namespace aos::testing
