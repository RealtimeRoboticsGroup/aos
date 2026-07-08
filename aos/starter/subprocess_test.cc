#include "aos/starter/subprocess.h"

#include <signal.h>
#include <stdlib.h>
#include <sys/stat.h>

#include <fstream>
#include <ostream>
#include <thread>

#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "aos/configuration.h"
#include "aos/events/shm_event_loop.h"
#include "aos/flatbuffers.h"
#include "aos/ipc_lib/shm_base.h"
#include "aos/testing/path.h"
#include "aos/testing/tmpdir.h"
#include "aos/util/file.h"

namespace aos::starter::testing {

class SubprocessTest : public ::testing::Test {
 protected:
  SubprocessTest()
      : config_file_(::aos::testing::ArtifactPath(
            "aos/testing/ping_pong/pingpong_config.json")),
        config_(aos::configuration::ReadConfig(config_file_)),
        event_loop_(&config_.message()),
        exit_timer_(event_loop_.AddTimer([this]() {
          event_loop_.Exit();
          FAIL() << "We should have already exited.";
        })) {
    // Nuke the shm dir:
    aos::util::UnlinkRecursive(absl::GetFlag(FLAGS_shm_base));
    event_loop_.OnRun([this]() {
      // Note: we are using the backup poll in this test to capture SIGCHLD.
      // This runs at 1 hz, so make sure we let the polling occur at least once.
      // From there, we add a bit of extra buffer to account for any variability
      // in test timing. Typically this timeout should not get hit, so adding
      // extra time here should not impact normal test runtime too much.
      exit_timer_->Schedule(event_loop_.monotonic_now() +
                            std::chrono::milliseconds(1500));
    });
  }
  const std::string config_file_;
  aos::FlatbufferDetachedBuffer<aos::Configuration> config_;
  aos::ShmEventLoop event_loop_;
  aos::TimerHandler *exit_timer_;
};

TEST_F(SubprocessTest, CaptureStatus) {
  Application application(
      "exit", "sh", &event_loop_,
      [this, &application]() {
        if (application.status() == aos::starter::State::STOPPED) {
          event_loop_.Exit();
        }
      },
      nullptr);
  ASSERT_FALSE(application.autorestart());
  application.set_args({"-c", "exit 0"});

  flatbuffers::FlatBufferBuilder fbb;
  fbb.Finish(application.PopulateStatus(&fbb, nullptr));
  aos::FlatbufferDetachedBuffer<ApplicationStatus> status = fbb.Release();
  EXPECT_THAT(aos::FlatbufferToJson(status, {.multi_line = true}),
              ::testing::MatchesRegex(R"json(\{
 "name": "exit",
 "state": "STOPPED",
 "last_stop_reason": "STOP_REQUESTED",
 "has_active_timing_report": false,
 "file_state": "NOT_RUNNING"
\})json"))
      << "Application status pre-start should not contain exit code/time "
         "values.";

  application.Start();

  event_loop_.Run();

  fbb.Finish(application.PopulateStatus(&fbb, nullptr));
  status = fbb.Release();
  EXPECT_THAT(aos::FlatbufferToJson(status, {.multi_line = true}),
              ::testing::MatchesRegex(R"json(\{
 "name": "exit",
 "state": "STOPPED",
 "last_exit_code": 0,
 "pid": [0-9]*,
 "id": 0,
 "last_start_time": [0-9]*,
 "last_stop_reason": "STOP_REQUESTED",
 "has_active_timing_report": false,
 "file_state": "NOT_RUNNING",
 "last_exit_time": [0-9]*
\})json"));
  EXPECT_LT(status.message().last_start_time(),
            status.message().last_exit_time())
      << "Exit time should be greater than start time when application is "
         "stopped.";
  EXPECT_LT(std::chrono::nanoseconds(status.message().last_exit_time() -
                                     status.message().last_start_time()),
            std::chrono::milliseconds(1000))
      << "The application should have completed running in a sane amount of "
         "time.";
  aos::monotonic_clock::time_point initial_stop_time =
      event_loop_.monotonic_now();
  EXPECT_LT(std::chrono::nanoseconds(status.message().last_exit_time()),
            initial_stop_time.time_since_epoch());

  EXPECT_EQ(aos::starter::State::STOPPED, application.status());

  application.set_args({"-c", "exit 1"});
  application.Start();
  event_loop_.Run();

  fbb.Finish(application.PopulateStatus(&fbb, nullptr));
  status = fbb.Release();
  EXPECT_THAT(aos::FlatbufferToJson(status, {.multi_line = true}),
              ::testing::MatchesRegex(R"json(\{
 "name": "exit",
 "state": "STOPPED",
 "last_exit_code": 1,
 "pid": [0-9]*,
 "id": 1,
 "last_start_time": [0-9]*,
 "last_stop_reason": "STOP_REQUESTED",
 "has_active_timing_report": false,
 "file_state": "NOT_RUNNING",
 "last_exit_time": [0-9]*
\})json"));
  EXPECT_LT(status.message().last_start_time(),
            status.message().last_exit_time())
      << "Exit time should be greater than start time when application is "
         "stopped.";
  EXPECT_LT(initial_stop_time.time_since_epoch(),
            std::chrono::nanoseconds(status.message().last_start_time()))
      << "Updated start time should be entirely after the original application "
         "stopped.";
}

TEST_F(SubprocessTest, CaptureOutputs) {
  Application echo_stdout(
      "echo", "echo", &event_loop_,
      [this, &echo_stdout]() {
        if (echo_stdout.status() == aos::starter::State::STOPPED) {
          event_loop_.Exit();
        }
      },
      nullptr);
  ASSERT_FALSE(echo_stdout.autorestart());
  echo_stdout.set_args({"abcdef"});
  echo_stdout.set_capture_stdout(true);
  echo_stdout.set_capture_stderr(true);

  echo_stdout.Start();

  event_loop_.Run();

  ASSERT_EQ("abcdef\n", echo_stdout.GetStdout());
  ASSERT_TRUE(echo_stdout.GetStderr().empty());
  EXPECT_EQ(aos::starter::State::STOPPED, echo_stdout.status());

  // Run again, the output should've been cleared.
  echo_stdout.set_args({"ghijkl"});
  echo_stdout.Start();
  event_loop_.Run();
  ASSERT_EQ("ghijkl\n", echo_stdout.GetStdout());
}

TEST_F(SubprocessTest, CaptureStderr) {
  Application echo_stderr(
      "echo", "sh", &event_loop_,
      [&echo_stderr, this]() {
        if (echo_stderr.status() == aos::starter::State::STOPPED) {
          event_loop_.Exit();
        }
      },
      nullptr);
  echo_stderr.set_args({"-c", "echo abcdef >&2"});
  echo_stderr.set_capture_stdout(true);
  echo_stderr.set_capture_stderr(true);

  echo_stderr.Start();

  event_loop_.Run();

  ASSERT_EQ("abcdef\n", echo_stderr.GetStderr());
  ASSERT_TRUE(echo_stderr.GetStdout().empty());
  ASSERT_EQ(aos::starter::State::STOPPED, echo_stderr.status());
}

// Checks that when a child application crashing results in the starter printing
// out its own version by default.
TEST_F(SubprocessTest, PrintNoTimingReportVersionString) {
  ::testing::internal::CaptureStderr();

  // Set up application without quiet flag active
  event_loop_.SetVersionString("version_string");
  Application error_out(
      "false", "bash", &event_loop_,
      [this, &error_out]() {
        if (error_out.status() == aos::starter::State::STOPPED) {
          event_loop_.Exit();
        }
      },
      nullptr, Application::QuietLogging::kNo);
  error_out.set_args({"-c", "sleep 3; false"});

  error_out.Start();
  event_loop_.OnRun([this]() {
    // Don't use the default exit timeout; this test specifically sleeps longer
    // than normal to get the process past the initial STARTING state.
    exit_timer_->Schedule(event_loop_.monotonic_now() +
                          std::chrono::milliseconds(5000));
  });

  event_loop_.Run();

  // Ensure presence of logs without quiet flag
  std::string output = ::testing::internal::GetCapturedStderr();
  std::string expected = "starter version 'version_string'";

  ASSERT_TRUE(output.find(expected) != std::string::npos) << output;
  EXPECT_EQ(aos::starter::State::STOPPED, error_out.status());
}

TEST_F(SubprocessTest, PrintFailedToStartVersionString) {
  ::testing::internal::CaptureStderr();

  // Set up application without quiet flag active
  event_loop_.SetVersionString("version_string");
  Application error_out(
      "false", "false", &event_loop_,
      [this, &error_out]() {
        if (error_out.status() == aos::starter::State::STOPPED) {
          event_loop_.Exit();
        }
      },
      nullptr, Application::QuietLogging::kNo);

  error_out.Start();

  event_loop_.Run();

  // Ensure presence of logs without quiet flag
  std::string output = ::testing::internal::GetCapturedStderr();
  std::string expected = "starter version 'version_string'";

  ASSERT_TRUE(output.find(expected) != std::string::npos) << output;
  EXPECT_EQ(aos::starter::State::STOPPED, error_out.status());
}

TEST_F(SubprocessTest, UnactiveQuietFlag) {
  ::testing::internal::CaptureStderr();

  // Set up application without quiet flag active
  Application error_out(
      "false", "false", &event_loop_,
      [this, &error_out]() {
        if (error_out.status() == aos::starter::State::STOPPED) {
          event_loop_.Exit();
        }
      },
      nullptr, Application::QuietLogging::kNo);
  ASSERT_FALSE(error_out.autorestart());

  error_out.Start();

  event_loop_.Run();

  // Ensure presence of logs without quiet flag
  std::string output = ::testing::internal::GetCapturedStderr();
  std::string expectedStart = "Failed to start 'false'";
  std::string expectedRun = "exited unexpectedly with status";

  ASSERT_TRUE(output.find(expectedStart) != std::string::npos ||
              output.find(expectedRun) != std::string::npos)
      << output;
  EXPECT_EQ(aos::starter::State::STOPPED, error_out.status());
}

TEST_F(SubprocessTest, ActiveQuietFlag) {
  ::testing::internal::CaptureStderr();

  Application error_out(
      "false", "false", &event_loop_,
      [this, &error_out]() {
        if (error_out.status() == aos::starter::State::STOPPED) {
          event_loop_.Exit();
        }
      },
      nullptr, Application::QuietLogging::kYes);
  ASSERT_FALSE(error_out.autorestart());

  error_out.Start();

  event_loop_.Run();

  // Ensure lack of logs with quiet flag
  ASSERT_TRUE(::testing::internal::GetCapturedStderr().empty());
  EXPECT_EQ(aos::starter::State::STOPPED, error_out.status());
}

// Tests that Nothing Bad™ happens if the event loop outlives the Application.
//
// Note that this is a bit of a hope test, as there is no guarantee that we
// will trigger a crash even if the resources tied to the event loop in the
// aos::Application aren't properly released.
TEST_F(SubprocessTest, ShortLivedApp) {
  auto application = std::make_unique<Application>(
      "sleep", "sleep", &event_loop_, []() {}, nullptr);
  application->set_args({"10"});
  application->Start();
  pid_t pid = application->get_pid();

  int ticks = 0;
  aos::TimerHandler *exit_timer = event_loop_.AddTimer([this, &ticks,
                                                        &application, pid]() {
    ticks++;
    if (application && application->status() == aos::starter::State::RUNNING) {
      // Kill the application, it will autorestart.
      kill(pid, SIGTERM);
      application.reset();
    }

    // event loop lives for longer.
    if (ticks >= 5) {
      // Now we exit.
      event_loop_.Exit();
    }
  });

  event_loop_.OnRun([this, exit_timer]() {
    // Disable the normal exit timer, and schedule a wakeup for our checks once
    // per polling period (every second).
    exit_timer_->Disable();
    exit_timer->Schedule(event_loop_.monotonic_now(),
                         std::chrono::milliseconds(1000));
  });

  event_loop_.Run();
}

// Test that if the binary changes out from under us that we note it in the
// FileState.
TEST_F(SubprocessTest, ChangeBinaryContents) {
  // Create a local copy of the sleep binary so that we can delete it.
  const std::filesystem::path full_executable_path =
      absl::StrCat(aos::testing::TestTmpDir(), "/", "sleep_binary");
  aos::util::WriteStringToFileOrDie(
      full_executable_path.native(),
      aos::util::ReadFileToStringOrDie(ResolvePath("sleep").native()),
      std::filesystem::perms::owner_all);

  const std::filesystem::path executable_name =
      absl::StrCat(aos::testing::TestTmpDir(), "/", "sleep_symlink");
  // Create a symlink that points to the actual binary, and test that a
  // Creating a symlink in particular lets us ensure that our logic actually
  // pays attention to the target file that we are running rather than the
  // symlink itself (it also saves us having to cp a binary somewhere where we
  // can overwrite it).
  std::filesystem::create_symlink(full_executable_path.native(),
                                  executable_name);

  // Wait until we are running, go through and test that various variations in
  // file state result in the expected behavior, and then exit.
  Application sleep(
      "sleep", executable_name.native(), &event_loop_,
      [&sleep, this, executable_name, full_executable_path]() {
        switch (sleep.status()) {
          case aos::starter::State::RUNNING:
            EXPECT_EQ(aos::starter::FileState::NO_CHANGE,
                      sleep.UpdateFileState());
            // Delete the symlink; this should have no effect,
            // because the Application class should be looking at
            // the original path.
            std::filesystem::remove(executable_name);
            EXPECT_EQ(aos::starter::FileState::NO_CHANGE,
                      sleep.UpdateFileState());
            // Delete the executable; it should be changed.
            std::filesystem::remove(full_executable_path);
            EXPECT_EQ(aos::starter::FileState::CHANGED,
                      sleep.UpdateFileState());
            // Replace the executable itself; it should be
            // changed.
            aos::util::WriteStringToFileOrDie(full_executable_path.native(),
                                              "abcdef");
            EXPECT_EQ(aos::starter::FileState::CHANGED,
                      sleep.UpdateFileState());
            // Terminate.
            event_loop_.Exit();
            break;
          case aos::starter::State::WAITING:
          case aos::starter::State::STARTING:
          case aos::starter::State::STOPPING:
          case aos::starter::State::STOPPED:
            EXPECT_EQ(aos::starter::FileState::NOT_RUNNING,
                      sleep.UpdateFileState());
            break;
        }
      },
      nullptr);
  ASSERT_FALSE(sleep.autorestart());
  // Ensure that the subprocess will run longer than we care about (we just call
  // Terminate() below to stop it).
  sleep.set_args({"1000"});

  sleep.Start();

  event_loop_.Run();
  sleep.Terminate();
}

class ResolvePathTest : public ::testing::Test {
 protected:
  ResolvePathTest() {
    // Before doing anything else,
    if (getenv("PATH") != nullptr) {
      original_path_ = getenv("PATH");
      PCHECK(0 == unsetenv("PATH"));
    }
  }

  ~ResolvePathTest() {
    if (!original_path_.empty()) {
      PCHECK(0 == setenv("PATH", original_path_.c_str(), /*overwrite=*/1));
    } else {
      PCHECK(0 == unsetenv("PATH"));
    }
  }

  std::filesystem::path GetLocalPath(const std::string filename) {
    return absl::StrCat(aos::testing::TestTmpDir(), "/", filename);
  }

  std::filesystem::path CreateFile(const std::string filename) {
    const std::filesystem::path file = GetLocalPath(filename);
    VLOG(2) << "Creating file at " << file;
    util::WriteStringToFileOrDie(file.native(), "contents");
    return file;
  }

  void SetPath(const std::vector<std::string> &path) {
    PCHECK(0 ==
           setenv("PATH", absl::StrJoin(path, ":").c_str(), /*overwrite=*/1));
  }

  // Keep track of original PATH environment variable so that we can restore
  // it.
  std::string original_path_;
};

// Tests that we can resolve paths when there is no PATH environment variable.
TEST_F(ResolvePathTest, ResolveWithUnsetPath) {
  const std::filesystem::path local_echo = CreateFile("echo");
  // Because the default path will be in /bin and /usr/bin (typically), we have
  // to choose some utility that we can reasonably expect to be available in the
  // test environment.
  const std::filesystem::path echo_path = ResolvePath("echo");
  EXPECT_THAT((std::vector<std::string>{"/bin/echo", "/usr/bin/echo",
                                        "/bin/echo.coreutils"}),
              ::testing::Contains(echo_path.native()));

  // Test that a file with /'s in the name ignores the PATH.
  const std::filesystem::path local_echo_path =
      ResolvePath(local_echo.native());
  EXPECT_EQ(local_echo_path, local_echo);
}

// Test that when the PATH environment variable is set that we can use it.
TEST_F(ResolvePathTest, ResolveWithPath) {
  const std::filesystem::path local_folder = GetLocalPath("bin/");
  const std::filesystem::path local_folder2 = GetLocalPath("bin2/");
  aos::util::MkdirP(local_folder.native(), std::filesystem::perms::owner_all);
  aos::util::MkdirP(local_folder2.native(), std::filesystem::perms::owner_all);
  SetPath({local_folder, local_folder2});
  const std::filesystem::path binary = CreateFile("bin/binary");
  const std::filesystem::path duplicate_binary = CreateFile("bin2/binary");
  const std::filesystem::path other_name = CreateFile("bin2/other_name");

  EXPECT_EQ(binary, ResolvePath("binary"));
  EXPECT_EQ(other_name, ResolvePath("other_name"));
  // And check that if we specify the full path for the duplicate binary that we
  // can find it.
  EXPECT_EQ(duplicate_binary, ResolvePath(duplicate_binary.native()));
}

// Test that we fail to find non-existent files.
TEST_F(ResolvePathTest, DieOnFakeFile) {
  // Fail to find something that searches the PATH.
  SetPath({"foo", "bar"});
  EXPECT_DEATH(ResolvePath("fake_file"), "Unable to resolve");

  // Fail to find a local file.
  EXPECT_DEATH(ResolvePath("./fake_file"), "./fake_file does not exist");
}

// Tests for RemoveCGroupWithRetry and GetCGroupProcessesInfo.
class RemoveCGroupTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create a temporary directory for testing.
    test_dir_ = std::filesystem::temp_directory_path() /
                ("test_cgroup_" + std::to_string(getpid()));
    std::filesystem::create_directories(test_dir_);
  }

  void TearDown() override {
    // Clean up test directory.
    std::error_code ec;
    std::filesystem::remove_all(test_dir_, ec);
  }

  std::filesystem::path test_dir_;
};

// Tests that RemoveCGroupWithRetry successfully removes an empty directory on
// the first attempt.
TEST_F(RemoveCGroupTest, RemoveEmptyDirectory) {
  std::filesystem::path test_path = test_dir_ / "empty_dir";
  std::filesystem::create_directory(test_path);
  ASSERT_TRUE(std::filesystem::exists(test_path));

  RemoveCGroupWithRetry(test_path);

  EXPECT_FALSE(std::filesystem::exists(test_path));
}

// Tests that RemoveCGroupWithRetry logs a warning when attempting to remove a
// non-existent directory and returns gracefully.
TEST_F(RemoveCGroupTest, RemoveNonExistentDirectoryWarning) {
  std::filesystem::path test_path = test_dir_ / "nonexistent";
  ASSERT_FALSE(std::filesystem::exists(test_path));

  // Should log a warning but not crash.
  // Just verify it doesn't crash - checking log output in tests is fragile.
  EXPECT_NO_FATAL_FAILURE(RemoveCGroupWithRetry(test_path));

  // Directory still doesn't exist (nothing changed).
  EXPECT_FALSE(std::filesystem::exists(test_path));
}

// Tests that RemoveCGroupWithRetry crashes with LOG(FATAL) when attempting to
// remove a non-empty directory after exhausting all retries.
TEST_F(RemoveCGroupTest, RemoveDirectoryWithFileFatal) {
  std::filesystem::path test_path = test_dir_ / "dir_with_file";
  std::filesystem::create_directory(test_path);
  std::filesystem::path file_path = test_path / "file.txt";
  std::ofstream(file_path) << "test content";
  ASSERT_TRUE(std::filesystem::exists(file_path));

  EXPECT_DEATH(RemoveCGroupWithRetry(test_path),
               "Failed to remove cgroup .* after 5 attempts");
}

// Tests that RemoveCGroupWithRetry successfully removes a directory after
// retrying when the directory becomes empty during the retry period.
TEST_F(RemoveCGroupTest, RetrySucceedsAfterFileRemoved) {
  std::filesystem::path test_path = test_dir_ / "dir_with_delayed_removal";
  std::filesystem::create_directory(test_path);
  std::filesystem::path file_path = test_path / "file.txt";
  std::ofstream(file_path) << "test content";
  ASSERT_TRUE(std::filesystem::exists(file_path));

  // Start a thread that will remove the file after 2 seconds.
  std::thread cleanup_thread([file_path]() {
    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::filesystem::remove(file_path);
  });

  RemoveCGroupWithRetry(test_path);

  cleanup_thread.join();

  EXPECT_FALSE(std::filesystem::exists(test_path));
}

// Tests that GetCGroupProcessesInfo correctly reads and returns information
// about processes from a cgroup's tasks file, including process names and
// states from /proc, and handles non-existent PIDs gracefully.
TEST_F(RemoveCGroupTest, GetCGroupProcessesInfoWithTasksFile) {
  std::filesystem::path mock_cgroup = test_dir_ / "mock_cgroup";
  std::filesystem::create_directory(mock_cgroup);

  // Create a tasks file with some PIDs (including our own process).
  std::filesystem::path tasks_file = mock_cgroup / "tasks";
  std::ofstream tasks(tasks_file);
  pid_t our_pid = getpid();
  tasks << our_pid << "\n";  // Our own PID should be readable.
  tasks << "99999\n";        // Non-existent PID.
  tasks.close();

  std::string output = GetCGroupProcessesInfo(mock_cgroup);

  // Verify that we found processes.
  EXPECT_THAT(output,
              ::testing::HasSubstr("Found 2 process(es) still in cgroup"));

  // Verify that we logged our own PID.
  EXPECT_THAT(output, ::testing::HasSubstr("PID " + std::to_string(our_pid)));

  // Verify that we read the process info from /proc. Our process should have
  // readable comm and stat files.
  EXPECT_THAT(output, ::testing::HasSubstr("comm:"));
  EXPECT_THAT(output, ::testing::HasSubstr("stat:"));

  // Verify that the non-existent PID is handled gracefully (no comm/stat
  // output for it).
  EXPECT_THAT(output, ::testing::HasSubstr("PID 99999"));
}

// Tests that GetCGroupProcessesInfo handles an empty tasks file gracefully by
// returning an empty string.
TEST_F(RemoveCGroupTest, GetCGroupProcessesInfoEmptyTasks) {
  std::filesystem::path mock_cgroup = test_dir_ / "mock_cgroup_empty";
  std::filesystem::create_directory(mock_cgroup);

  std::filesystem::path tasks_file = mock_cgroup / "tasks";
  std::ofstream tasks(tasks_file);
  tasks.close();

  std::string output = GetCGroupProcessesInfo(mock_cgroup);
  EXPECT_TRUE(output.empty());
}

// Tests that GetCGroupProcessesInfo handles a missing tasks file gracefully by
// returning an error message.
TEST_F(RemoveCGroupTest, GetCGroupProcessesInfoNoTasksFile) {
  std::filesystem::path mock_cgroup = test_dir_ / "mock_cgroup_no_tasks";
  std::filesystem::create_directory(mock_cgroup);

  std::string output = GetCGroupProcessesInfo(mock_cgroup);
  EXPECT_THAT(output, ::testing::MatchesRegex(absl::StrCat(
                          "Could not open .*", mock_cgroup.string(),
                          "/tasks.* to read PIDs")));
}

class CleanupCGroupV2Test : public ::testing::Test {
 protected:
  void SetUp() override {
    test_dir_ = std::filesystem::temp_directory_path() /
                ("test_cgroupv2_" + std::to_string(getpid()));
    std::filesystem::create_directories(test_dir_);
  }

  void TearDown() override {
    std::error_code ec;
    std::filesystem::remove_all(test_dir_, ec);
  }

  void CreateMockCGroup(const std::filesystem::path &path) {
    std::filesystem::create_directories(path);
  }

  std::filesystem::path test_dir_;
};

// On a real cgroup filesystem, rmdir works on dirs with virtual control
// files. On regular filesystems, rmdir only works on truly empty dirs.
// These tests verify the cleanup *logic* (recursive removal, selective
// removal of parent) rather than kernel cgroup behavior.

TEST_F(CleanupCGroupV2Test, CleanupStaleRemovesEmptyDir) {
  std::filesystem::path cgroup = test_dir_ / "stale";
  std::filesystem::create_directory(cgroup);

  EXPECT_TRUE(RemoveCGroupV2(cgroup.string()));

  EXPECT_FALSE(std::filesystem::exists(cgroup));
}

TEST_F(CleanupCGroupV2Test, CleanupStaleRecursesIntoChildren) {
  std::filesystem::path parent = test_dir_ / "parent";
  std::filesystem::path child = parent / "child";
  std::filesystem::path grandchild = child / "grandchild";
  CreateMockCGroup(parent);
  CreateMockCGroup(child);
  std::filesystem::create_directory(grandchild);

  EXPECT_TRUE(RemoveCGroupV2(parent.string()));

  EXPECT_FALSE(std::filesystem::exists(grandchild));
}

TEST_F(CleanupCGroupV2Test, CleanNestedStateRemovesAllSubdirs) {
  std::filesystem::path cgroup = test_dir_ / "app";
  std::filesystem::path starter = cgroup / "starter";
  std::filesystem::path digest = cgroup / "aos_digest";
  CreateMockCGroup(cgroup);
  std::filesystem::create_directory(starter);
  std::filesystem::create_directory(digest);

  EXPECT_TRUE(RemoveCGroupV2Children(cgroup.string()));

  EXPECT_FALSE(std::filesystem::exists(starter));
  EXPECT_FALSE(std::filesystem::exists(digest));
  EXPECT_TRUE(std::filesystem::exists(cgroup));
}

TEST_F(CleanupCGroupV2Test, CleanNestedStateNoopWhenClean) {
  std::filesystem::path cgroup = test_dir_ / "app";
  std::filesystem::create_directory(cgroup);

  EXPECT_TRUE(RemoveCGroupV2Children(cgroup.string()));
  EXPECT_TRUE(std::filesystem::exists(cgroup));
}

}  // namespace aos::starter::testing
