#include "aos/util/top.h"

#include <pthread.h>
#include <signal.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <limits>
#include <ostream>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/flags/reflection.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "flatbuffers/string.h"
#include "flatbuffers/vector.h"
#include "gtest/gtest.h"
#include <gmock/gmock.h>

#include "aos/configuration.h"
#include "aos/events/shm_event_loop.h"
#include "aos/flatbuffers.h"
#include "aos/ipc_lib/shm_base.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/testing/path.h"
#include "aos/testing/tmpdir.h"
#include "aos/util/file.h"

namespace aos::util::testing {

void SetThreadName(const std::string &name) {
  pthread_setname_np(pthread_self(), name.c_str());
}

constexpr std::string_view kTestCPUConsumer = "TestCPUConsumer";

class TopTest : public ::testing::Test {
 protected:
  TopTest()
      : shm_dir_(aos::testing::TestTmpDir()),
        cpu_consumer_([this]() {
          SetThreadName(std::string(kTestCPUConsumer));
          while (!stop_flag_.load()) {
          }
        }),
        config_file_(
            aos::testing::ArtifactPath("aos/events/pingpong_config.json")),
        config_(aos::configuration::ReadConfig(config_file_)),
        event_loop_(&config_.message()) {
    aos::testing::SetShmBase(shm_dir_);

    // Nuke the shm dir, to ensure we aren't being affected by any preexisting
    // tests.
    aos::util::UnlinkRecursive(shm_dir_ + "/aos");
  }
  ~TopTest() {
    stop_flag_ = true;
    cpu_consumer_.join();
  }

  absl::FlagSaver flag_saver_;
  std::string shm_dir_;

  std::thread cpu_consumer_;
  std::atomic<bool> stop_flag_{false};
  const std::string config_file_;
  const aos::FlatbufferDetachedBuffer<aos::Configuration> config_;
  aos::ShmEventLoop event_loop_;
};

TEST_F(TopTest, TestSelfStat) {
  const pid_t pid = getpid();
  std::optional<ProcStat> proc_stat = ReadProcStat(pid);
  ASSERT_TRUE(proc_stat.has_value());
  ASSERT_EQ(pid, proc_stat->pid);
  ASSERT_EQ("top_test", proc_stat->name);
  ASSERT_EQ('R', proc_stat->state);
  ASSERT_LT(1, proc_stat->num_threads);
}

TEST_F(TopTest, QuerySingleProcess) {
  const pid_t pid = getpid();
  Top top(&event_loop_, Top::TrackThreadsMode::kDisabled,
          Top::TrackPerThreadInfoMode::kDisabled);
  top.set_track_pids({pid});
  event_loop_.AddTimer([this]() { event_loop_.Exit(); })
      ->Schedule(event_loop_.monotonic_now() + std::chrono::seconds(2));
  event_loop_.Run();
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);
  fbb.Finish(top.InfoForProcess(&fbb, pid));
  aos::FlatbufferDetachedBuffer<ProcessInfo> info = fbb.Release();
  ASSERT_EQ(pid, info.message().pid());
  ASSERT_TRUE(info.message().has_name());
  ASSERT_EQ("top_test", info.message().name()->string_view());
  // Check that we did indeed consume ~1 CPU core (because we're multi-threaded,
  // we could've consumed a bit more; and on systems where we are competing with
  // other processes for CPU time, we may not get a full 100% load).
  ASSERT_LT(0.5, info.message().cpu_usage());
  ASSERT_GT(1.1, info.message().cpu_usage());
  // Sanity check memory usage.
  ASSERT_LT(1000000, info.message().physical_memory());
  ASSERT_GT(1000000000, info.message().physical_memory());

  // Verify no per-thread information is included by default.
  ASSERT_FALSE(info.message().has_threads());
}

TEST_F(TopTest, QuerySingleProcessWithThreads) {
  const pid_t pid = getpid();
  Top top(&event_loop_, Top::TrackThreadsMode::kDisabled,
          Top::TrackPerThreadInfoMode::kEnabled);
  top.set_track_pids({pid});
  event_loop_.AddTimer([this]() { event_loop_.Exit(); })
      ->Schedule(event_loop_.monotonic_now() + std::chrono::seconds(2));
  event_loop_.Run();
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);
  fbb.Finish(top.InfoForProcess(&fbb, pid));
  aos::FlatbufferDetachedBuffer<ProcessInfo> info = fbb.Release();
  ASSERT_EQ(pid, info.message().pid());
  ASSERT_TRUE(info.message().has_name());
  ASSERT_EQ("top_test", info.message().name()->string_view());
  // Check that we did indeed consume ~1 CPU core (because we're multi-threaded,
  // we could've consumed a bit more; and on systems where we are competing with
  // other processes for CPU time, we may not get a full 100% load).
  ASSERT_LT(0.5, info.message().cpu_usage());
  ASSERT_GT(1.1, info.message().cpu_usage());
  // Sanity check memory usage.
  ASSERT_LT(1000000, info.message().physical_memory());
  ASSERT_GT(1000000000, info.message().physical_memory());

  // Validate that we have some per-thread information.
  ASSERT_TRUE(info.message().has_threads());
  ASSERT_GT(info.message().threads()->size(), 0);
  std::set<std::string_view> thread_names;
  double thread_cpu_usage = 0.0;
  for (const ThreadInfo *thread_info : *info.message().threads()) {
    thread_names.insert(thread_info->name()->string_view());
    thread_cpu_usage += thread_info->cpu_usage();
    ASSERT_TRUE(thread_info->has_state());
  }
  // Validate that at least one thread was named correctly.
  ASSERT_THAT(thread_names, ::testing::Contains(kTestCPUConsumer));
  // Validate that we consumed at some cpu one a thread.
  ASSERT_GT(thread_cpu_usage, 0);
}

TEST_F(TopTest, TopProcesses) {
  // Make some dummy processes that will just spin and get killed off at the
  // end, so that we actually have things to query.
  constexpr int kNProcesses = 2;
  std::vector<pid_t> children;
  // This will create kNProcesses children + ourself, which means we have enough
  // processes to test that we correctly exclude extras when requesting fewer
  // processes than exist.
  for (int ii = 0; ii < kNProcesses; ++ii) {
    const pid_t pid = fork();
    PCHECK(pid >= 0);
    if (pid == 0) {
      LOG(INFO) << "In child process.";
      while (true) {
        // This is a "please don't optimize me out" thing for the compiler.
        // Otherwise, the entire if (pid == 0) block can get optimized away...
        asm("");
        continue;
      }
      LOG(FATAL) << "This should be unreachable.";
    } else {
      CHECK_NE(0, pid) << "The compiler is messing with you.";
      children.push_back(pid);
    }
  }

  Top top(&event_loop_, Top::TrackThreadsMode::kDisabled,
          Top::TrackPerThreadInfoMode::kDisabled);
  top.set_track_top_processes(true);
  event_loop_.AddTimer([this]() { event_loop_.Exit(); })
      ->Schedule(event_loop_.monotonic_now() + std::chrono::seconds(2));
  event_loop_.SkipTimingReport();
  event_loop_.SkipAosLog();
  event_loop_.Run();
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);
  fbb.Finish(top.TopProcesses(&fbb, kNProcesses));
  aos::FlatbufferDetachedBuffer<TopProcessesFbs> info = fbb.Release();
  ASSERT_EQ(kNProcesses, info.message().processes()->size());
  double last_cpu = std::numeric_limits<double>::infinity();
  std::set<pid_t> observed_pids;
  int process_index = 0;
  for (const ProcessInfo *info : *info.message().processes()) {
    SCOPED_TRACE(aos::FlatbufferToJson(info));
    ASSERT_EQ(0, observed_pids.count(info->pid()));
    observed_pids.insert(info->pid());
    ASSERT_TRUE(info->has_name());
    // Confirm that the top process has non-zero CPU usage, but allow the
    // lower-down processes to have not been scheduled in the last measurement
    // cycle.
    if (process_index < 1) {
      ASSERT_LT(0.0, info->cpu_usage());
    } else {
      ASSERT_LE(0.0, info->cpu_usage());
    }
    ++process_index;
    ASSERT_GE(last_cpu, info->cpu_usage());
    last_cpu = info->cpu_usage();
    ASSERT_LT(0, info->physical_memory());
  }

  for (const pid_t child : children) {
    kill(child, SIGINT);
  }
}

// Test thgat if we request arbitrarily many processes that we only get back as
// many processes as actually exist and that nothing breaks.
TEST_F(TopTest, AllTopProcesses) {
  constexpr int kNProcesses = 1000000;

  Top top(&event_loop_, Top::TrackThreadsMode::kDisabled,
          Top::TrackPerThreadInfoMode::kDisabled);
  top.set_track_top_processes(true);
  event_loop_.AddTimer([this]() { event_loop_.Exit(); })
      ->Schedule(event_loop_.monotonic_now() + std::chrono::seconds(2));
  event_loop_.Run();
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);
  // There should only be at most 2-3 processes visible inside the bazel
  // sandbox.
  fbb.Finish(top.TopProcesses(&fbb, kNProcesses));
  aos::FlatbufferDetachedBuffer<TopProcessesFbs> info = fbb.Release();
  ASSERT_GT(kNProcesses, info.message().processes()->size());
  double last_cpu = std::numeric_limits<double>::infinity();
  std::set<pid_t> observed_pids;
  for (const ProcessInfo *info : *info.message().processes()) {
    SCOPED_TRACE(aos::FlatbufferToJson(info));
    LOG(INFO) << aos::FlatbufferToJson(info);
    ASSERT_EQ(0, observed_pids.count(info->pid()));
    observed_pids.insert(info->pid());
    ASSERT_TRUE(info->has_name());
    ASSERT_LE(0.0, info->cpu_usage());
    ASSERT_GE(last_cpu, info->cpu_usage());
    last_cpu = info->cpu_usage();
    ASSERT_LE(0, info->physical_memory());
  }
}

}  // namespace aos::util::testing
