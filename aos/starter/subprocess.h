#ifndef AOS_STARTER_SUBPROCESS_H_
#define AOS_STARTER_SUBPROCESS_H_

#include <stdint.h>
#include <sys/signalfd.h>
#include <sys/types.h>

#include <algorithm>
#include <chrono>
#include <filesystem>  // IWYU pragma: keep
#include <functional>
#include <initializer_list>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/log/absl_check.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffers/string.h"
#include "flatbuffers/vector.h"

#include "aos/configuration_generated.h"
#include "aos/events/epoll.h"
#include "aos/events/event_loop.h"
#include "aos/events/event_loop_generated.h"
#include "aos/events/shm_event_loop.h"
#include "aos/ipc_lib/signalfd.h"
#include "aos/macros.h"
#include "aos/starter/starter_generated.h"
#include "aos/starter/starter_rpc_generated.h"
#include "aos/time/time.h"
#include "aos/util/scoped_pipe.h"
#include "aos/util/top.h"

namespace aos::starter {

// Replicates the path resolution that will be attempted by the shell or
// commands like execvp. Doing this manually allows us to conveniently know what
// is actually being executed (rather than, e.g., querying /proc/$pid/exe after
// the execvp() call is executed).
// This is also useful when using the below class with sudo or bash scripts,
// because in those circumstances /proc/$pid/exe contains sudo and /bin/bash (or
// similar binary), rather than the actual thing being executed.
std::filesystem::path ResolvePath(std::string_view command);

// Registers a signalfd listener with the given event loop and calls callback
// whenever a signal is received.
class SignalListener {
 public:
  SignalListener(aos::ShmEventLoop *loop,
                 std::function<void(signalfd_siginfo)> callback);
  SignalListener(aos::EPoll *epoll,
                 std::function<void(signalfd_siginfo)> callback);
  SignalListener(aos::ShmEventLoop *loop,
                 std::function<void(signalfd_siginfo)> callback,
                 std::initializer_list<unsigned int> signals);
  SignalListener(aos::EPoll *epoll,
                 std::function<void(signalfd_siginfo)> callback,
                 std::initializer_list<unsigned int> signals);

  ~SignalListener();

 private:
  aos::EPoll *epoll_;
  std::function<void(signalfd_siginfo)> callback_;
  aos::ipc_lib::SignalFd signalfd_;

  DISALLOW_COPY_AND_ASSIGN(SignalListener);
};

// Reads the PIDs from a cgroup's tasks file and returns information about them
// as a formatted string. Returns an empty string if no processes are found.
// Exposed for testing purposes.
std::string GetCGroupProcessesInfo(const std::filesystem::path &cgroup_path);

// Attempts to remove a cgroup directory with retry logic.
// The kernel may need time to clean up processes even after they've exited,
// so we retry with delays if the directory is busy.
// Exposed for testing purposes.
void RemoveCGroupWithRetry(const std::filesystem::path &cgroup_path);

// cgroupv2 allows removing a cgroup with rmdir if it has no children or live
// processes (zombie processes are OK). Recursively cleans child directories
// first, then attempts to remove the directory. Returns true if
// cleanup succeeded, false otherwise.
//
// Note: We don't migrate processes from child to parent before removal.
// Migrating orphaned live processes could put them under wrong resource
// limits/policies. If rmdir fails due to live processes, we surface the error
// rather than silently moving processes to an unexpected cgroup.
// Returns true if cleanup succeeded, false otherwise.
// Exposed for testing purposes.
bool RemoveCGroupV2(const std::string &cgroup_path);

// cgroupv2's "no internal processes" rule prevents writing to cgroup.procs
// when child cgroups exist. Cleans all subdirectories recursively without
// removing the parent directory. Returns true if
// cleanup succeeded, false otherwise.
// Exposed for testing only.
bool RemoveCGroupV2Children(const std::string &cgroup_path);

// Class to use the V1 cgroup API to limit memory usage.
class MemoryCGroup {
 public:
  // Enum to control if MemoryCGroup should create the cgroup and remove it on
  // its own, or if it should assume it already exists and just use it.
  enum class Create {
    kDoCreate,
    kDoNotCreate,
  };

  virtual ~MemoryCGroup() {}

  // Adds a thread ID to be managed by the cgroup.
  virtual void AddTid(pid_t pid = 0) = 0;

  // Sets the provided limit to the provided value.
  virtual void SetMemoryLimit(uint64_t limit_value) = 0;

  // Delegates ownership of this cgroup to the given user/group so that
  // non-root child processes can manage their own cgroup subtree.
  virtual void Delegate(std::string_view /*username*/) {}
};

// Factory class to create cgroups for applications.
//
// This needs to be a factory because cgroups v1 is very different from cgroups
// v2.  With cgroups v2, we need kernel resources to be allocated with a
// lifetime > the lifetime of the created sub-cgroups.
//
// Our Cgroups v1 usage is much more flat.
class CGroupManager {
 public:
  virtual ~CGroupManager() {}

  // Under cgroups v2, the new cgroup is created or searched for as a child of a
  // cgroup owned by this user named "user_<user_name>".
  // Under cgroups v1, user_name has no effect.
  // Note that if user name is empty, the current user is assumed.
  virtual std::unique_ptr<MemoryCGroup> MakeCGroup(
      std::string_view name, std::string_view user_name = "") = 0;
};

// Auto-detects what cgroups version is on this machine and constructs a manager
// for it.
// If should_create_cgroups is kDoCreate, the manager will create cgroups and
// also create a new cgroup for the current process to enable memory
// controllers. If kDoNotCreate, the manager will only manage existing cgroups.
std::unique_ptr<CGroupManager> MakeCGroupManager(
    MemoryCGroup::Create should_create_cgroups =
        MemoryCGroup::Create::kDoCreate);

// Manages a running process, allowing starting and stopping, and restarting
// automatically.
class Application {
 public:
  enum class QuietLogging {
    kYes,
    kNo,
    // For debugging child processes not behaving as expected. When a child
    // experiences an event such as exiting with an error code or dying to due a
    // signal, this option will cause a log statement to be printed.
    kNotForDebugging,
  };
  Application(const aos::Application *application, aos::EventLoop *event_loop,
              std::function<void()> on_change,
              std::unique_ptr<MemoryCGroup> memory_cgroup,
              std::vector<std::unique_ptr<MemoryCGroup>> extra_cgroups = {},
              QuietLogging quiet_flag = QuietLogging::kNo);

  Application(const aos::Application *application, aos::EventLoop *event_loop,
              std::function<void()> on_change,
              std::unique_ptr<MemoryCGroup> memory_cgroup,
              QuietLogging quiet_flag)
      : Application(application, event_loop, on_change,
                    std::move(memory_cgroup), {}, quiet_flag) {}

  // executable_name is the actual executable path.
  // When sudo is not used, name is used as argv[0] when exec'ing
  // executable_name. When sudo is used it's not possible to pass in a
  // distinct argv[0].
  Application(std::string_view name, std::string_view executable_name,
              aos::EventLoop *event_loop, std::function<void()> on_change,
              std::unique_ptr<MemoryCGroup> memory_cgroup,
              std::vector<std::unique_ptr<MemoryCGroup>> extra_cgroups = {},
              QuietLogging quiet_flag = QuietLogging::kNo);

  Application(std::string_view name, std::string_view executable_name,
              aos::EventLoop *event_loop, std::function<void()> on_change,
              std::unique_ptr<MemoryCGroup> memory_cgroup,
              QuietLogging quiet_flag)
      : Application(name, executable_name, event_loop, on_change,
                    std::move(memory_cgroup), {}, quiet_flag) {}

  ~Application();

  flatbuffers::Offset<aos::starter::ApplicationStatus> PopulateStatus(
      flatbuffers::FlatBufferBuilder *builder, util::Top *top);
  aos::starter::State status() const { return PublicState(status_); };

  // Returns the last pid of this process. -1 if not started yet.
  pid_t get_pid() const { return pid_; }

  // Handles a SIGCHLD signal received by the parent. Does nothing if this
  // process was not the target. Returns true if this Application should be
  // removed.
  bool MaybeHandleSignal();
  void DisableChildDeathPolling() { child_status_handler_->Disable(); }

  // Handles a command. May do nothing if application is already in the desired
  // state.
  void HandleCommand(aos::starter::Command cmd);

  void Start() { HandleCommand(aos::starter::Command::START); }

  // Stops the command by sending a SIGINT first, followed by a SIGKILL if it's
  // still alive in 1s.
  void Stop() { HandleCommand(aos::starter::Command::STOP); }

  // Stops the command the same way as Stop() does, but updates internal state
  // to reflect that the application was terminated.
  // Once an application is terminated, it cannot be restarted.
  void Terminate();

  // Adds a callback which gets notified when the application changes state.
  // This is in addition to any existing callbacks and doesn't replace any of
  // them.
  void AddOnChange(std::function<void()> fn) {
    on_change_.emplace_back(std::move(fn));
  }

  void set_args(std::vector<std::string> args);
  void set_capture_stdout(bool capture);
  void set_capture_stderr(bool capture);
  void set_run_as_sudo(bool value) { run_as_sudo_ = value; }

  // Sets the time for a process to stop gracefully. If an application is asked
  // to stop, but doesn't stop within the specified time limit, then it is
  // forcefully killed. Defaults to 1 second unless overridden by the
  // aos::Application instance in the constructor.
  void set_stop_grace_period(std::chrono::nanoseconds stop_grace_period) {
    stop_grace_period_ = stop_grace_period;
  }

  bool autostart() const { return autostart_; }

  bool autorestart() const { return autorestart_; }
  void set_autorestart(bool autorestart) { autorestart_ = autorestart; }

  LastStopReason stop_reason() const { return stop_reason_; }

  const std::string &GetStdout();
  const std::string &GetStderr();
  std::optional<int> exit_code() const { return exit_code_; }

  // Sets the memory limit for the application to the provided limit.
  void SetMemoryLimit(size_t limit) {
    ABSL_CHECK(main_cgroup_);

    main_cgroup_->SetMemoryLimit(limit);
  }

  // Sets the cgroup and memory limit to a pre-existing cgroup which is
  // externally managed.  This lets us configure the cgroup of an application
  // without root access.
  void SetExistingCgroupMemoryLimit(size_t limit) {
    ABSL_CHECK(main_cgroup_);

    main_cgroup_->SetMemoryLimit(limit);
  }

  // Observe a timing report message, and save it if it is relevant to us.
  // It is the responsibility of the caller to manage this, because the lifetime
  // of the Application itself is such that it cannot own Fetchers readily.
  void ObserveTimingReport(const aos::monotonic_clock::time_point send_time,
                           const aos::timing::Report *msg);

  FileState UpdateFileState();

 private:
  typedef aos::util::ScopedPipe::PipePair PipePair;

  static constexpr const char *const kSudo{"sudo"};

  void set_args(
      const flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>>
          &args);

  void DoStart();

  void DoStop(bool restart);

  void QueueStart();

  void OnChange();

  // Copy flatbuffer vector of strings to vector of std::string.
  static std::vector<std::string> FbsVectorToVector(
      const flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>> &v);

  static std::optional<uid_t> FindUid(const char *name);
  static std::optional<gid_t> FindPrimaryGidForUser(const char *name);

  void FetchOutputs();

  // Provides an std::vector of the args (such that CArgs().data() ends up being
  // suitable to pass to execve()).
  // The points are invalidated when args_ changes (e.g., due to a set_args
  // call).
  std::vector<char *> CArgs();

  // Next unique id for all applications
  static inline uint64_t next_id_ = 0;

  std::string name_;
  std::filesystem::path path_;
  // Inode of path_ immediately prior to the most recent fork() call.
  ino_t pre_fork_inode_;
  FileState file_state_ = FileState::NOT_RUNNING;
  std::vector<std::string> args_;
  std::string user_name_;
  std::optional<uid_t> user_;
  std::optional<gid_t> group_;
  bool run_as_sudo_ = false;
  std::chrono::nanoseconds stop_grace_period_ = std::chrono::seconds(1);

  bool capture_stdout_ = false;
  PipePair stdout_pipes_;
  std::string stdout_;
  bool capture_stderr_ = false;
  PipePair stderr_pipes_;
  std::string stderr_;

  pid_t pid_ = -1;
  PipePair status_pipes_;
  uint64_t id_ = 0;
  std::optional<int> exit_code_;
  std::optional<aos::monotonic_clock::time_point> start_time_, exit_time_;
  // Internal command vocabulary. Unlike the public Command enum, this
  // includes a kTerminate command used to stop the application and remove it
  // from the map once it reaches STOPPED.
  enum class ApplicationCommand {
    kNoOp,
    kStart,
    kStop,
    kRestart,
    kTerminate,
  };
  std::optional<ApplicationCommand> command_;

  bool autostart_ = false;
  bool autorestart_ = false;

  // Internal process state vocabulary. Includes kForking, which is never
  // visible externally.
  enum class ApplicationInternalState {
    kWaiting,
    kForking,
    kStarting,
    kRunning,
    kStopping,
    kStopped,
  };

  // Maps the internal process state to the public flatbuffer State enum.
  static aos::starter::State PublicState(
      ApplicationInternalState internal_state);

  // Events that drive the internal state machine.
  enum class Event {
    kStartCalled,
    kStopCalled,
    kRestartCalled,
    kTerminateCalled,
    kTimeout,
    kChildExited,
    kForkSuccess,
    kForkFailed,
  };

  // Side effects performed by transitions.
  enum class Action {
    kNone,
    kDoStart,
    kHandleStarted,
    kDoStop,
    kDoKill,
    kDoWait,
  };

  // A single row in the event-driven transition table.
  struct TransitionRule {
    ApplicationInternalState from_status;
    Event event;
    std::function<bool(bool autorestart,
                       std::optional<ApplicationCommand> last_command,
                       bool alive)>
        guard;
    ApplicationInternalState next_status;
    std::optional<ApplicationCommand> next_command;
    std::optional<Action> action;
  };

  // Dispatches an event through the transition table. Stub for now.
  void HandleEvent(Event event);

  ApplicationInternalState status_ = ApplicationInternalState::kStopped;
  aos::starter::LastStopReason stop_reason_ =
      aos::starter::LastStopReason::STOP_REQUESTED;

  aos::EventLoop *event_loop_;
  aos::TimerHandler *start_timer_, *restart_timer_, *stop_timer_, *pipe_timer_,
      *child_status_handler_;

  // Version string from the most recent valid timing report for this
  // application. Cleared when the application restarts.
  std::optional<std::string> latest_timing_report_version_;
  aos::monotonic_clock::time_point last_timing_report_ =
      aos::monotonic_clock::min_time;

  std::vector<std::function<void()>> on_change_;

  std::unique_ptr<MemoryCGroup> main_cgroup_;
  std::vector<std::unique_ptr<MemoryCGroup>> extra_cgroups_;

  QuietLogging quiet_flag_ = QuietLogging::kNo;

  DISALLOW_COPY_AND_ASSIGN(Application);
};

}  // namespace aos::starter
#endif  // AOS_STARTER_SUBPROCESS_H_
