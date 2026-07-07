#include "aos/starter/subprocess.h"

#include <errno.h>
#include <grp.h>
#include <pwd.h>
#include <signal.h>
#include <stdlib.h>
#include <sys/prctl.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <compare>
#include <fstream>
#include <iterator>
#include <ostream>
#include <ratio>
#include <unordered_map>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/log.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_split.h"

#include "aos/util/application_name.h"
#include "aos/util/file.h"
#include "aos/util/process_info_generated.h"

ABSL_FLAG(bool, require_path_resolution, true,
          "If set, names passed to Subprocess must correspond to a real file.");

namespace aos::starter {

// Blocks all signals while an instance of this class is in scope.
class ScopedCompleteSignalBlocker {
 public:
  ScopedCompleteSignalBlocker() {
    sigset_t mask;
    sigfillset(&mask);
    // Remember the current mask.
    ABSL_PCHECK(sigprocmask(SIG_SETMASK, &mask, &old_mask_) == 0);
  }

  ~ScopedCompleteSignalBlocker() {
    // Restore the remembered mask.
    ABSL_PCHECK(sigprocmask(SIG_SETMASK, &old_mask_, nullptr) == 0);
  }

 private:
  sigset_t old_mask_;
};

namespace {
std::optional<ino_t> GetInodeForPath(const std::filesystem::path &path) {
  struct stat stat_buf;
  if (0 != stat(path.c_str(), &stat_buf)) {
    return std::nullopt;
  }
  return stat_buf.st_ino;
}
bool InodeChanged(const std::filesystem::path &path, ino_t previous_inode) {
  const std::optional<ino_t> current_inode = GetInodeForPath(path);
  if (!current_inode.has_value()) {
    return true;
  }
  return current_inode.value() != previous_inode;
}
}  // namespace

std::filesystem::path ResolvePath(std::string_view command) {
  std::filesystem::path command_path = command;
  if (command.find("/") != std::string_view::npos) {
    if (!std::filesystem::exists(command_path)) {
      if (absl::GetFlag(FLAGS_require_path_resolution)) {
        ABSL_LOG(FATAL) << ": " << command << " does not exist.";
      } else {
        return "";
      }
    }
    return std::filesystem::canonical(command_path);
  }
  const char *system_path = getenv("PATH");
  std::string system_path_buffer;
  if (system_path == nullptr) {
    const size_t default_path_length = confstr(_CS_PATH, nullptr, 0);
    ABSL_PCHECK(default_path_length != 0) << ": Unable to resolve " << command;
    system_path_buffer.resize(default_path_length);
    confstr(_CS_PATH, system_path_buffer.data(), system_path_buffer.size());
    system_path = system_path_buffer.c_str();
    ABSL_VLOG(2) << "Using default path of " << system_path
                 << " in the absence of PATH being set.";
  }
  const std::vector<std::string_view> search_paths =
      absl::StrSplit(system_path, ':');
  for (const std::string_view search_path : search_paths) {
    const std::filesystem::path candidate =
        std::filesystem::path(search_path) / command_path;
    if (std::filesystem::exists(candidate)) {
      return std::filesystem::canonical(candidate);
    }
  }
  if (absl::GetFlag(FLAGS_require_path_resolution)) {
    ABSL_LOG(FATAL) << "Unable to resolve " << command;
  } else {
    return "";
  }
}

namespace {
// RAII class to become root and restore back to the original user and group
// afterwards.
class Sudo {
 public:
  Sudo() {
    // Save what we were.
    ABSL_PCHECK(getresuid(&ruid_, &euid_, &suid_) == 0);
    ABSL_PCHECK(getresgid(&rgid_, &egid_, &sgid_) == 0);

    // Become root.
    ABSL_PCHECK(
        setresuid(/* ruid */ 0 /* root */, /* euid */ 0, /* suid */ 0) == 0)
        << ": Failed to become root";
    ABSL_PCHECK(
        setresgid(/* ruid */ 0 /* root */, /* euid */ 0, /* suid */ 0) == 0)
        << ": Failed to become root";
  }

  ~Sudo() {
    // And recover.
    ABSL_PCHECK(setresgid(rgid_, egid_, sgid_) == 0);
    ABSL_PCHECK(setresuid(ruid_, euid_, suid_) == 0);
  }

  uid_t ruid_, euid_, suid_;
  gid_t rgid_, egid_, sgid_;
};

// Escalates to root when suid and sgid are both 0. If the current suid and
// sgid are 0, declares the RAII Sudo and calls the lambda. If the current suid
// and sgid are not 0, calls the lambda without using the Sudo class.
template <typename F>
void MaybeSudo(F &&f) {
  uid_t ruid, euid, suid;
  gid_t rgid, egid, sgid;
  ABSL_PCHECK(getresuid(&ruid, &euid, &suid) == 0);
  ABSL_PCHECK(getresgid(&rgid, &egid, &sgid) == 0);

  if (suid == 0 && sgid == 0) {
    Sudo sudo;
    f();
  } else {
    f();
  }
}

// Delegates ownership of a cgroup to the given user/group so that
// non-root child processes can manage their own cgroup subtree.
void DelegateCGroup(const std::string &cgroup_path, std::string_view username) {
  struct passwd *user_data = getpwnam(std::string(username).c_str());
  if (user_data == nullptr) {
    ABSL_LOG(FATAL) << "Could not find user " << username;
  }

  // Recursively chown the cgroup directory.
  ABSL_PCHECK(
      chown(cgroup_path.c_str(), user_data->pw_uid, user_data->pw_gid) == 0)
      << ": Failed to chown cgroup " << cgroup_path;
  std::error_code ec;
  for (const auto &entry :
       std::filesystem::recursive_directory_iterator(cgroup_path, ec)) {
    if (entry.is_regular_file() || entry.is_directory()) {
      ABSL_PCHECK(chown(entry.path().c_str(), user_data->pw_uid,
                        user_data->pw_gid) == 0)
          << ": Failed to chown " << entry.path();
    }
  }
  if (ec) {
    ABSL_LOG(WARNING) << "Error iterating cgroup " << cgroup_path << ": "
                      << ec.message();
  }
}
}  // namespace

class MemoryCGroupV1 : public MemoryCGroup {
 public:
  MemoryCGroupV1(std::string_view name, Create should_create)
      : cgroup_(absl::StrCat("/sys/fs/cgroup/memory/aos_", name)),
        should_create_(should_create) {
    if (should_create_ == Create::kDoCreate) {
      Sudo sudo;
      int ret = mkdir(cgroup_.c_str(), 0755);

      if (ret != 0) {
        if (errno == EEXIST) {
          ABSL_PCHECK(rmdir(cgroup_.c_str()) == 0)
              << ": Failed to remove previous cgroup " << cgroup_;
          ret = mkdir(cgroup_.c_str(), 0755);
        }
      }

      if (ret != 0) {
        ABSL_PLOG(FATAL) << ": Failed to create cgroup " << cgroup_
                         << ", do you have permission?";
      }
    }
  }
  ~MemoryCGroupV1() {
    if (should_create_ == Create::kDoCreate) {
      Sudo sudo;
      RemoveCGroupWithRetry(cgroup_);
    }
  }

  void AddTid(pid_t pid) override {
    if (pid == 0) {
      pid = getpid();
    }
    MaybeSudo([&] {
      util::WriteStringToFileOrDie(absl::StrCat(cgroup_, "/tasks").c_str(),
                                   std::to_string(pid));
    });
  }

  void SetMemoryLimit(uint64_t limit_value) override {
    MaybeSudo([&] {
      util::WriteStringToFileOrDie(
          absl::StrCat(cgroup_, "/memory.limit_in_bytes").c_str(),
          std::to_string(limit_value));
    });
  }

  void Delegate(std::string_view username) override {
    MaybeSudo([&] { DelegateCGroup(cgroup_, username); });
  }

 private:
  std::string cgroup_;
  Create should_create_;
};

class CGroupManagerV1 : public CGroupManager {
 public:
  CGroupManagerV1(MemoryCGroup::Create should_create_cgroups)
      : should_create_cgroups_(should_create_cgroups) {}
  virtual ~CGroupManagerV1() {}

  std::unique_ptr<MemoryCGroup> MakeCGroup(
      std::string_view name, std::string_view /* user_name */) override {
    return std::make_unique<MemoryCGroupV1>(name, should_create_cgroups_);
  }

 private:
  MemoryCGroup::Create should_create_cgroups_;
};

bool RemoveCGroupV2Hierarchy(const std::string &cgroup_path, bool remove_self) {
  namespace fs = std::filesystem;

  std::error_code ec;
  for (const auto &entry : fs::directory_iterator(cgroup_path, ec)) {
    if (entry.is_directory() && !entry.is_symlink()) {
      if (!RemoveCGroupV2Hierarchy(entry.path().string(), true)) {
        return false;
      }
    }
  }

  if (ec) {
    ABSL_LOG(WARNING) << "Failed to iterate cgroup " << cgroup_path << ": "
                      << ec.message() << " (errno=" << ec.value() << ")";
    return false;
  }

  if (remove_self) {
    if (rmdir(cgroup_path.c_str()) != 0) {
      ABSL_PLOG(WARNING) << ": Failed to remove stale cgroup " << cgroup_path;
      return false;
    }
  }

  return true;
}

bool RemoveCGroupV2(const std::string &cgroup_path) {
  return RemoveCGroupV2Hierarchy(cgroup_path, true);
}

// This is currently dead code, consider removing later.
bool RemoveCGroupV2Children(const std::string &cgroup_path) {
  return RemoveCGroupV2Hierarchy(cgroup_path, false);
}

class MemoryCGroupV2 : public MemoryCGroup {
 public:
  MemoryCGroupV2(std::string_view name,
                 Create should_create = Create::kDoCreate)
      : cgroup_(name), should_create_(should_create) {
    if (should_create_ == Create::kDoCreate) {
      MaybeSudo([&] {
        int ret = mkdir(cgroup_.c_str(), 0755);

        if (ret != 0) {
          if (errno == EEXIST) {
            RemoveCGroupV2(cgroup_);
            ret = mkdir(cgroup_.c_str(), 0755);
          }
        }

        if (ret != 0) {
          ABSL_PLOG(FATAL) << ": Failed to create cgroup aos_" << cgroup_
                           << ", do you have permission?";
        }
      });
    } else if (!std::filesystem::exists(cgroup_)) {
      ABSL_PLOG(FATAL) << ": Cgroup " << cgroup_ << "does not exist.";
    }
  }
  ~MemoryCGroupV2() {
    if (should_create_ == Create::kDoCreate) {
      MaybeSudo([&] { RemoveCGroupV2(cgroup_); });
    }
  }

  void AddTid(pid_t pid) override {
    MaybeSudo([&] {
      if (pid == 0) {
        pid = getpid();
      }
      util::WriteStringToFileOrDie(
          absl::StrCat(cgroup_, "/cgroup.procs").c_str(), std::to_string(pid));
    });
  }

  void SetMemoryLimit(uint64_t limit_value) override {
    MaybeSudo([&] {
      util::WriteStringToFileOrDie(absl::StrCat(cgroup_, "/memory.max").c_str(),
                                   std::to_string(limit_value));
    });
  }

  void Delegate(std::string_view username) override {
    MaybeSudo([&] { DelegateCGroup(cgroup_, username); });
  }

 private:
  std::string cgroup_;
  Create should_create_;
};

// cgroups v2 won't let you enable cgroups managers on cgroups which have
// anything in them directly.  This requires us to move ourselves (starter) to a
// nested cgroup.
//
// Step 1, make a new cgroup for starterd.
// Step 2, move starterd to a new cgroup inside the current cgroup.
// Step 3, Enable memory cgroups on the original cgroup, so the nested cgroups
// we create can limit memory usage.

// Manages a per-user intermediate cgroup directory for cgroupsv2.
// This intermediate cgroup is a parent for all application cgroups running as
// a specific user, and has +memory enabled in subtree_control so child cgroups
// can set memory limits.
class UserCGroupV2 {
 public:
  UserCGroupV2(std::string_view root_path, std::string_view username,
               MemoryCGroup::Create should_create)
      : cgroup_(absl::StrCat(root_path, "/user_", username)),
        should_create_(should_create) {
    if (should_create_ != MemoryCGroup::Create::kDoCreate) {
      return;
    }
    MaybeSudo([&] {
      int ret = mkdir(cgroup_.c_str(), 0755);
      if (ret != 0 && errno == EEXIST) {
        RemoveCGroupV2(cgroup_);
        ret = mkdir(cgroup_.c_str(), 0755);
      }
      ABSL_PCHECK(ret == 0) << ": Failed to create user cgroup " << cgroup_;
      // Enable memory controller on the user intermediate cgroup so child
      // cgroups can set memory limits.
      util::WriteStringToFileOrDie(
          absl::StrCat(cgroup_, "/cgroup.subtree_control"), "+memory");

      // Delegate the cgroup to the specified user.
      DelegateCGroup(cgroup_, username);
    });
  }

  ~UserCGroupV2() {
    if (should_create_ != MemoryCGroup::Create::kDoCreate) {
      return;
    }
    MaybeSudo([&] {
      // Best-effort cleanup: disable memory controller and remove directory.
      util::MaybeWriteStringToFile(
          absl::StrCat(cgroup_, "/cgroup.subtree_control"), "-memory");
      RemoveCGroupV2(cgroup_);
    });
  }

  const std::string &cgroup() const { return cgroup_; }

 private:
  std::string cgroup_;
  MemoryCGroup::Create should_create_;
};

class CGroupManagerV2 : public CGroupManager {
 public:
  explicit CGroupManagerV2(std::string name,
                           MemoryCGroup::Create should_create_cgroups)
      : cgroup_(name),
        this_cgroup_(
            absl::StrCat(cgroup_, "/", absl::GetFlag(FLAGS_application_name))),
        should_create_cgroups_(should_create_cgroups) {
    // Resolve the starter's effective username to use as default when apps
    // don't specify a user.
    struct passwd *pw = getpwuid(getuid());
    if (pw != nullptr) {
      default_user_name_ = pw->pw_name;
    }

    if (should_create_cgroups_ != MemoryCGroup::Create::kDoCreate) {
      return;
    }
    MaybeSudo([&] {
      // First, make a cgroup to move ourselves to.
      int ret = mkdir(this_cgroup_.c_str(), 0755);

      if (ret != 0) {
        if (errno == EEXIST) {
          // Nuke the previous one so we don't retain any memory that
          // were in it.  Also, cgroups v2 can manage more than memory, so there
          // is more to diverge.
          ABSL_PCHECK(rmdir(this_cgroup_.c_str()) == 0)
              << ": Failed to remove previous cgroup " << this_cgroup_;
          ret = mkdir(this_cgroup_.c_str(), 0755);
        }
      }

      ABSL_PCHECK(ret == 0) << ": Failed to create cgroup " << this_cgroup_
                            << ", do you have permission?";

      const pid_t pid = getpid();

      // Then, move ourselves to the sub cgroup.
      util::WriteStringToFileOrDie(absl::StrCat(this_cgroup_, "/cgroup.procs"),
                                   std::to_string(pid));

      // Now, enable memory
      util::WriteStringToFileOrDie(
          absl::StrCat(cgroup_, "/cgroup.subtree_control"), "+memory");
    });
  }

  ~CGroupManagerV2() override {
    if (should_create_cgroups_ != MemoryCGroup::Create::kDoCreate) {
      return;
    }
    const pid_t pid = getpid();
    MaybeSudo([&] {
      // Best-effort: we may be shutting down after a crash.
      util::MaybeWriteStringToFile(
          absl::StrCat(cgroup_, "/cgroup.subtree_control"), "-memory");
      util::MaybeWriteStringToFile(absl::StrCat(cgroup_, "/cgroup.procs"),
                                   std::to_string(pid));
      if (rmdir(this_cgroup_.c_str()) != 0) {
        ABSL_PLOG(WARNING) << "Failed to remove cgroup " << this_cgroup_;
      }
    });
  }

  std::unique_ptr<MemoryCGroup> MakeCGroup(
      std::string_view name, std::string_view user_name) override {
    // Use the provided user_name, or fall back to the starter's default
    // username if empty.
    std::string resolved_user_name =
        user_name.empty() ? default_user_name_ : std::string(user_name);

    // Look up or create the per-user intermediate cgroup.
    auto it = user_cgroups_.find(resolved_user_name);
    if (it == user_cgroups_.end()) {
      it = user_cgroups_
               .emplace(std::piecewise_construct,
                        std::forward_as_tuple(resolved_user_name),
                        std::forward_as_tuple(cgroup_, resolved_user_name,
                                              should_create_cgroups_))
               .first;
    }

    const std::string &user_cgroup_path = it->second.cgroup();

    // Default behavior when creating cgroups
    if (should_create_cgroups_ == MemoryCGroup::Create::kDoCreate) {
      return std::make_unique<MemoryCGroupV2>(
          absl::StrCat(user_cgroup_path, "/aos_", name),
          should_create_cgroups_);
    }

    // Fall back to searching up the hierarchy from the user cgroup path.
    std::string cgroup_path = user_cgroup_path;
    while (true) {
      const std::string candidate = absl::StrCat(cgroup_path, "/aos_", name);
      if (std::filesystem::exists(candidate)) {
        return std::make_unique<MemoryCGroupV2>(candidate,
                                                should_create_cgroups_);
      }
      // Strip off the last element of the path, to look up a level.
      size_t last_slash = cgroup_path.rfind('/');
      if (last_slash == std::string::npos || last_slash == 0) {
        // No more parent directories to try
        ABSL_LOG(FATAL) << "Could not find existing cgroup for " << name
                        << ", tried starting from " << cgroup_;
      }
      cgroup_path = cgroup_path.substr(0, last_slash);
    }
  }

 private:
  std::string cgroup_;
  std::string this_cgroup_;
  MemoryCGroup::Create should_create_cgroups_;
  // Default username to use when apps don't specify a user.
  std::string default_user_name_;
  // Map of username to per-user intermediate cgroup.
  std::unordered_map<std::string, UserCGroupV2> user_cgroups_;
};

std::unique_ptr<CGroupManager> MakeCGroupManager(
    MemoryCGroup::Create should_create_cgroups) {
  // The kernel only supports cgroups v1 or v2, not both.  The standard
  // autodetection mechanism for other programs appears to be to check for the
  // cgroup.controllers file, which signifies that cgroups v2 is in use.
  if (!std::filesystem::exists("/sys/fs/cgroup/cgroup.controllers")) {
    return std::make_unique<CGroupManagerV1>(should_create_cgroups);
  } else {
    // Read the current cgroup out of /proc/self/cgroup
    std::string cgroup = util::ReadFileToStringOrDie("/proc/self/cgroup");
    // Remove the trailing newline.
    cgroup.erase(cgroup.length() - 1);
    ABSL_CHECK(cgroup.starts_with("0::"));
    cgroup = "/sys/fs/cgroup" + cgroup.substr(3);
    ABSL_LOG(INFO) << "Cgroup of: " << cgroup;
    return std::make_unique<CGroupManagerV2>(cgroup, should_create_cgroups);
  }
};

std::string GetCGroupProcessesInfo(const std::filesystem::path &cgroup_path) {
  const std::filesystem::path tasks_file = cgroup_path / "tasks";

  std::ifstream tasks(tasks_file);
  if (!tasks.is_open()) {
    return absl::StrCat("Could not open ", tasks_file.string(),
                        " to read PIDs");
  }

  std::vector<pid_t> pids;
  pid_t pid;
  while (tasks >> pid) {
    pids.push_back(pid);
  }

  if (pids.empty()) {
    return "";
  }

  std::string result =
      absl::StrCat("Found ", pids.size(), " process(es) still in cgroup ",
                   cgroup_path.string(), ":");

  for (pid_t pid : pids) {
    absl::StrAppend(&result, " [PID ", pid);

    // Dump /proc/<pid>/comm.
    const std::filesystem::path comm_file =
        std::filesystem::path("/proc") / std::to_string(pid) / "comm";
    const std::optional<std::string> comm_contents =
        util::MaybeReadFileToString(comm_file.native());
    if (comm_contents.has_value()) {
      // Trim trailing newline from comm if present.
      std::string_view comm_view = *comm_contents;
      if (!comm_view.empty() && comm_view.back() == '\n') {
        comm_view.remove_suffix(1);
      }
      absl::StrAppend(&result, " comm:", comm_view);
    }

    // Dump /proc/<pid>/stat.
    const std::filesystem::path stat_file =
        std::filesystem::path("/proc") / std::to_string(pid) / "stat";
    const std::optional<std::string> stat_contents =
        util::MaybeReadFileToString(stat_file.native());
    if (stat_contents.has_value()) {
      // Trim trailing newline from stat if present.
      std::string_view stat_view = stat_contents.value();
      if (!stat_view.empty() && stat_view.back() == '\n') {
        stat_view.remove_suffix(1);
      }
      absl::StrAppend(&result, " stat:", stat_view);
    }

    absl::StrAppend(&result, "]");
  }

  return result;
}

void RemoveCGroupWithRetry(const std::filesystem::path &cgroup_path) {
  // Check if the path exists first to avoid retrying on non-existent paths.
  if (!std::filesystem::exists(cgroup_path)) {
    ABSL_LOG(WARNING) << "Cgroup " << cgroup_path
                      << " does not exist, nothing to remove";
    return;
  }

  constexpr int kMaxRetries = 5;
  constexpr std::chrono::seconds kRetryDelay{1};

  std::error_code last_error;
  for (int attempt = 0; attempt < kMaxRetries; ++attempt) {
    std::error_code ec;
    if (std::filesystem::remove(cgroup_path, ec)) {
      if (attempt > 0) {
        ABSL_LOG(INFO) << "Successfully removed cgroup " << cgroup_path
                       << " after " << (attempt + 1) << " attempts";
      }
      return;
    }

    last_error = ec;

    // Cgroup is busy or not empty, wait and retry.
    if (attempt < kMaxRetries - 1) {
      const std::string process_info = GetCGroupProcessesInfo(cgroup_path);
      ABSL_LOG(WARNING) << "Failed to remove cgroup " << cgroup_path
                        << " (attempt " << (attempt + 1) << "/" << kMaxRetries
                        << "): " << ec.message() << ". Retrying in 1 second... "
                        << (process_info.empty() ? "" : process_info);

      std::this_thread::sleep_for(kRetryDelay);
    }
  }

  // All attempts failed, log final state.
  const std::string final_process_info = GetCGroupProcessesInfo(cgroup_path);
  ABSL_LOG(ERROR) << "Failed to remove cgroup " << cgroup_path << " after "
                  << kMaxRetries
                  << " attempts. Last error: " << last_error.message() << " ("
                  << last_error.value() << "). Final cgroup state: "
                  << (final_process_info.empty() ? "(no processes found)"
                                                 : final_process_info);

  ABSL_LOG(FATAL) << "Giving up on removing cgroup " << cgroup_path;
}

SignalListener::SignalListener(aos::ShmEventLoop *loop,
                               std::function<void(signalfd_siginfo)> callback)
    : SignalListener(loop->epoll(), std::move(callback)) {}

SignalListener::SignalListener(aos::EPoll *epoll,
                               std::function<void(signalfd_siginfo)> callback)
    : SignalListener(epoll, callback,
                     {SIGHUP, SIGINT, SIGQUIT, SIGABRT, SIGFPE, SIGSEGV,
                      SIGPIPE, SIGTERM, SIGBUS, SIGXCPU, SIGCHLD}) {}

SignalListener::SignalListener(aos::ShmEventLoop *loop,
                               std::function<void(signalfd_siginfo)> callback,
                               std::initializer_list<unsigned int> signals)
    : SignalListener(loop->epoll(), std::move(callback), std::move(signals)) {}

SignalListener::SignalListener(aos::EPoll *epoll,
                               std::function<void(signalfd_siginfo)> callback,
                               std::initializer_list<unsigned int> signals)
    : epoll_(epoll), callback_(std::move(callback)), signalfd_(signals) {
  epoll_->OnReadable(signalfd_.fd(), [this] {
    signalfd_siginfo info = signalfd_.Read();

    if (info.ssi_signo == 0) {
      ABSL_LOG(WARNING) << "Could not read " << sizeof(signalfd_siginfo)
                        << " bytes";
      return;
    }

    callback_(info);
  });
}

SignalListener::~SignalListener() { epoll_->DeleteFd(signalfd_.fd()); }

Application::Application(
    std::string_view name, std::string_view executable_name,
    aos::EventLoop *event_loop, std::function<void()> on_change,
    std::unique_ptr<MemoryCGroup> memory_cgroup,
    std::vector<std::unique_ptr<MemoryCGroup>> extra_cgroups,
    QuietLogging quiet_flag)
    : name_(name),
      path_(ResolvePath(executable_name)),
      event_loop_(event_loop),
      start_timer_(event_loop_->AddTimer([this] {
        status_ = aos::starter::State::RUNNING;
        ABSL_LOG_IF(INFO, quiet_flag_ == QuietLogging::kNo)
            << "Started '" << name_ << "' pid: " << pid_;
        // Check if the file on disk changed while we were starting up. We allow
        // this state for the same reason that we don't just use /proc/$pid/exe
        // to determine if the file is deleted--we may be running a script or
        // sudo or the such and determining the state of the file that we
        // actually care about sounds like more work than we want to deal with.
        if (InodeChanged(path_, pre_fork_inode_)) {
          file_state_ = FileState::CHANGED_DURING_STARTUP;
        } else {
          file_state_ = FileState::NO_CHANGE;
        }

        OnChange();
      })),
      restart_timer_(event_loop_->AddTimer([this] { DoStart(); })),
      stop_timer_(event_loop_->AddTimer([this] {
        if (kill(pid_, SIGKILL) == 0) {
          ABSL_LOG_IF(WARNING,
                      quiet_flag_ == QuietLogging::kNo ||
                          quiet_flag_ == QuietLogging::kNotForDebugging)
              << "Failed to stop, sending SIGKILL to '" << name_
              << "' pid: " << pid_;
        } else {
          ABSL_PLOG_IF(WARNING,
                       quiet_flag_ == QuietLogging::kNo ||
                           quiet_flag_ == QuietLogging::kNotForDebugging)
              << "Failed to send SIGKILL to '" << name_ << "' pid: " << pid_;
          stop_timer_->Schedule(event_loop_->monotonic_now() +
                                std::chrono::seconds(1));
        }
      })),
      pipe_timer_(event_loop_->AddTimer([this]() { FetchOutputs(); })),
      child_status_handler_(
          event_loop_->AddTimer([this]() { MaybeHandleSignal(); })),
      on_change_({on_change}),
      main_cgroup_(std::move(memory_cgroup)),
      extra_cgroups_(std::move(extra_cgroups)),
      quiet_flag_(quiet_flag) {
  // Keep the length of the timer name bounded to some reasonable length.
  start_timer_->set_name(absl::StrCat("app_start_", name.substr(0, 10)));
  restart_timer_->set_name(absl::StrCat("app_restart_", name.substr(0, 10)));
  stop_timer_->set_name(absl::StrCat("app_stop_", name.substr(0, 10)));
  pipe_timer_->set_name(absl::StrCat("app_pipe_", name.substr(0, 10)));
  child_status_handler_->set_name(
      absl::StrCat("app_status_handler_", name.substr(0, 10)));
  // Every second poll to check if the child is dead. This is used as a
  // default for the case where the user is not directly catching SIGCHLD
  // and calling MaybeHandleSignal for us.
  child_status_handler_->Schedule(event_loop_->monotonic_now(),
                                  std::chrono::seconds(1));
}

Application::Application(
    const aos::Application *application, aos::EventLoop *event_loop,
    std::function<void()> on_change,
    std::unique_ptr<MemoryCGroup> memory_cgroup,
    std::vector<std::unique_ptr<MemoryCGroup>> extra_cgroups,
    QuietLogging quiet_flag)
    : Application(application->name()->string_view(),
                  application->has_executable_name()
                      ? application->executable_name()->string_view()
                      : application->name()->string_view(),
                  event_loop, on_change, std::move(memory_cgroup),
                  std::move(extra_cgroups), quiet_flag) {
  user_name_ = application->has_user() ? application->user()->str() : "";
  user_ = application->has_user() ? FindUid(user_name_.c_str()) : std::nullopt;
  group_ = application->has_user() ? FindPrimaryGidForUser(user_name_.c_str())
                                   : std::nullopt;
  autostart_ = application->autostart();
  autorestart_ = application->autorestart();
  if (application->has_args()) {
    set_args(*application->args());
  }

  if (application->has_memory_limit() && application->memory_limit() > 0) {
    SetMemoryLimit(application->memory_limit());
  }

  set_stop_grace_period(std::chrono::nanoseconds(application->stop_time()));
}

void Application::DoStart() {
  if (status_ != aos::starter::State::WAITING) {
    return;
  }

  start_timer_->Disable();
  restart_timer_->Disable();

  status_pipes_ = util::ScopedPipe::MakePipe();

  if (capture_stdout_) {
    stdout_pipes_ = util::ScopedPipe::MakePipe();
    stdout_.clear();
  }
  if (capture_stderr_) {
    stderr_pipes_ = util::ScopedPipe::MakePipe();
    stderr_.clear();
  }

  pipe_timer_->Schedule(event_loop_->monotonic_now(),
                        std::chrono::milliseconds(100));

  {
    // Block all signals during the fork() call. Together with the default
    // signal handler restoration below, This prevents signal handlers from
    // getting called in the child and accidentally affecting the parent. In
    // particular, the exit handler for shm_event_loop could be called here if
    // we don't exec() quickly enough.
    ScopedCompleteSignalBlocker signal_blocker;
    {
      const std::optional<ino_t> inode = GetInodeForPath(path_);
      if (inode.has_value()) {
        pre_fork_inode_ = inode.value();
      } else {
        if (absl::GetFlag(FLAGS_require_path_resolution)) {
          ABSL_CHECK(inode.has_value())
              << ": " << path_ << " does not seem to be stat'able.";
        }
      }
    }
    const pid_t pid = fork();

    if (pid != 0) {
      if (pid == -1) {
        ABSL_PLOG_IF(WARNING, quiet_flag_ == QuietLogging::kNo ||
                                  quiet_flag_ == QuietLogging::kNotForDebugging)
            << "Failed to fork '" << name_ << "'";
        stop_reason_ = aos::starter::LastStopReason::FORK_ERR;
        status_ = aos::starter::State::STOPPED;
      } else {
        pid_ = pid;
        id_ = next_id_++;
        start_time_ = event_loop_->monotonic_now();
        status_ = aos::starter::State::STARTING;
        latest_timing_report_version_.reset();
        ABSL_LOG_IF(INFO, quiet_flag_ == QuietLogging::kNo)
            << "Starting '" << name_ << "' pid " << pid_;

        // Set up timer which moves application to RUNNING state if it is still
        // alive in 1 second.
        start_timer_->Schedule(event_loop_->monotonic_now() +
                               std::chrono::seconds(1));
        // Since we are the parent process, clear our write-side of all the
        // pipes.
        status_pipes_.write.reset();
        stdout_pipes_.write.reset();
        stderr_pipes_.write.reset();
      }
      OnChange();
      return;
    }

    // Clear any signal handlers so that they don't accidentally interfere with
    // the parent process. Is there a better way to iterate over all the
    // signals? Right now we're just dealing with the most common ones.
    for (int signal : {SIGINT, SIGHUP, SIGTERM}) {
      struct sigaction action;
      sigemptyset(&action.sa_mask);
      action.sa_flags = 0;
      action.sa_handler = SIG_DFL;
      ABSL_PCHECK(sigaction(signal, &action, nullptr) == 0);
    }
  }

  if (main_cgroup_) {
    main_cgroup_->AddTid();
  }

  // Since we are the child process, clear our read-side of all the pipes.
  status_pipes_.read.reset();
  stdout_pipes_.read.reset();
  stderr_pipes_.read.reset();

  // The status pipe will not be needed if the execve succeeds.
  status_pipes_.write->SetCloexec();

  // Clear out signal mask of parent so forked process receives all signals
  // normally.
  sigset_t empty_mask;
  sigemptyset(&empty_mask);
  sigprocmask(SIG_SETMASK, &empty_mask, nullptr);

  // Cleanup children if starter dies in a way that is not handled gracefully.
  if (prctl(PR_SET_PDEATHSIG, SIGKILL) == -1) {
    status_pipes_.write->Write(
        static_cast<uint32_t>(aos::starter::LastStopReason::SET_PRCTL_ERR));
    ABSL_PLOG(FATAL) << "Could not set PR_SET_PDEATHSIG to SIGKILL";
  }

  if (group_) {
    ABSL_CHECK(!user_name_.empty());
    // The manpage for setgroups says we just need CAP_SETGID, but empirically
    // we also need the effective UID to be 0 to make it work. user_ must also
    // be set so we change this effective UID back later.
    ABSL_CHECK(user_);
    if (seteuid(0) == -1) {
      status_pipes_.write->Write(
          static_cast<uint32_t>(aos::starter::LastStopReason::SET_GRP_ERR));
      ABSL_PLOG(FATAL) << "Could not seteuid(0) for " << name_
                       << " in preparation for setting groups";
    }
    if (initgroups(user_name_.c_str(), *group_) == -1) {
      status_pipes_.write->Write(
          static_cast<uint32_t>(aos::starter::LastStopReason::SET_GRP_ERR));
      ABSL_PLOG(FATAL) << "Could not initialize normal groups for " << name_
                       << " as " << user_name_ << " with " << *group_;
    }
    if (setgid(*group_) == -1) {
      status_pipes_.write->Write(
          static_cast<uint32_t>(aos::starter::LastStopReason::SET_GRP_ERR));
      ABSL_PLOG(FATAL) << "Could not set group for " << name_ << " to "
                       << *group_;
    }
  }

  if (user_) {
    if (setuid(*user_) == -1) {
      status_pipes_.write->Write(
          static_cast<uint32_t>(aos::starter::LastStopReason::SET_USR_ERR));
      ABSL_PLOG(FATAL) << "Could not set user for " << name_ << " to "
                       << *user_;
    }
  }

  if (capture_stdout_) {
    ABSL_PCHECK(STDOUT_FILENO ==
                dup2(stdout_pipes_.write->fd(), STDOUT_FILENO));
    stdout_pipes_.write.reset();
  }

  if (capture_stderr_) {
    ABSL_PCHECK(STDERR_FILENO ==
                dup2(stderr_pipes_.write->fd(), STDERR_FILENO));
    stderr_pipes_.write.reset();
  }

  if (run_as_sudo_) {
    // For sudo we must supply the actual path
    args_.insert(args_.begin(), path_.c_str());
    args_.insert(args_.begin(), kSudo);
  } else {
    // argv[0] should be the program name
    args_.insert(args_.begin(), name_);
  }

  std::vector<char *> cargs = CArgs();
  const char *path = run_as_sudo_ ? kSudo : path_.c_str();
  execvp(path, cargs.data());

  // If we got here, something went wrong
  status_pipes_.write->Write(
      static_cast<uint32_t>(aos::starter::LastStopReason::EXECV_ERR));
  ABSL_PLOG_IF(WARNING, quiet_flag_ == QuietLogging::kNo ||
                            quiet_flag_ == QuietLogging::kNotForDebugging)
      << "Could not execute " << name_ << " (" << path_ << ')';

  _exit(EXIT_FAILURE);
}

void Application::ObserveTimingReport(
    const aos::monotonic_clock::time_point send_time,
    const aos::timing::Report *msg) {
  if (msg->name()->string_view() == name_ && msg->pid() == pid_ &&
      msg->has_version()) {
    latest_timing_report_version_ = msg->version()->str();
    last_timing_report_ = send_time;
  }
}

void Application::FetchOutputs() {
  if (capture_stdout_) {
    stdout_pipes_.read->Read(&stdout_);
  }
  if (capture_stderr_) {
    stderr_pipes_.read->Read(&stderr_);
  }
}

const std::string &Application::GetStdout() {
  ABSL_CHECK(capture_stdout_);
  FetchOutputs();
  return stdout_;
}

const std::string &Application::GetStderr() {
  ABSL_CHECK(capture_stderr_);
  FetchOutputs();
  return stderr_;
}

void Application::DoStop(bool restart) {
  // If stop or restart received, the old state of these is no longer applicable
  // so cancel both.
  restart_timer_->Disable();
  start_timer_->Disable();

  FetchOutputs();

  switch (status_) {
    case aos::starter::State::STARTING:
    case aos::starter::State::RUNNING: {
      file_state_ = FileState::NOT_RUNNING;
      ABSL_LOG_IF(INFO, quiet_flag_ == QuietLogging::kNo ||
                            quiet_flag_ == QuietLogging::kNotForDebugging)
          << "Stopping '" << name_ << "' pid: " << pid_ << " with signal "
          << SIGINT;
      status_ = aos::starter::State::STOPPING;

      if (kill(pid_, SIGINT) != 0) {
        ABSL_PLOG_IF(INFO, quiet_flag_ == QuietLogging::kNo ||
                               quiet_flag_ == QuietLogging::kNotForDebugging)
            << "Failed to send signal " << SIGINT << " to '" << name_
            << "' pid: " << pid_;
      }

      // Watchdog timer to SIGKILL application if it is still running 1 second
      // after SIGINT
      stop_timer_->Schedule(event_loop_->monotonic_now() + stop_grace_period_);
      OnChange();
      break;
    }
    case aos::starter::State::WAITING: {
      // If waiting to restart, and receives restart, skip the waiting period
      // and restart immediately. If stop received, all we have to do is move
      // to the STOPPED state.
      if (restart) {
        DoStart();
      } else {
        status_ = aos::starter::State::STOPPED;
        OnChange();
      }
      break;
    }
    case aos::starter::State::STOPPING: {
      break;
    }
    case aos::starter::State::STOPPED: {
      // Restart immediately if the application is already stopped
      if (restart) {
        status_ = aos::starter::State::WAITING;
        DoStart();
      }
      break;
    }
  }
}

void Application::QueueStart() {
  status_ = aos::starter::State::WAITING;

  ABSL_LOG_IF(INFO, quiet_flag_ == QuietLogging::kNo)
      << "Restarting " << name_ << " in 3 seconds";
  restart_timer_->Schedule(event_loop_->monotonic_now() +
                           std::chrono::seconds(3));
  start_timer_->Disable();
  stop_timer_->Disable();
  OnChange();
}

std::vector<char *> Application::CArgs() {
  std::vector<char *> cargs;
  std::transform(args_.begin(), args_.end(), std::back_inserter(cargs),
                 [](std::string &str) { return str.data(); });
  cargs.push_back(nullptr);
  return cargs;
}

void Application::set_args(
    const flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>> &v) {
  args_.clear();
  std::transform(v.begin(), v.end(), std::back_inserter(args_),
                 [](const flatbuffers::String *str) { return str->str(); });
}

void Application::set_args(std::vector<std::string> args) {
  args_ = std::move(args);
}

void Application::set_capture_stdout(bool capture) {
  capture_stdout_ = capture;
}

void Application::set_capture_stderr(bool capture) {
  capture_stderr_ = capture;
}

std::optional<uid_t> Application::FindUid(const char *name) {
  // TODO(austin): Use the reentrant version.  This should be safe.
  struct passwd *user_data = getpwnam(name);
  if (user_data != nullptr) {
    return user_data->pw_uid;
  } else {
    ABSL_LOG(FATAL) << "Could not find user " << name;
    return std::nullopt;
  }
}

std::optional<gid_t> Application::FindPrimaryGidForUser(const char *name) {
  // TODO(austin): Use the reentrant version.  This should be safe.
  struct passwd *user_data = getpwnam(name);
  if (user_data != nullptr) {
    return user_data->pw_gid;
  } else {
    ABSL_LOG(FATAL) << "Could not find user " << name;
    return std::nullopt;
  }
}

FileState Application::UpdateFileState() {
  // On every call, check if a different file is present on disk. Note that
  // while the applications is running, the file cannot be changed without the
  // inode changing.
  // We could presumably use inotify or the such to watch the file instead,
  // but this works and we do not expect substantial cost from reading the inode
  // of a file every time we send out a status message.
  if (InodeChanged(path_, pre_fork_inode_)) {
    switch (file_state_) {
      case FileState::NO_CHANGE:
        file_state_ = FileState::CHANGED;
        break;
      case FileState::NOT_RUNNING:
      case FileState::CHANGED_DURING_STARTUP:
      case FileState::CHANGED:
        break;
    }
  }
  return file_state_;
}

flatbuffers::Offset<aos::starter::ApplicationStatus>
Application::PopulateStatus(flatbuffers::FlatBufferBuilder *builder,
                            util::Top *top) {
  UpdateFileState();

  ABSL_CHECK(builder != nullptr);
  auto name_fbs = builder->CreateString(name_);

  const bool valid_pid = pid_ > 0 && status_ != aos::starter::State::STOPPED;
  const flatbuffers::Offset<util::ProcessInfo> process_info =
      (valid_pid && top != nullptr) ? top->InfoForProcess(builder, pid_)
                                    : flatbuffers::Offset<util::ProcessInfo>();

  aos::starter::ApplicationStatus::Builder status_builder(*builder);
  status_builder.add_name(name_fbs);
  status_builder.add_state(status_);
  if (exit_code_.has_value()) {
    status_builder.add_last_exit_code(exit_code_.value());
  }
  status_builder.add_has_active_timing_report(
      last_timing_report_ +
          // Leave a bit of margin on the timing report receipt time, to allow
          // for timing errors.
          3 * std::chrono::milliseconds(absl::GetFlag(FLAGS_timing_report_ms)) >
      event_loop_->monotonic_now());
  status_builder.add_last_stop_reason(stop_reason_);
  if (pid_ != -1) {
    status_builder.add_pid(pid_);
    status_builder.add_id(id_);
  }
  // Note that even if process_info is null, calling add_process_info is fine.
  status_builder.add_process_info(process_info);
  if (start_time_.has_value()) {
    status_builder.add_last_start_time(
        start_time_.value().time_since_epoch().count());
  }
  if (exit_time_.has_value()) {
    status_builder.add_last_exit_time(
        exit_time_.value().time_since_epoch().count());
  }
  status_builder.add_file_state(file_state_);
  return status_builder.Finish();
}

void Application::Terminate() {
  stop_reason_ = aos::starter::LastStopReason::TERMINATE;
  DoStop(false);
  command_ = ApplicationCommand::kTerminate;
}

void Application::HandleCommand(aos::starter::Command cmd) {
  if (command_ == ApplicationCommand::kTerminate) {
    return;
  }
  switch (cmd) {
    case aos::starter::Command::START: {
      command_ = ApplicationCommand::kStart;
      switch (status_) {
        case aos::starter::State::WAITING: {
          restart_timer_->Disable();
          DoStart();
          break;
        }
        case aos::starter::State::STARTING: {
          break;
        }
        case aos::starter::State::RUNNING: {
          break;
        }
        case aos::starter::State::STOPPING: {
          command_ = ApplicationCommand::kRestart;
          break;
        }
        case aos::starter::State::STOPPED: {
          status_ = aos::starter::State::WAITING;
          DoStart();
          break;
        }
      }
      break;
    }
    case aos::starter::Command::STOP: {
      command_ = ApplicationCommand::kStop;
      stop_reason_ = aos::starter::LastStopReason::STOP_REQUESTED;
      DoStop(false);
      break;
    }
    case aos::starter::Command::RESTART: {
      command_ = ApplicationCommand::kRestart;
      stop_reason_ = aos::starter::LastStopReason::RESTART_REQUESTED;
      DoStop(true);
      break;
    }
  }
}

bool Application::MaybeHandleSignal() {
  int status;

  if (status_ == aos::starter::State::WAITING ||
      status_ == aos::starter::State::STOPPED) {
    // We can't possibly have received a signal meant for this process.
    return false;
  }

  // Check if the status of this process has changed
  // The PID won't be -1 if this application has ever been run successfully
  if (pid_ == -1 || waitpid(pid_, &status, WNOHANG) != pid_) {
    return false;
  }

  // Check that the event was the process exiting
  if (!WIFEXITED(status) && !WIFSIGNALED(status)) {
    return false;
  }

  start_timer_->Disable();
  exit_time_ = event_loop_->monotonic_now();
  exit_code_ = WIFEXITED(status) ? WEXITSTATUS(status) : WTERMSIG(status);
  file_state_ = FileState::NOT_RUNNING;

  if (auto read_result = status_pipes_.read->Read()) {
    stop_reason_ = static_cast<aos::starter::LastStopReason>(*read_result);
  }

  const std::string starter_version_string =
      absl::StrCat("starter version '",
                   event_loop_->VersionString().value_or("unknown"), "'");
  switch (status_) {
    case aos::starter::State::STARTING: {
      if (exit_code_.value() == 0) {
        ABSL_LOG_IF(INFO, quiet_flag_ == QuietLogging::kNo)
            << "Application '" << name_ << "' pid " << pid_
            << " exited with status " << exit_code_.value() << " and "
            << starter_version_string;
      } else {
        ABSL_LOG_IF(WARNING, quiet_flag_ == QuietLogging::kNo ||
                                 quiet_flag_ == QuietLogging::kNotForDebugging)
            << "Failed to start '" << name_ << "' on pid " << pid_
            << " : Exited with status " << exit_code_.value() << " and "
            << starter_version_string;
      }
      if (autorestart()) {
        QueueStart();
      } else {
        status_ = aos::starter::State::STOPPED;
        OnChange();
      }
      break;
    }
    case aos::starter::State::RUNNING: {
      if (exit_code_.value() == 0) {
        ABSL_LOG_IF(INFO, quiet_flag_ == QuietLogging::kNo)
            << "Application '" << name_ << "' pid " << pid_
            << " exited with status " << exit_code_.value();
      } else {
        if (quiet_flag_ == QuietLogging::kNo ||
            quiet_flag_ == QuietLogging::kNotForDebugging) {
          const std::string version_string =
              latest_timing_report_version_.has_value()
                  ? absl::StrCat("version '",
                                 latest_timing_report_version_.value(), "'")
                  : starter_version_string;
          ABSL_LOG_IF(WARNING, quiet_flag_ == QuietLogging::kNo)
              << "Application '" << name_ << "' pid " << pid_ << " "
              << version_string << " exited unexpectedly with status "
              << exit_code_.value();
        }
      }
      if (autorestart()) {
        QueueStart();
      } else {
        status_ = aos::starter::State::STOPPED;
        OnChange();
      }
      break;
    }
    case aos::starter::State::STOPPING: {
      ABSL_LOG_IF(INFO, quiet_flag_ == QuietLogging::kNo)
          << "Successfully stopped '" << name_ << "' pid: " << pid_
          << " with status " << exit_code_.value();
      status_ = aos::starter::State::STOPPED;

      // Disable force stop timer since the process already died
      stop_timer_->Disable();

      OnChange();
      if (command_ == ApplicationCommand::kTerminate) {
        return true;
      }

      if (command_ == ApplicationCommand::kRestart) {
        command_ = std::nullopt;
        status_ = aos::starter::State::WAITING;
        DoStart();
      }
      break;
    }
    case aos::starter::State::WAITING:
    case aos::starter::State::STOPPED: {
      __builtin_unreachable();
      break;
    }
  }

  return false;
}

void Application::OnChange() {
  for (auto &fn : on_change_) {
    fn();
  }
}

Application::~Application() {
  start_timer_->Disable();
  restart_timer_->Disable();
  stop_timer_->Disable();
  pipe_timer_->Disable();
  child_status_handler_->Disable();
}

}  // namespace aos::starter
