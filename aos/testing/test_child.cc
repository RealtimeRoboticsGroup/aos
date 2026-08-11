#include "aos/testing/test_child.h"

#include <functional>
#include <memory>
#include <utility>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#ifdef _WIN32
#include <thread>
#else
#include <errno.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cstdlib>

#include "aos/testing/prevent_exit.h"
#endif

namespace aos::testing {

#ifndef _WIN32

// The child is a fork(2)ed process.  It can be killed asynchronously
// mid-anything, and PreventExit() keeps the forked copy of gtest from running
// the parent's atexit handlers.
class TestChild::Impl {
 public:
  ~Impl() { ABSL_CHECK_EQ(child_, -1) << ": child never reaped"; }

  void Start(std::function<void()> fn) {
    ABSL_CHECK_EQ(child_, -1) << ": already started";
    child_ = fork();
    if (child_ == 0) {  // in child
      PreventExit();
      fn();
      exit(EXIT_SUCCESS);
    }
    ABSL_PCHECK(child_ != -1) << ": fork() failed";
  }

  bool running() const { return child_ != -1; }

  void Join() {
    const int status = Wait();
    ABSL_CHECK(WIFEXITED(status) && WEXITSTATUS(status) == 0)
        << ": child " << child_ << " died with status " << status;
    child_ = -1;
  }

  // A forked process can be killed outright.  Dying by signal is expected.
  void Terminate() {
    ABSL_CHECK_NE(child_, -1) << ": not started";
    ABSL_PCHECK(kill(child_, SIGTERM) != -1);
    const int status = Wait();
    ABSL_CHECK(WIFEXITED(status) || WIFSIGNALED(status));
    child_ = -1;
  }

 private:
  // Waits for the child to change state and returns its wait status.
  int Wait() {
    ABSL_CHECK_NE(child_, -1) << ": not started";
    while (true) {
      int status;
      const pid_t waited_on = waitpid(child_, &status, 0);
      if (waited_on == -1) {
        if (errno == EINTR) continue;
        ABSL_PLOG(FATAL) << ": waitpid(" << child_ << ") failed";
      }
      ABSL_CHECK_EQ(waited_on, child_);
      if (WIFEXITED(status) || WIFSIGNALED(status)) {
        return status;
      }
    }
  }

  pid_t child_ = -1;
};

#else

// There is no fork, and no safe way to kill a running thread, so the child is
// a std::thread and Terminate() has to talk it into returning on its own.
class TestChild::Impl {
 public:
  ~Impl() { ABSL_CHECK(!thread_.joinable()) << ": child never joined"; }

  void Start(std::function<void()> fn) {
    ABSL_CHECK(!thread_.joinable()) << ": already started";
    thread_ = std::thread(std::move(fn));
  }

  bool running() const { return thread_.joinable(); }

  // A thread that returns has by definition finished cleanly; a child that
  // ASSERTed already failed the test.
  void Join() {
    ABSL_CHECK(thread_.joinable()) << ": not started";
    thread_.join();
  }

  void Terminate() {
    // A thread cannot be killed.  The only reason to get here is a child that
    // is blocked when the test expected it to finish, which means the test
    // has already failed; die loudly instead of hanging the whole binary.
    ABSL_LOG(FATAL) << "cannot kill a blocked thread on Windows; a hung "
                       "child means this test already failed";
  }

 private:
  std::thread thread_;
};

#endif

TestChild::TestChild() : impl_(std::make_unique<Impl>()) {}

// Out of line so that Impl only has to be complete here.
TestChild::~TestChild() = default;

void TestChild::Start(std::function<void()> fn) { impl_->Start(std::move(fn)); }

bool TestChild::running() const { return impl_->running(); }

void TestChild::Join() { impl_->Join(); }

void TestChild::Terminate() { impl_->Terminate(); }

}  // namespace aos::testing
