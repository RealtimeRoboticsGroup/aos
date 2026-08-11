#ifndef AOS_TESTING_TEST_CHILD_H_
#define AOS_TESTING_TEST_CHILD_H_

#include <functional>
#include <memory>

namespace aos::testing {

// Runs test code in a "child": an execution context that shares the test's
// (shared) memory but runs, finishes, and dies independently of it.  The sync
// tests use this to exercise cross-owner behavior -- one side blocks on a
// primitive while the other signals it, dies holding it, or hangs and needs
// to be torn down.
//
// What a child actually is depends on the platform, and the difference is not
// hidden completely: where there is a fork(2) the child is a process which can
// be killed outright mid-anything, and where there isn't it is a thread,
// which cannot be killed at all.  Terminate() therefore only works where the
// child is a process; on Windows it LOG(FATAL)s.  Tests which deliberately
// leave a child blocked and then reap it are testing a cross-process property
// and should be skipped on Windows -- for a test whose child is expected to
// finish, reaching Terminate() means the test already failed, and dying
// loudly beats hanging the binary.
//
// The child function's normal completion must mean "exited cleanly": Join()
// CHECK-fails if the child died any other way.
class TestChild {
 public:
  TestChild();
  ~TestChild();

  TestChild(const TestChild &) = delete;
  TestChild &operator=(const TestChild &) = delete;

  // Starts fn running in the child.  May only be called once.
  void Start(std::function<void()> fn);

  // Returns true between Start() and whichever of Join() or Terminate() reaps
  // the child.
  bool running() const;

  // Waits for the child to finish.  The child must finish cleanly: anything
  // else fails the test.
  void Join();

  // Stops the child and reaps it.  Only works where the child is a process;
  // see above.
  void Terminate();

 private:
  class Impl;

  std::unique_ptr<Impl> impl_;
};

}  // namespace aos::testing

#endif  // AOS_TESTING_TEST_CHILD_H_
