#ifndef AOS_EVENTS_AIO_TEST_LIB_H_
#define AOS_EVENTS_AIO_TEST_LIB_H_

#include <functional>
#include <memory>
#include <ostream>
#include <span>
#include <string>

#include "absl/flags/declare.h"
#include "absl/flags/reflection.h"
#include "gtest/gtest.h"

#include "aos/events/aio.h"
#include "aos/ipc_lib/thread_signal.h"

ABSL_DECLARE_FLAG(std::string, aio_backend);

namespace aos::testing {

// The Aio contract suite, in a form every backend can be held to.
//
// The bodies live in aio_test_lib.cc and are compiled once, into a library.
// Whoever links that library instantiates the suite for the backends it can
// reach.  aio_test_native.cc does it for the four that own their loop, and a
// backend that borrows somebody else's loop gets a file of its own.  That
// split is what keeps a guest backend's library out of a test binary that
// does not have it, without an #ifdef in a single test.
//
// What differs between backends is therefore data below rather than
// preprocessor conditionals beside each test.  A guest backend is not a
// platform; it is a set of promises it cannot make, and each of those is a
// field.

// How the suite gets at one backend: the Aio itself, and what Poll() and Run()
// mean on it.
class TestAioDriver {
 public:
  virtual ~TestAioDriver() = default;

  virtual Aio *aio() = 0;

  // A backend that owns its loop forwards these to Aio's.  A guest cannot:
  // Aio::Poll() and Aio::Run() are fatal there, because driving a borrowed
  // loop is its owner's job -- and here that owner is the test, so the driver
  // runs the loop itself.
  virtual bool Poll(bool block) = 0;
  virtual void Run() = 0;
};

struct AioBackend {
  // The name --aio_backend takes, and the name the backend calls itself by in
  // its own messages, so a failure says which one it was.
  std::string name;

  std::unique_ptr<TestAioDriver> (*make)() = nullptr;

  // Poll() rations completions one per call, and notices being reentered.
  // How much one iteration of a borrowed loop does is its owner's to decide,
  // so a guest promises neither.
  bool drives_its_own_loop = true;

  // The kernel object survives fork(), or is rebuilt in the child.  A
  // borrowed loop is neither this backend's to rebuild nor safe to keep: a
  // kqueue under one is not even inherited.
  bool survives_fork = true;

  // Misuse of what the backend tracks is caught here: destruction with work
  // still registered, a fd deleted with a request still pending on it, and
  // a cancelled request still in flight.  On a borrowed loop what outlives
  // it is bounded by the loop, not by the backend on it, and a cancelled
  // request stops being tracked once it is detached.
  bool checks_teardown = true;

  // Each Aio::Timer is a descriptor of the loop's own (a timerfd), which a
  // caller's fd can collide with.
  bool timers_are_descriptors = true;

  // io_uring's SINGLE_ISSUER rules, orphaned destruction, and its
  // one-completion-per-Poll() batching are its own.  Tests for them live in
  // AioIoUringTest rather than being guarded here.
  bool is_io_uring = false;

  // --aio_backend picks this one.  A guest backend is chosen by construction
  // instead, and its name is not a value the flag accepts.
  bool selected_by_flag = true;
};

// Self-imposed backpressure for ring-churning loops.  Closing an io_uring fd
// is fire-and-forget: the kernel frees the ring asynchronously on a
// workqueue, and IORING_SETUP_DEFER_TASKRUN rings additionally block that
// work on a full RCU grace period each (io_ring_exit_work).  Teardown
// throughput is therefore capped -- ~2500 rings/s on an idle machine,
// collapsing under CPU load as grace periods stretch -- while creation is
// effectively unbounded.  A loop that creates rings faster than the kernel
// retires them accumulates gigabytes of unreclaimable slab (pinned ctx,
// request, and ring-buffer memory), which is exactly what a heavily parallel
// CI run did to the whole build cluster.
//
// Backpressure only, never an assertion: what it watches is machine-global,
// so a noisy neighbor can only ever make this throttle extra, never make a
// test pass wrongly.  Linux-only; no-op elsewhere.
void ThrottleOnKernelRingTeardown();

// Names the instantiated tests after the backend rather than an index.
std::string BackendTestName(const ::testing::TestParamInfo<AioBackend> &info);
void PrintTo(const AioBackend &backend, std::ostream *os);

// The portable contract: everything aio.h promises of every backend.
class AioTest : public ::testing::TestWithParam<AioBackend> {
 protected:
  void SetUp() override;
  void TearDown() override;

  static const AioBackend &Backend() { return GetParam(); }
  static std::string BackendName() { return GetParam().name; }

  // Restores every flag SetUp() (or a test body) touched.
  absl::FlagSaver flag_saver_;
};

// io_uring's own behavior, which no other backend promises.  A separate suite
// rather than a skip inside the portable one: these are not contracts other
// backends fail to meet, they are a different contract.
class AioIoUringTest : public AioTest {};

// What every body constructs instead of an Aio.
//
// A guest backend cannot be default-constructed -- it borrows a loop somebody
// else has to own -- and Poll()/Run() mean "run that loop" there rather
// than "drive ours".  Both differences stop here, so no body below has to know
// which backend it got.  Everything else is Aio's, forwarded unchanged.
class TestAio {
 public:
  // Uses the backend of the test currently running.
  TestAio();
  ~TestAio();

  TestAio(const TestAio &) = delete;
  TestAio &operator=(const TestAio &) = delete;

  // The Aio underneath, for handing to something that takes one -- Aio::Timer,
  // mostly.
  Aio *get() { return driver_->aio(); }

  bool Poll(bool block) { return driver_->Poll(block); }
  void Run() { driver_->Run(); }

  void Quit() { get()->Quit(); }
  bool should_run() const { return driver_->aio()->should_run(); }

  void AsyncRead(FileDescriptor fd, std::span<char> buffer,
                 AsyncRequest *request) {
    get()->AsyncRead(fd, buffer, request);
  }
  void AsyncWrite(FileDescriptor fd, std::span<const char> buffer,
                  AsyncRequest *request) {
    get()->AsyncWrite(fd, buffer, request);
  }
  void Cancel(AsyncRequest *request) { get()->Cancel(request); }

  Aio::BeforeWaitId BeforeWait(std::function<void()> function) {
    return get()->BeforeWait(std::move(function));
  }
  void RemoveBeforeWait(Aio::BeforeWaitId id) { get()->RemoveBeforeWait(id); }

  void OnReadable(FileDescriptor fd, std::function<void()> callback) {
    get()->OnReadable(fd, std::move(callback));
  }
  void OnError(FileDescriptor fd, std::function<void()> callback) {
    get()->OnError(fd, std::move(callback));
  }
  void OnWritable(FileDescriptor fd, std::function<void()> callback) {
    get()->OnWritable(fd, std::move(callback));
  }
  void OnEvents(FileDescriptor fd, std::function<void(uint32_t)> callback) {
    get()->OnEvents(fd, std::move(callback));
  }
  void DeleteFd(FileDescriptor fd) { get()->DeleteFd(fd); }
  void ForgetClosedFd(FileDescriptor fd) { get()->ForgetClosedFd(fd); }
  void EnableWritable(FileDescriptor fd) { get()->EnableWritable(fd); }
  void DisableWritable(FileDescriptor fd) { get()->DisableWritable(fd); }
  void SetEvents(FileDescriptor fd, uint32_t events) {
    get()->SetEvents(fd, events);
  }

  void RegisterThreadSignalReceiver(ipc_lib::ThreadSignalReceiver *receiver,
                                    std::function<void()> callback) {
    get()->RegisterThreadSignalReceiver(receiver, std::move(callback));
  }
  void UnregisterThreadSignalReceiver(ipc_lib::ThreadSignalReceiver *receiver) {
    get()->UnregisterThreadSignalReceiver(receiver);
  }
  void ConsumeThreadSignalReceiver(ipc_lib::ThreadSignalReceiver *receiver) {
    get()->ConsumeThreadSignalReceiver(receiver);
  }

 private:
  std::unique_ptr<TestAioDriver> driver_;
};

// A driver for a backend that owns its loop: Aio's own Poll() and Run() are
// what the bodies mean.  Shared by every hosting backend, which differ from
// each other only in what --aio_backend says.
class HostingAioDriver : public TestAioDriver {
 public:
  Aio *aio() override { return &aio_; }
  bool Poll(bool block) override { return aio_.Poll(block); }
  void Run() override { aio_.Run(); }

 private:
  Aio aio_;
};

}  // namespace aos::testing

#endif  // AOS_EVENTS_AIO_TEST_LIB_H_
