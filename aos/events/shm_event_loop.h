#ifndef AOS_EVENTS_SHM_EVENT_LOOP_H_
#define AOS_EVENTS_SHM_EVENT_LOOP_H_

#include <thread>
#include <vector>

#include "absl/types/span.h"

#include "aos/events/aio.h"
#include "aos/events/event_loop.h"
#include "aos/events/event_loop_generated.h"
#include "aos/ipc_lib/shm_base.h"
#include "aos/ipc_lib/thread_signal.h"
#include "aos/stl_mutex/stl_mutex.h"

namespace aos {

class AosLogToFbs;
namespace logging {
class ScopedLogRestorer;
}  // namespace logging

namespace shm_event_loop_internal {

class ShmWatcherState;
class ShmTimerHandler;
class ShmPhasedLoopHandler;
class ShmSender;
class SimpleShmFetcher;
class ShmFetcher;
class ShmExitHandle;
class ShmThreadHandle;

}  // namespace shm_event_loop_internal

// Concrete implementation of EventLoop that is built from queues running out of
// shared memory.
//
// TODO(austin): Timing reports break multiple threads.  Need to add back in a
// mutex.
// This object must be interacted with from one thread, but the Senders
// and Fetchers may be used from multiple threads afterwords (as long as their
// destructors are called back in one thread again)
class ShmEventLoop : public EventLoop {
 public:
  ShmEventLoop(const Flatbuffer<Configuration> &configuration)
      : ShmEventLoop(&configuration.message()) {}
  ShmEventLoop(const Configuration *configuration);

  // Builds an event loop that runs on somebody else's Aio rather than its own.
  //
  // This is how AOS gets scheduled onto a foreign event loop: pair it with an
  // Aio subclass backed by that loop, and the watchers and timers here become
  // work that loop is already waiting on.
  //
  // The Aio is borrowed and must outlive this.  Run() is not usable in this
  // mode, because whoever owns the Aio is the one driving it; call Startup()
  // and Shutdown() around their loop instead.  No other event loop may be on
  // that Aio, because an Aio carries one event loop's wakeups.
  //
  // Exit(), ExitWithStatus() and an ExitHandle Quit() that Aio.
  ShmEventLoop(const Flatbuffer<Configuration> &configuration, Aio *aio)
      : ShmEventLoop(&configuration.message(), aio) {}
  ShmEventLoop(const Configuration *configuration, Aio *aio);

  ShmEventLoop(const ShmEventLoop &) = delete;
  ~ShmEventLoop() override;

  void operator=(ShmEventLoop const &) = delete;

  // Runs the event loop until Exit is called, or ^C is caught.
  // TODO(james): Upgrade this to [[nodiscard]].
  //
  // Only for an event loop that owns its Aio.  On a borrowed Aio, use Startup()
  // and Shutdown().
  Status Run();

  // Run() without the polling in the middle, for an event loop on a borrowed
  // Aio: Startup() before the Aio is first polled, and Shutdown() after it is
  // last polled, before anything made from this event loop is destroyed.
  // Polling the Aio while this is not started, or destroying it while it is,
  // dies.  Shutdown() returns what Run() would have.
  //
  // Both run on the thread that polls the Aio, and so does the destructor.
  // Shutdown() may not run from one of this event loop's callbacks, and only
  // one event loop may be started on a thread at a time.  Startup() names the
  // thread and sets its affinity and realtime priority, and Shutdown() drops
  // the priority; neither restores what the thread had before.  The
  // destructor takes a before-wait function off the Aio, so destroying this
  // from one, a glib callback under GlibMainLoop included, dies.
  void Startup();
  Status Shutdown();

  // Sets whether Startup() installs handlers that exit this event loop on
  // SIGINT, SIGHUP and SIGTERM, or on Ctrl-C, Ctrl-Break and console close on
  // Windows.  Defaults to true.  Must be called before Startup().
  void set_handle_signals(bool handle_signals);
  bool handle_signals() const { return handle_signals_; }

  // Exits the event loop.  async-signal-safe (see
  // https://man7.org/linux/man-pages/man7/signal-safety.7.html).
  // Run() then returns a successful result.  On a borrowed Aio this stops the
  // owner's loop instead; see the constructor.
  void Exit();

  // Exits the event loop with the provided status. Thread-safe, but not
  // async-safe.
  void ExitWithStatus(Status status = {});

  // Constructs an exit handle for the EventLoop. The provided ExitHandle uses
  // ExitWithStatus().
  std::unique_ptr<ExitHandle> MakeExitHandle();

  aos::monotonic_clock::time_point monotonic_now() const override {
    return aos::monotonic_clock::now();
  }
  aos::realtime_clock::time_point realtime_now() const override {
    return aos::realtime_clock::now();
  }

  std::unique_ptr<RawSender> MakeRawSender(const Channel *channel) override;
  std::unique_ptr<RawFetcher> MakeRawFetcher(const Channel *channel) override;

  void MakeRawWatcher(
      const Channel *channel,
      std::function<void(const Context &context, const void *message)> watcher)
      override;
  void MakeRawNoArgWatcher(
      const Channel *channel,
      std::function<void(const Context &context)> watcher) override;

  TimerHandler *AddTimer(std::function<void()> callback) override;
  PhasedLoopHandler *AddPhasedLoop(std::function<void(int)> callback,
                                   const monotonic_clock::duration interval,
                                   const monotonic_clock::duration offset =
                                       std::chrono::seconds(0)) override;

  void OnRun(std::function<void()> on_run) override;

  void SetRuntimeAffinity(const CpuSet &cpuset) override;
  void SetRuntimeRealtimePriority(
      int priority,
      SchedulingPolicy scheduling_policy = SchedulingPolicy::SCHEDULER_FIFO,
      RealtimePolicy realtime_policy =
          RealtimePolicy::REALTIME_MODE_DENY_MALLOC) override;

  void set_name(const std::string_view name) override;
  const std::string_view name() const override { return name_; }
  const Node *node() const override { return node_; }

  const CpuSet &runtime_affinity() const override { return runtime_affinity_; }
  int runtime_realtime_priority() const override {
    return (runtime_scheduling_policy_ == SchedulingPolicy::SCHEDULER_FIFO ||
            runtime_scheduling_policy_ == SchedulingPolicy::SCHEDULER_RR)
               ? runtime_priority_
               : 0;
  }
  SchedulingPolicy runtime_scheduling_policy() const override {
    return runtime_scheduling_policy_;
  }
  RealtimePolicy runtime_realtime_policy() const override {
    return runtime_realtime_policy_;
  }

  const UUID &boot_uuid() const override { return boot_uuid_; }

  // Returns the Aio loop used to run the event loop.
  Aio *aio() { return aio_; }

  // Returns the local mapping of the shared memory used by the watcher on the
  // specified channel. A watcher must be created on this channel before calling
  // this.
  absl::Span<char> GetWatcherSharedMemory(const Channel *channel);

  // Setting "use writable memory" to false (the default) means that the
  // watcher will provide messages in a read-only memory region. Setting "use
  // writable memory" to true means that the watcher will provide messages in a
  // writable memory. Only use this if you absolutely know what you're doing.
  // You should only use this if you're interacting with something like CUDA
  // which expects writable memory in its API. Note that regardless of this
  // setting, the API for watcher doesn't change. The messages will still be
  // `const`.
  void SetWatcherUseWritableMemory(const Channel *channel,
                                   bool use_writable_memory);

  // Returns the local mapping of the shared memory used by the provided Sender.
  template <typename T>
  absl::Span<char> GetSenderSharedMemory(aos::Sender<T> *sender) const {
    CheckCurrentThread();
    return GetShmSenderSharedMemory(GetRawSender(sender));
  }

  // Returns the local mapping of the private memory used by the provided
  // Fetcher to hold messages.
  //
  // Note that this may be the entire shared memory region held by this fetcher,
  // depending on its channel's read_method.
  template <typename T>
  absl::Span<const char> GetFetcherPrivateMemory(
      aos::Fetcher<T> *fetcher) const {
    CheckCurrentThread();
    return GetShmFetcherPrivateMemory(GetRawFetcher(fetcher));
  }

  // Returns the local mapping of the shared memory used by the provided
  // Fetcher to hold messages.
  //
  // Note that this may be the entire shared memory region held by this fetcher,
  // depending on its channel's read_method.
  //
  // Only use this if you really know what you're doing. See the docs for
  // SetFetcherUseWritableMemory() for more information.
  template <typename T>
  absl::Span<char> GetFetcherSharedMemory(aos::Fetcher<T> *fetcher) const {
    CheckCurrentThread();
    return GetShmFetcherSharedMemory(GetRawFetcher(fetcher));
  }

  // Setting "use writable memory" to false (the default) means that the
  // fetcher will provide messages in a read-only memory region. Setting "use
  // writable memory" to true means that the fetcher will provide messages in a
  // writable memory. Only use this if you absolutely know what you're doing.
  // You should only use this if you're interacting with something like CUDA
  // which expects writable memory in its API. Note that regardless of this
  // setting, the API for fetchers doesn't change. The messages will still be
  // `const`.
  template <typename T>
  void SetFetcherUseWritableMemory(aos::Fetcher<T> *fetcher,
                                   bool use_writable_memory) const {
    CheckCurrentThread();
    SetShmFetcherUseWritableMemory(GetRawFetcher(fetcher), use_writable_memory);
  }

  int NumberBuffers(const Channel *channel) override;

  // All public-facing APIs will verify this mutex is held when they are called.
  // For normal use with everything in a single thread, this is unnecessary.
  //
  // This is helpful as a safety check when using a ShmEventLoop with external
  // synchronization across multiple threads. It will NOT reliably catch race
  // conditions, but if you have a race condition triggered repeatedly it'll
  // probably catch it eventually.
  //
  // Holding the mutex does not make timers and phased loops usable from other
  // threads.  While the event loop runs, only the thread running it may
  // schedule or disable them; see CheckLoopThread().
  void CheckForMutex(aos::stl_mutex *check_mutex) {
    check_mutex_ = check_mutex;
  }

  // All public-facing APIs will verify they are called in this thread.
  // For normal use with the whole program in a single thread, this is
  // unnecessary. It's helpful as a safety check for programs with multiple
  // threads, where the EventLoop should only be interacted with from a single
  // one.
  //
  // This must be called before any threads are started.
  void LockToThread() { check_tid_ = GetTid(); }

 private:
  friend class shm_event_loop_internal::ShmWatcherState;
  friend class shm_event_loop_internal::ShmTimerHandler;
  friend class shm_event_loop_internal::ShmPhasedLoopHandler;
  friend class shm_event_loop_internal::ShmSender;
  friend class shm_event_loop_internal::SimpleShmFetcher;
  friend class shm_event_loop_internal::ShmFetcher;
  friend class shm_event_loop_internal::ShmExitHandle;
  friend class shm_event_loop_internal::ShmThreadHandle;

  using EventLoop::SendTimingReport;

  std::unique_ptr<ThreadHandle> ConfigureThreadImpl(
      const ThreadConfiguration &thread_configuration) override;

  void IgnoreThreadImpl() override;

  void CheckCurrentThread() const;

  // CheckCurrentThread(), and between Startup() and Shutdown(), also checks
  // that this is the thread that ran Startup().  Timers and phased loops arm
  // Aio timers, which only the thread driving the Aio may do, so they check
  // this.  Before Startup() and after Shutdown(), any thread may set them up
  // or tear them down, as far as the Aio allows.
  void CheckLoopThread() const;

  // Validates that the current thread is not the main thread.
  // Requires that the user has called LockToThread() before.
  void CheckNotMainThread() const;

  void HandleEvent();

  // Returns the TID of the event loop.
  pid_t GetTid() const override;

  static void IgnoreWakeupSignal();
  void RegisterSignalHandler();
  void UnregisterSignalHandler();

  // Private method to access the shared memory mapping of a ShmSender.
  absl::Span<char> GetShmSenderSharedMemory(const aos::RawSender *sender) const;

  // Private method to access the private memory mapping of a ShmFetcher.
  absl::Span<const char> GetShmFetcherPrivateMemory(
      const aos::RawFetcher *fetcher) const;

  // Private method to access the shared memory mapping of a ShmFetcher.
  absl::Span<char> GetShmFetcherSharedMemory(
      const aos::RawFetcher *fetcher) const;

  void SetShmFetcherUseWritableMemory(aos::RawFetcher *fetcher,
                                      bool use_writable_memory) const;

  const UUID boot_uuid_;

  int exit_handle_count_ = 0;

  // Capture the --shm_base flag at construction time.  This makes it much
  // easier to make different shared memory regions for doing things like
  // multi-node tests.
  std::string shm_base_;

  std::vector<std::function<void()>> on_run_;

  aos::stl_mutex *check_mutex_ = nullptr;
  std::optional<pid_t> check_tid_;
  // The thread that ran Startup(), and so drives the Aio, until Shutdown().
  // A std::thread::id because reading it is free, where gettid() is a
  // syscall on every timer Schedule().
  std::optional<std::thread::id> loop_thread_;

  // Shared by both constructors.
  void Initialize(const Configuration *configuration);

  // Startup() and Shutdown() each run once per run.  Shutdown() clears them,
  // because an event loop may be Run() more than once.
  bool started_ = false;
  bool shut_down_ = false;
  // Whether one of this event loop's callbacks is running, which Shutdown()
  // may not be called from.
  bool in_callback_ = false;
  // Whether Startup() installs signal handlers, and whether it did.
  bool handle_signals_ = true;
  bool registered_signal_handler_ = false;

  // Live from Startup() to Shutdown(), which is why they are not locals.
  // Held by pointer so this header does not have to define them.
  std::unique_ptr<logging::ScopedLogRestorer> log_restorer_;
  std::unique_ptr<AosLogToFbs> aos_logger_;

  // A borrowed Aio's before-wait hook, which the destructor removes.
  Aio::BeforeWaitId check_polled_hook_ = 0;

  // Engaged only when this event loop owns its Aio; aio_ points into it.
  std::optional<Aio> owned_aio_;
  // The Aio actually in use, owned or borrowed.  Never null.
  Aio *const aio_;

  // Only set from Startup() to Shutdown(), and only with watchers.
  std::unique_ptr<ipc_lib::ThreadSignalReceiver> signal_receiver_;

  // Calls to Exit() are guaranteed to be thread-safe, so the exit_status_mutex_
  // guards access to the exit_status_.
  aos::stl_mutex exit_status_mutex_;
  // Once exit_status_ is set once, we will not set it again until we have
  // actually exited. This is to try to provide consistent behavior in cases
  // where Exit() is called multiple times before Run() is aactually terminates
  // execution.
  std::optional<Status> exit_status_{};
  // Used by the Exit() call to provide an async-safe way of indicating that
  // Exit() was called.
  // Will be set once Exit() or ExitWithStatus() has been called.
  // Note: std::atomic<> is not necessarily guaranteed to be lock-free, although
  // std::atomic_flag is, and so is safe to use in Exit().
  std::atomic_flag observed_exit_ = ATOMIC_FLAG_INIT;
};

}  // namespace aos

#endif  // AOS_EVENTS_SHM_EVENT_LOOP_H_
