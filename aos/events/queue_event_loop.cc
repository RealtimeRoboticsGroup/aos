#include "aos/events/queue_event_loop.h"

#include <sys/stat.h>
#include <sys/types.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <iterator>
#include <mutex>
#include <ranges>
#include <set>
#include <stdexcept>

#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "absl/log/die_if_null.h"

#include "aos/events/aio.h"
#include "aos/events/aos_logging.h"
#include "aos/events/event_loop_generated.h"
#include "aos/events/timing_statistics.h"
#include "aos/init.h"
#include "aos/ipc_lib/lockless_queue.h"
#include "aos/ipc_lib/memory_mapped_queue.h"
#include "aos/ipc_lib/queue_memory.h"
#include "aos/macros.h"
#include "aos/realtime.h"
#include "aos/stl_mutex/stl_mutex.h"
#include "aos/util/application_name.h"
#include "aos/util/phased_loop.h"

namespace aos {

using namespace queue_event_loop_internal;

namespace {

const Node *MaybeMyNode(const Configuration *configuration) {
  if (!configuration->has_nodes()) {
    return nullptr;
  }

  return configuration::GetMyNode(configuration);
}

// Returns true if the given scheduling policy is realtime.
bool SchedulingPolicyIsRealtime(SchedulingPolicy policy) {
  return policy == SchedulingPolicy::SCHEDULER_FIFO ||
         policy == SchedulingPolicy::SCHEDULER_RR;
}

// Returns true if the given thread is configured to be realtime.
bool ThreadIsConfiguredToBeRealtime(
    const ThreadConfiguration *thread_configuration) {
  return SchedulingPolicyIsRealtime(
      ABSL_DIE_IF_NULL(thread_configuration)->scheduling_policy());
}

// The Aios that have an event loop on them.  An Aio has room for one
// ThreadSignalReceiver, which is how an event loop hears about new messages,
// so a second event loop on it would never be woken.
struct AiosInUse {
  std::mutex mutex;
  std::set<const Aio *> aios;
};

AiosInUse &aios_in_use() {
  static AiosInUse *const in_use = new AiosInUse();
  return *in_use;
}

void ClaimAio(const Aio *aio) {
  AiosInUse &in_use = aios_in_use();
  std::unique_lock<std::mutex> locker(in_use.mutex);
  ABSL_CHECK(in_use.aios.insert(aio).second)
      << ": Only one event loop may be on an Aio, because an Aio carries one "
         "event loop's wakeups.";
}

void ReleaseAio(const Aio *aio) {
  AiosInUse &in_use = aios_in_use();
  std::unique_lock<std::mutex> locker(in_use.mutex);
  ABSL_CHECK_EQ(in_use.aios.erase(aio), 1u);
}

// The event loop started on this thread, from its Startup() to its
// Shutdown().  What an event loop takes over is per thread, not per Aio: its
// wakeup signal, its realtime priority and its logging, so two started on
// one thread, on two Aios that one thread polls, would take each other's.
thread_local const QueueEventLoop *started_on_this_thread = nullptr;

}  // namespace

QueueEventLoop::QueueEventLoop(const Configuration *configuration)
    : EventLoop(configuration, absl::GetFlag(FLAGS_application_name),
                MaybeMyNode(configuration)),
      boot_uuid_(UUID::BootUUID()),
      owned_aio_(std::in_place),
      aio_(&owned_aio_.value()) {
  Initialize(configuration);
}

QueueEventLoop::QueueEventLoop(const Configuration *configuration, Aio *aio)
    : EventLoop(configuration, absl::GetFlag(FLAGS_application_name),
                MaybeMyNode(configuration)),
      boot_uuid_(UUID::BootUUID()),
      aio_(ABSL_DIE_IF_NULL(aio)) {
  Initialize(configuration);
  // Every poll has to find this event loop started.  An owned Aio is only
  // polled inside Run(), so holding a borrowed one to the same rule keeps a
  // shut-down event loop the same on both: what it scheduled stays
  // scheduled, and nothing runs.  The destructor removes it, since the Aio
  // outlives this.
  check_polled_hook_ =
      aio_->BeforeWait([this]() {
        ABSL_CHECK(started_)
            << ": Polled the Aio while an event loop on it was not started.  "
               "Startup() it before polling the Aio, and Shutdown() it after.";
        ABSL_CHECK(started_on_this_thread == this)
            << ": Polled the Aio on a different thread from the one that ran "
               "this event loop's Startup().  Its wakeups and timers belong "
               "to that thread, so Startup() has to run on the thread that "
               "polls the Aio.";
      });
}

void QueueEventLoop::Initialize(const Configuration *configuration) {
  ClaimAio(aio_);
  // Ignore the wakeup signal by default. Otherwise, we have race conditions on
  // shutdown where a wakeup signal will uncleanly terminate the process.
  // See LocklessQueueWakeUpper::Wakeup() for some more information.
  IgnoreWakeupSignal();

  ABSL_CHECK(IsInitialized()) << ": Need to initialize AOS first.";
  ClearContext();
  if (configuration->has_nodes()) {
    ABSL_CHECK(node_ != nullptr) << ": Couldn't find node in config.";
  }
}

namespace queue_event_loop_internal {

class SimpleShmFetcher {
 public:
  explicit SimpleShmFetcher(QueueEventLoop *event_loop, const Channel *channel)
      : channel_(channel),
        lockless_queue_memory_(event_loop->MakeQueueMemory(channel)),
        reader_(lockless_queue_memory_->queue()) {
    context_.data = nullptr;
    // Point the queue index at the next index to read starting now.  This
    // makes it such that FetchNext will read the next message sent after
    // the fetcher is created.
    PointAtNextQueueIndex();
  }

  ~SimpleShmFetcher() {}

  // Sets this object to pin or copy data, as configured in the channel.
  void RetrieveData() {
    if (channel_->read_method() == ReadMethod::PIN) {
      PinDataOnFetch();
    } else {
      CopyDataOnFetch();
    }
  }

  // Sets this object to copy data out of the shared memory into a private
  // buffer when fetching.
  void CopyDataOnFetch() {
    ABSL_CHECK(!pin_data());
    data_storage_.reset(static_cast<char *>(
        malloc(channel_->max_size() + kChannelDataAlignment - 1)));
  }

  // Sets this object to pin data in shared memory when fetching.
  void PinDataOnFetch() {
    ABSL_CHECK(!copy_data());
    auto maybe_pinner =
        ipc_lib::LocklessQueuePinner::Make(lockless_queue_memory_->queue());
    if (!maybe_pinner) {
      ABSL_LOG(FATAL) << "Failed to create reader on "
                      << configuration::CleanedChannelToString(channel_)
                      << ", too many readers.";
    }
    pinner_ = std::move(maybe_pinner.value());
  }

  // Points the next message to fetch at the queue index which will be
  // populated next.
  void PointAtNextQueueIndex() {
    actual_queue_index_ = reader_.LatestIndex();
    if (!actual_queue_index_.valid()) {
      // Nothing in the queue.  The next element will show up at the 0th
      // index in the queue.
      actual_queue_index_ = ipc_lib::QueueIndex::Zero(
          LocklessQueueSize(lockless_queue_memory_->memory()));
    } else {
      actual_queue_index_ = actual_queue_index_.Increment();
    }
  }

  RawFetcher::Result FetchNext() { return FetchNextIf(should_fetch_); }

  RawFetcher::Result FetchNextIf(std::function<bool(const Context &)> fn) {
    const ipc_lib::LocklessQueueReader::Result read_result =
        DoFetch(actual_queue_index_, std::move(fn));

    return read_result;
  }

  RawFetcher::Result FetchIf(std::function<bool(const Context &)> fn) {
    const ipc_lib::QueueIndex queue_index = reader_.LatestIndex();
    // actual_queue_index_ is only meaningful if it was set by Fetch or
    // FetchNext.  This happens when valid_data_ has been set.  So, only
    // skip checking if valid_data_ is true.
    //
    // Also, if the latest queue index is invalid, we are empty.  So there
    // is nothing to fetch.
    if ((context_.data != nullptr &&
         queue_index == actual_queue_index_.DecrementBy(1u)) ||
        !queue_index.valid()) {
      return RawFetcher::Result::NOTHING_NEW;
    }

    const RawFetcher::Result read_result = DoFetch(queue_index, std::move(fn));

    ABSL_CHECK(read_result != ipc_lib::LocklessQueueReader::Result::NOTHING_NEW)
        << ": Queue index went backwards.  This should never happen.  "
        << configuration::CleanedChannelToString(channel_);

    return read_result;
  }

  RawFetcher::Result Fetch() { return FetchIf(should_fetch_); }

  Context context() const { return context_; }

  bool RegisterWakeup(int priority) {
    ABSL_CHECK(!watcher_);
    watcher_ = ipc_lib::LocklessQueueWatcher::Make(
        lockless_queue_memory_->queue(), priority);
    return static_cast<bool>(watcher_);
  }

  void UnregisterWakeup() {
    ABSL_CHECK(watcher_);
    watcher_ = std::nullopt;
  }

  absl::Span<char> GetMutableSharedMemory() const {
    return lockless_queue_memory_->GetMutableSharedMemory();
  }

  absl::Span<const char> GetConstSharedMemory() const {
    return lockless_queue_memory_->GetConstSharedMemory();
  }

  absl::Span<const char> GetPrivateMemory() const {
    if (pin_data()) {
      return lockless_queue_memory_->GetConstSharedMemory();
    }
    return absl::Span<char>(
        const_cast<SimpleShmFetcher *>(this)->data_storage_start(),
        LocklessQueueMessageDataSize(lockless_queue_memory_->memory()));
  }

  void SetUseWritableMemory(bool use_writable_memory) {
    if (pin_data()) {
      pinner_->set_use_writable_memory(use_writable_memory);
    }
    reader_.set_use_writable_memory(use_writable_memory);
  }

 private:
  ipc_lib::LocklessQueueReader::Result DoFetch(
      ipc_lib::QueueIndex queue_index,
      std::function<bool(const Context &context)> fn) {
    // TODO(austin): Get behind and make sure it dies.
    char *copy_buffer = nullptr;
    if (copy_data()) {
      copy_buffer = data_storage_start();
    }
    ipc_lib::LocklessQueueReader::Result read_result = reader_.Read(
        queue_index.index(), &context_.monotonic_event_time,
        &context_.realtime_event_time, &context_.monotonic_remote_time,
        &context_.monotonic_remote_transmit_time,
        &context_.realtime_remote_time, &context_.remote_queue_index,
        &context_.source_boot_uuid, &context_.size, copy_buffer, std::move(fn));

    if (read_result == ipc_lib::LocklessQueueReader::Result::GOOD) {
      if (pin_data()) {
        const int pin_result = pinner_->PinIndex(queue_index.index());
        ABSL_CHECK(pin_result >= 0)
            << ": Got behind while reading and the last message was modified "
               "out from under us while we tried to pin it. Don't get so far "
               "behind on: "
            << configuration::CleanedChannelToString(channel_);
        context_.buffer_index = pin_result;
      } else {
        context_.buffer_index = -1;
      }

      context_.queue_index = queue_index.index();
      if (context_.remote_queue_index == 0xffffffffu) {
        context_.remote_queue_index = context_.queue_index;
      }
      if (context_.monotonic_remote_time == aos::monotonic_clock::min_time) {
        context_.monotonic_remote_time = context_.monotonic_event_time;
      }
      if (context_.realtime_remote_time == aos::realtime_clock::min_time) {
        context_.realtime_remote_time = context_.realtime_event_time;
      }
      const char *const data = DataBuffer();
      if (data) {
        context_.data =
            data +
            LocklessQueueMessageDataSize(lockless_queue_memory_->memory()) -
            context_.size;
      } else {
        context_.data = nullptr;
      }
      actual_queue_index_ = queue_index.Increment();
    }
    return read_result;
  }

  char *data_storage_start() const {
    ABSL_CHECK(copy_data());
    return RoundChannelData(data_storage_.get(), channel_->max_size());
  }

  // Note that for some modes the return value will change as new messages are
  // read.
  const char *DataBuffer() const {
    if (copy_data()) {
      return data_storage_start();
    }
    if (pin_data()) {
      return static_cast<const char *>(pinner_->Data());
    }
    return nullptr;
  }

  bool copy_data() const { return static_cast<bool>(data_storage_); }
  bool pin_data() const { return static_cast<bool>(pinner_); }

  const Channel *const channel_;
  std::unique_ptr<ipc_lib::QueueMemory> lockless_queue_memory_;
  ipc_lib::LocklessQueueReader reader_;
  // This being nullopt indicates we're not looking for wakeups right now.
  std::optional<ipc_lib::LocklessQueueWatcher> watcher_;

  ipc_lib::QueueIndex actual_queue_index_ = ipc_lib::QueueIndex::Invalid();

  // This being empty indicates we're not going to copy data.
  std::unique_ptr<char, decltype(&free)> data_storage_{nullptr, &free};

  // This being nullopt indicates we're not going to pin messages.
  std::optional<ipc_lib::LocklessQueuePinner> pinner_;

  Context context_;

  // Pre-allocated should_fetch function so we don't allocate.
  const std::function<bool(const Context &)> should_fetch_;
};

class ShmFetcher : public RawFetcher {
 public:
  explicit ShmFetcher(QueueEventLoop *event_loop, const Channel *channel)
      : RawFetcher(event_loop, channel),
        simple_shm_fetcher_(event_loop, channel) {
    simple_shm_fetcher_.RetrieveData();
  }

  ~ShmFetcher() override {
    ABSL_CHECK(!event_loop()->is_running())
        << ": Can't destroy Fetcher while running";
    queue_event_loop()->CheckCurrentThread();
    context_.data = nullptr;
  }

  std::pair<RawFetcher::Result, monotonic_clock::time_point> DoFetchNext()
      override {
    queue_event_loop()->CheckCurrentThread();
    RawFetcher::Result result = simple_shm_fetcher_.FetchNext();
    if (result == RawFetcher::Result::GOOD) {
      context_ = simple_shm_fetcher_.context();
      return std::make_pair(result, monotonic_clock::now());
    }
    return std::make_pair(result, monotonic_clock::min_time);
  }

  std::pair<RawFetcher::Result, monotonic_clock::time_point> DoFetchNextIf(
      std::function<bool(const Context &context)> fn) override {
    queue_event_loop()->CheckCurrentThread();
    RawFetcher::Result result = simple_shm_fetcher_.FetchNextIf(std::move(fn));
    if (result == RawFetcher::Result::GOOD) {
      context_ = simple_shm_fetcher_.context();
      return std::make_pair(result, monotonic_clock::now());
    }
    return std::make_pair(result, monotonic_clock::min_time);
  }

  std::pair<bool, monotonic_clock::time_point> DoFetch() override {
    queue_event_loop()->CheckCurrentThread();
    if (ConvertReaderResultToBoolOrDie(simple_shm_fetcher_.Fetch())) {
      context_ = simple_shm_fetcher_.context();
      return std::make_pair(true, monotonic_clock::now());
    }
    return std::make_pair(false, monotonic_clock::min_time);
  }

  std::pair<bool, monotonic_clock::time_point> DoFetchIf(
      std::function<bool(const Context &context)> fn) override {
    queue_event_loop()->CheckCurrentThread();
    if (ConvertReaderResultToBoolOrDie(
            simple_shm_fetcher_.FetchIf(std::move(fn)))) {
      context_ = simple_shm_fetcher_.context();
      return std::make_pair(true, monotonic_clock::now());
    }
    return std::make_pair(false, monotonic_clock::min_time);
  }

  absl::Span<const char> GetPrivateMemory() const {
    return simple_shm_fetcher_.GetPrivateMemory();
  }

  absl::Span<char> GetMutableSharedMemory() const {
    return simple_shm_fetcher_.GetMutableSharedMemory();
  }

  void SetUseWritableMemory(bool use_writable_memory) {
    simple_shm_fetcher_.SetUseWritableMemory(use_writable_memory);
  }

 private:
  const QueueEventLoop *queue_event_loop() const {
    return static_cast<const QueueEventLoop *>(event_loop());
  }

  SimpleShmFetcher simple_shm_fetcher_;
};

class ShmExitHandle : public ExitHandle {
 public:
  ShmExitHandle(QueueEventLoop *event_loop) : event_loop_(event_loop) {
    ++event_loop_->exit_handle_count_;
  }
  ~ShmExitHandle() override {
    ABSL_CHECK_GT(event_loop_->exit_handle_count_, 0);
    --event_loop_->exit_handle_count_;
  }
  // Because of how we handle reference counting, we either need to implement
  // reference counting in the copy/move constructors or just not support them.
  // If we ever develop a need for this object to be movable/copyable,
  // supporting it should be straightforwards.
  DISALLOW_COPY_AND_ASSIGN(ShmExitHandle);

  void Exit(Status status) override { event_loop_->ExitWithStatus(status); }

 private:
  QueueEventLoop *const event_loop_;
};

class ShmSender : public RawSender {
 public:
  explicit ShmSender(QueueEventLoop *event_loop, const Channel *channel)
      : RawSender(event_loop, channel),
        lockless_queue_memory_(event_loop->MakeQueueMemory(channel)),
        lockless_queue_sender_(
            VerifySender(ipc_lib::LocklessQueueSender::Make(
                             lockless_queue_memory_->queue(),
                             configuration::ChannelStorageDuration(
                                 event_loop->configuration(), channel)),
                         channel)),
        wake_upper_(lockless_queue_memory_->queue()) {}

  ~ShmSender() override { queue_event_loop()->CheckCurrentThread(); }

  static ipc_lib::LocklessQueueSender VerifySender(
      std::optional<ipc_lib::LocklessQueueSender> sender,
      const Channel *channel) {
    if (sender) {
      return std::move(sender.value());
    }
    ABSL_LOG(FATAL) << "Failed to create sender on "
                    << configuration::CleanedChannelToString(channel)
                    << ", too many senders.";
    AOS_UNREACHABLE();
  }

  void *data() override {
    queue_event_loop()->CheckCurrentThread();
    return lockless_queue_sender_.Data();
  }
  size_t size() override {
    queue_event_loop()->CheckCurrentThread();
    return lockless_queue_sender_.size();
  }

  Error DoSend(size_t length,
               aos::monotonic_clock::time_point monotonic_remote_time,
               aos::realtime_clock::time_point realtime_remote_time,
               aos::monotonic_clock::time_point monotonic_remote_transmit_time,
               uint32_t remote_queue_index,
               const UUID &source_boot_uuid) override {
    queue_event_loop()->CheckCurrentThread();
    ABSL_CHECK_LE(length, static_cast<size_t>(channel()->max_size()))
        << ": Sent too big a message on "
        << configuration::CleanedChannelToString(channel());
    const auto result = lockless_queue_sender_.Send(
        length, monotonic_remote_time, realtime_remote_time,
        monotonic_remote_transmit_time, remote_queue_index, source_boot_uuid,
        &monotonic_sent_time_, &realtime_sent_time_, &sent_queue_index_);
    ABSL_CHECK_NE(result, ipc_lib::LocklessQueueSender::Result::INVALID_REDZONE)
        << ": Somebody wrote outside the buffer of their message on channel "
        << configuration::CleanedChannelToString(channel());

    wake_upper_.Wakeup(event_loop()->is_running()
                           ? event_loop()->runtime_realtime_priority()
                           : 0);
    return CheckLocklessQueueResult(result);
  }

  Error DoSend(const void *msg, size_t length,
               aos::monotonic_clock::time_point monotonic_remote_time,
               aos::realtime_clock::time_point realtime_remote_time,
               aos::monotonic_clock::time_point monotonic_remote_transmit_time,
               uint32_t remote_queue_index,
               const UUID &source_boot_uuid) override {
    queue_event_loop()->CheckCurrentThread();
    ABSL_CHECK_LE(length, static_cast<size_t>(channel()->max_size()))
        << ": Sent too big a message on "
        << configuration::CleanedChannelToString(channel());
    const auto result = lockless_queue_sender_.Send(
        reinterpret_cast<const char *>(msg), length, monotonic_remote_time,
        realtime_remote_time, monotonic_remote_transmit_time,
        remote_queue_index, source_boot_uuid, &monotonic_sent_time_,
        &realtime_sent_time_, &sent_queue_index_);

    ABSL_CHECK_NE(result, ipc_lib::LocklessQueueSender::Result::INVALID_REDZONE)
        << ": Somebody wrote outside the buffer of their message on "
           "channel "
        << configuration::CleanedChannelToString(channel());
    wake_upper_.Wakeup(event_loop()->is_running()
                           ? event_loop()->runtime_realtime_priority()
                           : 0);

    return CheckLocklessQueueResult(result);
  }

  absl::Span<char> GetSharedMemory() const {
    return lockless_queue_memory_->GetMutableSharedMemory();
  }

  int buffer_index() override {
    queue_event_loop()->CheckCurrentThread();
    return lockless_queue_sender_.buffer_index();
  }

 private:
  const QueueEventLoop *queue_event_loop() const {
    return static_cast<const QueueEventLoop *>(event_loop());
  }

  RawSender::Error CheckLocklessQueueResult(
      const ipc_lib::LocklessQueueSender::Result &result) {
    switch (result) {
      case ipc_lib::LocklessQueueSender::Result::GOOD:
        return Error::kOk;
      case ipc_lib::LocklessQueueSender::Result::MESSAGES_SENT_TOO_FAST:
        return Error::kMessagesSentTooFast;
      case ipc_lib::LocklessQueueSender::Result::INVALID_REDZONE:
        return Error::kInvalidRedzone;
    }
    ABSL_LOG(FATAL) << "Unknown lockless queue sender result"
                    << static_cast<int>(result);
    AOS_UNREACHABLE();
  }

  std::unique_ptr<ipc_lib::QueueMemory> lockless_queue_memory_;
  ipc_lib::LocklessQueueSender lockless_queue_sender_;
  ipc_lib::LocklessQueueWakeUpper wake_upper_;
};

// Class to manage the state for a Watcher.
class ShmWatcherState : public WatcherState {
 public:
  ShmWatcherState(
      QueueEventLoop *event_loop, const Channel *channel,
      std::function<void(const Context &context, const void *message)> fn,
      bool copy_data)
      : WatcherState(event_loop, channel, std::move(fn)),
        event_loop_(event_loop),
        event_(this),
        simple_shm_fetcher_(event_loop, channel) {
    if (copy_data) {
      simple_shm_fetcher_.RetrieveData();
    }
  }

  ~ShmWatcherState() override {
    event_loop_->CheckCurrentThread();
    event_loop_->RemoveEvent(&event_);
  }

  void Construct() override {
    event_loop_->CheckCurrentThread();
    ABSL_CHECK(RegisterWakeup(event_loop_->runtime_realtime_priority()));
  }

  void Startup() override {
    event_loop_->CheckCurrentThread();
    simple_shm_fetcher_.PointAtNextQueueIndex();
  }

  // Returns true if there is new data available.
  bool CheckForNewData() {
    if (!has_new_data_) {
      has_new_data_ =
          simple_shm_fetcher_.FetchNext() == RawFetcher::Result::GOOD;

      if (has_new_data_) {
        event_.set_event_time(
            simple_shm_fetcher_.context().monotonic_event_time);
        event_loop_->AddEvent(&event_);
      }
    }

    return has_new_data_;
  }

  // Consumes the data by calling the callback.
  void HandleEvent() {
    ABSL_CHECK(has_new_data_);
    DoCallCallback(monotonic_clock::now, simple_shm_fetcher_.context());
    has_new_data_ = false;
    CheckForNewData();
  }

  // Registers us to receive a signal on event reception.
  bool RegisterWakeup(int priority) {
    return simple_shm_fetcher_.RegisterWakeup(priority);
  }

  void UnregisterWakeup() { return simple_shm_fetcher_.UnregisterWakeup(); }

  absl::Span<char> GetMutableSharedMemory() const {
    return simple_shm_fetcher_.GetMutableSharedMemory();
  }

  void SetUseWritableMemory(bool use_writable_memory) {
    simple_shm_fetcher_.SetUseWritableMemory(use_writable_memory);
  }

 private:
  bool has_new_data_ = false;

  QueueEventLoop *event_loop_;
  EventHandler<ShmWatcherState> event_;
  SimpleShmFetcher simple_shm_fetcher_;
};

// Adapts an Aio::Timer to a TimerHandler.  An Aio::Timer fires once per
// Schedule(), so HandleEvent() re-arms it for a repeating timer.
class ShmTimerHandler final : public TimerHandler {
 public:
  ShmTimerHandler(QueueEventLoop *queue_event_loop, ::std::function<void()> fn)
      : TimerHandler(queue_event_loop, std::move(fn)),
        queue_event_loop_(queue_event_loop),
        event_(this),
        timer_(queue_event_loop_->aio_) {}

  ~ShmTimerHandler() {
    queue_event_loop_->CheckCurrentThread();
    DisableTimer();
  }

  void HandleEvent() {
    ABSL_CHECK(!event_.valid());
    disabled_ = false;
    const auto monotonic_now = Call(monotonic_clock::now, base_);
    if (event_.valid()) {
      // If someone called Schedule inside Call, rescheduling is already taken
      // care of. Bail.
      return;
    }
    if (disabled_) {
      // Somebody called Disable inside Call, so we don't want to reschedule.
      // Bail.
      return;
    }

    if (repeat_offset_ == std::chrono::seconds(0)) {
      timer_.Cancel();
      disabled_ = true;
    } else {
      // Compute how many cycles have elapsed and schedule the next iteration
      // for the next iteration in the future.
      const int elapsed_cycles =
          std::max<int>(0, (monotonic_now - base_ + repeat_offset_ -
                            std::chrono::nanoseconds(1)) /
                               repeat_offset_);
      base_ += repeat_offset_ * elapsed_cycles;

      // Update the heap and schedule the timer wakeup.
      event_.set_event_time(base_);
      queue_event_loop_->AddEvent(&event_);
      timer_.Schedule(base_, &OnTimerComplete, this);
      disabled_ = false;
    }
  }

  void Schedule(monotonic_clock::time_point base,
                monotonic_clock::duration repeat_offset) override {
    queue_event_loop_->CheckLoopThread();
    ABSL_CHECK_GE(repeat_offset, monotonic_clock::zero());
    if (event_.valid()) {
      queue_event_loop_->RemoveEvent(&event_);
    }

    // Aio timers are one-shot, so a repeating timer reschedules itself:
    // HandleEvent() advances base_ to the next period after now and arms the
    // timer for it.  base_ is the event time for both the event heap and the
    // timing report, so it comes from the clock, not from how many
    // expirations the kernel counted.
    timer_.Schedule(base, &OnTimerComplete, this);
    base_ = base;
    repeat_offset_ = repeat_offset;
    event_.set_event_time(base_);
    queue_event_loop_->AddEvent(&event_);
    disabled_ = false;
  }

  void Disable() override {
    queue_event_loop_->CheckLoopThread();
    DisableTimer();
  }

  bool IsDisabled() override { return disabled_; }

 private:
  void DisableTimer() {
    queue_event_loop_->RemoveEvent(&event_);
    timer_.Cancel();
    disabled_ = true;
  }

  // Timer completions are Ok() by contract (see Aio::Timer::Schedule());
  // CHECK rather than handle.
  static void OnTimerComplete(Completion completion, void *context) {
    ABSL_CHECK(completion.status.has_value());
    static_cast<ShmTimerHandler *>(context)->queue_event_loop_->HandleEvent();
  }

  QueueEventLoop *queue_event_loop_;
  EventHandler<ShmTimerHandler> event_;

  Aio::Timer timer_;

  monotonic_clock::time_point base_;
  monotonic_clock::duration repeat_offset_;

  // Used to track if Disable() was called during the callback, so we know not
  // to reschedule.
  bool disabled_ = true;
};

// Adapts an Aio::Timer and a PhasedLoop to a PhasedLoopHandler.
class ShmPhasedLoopHandler final : public PhasedLoopHandler {
 public:
  ShmPhasedLoopHandler(QueueEventLoop *queue_event_loop,
                       ::std::function<void(int)> fn,
                       const monotonic_clock::duration interval,
                       const monotonic_clock::duration offset)
      : PhasedLoopHandler(queue_event_loop, std::move(fn), interval, offset),
        queue_event_loop_(queue_event_loop),
        event_(this),
        timer_(queue_event_loop_->aio_) {}

  void HandleEvent() {
    event_.Invalidate();
    Call(monotonic_clock::now);
  }

  ~ShmPhasedLoopHandler() override {
    queue_event_loop_->CheckCurrentThread();
    queue_event_loop_->RemoveEvent(&event_);
  }

 private:
  // Reschedules the timer.
  void Schedule(monotonic_clock::time_point sleep_time) override {
    queue_event_loop_->CheckLoopThread();
    if (event_.valid()) {
      queue_event_loop_->RemoveEvent(&event_);
    }

    timer_.Schedule(sleep_time, &OnTimerComplete, this);
    event_.set_event_time(sleep_time);
    queue_event_loop_->AddEvent(&event_);
  }

  // Same contract CHECK as ShmTimerHandler::OnTimerComplete().
  static void OnTimerComplete(Completion completion, void *context) {
    ABSL_CHECK(completion.status.has_value());
    static_cast<ShmPhasedLoopHandler *>(context)
        ->queue_event_loop_->HandleEvent();
  }

  QueueEventLoop *queue_event_loop_;
  EventHandler<ShmPhasedLoopHandler> event_;
  Aio::Timer timer_;
};

class ShmThreadHandle : public ThreadHandle {
 public:
  ShmThreadHandle(QueueEventLoop *event_loop,
                  const ThreadConfiguration &thread_configuration)
      : event_loop_(event_loop) {
    // Make sure that we're not accidentally constructing this in the main
    // thread.
    event_loop_->CheckNotMainThread();
    ABSL_CHECK(thread_configuration.has_name());

    ABSL_LOG(INFO) << "Thread " << thread_configuration.name()->string_view()
                   << " waiting for the event loop to start running.";

    // Unblock the Run() call.
    event_loop_->thread_ready_semaphore_.release();

    // Wait for the main thread to call InitRT().
    event_loop_->thread_running_semaphore_.acquire();

    // Set up the thread name so that it shows up in top(1).
    SetCurrentThreadName(thread_configuration.name()->string_view());
    // Set the CPU affinity.
    if (thread_configuration.has_cpu_affinity()) {
      SetCurrentThreadAffinity(MakeCpusetFromCpus(
          flatbuffers::make_span(thread_configuration.cpu_affinity())));
    }
    // Set the realtime priority.
    if (thread_configuration.has_scheduling_policy()) {
      switch (thread_configuration.scheduling_policy()) {
        case SchedulingPolicy::SCHEDULER_FIFO:
          SetCurrentThreadRealtimePriority(thread_configuration.priority(),
                                           SCHED_FIFO);
          break;
        case SchedulingPolicy::SCHEDULER_RR:
          SetCurrentThreadRealtimePriority(thread_configuration.priority(),
                                           SCHED_RR);
          break;
        case SchedulingPolicy::SCHEDULER_OTHER:
          break;
      }
    }
  }

  ~ShmThreadHandle() override {
    // When the thread is shutting down, we should restore normal priority.
    UnsetCurrentThreadRealtimePriority();

    // Reset the CPU affinity.
    SetCurrentThreadAffinity(DefaultAffinity());
  }

 private:
  QueueEventLoop *event_loop_;
};

}  // namespace queue_event_loop_internal

::std::unique_ptr<RawFetcher> QueueEventLoop::MakeRawFetcher(
    const Channel *channel) {
  ABSL_CHECK(!is_running()) << ": Can't make Fetcher while running";
  CheckCurrentThread();
  if (!configuration::ChannelIsReadableOnNode(channel, node())) {
    ABSL_LOG(FATAL)
        << "Channel { \"name\": \"" << channel->name()->string_view()
        << "\", \"type\": \"" << channel->type()->string_view()
        << "\" } is not able to be fetched on this node.  Check your "
           "configuration.";
  }

  return ::std::unique_ptr<RawFetcher>(new ShmFetcher(this, channel));
}

::std::unique_ptr<RawSender> QueueEventLoop::MakeRawSender(
    const Channel *channel) {
  CheckCurrentThread();
  TakeSender(channel);

  return ::std::unique_ptr<RawSender>(new ShmSender(this, channel));
}

void QueueEventLoop::MakeRawWatcher(
    const Channel *channel,
    std::function<void(const Context &context, const void *message)> watcher) {
  CheckCurrentThread();
  TakeWatcher(channel);

  NewWatcher(::std::unique_ptr<WatcherState>(
      new ShmWatcherState(this, channel, std::move(watcher), true)));
}

void QueueEventLoop::MakeRawNoArgWatcher(
    const Channel *channel,
    std::function<void(const Context &context)> watcher) {
  CheckCurrentThread();
  TakeWatcher(channel);

  NewWatcher(::std::unique_ptr<WatcherState>(new ShmWatcherState(
      this, channel,
      [watcher](const Context &context, const void *) { watcher(context); },
      false)));
}

TimerHandler *QueueEventLoop::AddTimer(::std::function<void()> callback) {
  CheckCurrentThread();
  return NewTimer(::std::unique_ptr<TimerHandler>(
      new ShmTimerHandler(this, ::std::move(callback))));
}

PhasedLoopHandler *QueueEventLoop::AddPhasedLoop(
    ::std::function<void(int)> callback,
    const monotonic_clock::duration interval,
    const monotonic_clock::duration offset) {
  CheckCurrentThread();
  return NewPhasedLoop(::std::unique_ptr<PhasedLoopHandler>(
      new ShmPhasedLoopHandler(this, ::std::move(callback), interval, offset)));
}

void QueueEventLoop::OnRun(::std::function<void()> on_run) {
  CheckCurrentThread();
  on_run_.push_back(::std::move(on_run));
}

void QueueEventLoop::CheckCurrentThread() const {
  if (AOS_UNLIKELY(check_mutex_ != nullptr)) {
    ABSL_CHECK(check_mutex_->is_locked())
        << ": The configured mutex is not locked while calling an event loop "
           "function";
  }
  if (AOS_UNLIKELY(!!check_tid_)) {
    ABSL_CHECK_EQ(aos::GetThreadId(), *check_tid_)
        << ": Being called from the wrong thread. Call from the main thread "
           "instead.";
  }
}

void QueueEventLoop::CheckLoopThread() const {
  CheckCurrentThread();
  if (loop_thread_) {
    ABSL_CHECK(std::this_thread::get_id() == *loop_thread_)
        << ": Timers and phased loops can only be scheduled or disabled from "
           "the thread that runs the event loop.";
  }
}

void QueueEventLoop::CheckNotMainThread() const {
  std::optional<pid_t> main_tid = check_tid_;

  ABSL_CHECK(main_tid.has_value())
      << ": Call LockToThread() before constructing any threads.";
  ABSL_CHECK_NE(aos::GetThreadId(), *main_tid)
      << ": Do not call this function from the main thread.";
}

// This is a bit tricky because watchers can generate new events at any time (as
// long as it's in the past). We want to check the watchers at least once before
// declaring there are no events to handle, and we want to check them again if
// event processing takes long enough that we find an event after that point in
// time to handle.
void QueueEventLoop::HandleEvent() {
  // Time through which we've checked for new events in watchers.
  monotonic_clock::time_point checked_until = monotonic_clock::min_time;
  if (!signal_receiver_) {
    // Nothing to check, so we can bail out immediately once we're out of
    // events.
    ABSL_CHECK(watchers_.empty());
    checked_until = monotonic_clock::max_time;
  }

  // Loop until we run out of events to check.
  while (true) {
    // Time of the next event we know about. If this is before checked_until, we
    // know there aren't any new events before the next one that we already know
    // about, so no need to check the watchers.
    monotonic_clock::time_point next_time = monotonic_clock::max_time;

    if (EventCount() == 0) {
      if (checked_until != monotonic_clock::min_time) {
        // No events, and we've already checked the watchers at least once, so
        // we're all done.
        //
        // There's a small chance that a watcher has gotten another event in
        // between checked_until and now. If so, then the signal receiver will
        // be triggered now and we'll re-enter HandleEvent immediately. This is
        // unlikely though, so we don't want to spend time checking all the
        // watchers unnecessarily.
        break;
      }
    } else {
      next_time = PeekEvent()->event_time();
    }
    monotonic_clock::time_point now;
    bool new_data = false;

    if (next_time > checked_until) {
      // Consume all pending wakeup signals, so we don't wake up again
      // immediately to handle them.
      aio_->ConsumeThreadSignalReceiver(signal_receiver_.get());
      // This is the last time we can guarantee that if a message is published
      // before, we will notice it.
      now = monotonic_clock::now();

      // Check all the watchers for new events.
      for (std::unique_ptr<WatcherState> &base_watcher : watchers_) {
        ShmWatcherState *const watcher =
            reinterpret_cast<ShmWatcherState *>(base_watcher.get());

        // Track if we got a message.
        if (watcher->CheckForNewData()) {
          new_data = true;
        }
      }
      if (EventCount() == 0) {
        // Still no events, all done now.
        break;
      }

      checked_until = now;
      // Check for any new events we found.
      next_time = PeekEvent()->event_time();
    } else {
      now = monotonic_clock::now();
    }

    if (next_time > now) {
      // Ok, we got a message with a timestamp *after* we wrote down time.  We
      // need to process it (otherwise we will go to sleep without processing
      // it), but we also need to make sure no other messages have come in
      // before it that we would process out of order.  Just go around again to
      // redo the checks.
      if (new_data) {
        continue;
      }
      break;
    }

    EventLoopEvent *const event = PopEvent();
    in_callback_ = true;
    event->HandleEvent();
    in_callback_ = false;
  }
}

void QueueEventLoop::set_handle_signals(bool handle_signals) {
  CheckCurrentThread();
  ABSL_CHECK(!started_) << ": Must be called before Startup().";
  handle_signals_ = handle_signals;
}

void QueueEventLoop::Startup() {
  ABSL_CHECK(!started_) << ": Startup() has already run.";
  ABSL_CHECK(started_on_this_thread == nullptr)
      << ": Another event loop is already started on this thread.  An event "
         "loop's wakeups, realtime priority and logging are per thread, so "
         "only one may be started on a thread at a time.  Shut the other "
         "down first, or start this one on a thread of its own.";
  started_on_this_thread = this;
  started_ = true;
  CheckCurrentThread();
  loop_thread_ = std::this_thread::get_id();
  if (handle_signals_) {
    RegisterSignalHandler();
    registered_signal_handler_ = true;
  }

  if (watchers_.size() > 0) {
    signal_receiver_.reset(new ipc_lib::ThreadSignalReceiver());
    signal_receiver_->LeaveSignalBlocked();

    aio_->RegisterThreadSignalReceiver(signal_receiver_.get(),
                                       [this]() { HandleEvent(); });
  }

  MaybeScheduleTimingReports();

  ReserveEvents();

  // These outlive Startup() rather than being scoped locals, because the
  // waiting they bracket happens in the caller.  The restorer puts back the
  // thread's logging implementation at Shutdown().
  log_restorer_ = std::make_unique<logging::ScopedLogRestorer>();
  aos_logger_ = std::make_unique<AosLogToFbs>();
  if (!skip_logger_) {
    aos_logger_->Initialize(&name_, MakeSender<logging::LogMessageFbs>("/aos"));
    log_restorer_->Swap(aos_logger_->implementation());
  }

  aos::SetCurrentThreadName(name_.substr(0, 16));
  const CpuSet default_affinity = DefaultAffinity();
  if (runtime_affinity_ != default_affinity) {
    ::aos::SetCurrentThreadAffinity(runtime_affinity_);
  }

  // Construct the watchers, but don't update the next pointer. This also
  // cleans up any watchers that previously died, and puts the nonrt work
  // before going realtime.  After this happens, we will start queueing
  // signals (which may be a bit of extra work to process, but won't cause any
  // messages to be lost).
  for (::std::unique_ptr<WatcherState> &watcher : watchers_) {
    watcher->Construct();
  }

  // Wait for the threads to start up before moving on.
  WaitForNonIgnoredThreads();

  const bool need_realtime =
      SchedulingPolicyIsRealtime(runtime_scheduling_policy_) ||
      (threads_ &&
       std::ranges::any_of(*threads_, ThreadIsConfiguredToBeRealtime));
  if (need_realtime) {
    ::aos::InitRT();
  }

  // Tell the threads that they can start realtime configuration and start
  // running.
  AllowNonIgnoredThreadsToStart();

  // Now, all the callbacks are setup.  Lock everything into memory and go RT.
  if (SchedulingPolicyIsRealtime(runtime_scheduling_policy_)) {
    const int scheduling_policy_id =
        (runtime_scheduling_policy_ == SchedulingPolicy::SCHEDULER_FIFO)
            ? SCHED_FIFO
            : SCHED_RR;

    ABSL_LOG(INFO) << "Setting scheduling policy to "
                   << runtime_scheduling_policy_ << " and realtime priority to "
                   << runtime_priority_ << " for " << name_;
    ::aos::SetCurrentThreadRealtimePriority(
        runtime_priority_, scheduling_policy_id, runtime_realtime_policy_);
  }

  set_is_running(true);

  // Now that we are realtime (but before the OnRun handlers run), snap the
  // queue index pointer to the newest message. This happens in RT so that we
  // minimize the risk of losing messages.
  for (::std::unique_ptr<WatcherState> &watcher : watchers_) {
    watcher->Startup();
  }

  // Now that we are RT, run all the OnRun handlers.
  SetTimerContext(monotonic_clock::now());
  in_callback_ = true;
  for (const auto &run : on_run_) {
    run();
  }
  in_callback_ = false;
}

Status QueueEventLoop::Shutdown() {
  ABSL_CHECK(started_) << ": Shutdown() without a Startup() to undo.";
  ABSL_CHECK(!shut_down_) << ": Shutdown() reentered from inside itself.";
  ABSL_CHECK(!in_callback_)
      << ": Shutdown() from inside one of this event loop's own callbacks.  "
         "It is what Run() returning is on an owned Aio, which never happens "
         "there either.  Exit() from the callback, and Shutdown() once the "
         "Aio is no longer polled.";
  // The watchers it unregisters belong to the thread that ran Startup(), and
  // that is the thread whose realtime priority it drops.
  CheckCurrentThread();
  ABSL_CHECK(started_on_this_thread == this)
      << ": Shutdown() has to run on the thread that ran Startup(), which is "
         "the one driving the Aio.";
  shut_down_ = true;
  // This event loop's work stops here, whether or not the Aio keeps going.
  set_is_running(false);

  // Nothing time or synchronization critical needs to happen after this
  // point. Drop RT priority.
  ::aos::UnsetCurrentThreadRealtimePriority();

  for (::std::unique_ptr<WatcherState> &base_watcher : watchers_) {
    ShmWatcherState *watcher =
        reinterpret_cast<ShmWatcherState *>(base_watcher.get());
    watcher->UnregisterWakeup();
  }

  if (watchers_.size() > 0) {
    aio_->UnregisterThreadSignalReceiver(signal_receiver_.get());
    signal_receiver_.reset();
  }

  if (registered_signal_handler_) {
    UnregisterSignalHandler();
    registered_signal_handler_ = false;
  }
  loop_thread_.reset();
  started_on_this_thread = nullptr;

  // Trigger any remaining senders or fetchers to be cleared before destroying
  // the event loop so the book keeping matches.  Do this in the thread that
  // created the timing reporter.
  timing_report_sender_.reset();
  ClearContext();
  aos_logger_.reset();
  log_restorer_.reset();
  // Back to the state a fresh event loop is in, because Run() may be called
  // again on this one -- aos/starter/subprocess_test does exactly that.  The
  // CHECKs above are about one run starting or stopping twice, not about
  // making an event loop single-use.
  started_ = false;
  shut_down_ = false;
  std::unique_lock<aos::stl_mutex> locker(exit_status_mutex_);
  std::optional<Status> exit_status;
  // Clear the stored exit_status_ and extract it to be returned.
  exit_status_.swap(exit_status);
  return exit_status.value_or(Status{});
}

Status QueueEventLoop::Run() {
  ABSL_CHECK(owned_aio_.has_value())
      << ": Run() drives the Aio, so it is only for an event loop that "
         "owns one.  This one was built on a borrowed Aio; drive that "
         "Aio yourself, with Startup() and Shutdown() around it.";
  Startup();
  // Run all the timers and handle Quit.
  aio_->Run();
  return Shutdown();
}

void QueueEventLoop::Exit() {
  observed_exit_.test_and_set();
  // Implicitly defaults exit_status_ to success by not setting it.

  aio_->Quit();
}

void QueueEventLoop::ExitWithStatus(Status status) {
  // Only set the exit status if no other Exit*() call got here first.
  if (!observed_exit_.test_and_set()) {
    std::unique_lock<aos::stl_mutex> locker(exit_status_mutex_);
    exit_status_ = std::move(status);
  } else {
    ABSL_VLOG(1) << "Exit status is already set; not setting it again.";
  }
  Exit();
}

std::unique_ptr<ExitHandle> QueueEventLoop::MakeExitHandle() {
  return std::make_unique<ShmExitHandle>(this);
}

void QueueEventLoop::CheckNotRunningForDestruction() const {
  // Run() shuts an owned event loop down before it returns, so one still
  // running is being destroyed from inside one of its own callbacks.  One on
  // a borrowed Aio is shut down by whoever started it.
  ABSL_CHECK(!is_running())
      << ": Event loop destroyed while running.  On a borrowed Aio, "
         "Shutdown() it first.";
}

QueueEventLoop::~QueueEventLoop() {
  CheckCurrentThread();
  CheckNotRunningForDestruction();
  if (!owned_aio_.has_value()) {
    aio_->RemoveBeforeWait(check_polled_hook_);
  }
  // Destroy the timers, phased loops and watchers now, while the Aio they are
  // registered with still exists.
  timers_.clear();
  phased_loops_.clear();
  watchers_.clear();

  ABSL_CHECK(!is_running()) << ": Event loop destroyed while running";
  ABSL_CHECK_EQ(0, exit_handle_count_)
      << ": All ExitHandles must be destroyed before the event loop";
  ReleaseAio(aio_);
}

void QueueEventLoop::SetRuntimeRealtimePriority(
    int priority, SchedulingPolicy scheduling_policy,
    RealtimePolicy realtime_policy) {
  CheckCurrentThread();
  ABSL_CHECK(!is_running()) << "Cannot set realtime priority while running.";

  if (priority == 0) {
    runtime_priority_ = 0;
    runtime_scheduling_policy_ = SchedulingPolicy::SCHEDULER_OTHER;
  } else {
    ABSL_CHECK(scheduling_policy == SchedulingPolicy::SCHEDULER_FIFO ||
               scheduling_policy == SchedulingPolicy::SCHEDULER_RR)
        << ": Attempted to set realtime priority without a realtime scheduling "
           "policy";
    runtime_priority_ = priority;
    runtime_scheduling_policy_ = scheduling_policy;
  }
  runtime_realtime_policy_ = realtime_policy;
}

void QueueEventLoop::SetRuntimeAffinity(const CpuSet &cpuset) {
  CheckCurrentThread();
  if (is_running()) {
    ABSL_LOG(FATAL) << "Cannot set affinity while running.";
  }
  runtime_affinity_ = cpuset;
}

std::unique_ptr<ThreadHandle> QueueEventLoop::ConfigureThreadImpl(
    const ThreadConfiguration &thread_configuration) {
  return std::make_unique<ShmThreadHandle>(this, thread_configuration);
}

void QueueEventLoop::IgnoreThreadImpl() {
  ABSL_CHECK(check_tid_.has_value())
      << ": Call LockToThread() before ignoring any threads.";
  CheckCurrentThread();
}

void QueueEventLoop::set_name(const std::string_view name) {
  CheckCurrentThread();
  name_ = std::string(name);
  ParseSchedulingSettings();
  UpdateTimingReport();
}

void QueueEventLoop::SetWatcherUseWritableMemory(const Channel *channel,
                                                 bool use_writable_memory) {
  CheckCurrentThread();
  ShmWatcherState *const watcher_state =
      static_cast<ShmWatcherState *>(GetWatcherState(channel));
  return watcher_state->SetUseWritableMemory(use_writable_memory);
}

absl::Span<char> QueueEventLoop::GetWatcherSharedMemory(
    const Channel *channel) {
  CheckCurrentThread();
  ShmWatcherState *const watcher_state =
      static_cast<ShmWatcherState *>(GetWatcherState(channel));
  return watcher_state->GetMutableSharedMemory();
}

int QueueEventLoop::NumberBuffers(const Channel *channel) {
  CheckCurrentThread();
  return ipc_lib::MakeQueueConfiguration(configuration(), channel)
      .num_messages();
}

absl::Span<char> QueueEventLoop::GetShmSenderSharedMemory(
    const aos::RawSender *sender) const {
  CheckCurrentThread();
  return static_cast<const ShmSender *>(sender)->GetSharedMemory();
}

absl::Span<const char> QueueEventLoop::GetShmFetcherPrivateMemory(
    const aos::RawFetcher *fetcher) const {
  CheckCurrentThread();
  return static_cast<const ShmFetcher *>(fetcher)->GetPrivateMemory();
}

absl::Span<char> QueueEventLoop::GetShmFetcherSharedMemory(
    const aos::RawFetcher *fetcher) const {
  CheckCurrentThread();
  return static_cast<const ShmFetcher *>(fetcher)->GetMutableSharedMemory();
}

void QueueEventLoop::SetShmFetcherUseWritableMemory(
    aos::RawFetcher *fetcher, bool use_writable_memory) const {
  CheckCurrentThread();
  static_cast<ShmFetcher *>(fetcher)->SetUseWritableMemory(use_writable_memory);
}

pid_t QueueEventLoop::GetTid() const {
  CheckCurrentThread();
  return aos::GetThreadId();
}

}  // namespace aos
