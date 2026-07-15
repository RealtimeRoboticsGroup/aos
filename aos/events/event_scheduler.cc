#include "aos/events/event_scheduler.h"

#include <algorithm>
#include <deque>

#include "aos/events/event_loop.h"
#include "aos/logging/implementations.h"

namespace aos {

EventScheduler::Token EventScheduler::Schedule(monotonic_clock::time_point time,
                                               Event *callback) {
  ABSL_CHECK_LE(monotonic_clock::epoch(), time);
  return events_list_.emplace(time, callback);
}

void EventScheduler::Deschedule(EventScheduler::Token token) {
  // We basically want to DCHECK some nontrivial logic. Guard it with NDEBUG to
  // ensure the compiler realizes it's all unnecessary when not doing debug
  // checks.
#ifndef NDEBUG
  {
    bool found = false;
    auto i = events_list_.begin();
    while (i != events_list_.end()) {
      if (i == token) {
        ABSL_CHECK(!found) << ": The same iterator is in the multimap twice??";
        found = true;
      }
      ++i;
    }
    ABSL_CHECK(found)
        << ": Trying to deschedule an event which is not scheduled";
  }
#endif
  events_list_.erase(token);
}

Result<std::pair<distributed_clock::time_point, monotonic_clock::time_point>>
EventScheduler::OldestEvent() {
  // If we haven't started yet, schedule a special event for the epoch to allow
  // ourselves to boot.
  if (!called_started_) {
    if (!cached_epoch_) {
      AOS_ASSIGN_OR_RETURN_ERROR(cached_epoch_,
                                 ToDistributedClock(monotonic_clock::epoch()));
    }
    return std::make_pair(*cached_epoch_, monotonic_clock::epoch());
  }

  if (events_list_.empty()) {
    return std::make_pair(distributed_clock::max_time,
                          monotonic_clock::max_time);
  }

  const monotonic_clock::time_point monotonic_time =
      events_list_.begin()->first;
  if (cached_event_list_monotonic_time_ != monotonic_time) {
    AOS_ASSIGN_OR_RETURN_ERROR(cached_event_list_time_,
                               ToDistributedClock(monotonic_time));
    cached_event_list_monotonic_time_ = monotonic_time;
  }

  return std::make_pair(cached_event_list_time_, monotonic_time);
}

void EventScheduler::Shutdown() {
  ABSL_CHECK(!is_running_);
  on_shutdown_();
}

Status EventScheduler::Startup() {
  ++boot_count_;
  cached_event_list_monotonic_time_ = kInvalidCachedTime();
  ABSL_CHECK(!is_running_);
  AOS_RETURN_IF_ERROR(MaybeRunOnStartup());
  ABSL_CHECK(called_started_);
  return Ok();
}

Status EventScheduler::CallOldestEvent() {
  if (!called_started_) {
    // If we haven't started, start.
    AOS_RETURN_IF_ERROR(MaybeRunOnStartup());
    MaybeRunOnRun();
    ABSL_CHECK(called_started_);
    return Ok();
  }
  ABSL_CHECK(is_running_);
  ABSL_CHECK_GT(events_list_.size(), 0u);
  auto iter = events_list_.begin();
  logger::BootTimestamp t;
  AOS_ASSIGN_OR_RETURN_ERROR(
      t, FromDistributedClock(scheduler_scheduler_->distributed_now()));
  ABSL_VLOG(2) << "Got time back " << t;
  ABSL_CHECK_EQ(t.boot, boot_count_);
  ABSL_CHECK_EQ(t.time, iter->first)
      << ": Time is wrong on node " << node_index_;

  Event *callback = iter->second;
  events_list_.erase(iter);
  callback->Handle();

  converter_->ObserveTimePassed(scheduler_scheduler_->distributed_now());
  return Ok();
}

void EventScheduler::RunOnRun() {
  ABSL_CHECK(is_running_);
  while (!on_run_.empty()) {
    Event *event = *on_run_.begin();
    on_run_.erase(on_run_.begin());
    event->Handle();
  }
}

void EventScheduler::RunOnStartup() noexcept {
  while (!on_startup_.empty()) {
    ABSL_CHECK(!is_running_);
    std::function<void()> fn = std::move(*on_startup_.begin());
    on_startup_.erase(on_startup_.begin());
    fn();
  }
}

void EventScheduler::RunStarted() {
  ABSL_CHECK(!is_running_);
  if (started_) {
    started_();
  }
  is_running_ = true;
}

void EventScheduler::MaybeRunStopped() {
  ABSL_CHECK(is_running_);
  is_running_ = false;
  if (called_started_) {
    called_started_ = false;
    if (stopped_) {
      stopped_();
    }
  }
}

Status EventScheduler::MaybeRunOnStartup() {
  ABSL_CHECK(!called_started_);
  ABSL_CHECK(!is_running_);
  logger::BootTimestamp t;
  AOS_ASSIGN_OR_RETURN_ERROR(
      t, FromDistributedClock(scheduler_scheduler_->distributed_now()));
  if (t.boot == boot_count_ && t.time >= monotonic_clock::epoch()) {
    called_started_ = true;
    RunOnStartup();
  }
  return Ok();
}

void EventScheduler::MaybeRunOnRun() {
  if (called_started_) {
    RunStarted();
    RunOnRun();
  }
}

std::ostream &operator<<(std::ostream &stream,
                         const aos::distributed_clock::time_point &now) {
  // Print it the same way we print a monotonic time.  Literally.
  stream << monotonic_clock::time_point(now.time_since_epoch());
  return stream;
}

void EventSchedulerScheduler::AddEventScheduler(EventScheduler *scheduler) {
  ABSL_CHECK(std::find(schedulers_.begin(), schedulers_.end(), scheduler) ==
             schedulers_.end());
  ABSL_CHECK(scheduler->scheduler_scheduler_ == nullptr);
  ABSL_CHECK_EQ(scheduler->node_index(), schedulers_.size());

  schedulers_.emplace_back(scheduler);
  scheduler->scheduler_scheduler_ = this;
}

void EventSchedulerScheduler::MaybeRunStopped() {
  ABSL_CHECK(!is_running_);
  for (EventScheduler *scheduler : schedulers_) {
    if (scheduler->is_running()) {
      scheduler->MaybeRunStopped();
    }
  }
}

Result<bool> EventSchedulerScheduler::RunUntil(
    realtime_clock::time_point end_time, EventScheduler *scheduler,
    std::function<std::chrono::nanoseconds()> fn_realtime_offset) {
  logging::ScopedLogRestorer prev_logger;
  AOS_RETURN_IF_ERROR(MaybeRunOnStartup());

  bool reached_end_time = false;

  const Status result = RunMaybeRealtimeLoop([this, scheduler, end_time,
                                              fn_realtime_offset,
                                              &reached_end_time]() -> Status {
    std::tuple<distributed_clock::time_point, EventScheduler *> oldest_event;
    AOS_ASSIGN_OR_RETURN_ERROR(oldest_event, OldestEvent());
    aos::distributed_clock::time_point oldest_event_time_distributed =
        std::get<0>(oldest_event);
    logger::BootTimestamp test_time_monotonic;
    AOS_ASSIGN_OR_RETURN_ERROR(
        test_time_monotonic,
        scheduler->FromDistributedClock(oldest_event_time_distributed));
    realtime_clock::time_point oldest_event_realtime(
        test_time_monotonic.time_since_epoch() + fn_realtime_offset());

    if ((std::get<0>(oldest_event) == distributed_clock::max_time) ||
        (oldest_event_realtime > end_time &&
         (reboots_.empty() ||
          std::get<0>(reboots_.front()) > oldest_event_time_distributed))) {
      is_running_ = false;
      reached_end_time = true;

      // We have to nudge our time back to the distributed time
      // corresponding to our desired realtime time.
      const monotonic_clock::time_point end_monotonic =
          monotonic_clock::epoch() + end_time.time_since_epoch() -
          fn_realtime_offset();

      AOS_ASSIGN_OR_RETURN_ERROR(now_,
                                 scheduler->ToDistributedClock(end_monotonic));

      return Ok();
    }

    if (!reboots_.empty() &&
        std::get<0>(reboots_.front()) <= std::get<0>(oldest_event)) {
      // Reboot is next.
      ABSL_CHECK_LE(now_,
                    std::get<0>(reboots_.front()) + std::chrono::nanoseconds(1))
          << ": Simulated time went backwards by too much.  Please "
             "investigate.";
      now_ = std::get<0>(reboots_.front());
      AOS_RETURN_IF_ERROR(Reboot());
      reboots_.erase(reboots_.begin());
      return Ok();
    }

    // We get to pick our tradeoffs here.  Either we assume that there are
    // no backward step changes in our time function for each node, or we
    // have to let time go backwards.  We currently only really see this
    // happen when 2 events are scheduled for "now", time changes, and
    // there is a nanosecond or two of rounding due to integer math.
    //
    // //aos/events/logging:logger_test triggers this.
    ABSL_CHECK_LE(now_, std::get<0>(oldest_event) + std::chrono::nanoseconds(1))
        << ": Simulated time went backwards by too much.  Please "
           "investigate.";

    now_ = std::get<0>(oldest_event);

    return std::get<1>(oldest_event)->CallOldestEvent();
  });

  MaybeRunStopped();

  return result.transform([reached_end_time]() { return reached_end_time; });
}

Status EventSchedulerScheduler::Reboot() {
  const std::vector<logger::BootTimestamp> &times =
      std::get<1>(reboots_.front());
  ABSL_CHECK_EQ(times.size(), schedulers_.size());

  ABSL_VLOG(1) << "Rebooting at " << now_;
  for (const auto &time : times) {
    ABSL_VLOG(1) << "  " << time;
  }

  is_running_ = false;

  // Shut everything down.
  std::vector<size_t> rebooted;
  for (size_t node_index = 0; node_index < schedulers_.size(); ++node_index) {
    if (schedulers_[node_index]->boot_count() == times[node_index].boot) {
      continue;
    } else {
      rebooted.emplace_back(node_index);
      ABSL_CHECK_EQ(schedulers_[node_index]->boot_count() + 1,
                    times[node_index].boot);
      schedulers_[node_index]->MaybeRunStopped();
      schedulers_[node_index]->Shutdown();
    }
  }

  // And start it back up again to reboot.  When something starts back up
  // (especially message_bridge), it could try to send stuff out.  We want
  // to move everything over to the new boot before doing that.
  for (const size_t node_index : rebooted) {
    AOS_RETURN_IF_ERROR(schedulers_[node_index]->Startup());
  }
  for (const size_t node_index : rebooted) {
    schedulers_[node_index]->MaybeRunOnRun();
  }
  is_running_ = true;
  return Ok();
}

Status EventSchedulerScheduler::RunFor(distributed_clock::duration duration) {
  distributed_clock::time_point end_time = now_ + duration;
  logging::ScopedLogRestorer prev_logger;
  AOS_RETURN_IF_ERROR(MaybeRunOnStartup());

  // Run all the sub-event-schedulers.
  const Status result = RunMaybeRealtimeLoop([this, end_time]() -> Status {
    std::tuple<distributed_clock::time_point, EventScheduler *> oldest_event;
    AOS_ASSIGN_OR_RETURN_ERROR(oldest_event, OldestEvent());
    if (!reboots_.empty() &&
        std::get<0>(reboots_.front()) <= std::get<0>(oldest_event)) {
      // Reboot is next.
      if (std::get<0>(reboots_.front()) > end_time) {
        // Reboot is after our end time, give up.
        is_running_ = false;
        return Ok();
      }

      ABSL_CHECK_LE(now_,
                    std::get<0>(reboots_.front()) + std::chrono::nanoseconds(1))
          << ": Simulated time went backwards by too much.  Please "
             "investigate.";
      now_ = std::get<0>(reboots_.front());
      AOS_RETURN_IF_ERROR(Reboot());
      reboots_.erase(reboots_.begin());
      return Ok();
    }

    // No events left, bail.
    if (std::get<0>(oldest_event) == distributed_clock::max_time ||
        std::get<0>(oldest_event) > end_time) {
      is_running_ = false;
      return Ok();
    }

    // We get to pick our tradeoffs here.  Either we assume that there are no
    // backward step changes in our time function for each node, or we have to
    // let time go backwards.  We currently only really see this happen when 2
    // events are scheduled for "now", time changes, and there is a nanosecond
    // or two of rounding due to integer math.
    //
    // //aos/events/logging:logger_test triggers this.
    ABSL_CHECK_LE(now_, std::get<0>(oldest_event) + std::chrono::nanoseconds(1))
        << ": Simulated time went backwards by too much.  Please investigate.";
    // push time forwards
    now_ = std::get<0>(oldest_event);

    if (on_event_) {
      on_event_(oldest_event);
    }

    return std::get<1>(oldest_event)->CallOldestEvent();
  });

  now_ = end_time;

  MaybeRunStopped();

  return result;
}

Status EventSchedulerScheduler::Run() {
  logging::ScopedLogRestorer prev_logger;
  AOS_RETURN_IF_ERROR(MaybeRunOnStartup());

  // Run all the sub-event-schedulers.
  const Status result = RunMaybeRealtimeLoop([this]() -> Status {
    std::tuple<distributed_clock::time_point, EventScheduler *> oldest_event;
    AOS_ASSIGN_OR_RETURN_ERROR(oldest_event, OldestEvent());
    if (!reboots_.empty() &&
        std::get<0>(reboots_.front()) <= std::get<0>(oldest_event)) {
      // Reboot is next.
      ABSL_CHECK_LE(now_,
                    std::get<0>(reboots_.front()) + std::chrono::nanoseconds(1))
          << ": Simulated time went backwards by too much.  Please "
             "investigate.";
      now_ = std::get<0>(reboots_.front());
      AOS_RETURN_IF_ERROR(Reboot());
      reboots_.erase(reboots_.begin());
      return Ok();
    }
    // No events left, bail.
    if (std::get<0>(oldest_event) == distributed_clock::max_time) {
      is_running_ = false;
      return Ok();
    }

    // We get to pick our tradeoffs here.  Either we assume that there are no
    // backward step changes in our time function for each node, or we have to
    // let time go backwards.  We currently only really see this happen when 2
    // events are scheduled for "now", time changes, and there is a nanosecond
    // or two of rounding due to integer math.
    //
    // //aos/events/logging:logger_test triggers this.
    ABSL_CHECK_LE(now_, std::get<0>(oldest_event) + std::chrono::nanoseconds(1))
        << ": Simulated time went backwards by too much.  Please investigate.";
    now_ = std::get<0>(oldest_event);

    return std::get<1>(oldest_event)->CallOldestEvent();
  });

  MaybeRunStopped();
  return result;
}

template <typename F>
Result<void> EventSchedulerScheduler::RunMaybeRealtimeLoop(F loop_body) {
  Aio::Timer timer(&aio_);
  ABSL_CHECK_LT(0.0, replay_rate_) << "Replay rate must be positive.";
  std::tuple<distributed_clock::time_point, EventScheduler *> oldest_event;
  AOS_ASSIGN_OR_RETURN_ERROR(oldest_event, OldestEvent());
  distributed_clock::time_point last_distributed_clock =
      std::get<0>(oldest_event);
  monotonic_clock::time_point last_monotonic_clock = monotonic_clock::now();

  Result<void> result{};

  struct Context {
    EventSchedulerScheduler *self;
    Aio::Timer *timer;
    distributed_clock::time_point *last_distributed_clock;
    monotonic_clock::time_point *last_monotonic_clock;
    Result<void> *result;
    std::function<Result<void>()> loop_body;

    // Timer completions are Ok() by contract (see Aio::Timer::Schedule()),
    // so CHECK rather than handle.  This timer is not the only thing on
    // aio_: log_web_proxy_main, in_process_plotter, localizer_replay, and
    // glib_main_loop_test register their own fds on the same loop via
    // scheduler_aio().
    static void OnTimer(Completion completion, void *ctx) {
      ABSL_CHECK(completion.status.has_value());
      auto c = static_cast<Context *>(ctx);
      if (!c->self->is_running_) {
        c->self->aio_.Quit();
        return;
      }
      // Call loop_body() at least once; if we are in infinite-speed replay,
      // we don't actually want/need the context switches from the timer, so
      // just loop.
      // This is deliberately written to support the user changing replay
      // rates dynamically.
      do {
        *c->result = c->loop_body();
        if (!c->result->has_value()) {
          c->self->is_running_ = false;
        }
        if (c->self->is_running_) {
          // Sleep until the *next* event, not the one we just handled --
          // otherwise every event fires immediately no matter how far apart
          // they were scheduled.
          const Result<
              std::tuple<distributed_clock::time_point, EventScheduler *>>
              next_oldest_event = c->self->OldestEvent();
          if (!next_oldest_event.has_value()) {
            // Propagate the error.
            *c->result = MakeError(next_oldest_event.error());
            c->self->is_running_ = false;
          } else if (distributed_clock::time_point next_event_time =
                         std::get<0>(next_oldest_event.value());
                     next_event_time != distributed_clock::max_time) {
            // Only arm the timer when something will wait on it.  At
            // infinite rate the do/while below drains every event without
            // returning to the loop, so a timer would never drive progress,
            // and Schedule() is not free.
            //
            // A rate change inside loop_body() still works: this test and
            // the while condition below read replay_rate_ at the same point,
            // so switching off infinity here takes this branch and leaves
            // with a timer armed.
            if (c->self->replay_rate_ !=
                std::numeric_limits<double>::infinity()) {
              // Schedule the next event.
              const monotonic_clock::time_point next_trigger =
                  *c->last_monotonic_clock +
                  std::chrono::duration_cast<std::chrono::nanoseconds>(
                      (next_event_time - *c->last_distributed_clock) /
                      c->self->replay_rate_);
              c->timer->Schedule(next_trigger, &Context::OnTimer, c);
              *c->last_monotonic_clock = next_trigger;
            }
            *c->last_distributed_clock = next_event_time;
          } else {
            // We're out of events.  loop_body deals with that: it sets
            // is_running_ to false and does any other bookkeeping it needs.
            // At an infinite rate the loop below runs it again right away.
            // Otherwise there is no next event to arm the timer for, so arm
            // it for now, and the next callback runs it.
            if (c->self->replay_rate_ !=
                std::numeric_limits<double>::infinity()) {
              c->timer->Schedule(monotonic_clock::now(), &Context::OnTimer, c);
            }
            continue;
          }
        }
      } while (c->self->replay_rate_ ==
                   std::numeric_limits<double>::infinity() &&
               c->self->is_running_);

      if (!c->self->is_running_) {
        c->self->aio_.Quit();
      }
    }
  };

  Context ctx{this,
              &timer,
              &last_distributed_clock,
              &last_monotonic_clock,
              &result,
              std::move(loop_body)};
  timer.Schedule(last_monotonic_clock, &Context::OnTimer, &ctx);

  aio_.Run();
  return result;
}

Result<std::tuple<distributed_clock::time_point, EventScheduler *>>
EventSchedulerScheduler::OldestEvent() {
  distributed_clock::time_point min_event_time = distributed_clock::max_time;
  EventScheduler *min_scheduler = nullptr;

  // TODO(austin): Don't linearly search...  But for N=3, it is probably the
  // fastest way to do this.
  for (EventScheduler *scheduler : schedulers_) {
    std::pair<distributed_clock::time_point, monotonic_clock::time_point>
        event_time;
    AOS_ASSIGN_OR_RETURN_ERROR(event_time, scheduler->OldestEvent());
    if (event_time.second != monotonic_clock::max_time) {
      if (event_time.first < min_event_time) {
        min_event_time = event_time.first;
        min_scheduler = scheduler;
      }
    }
  }

  if (min_scheduler) {
    ABSL_VLOG(2) << "Oldest event " << min_event_time << " on scheduler "
                 << min_scheduler->node_index_;
  }
  return std::make_tuple(min_event_time, min_scheduler);
}

Result<void> EventSchedulerScheduler::TemporarilyStopAndRun(
    std::function<void()> fn) {
  if (in_on_run_) {
    ABSL_LOG(FATAL)
        << "Can't call AllowApplicationCreationDuring from an OnRun callback.";
  }
  const bool was_running = is_running_;
  if (is_running_) {
    is_running_ = false;
    MaybeRunStopped();
  }
  fn();
  if (was_running) {
    AOS_RETURN_IF_ERROR(MaybeRunOnStartup());
  }
  return Ok();
}

Result<void> EventSchedulerScheduler::MaybeRunOnStartup() {
  is_running_ = true;
  for (EventScheduler *scheduler : schedulers_) {
    AOS_RETURN_IF_ERROR(scheduler->MaybeRunOnStartup());
  }
  in_on_run_ = true;
  // We must trigger all the OnRun's *after* all the OnStartup callbacks are
  // triggered because that is the contract that we have stated.
  for (EventScheduler *scheduler : schedulers_) {
    scheduler->MaybeRunOnRun();
  }
  in_on_run_ = false;
  return Ok();
}

}  // namespace aos
