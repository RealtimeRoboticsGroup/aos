#include "aos/events/event_loop_c.h"

#include <string.h>

#include <chrono>
#include <memory>

#include "absl/log/check.h"
#include "absl/log/die_if_null.h"
#include "absl/log/log.h"
#include "flatbuffers/buffer.h"

#include "aos/configuration.h"
#include "aos/configuration_generated.h"
#include "aos/events/context.h"
#include "aos/events/event_loop.h"
#include "aos/events/shm_event_loop.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"

namespace {

aos_context_t to_context_t(const aos::Context &context) {
  aos_context_t c_context;
  c_context.monotonic_event_time = static_cast<int64_t>(
      context.monotonic_event_time.time_since_epoch().count());
  c_context.realtime_event_time = static_cast<int64_t>(
      context.realtime_event_time.time_since_epoch().count());
  c_context.queue_index = context.queue_index;
  c_context.remote_queue_index = context.remote_queue_index;
  c_context.size = context.size;
  c_context.data = context.data;
  return c_context;
}

aos_error_t *to_error_t(aos::Status status) {
  if (aos::IsOk(status)) {
    return nullptr;
  }
  return reinterpret_cast<aos_error_t *>(new aos::Status(std::move(status)));
}

}  // namespace

int aos_const_raw_sender_error_ok(void) {
  return static_cast<int>(aos::RawSender::Error::kOk);
}
int aos_const_raw_sender_error_messages_sent_too_fast(void) {
  return static_cast<int>(aos::RawSender::Error::kMessagesSentTooFast);
}
int aos_const_raw_sender_error_invalid_redzone(void) {
  return static_cast<int>(aos::RawSender::Error::kInvalidRedzone);
}
int aos_const_error_status_code_ok(void) {
  return static_cast<int>(aos::ErrorType::StatusCode::kOk);
}
int aos_const_error_status_code_error(void) {
  return static_cast<int>(aos::ErrorType::StatusCode::kError);
}
int aos_const_error_status_code_python_exception(void) {
  return static_cast<int>(aos::ErrorType::StatusCode::kPythonException);
}
int aos_const_scheduling_policy_other(void) {
  return static_cast<int>(aos::SchedulingPolicy::SCHEDULER_OTHER);
}
int aos_const_scheduling_policy_fifo(void) {
  return static_cast<int>(aos::SchedulingPolicy::SCHEDULER_FIFO);
}
int aos_const_scheduling_policy_rr(void) {
  return static_cast<int>(aos::SchedulingPolicy::SCHEDULER_RR);
}
int aos_const_realtime_policy_realtime_mode_deny_malloc(void) {
  return static_cast<int>(aos::RealtimePolicy::REALTIME_MODE_DENY_MALLOC);
}
int aos_const_realtime_policy_no_mode(void) {
  return static_cast<int>(aos::RealtimePolicy::NO_MODE);
}

void aos_fetcher_destroy(aos_fetcher_t *self) {
  delete reinterpret_cast<aos::RawFetcher *>(self);
}

bool aos_fetcher_fetch(aos_fetcher_t *self) {
  aos::RawFetcher *fetcher =
      reinterpret_cast<aos::RawFetcher *>(ABSL_DIE_IF_NULL(self));
  return fetcher->Fetch();
}

bool aos_fetcher_fetch_next(aos_fetcher_t *self) {
  aos::RawFetcher *fetcher =
      reinterpret_cast<aos::RawFetcher *>(ABSL_DIE_IF_NULL(self));
  return fetcher->FetchNext();
}

aos_context_t aos_fetcher_context(aos_fetcher_t *self) {
  aos::RawFetcher *fetcher =
      reinterpret_cast<aos::RawFetcher *>(ABSL_DIE_IF_NULL(self));
  return to_context_t(fetcher->context());
}

void aos_sender_destroy(aos_sender_t *self) {
  delete reinterpret_cast<aos::RawSender *>(self);
}

size_t aos_sender_size(aos_sender_t *self) {
  aos::RawSender *sender =
      reinterpret_cast<aos::RawSender *>(ABSL_DIE_IF_NULL(self));
  return sender->size();
}

void *aos_sender_data(aos_sender_t *self) {
  aos::RawSender *sender =
      reinterpret_cast<aos::RawSender *>(ABSL_DIE_IF_NULL(self));
  return sender->data();
}

int aos_sender_send_copy(aos_sender_t *self, const void *data, size_t size) {
  aos::RawSender *sender =
      reinterpret_cast<aos::RawSender *>(ABSL_DIE_IF_NULL(self));
  const aos::RawSender::Error status = sender->Send(data, size);
  return static_cast<int>(status);
}

int aos_sender_send(aos_sender_t *self, size_t size) {
  aos::RawSender *sender =
      reinterpret_cast<aos::RawSender *>(ABSL_DIE_IF_NULL(self));
  const aos::RawSender::Error status = sender->Send(size);
  return static_cast<int>(status);
}

void aos_timer_handler_schedule(aos_timer_handler_t *self, int64_t base_ns,
                                int64_t repeat_offset_ns) {
  aos::TimerHandler *timer_handler =
      reinterpret_cast<aos::TimerHandler *>(ABSL_DIE_IF_NULL(self));
  timer_handler->Schedule(
      aos::monotonic_clock::epoch() + std::chrono::nanoseconds(base_ns),
      std::chrono::nanoseconds(repeat_offset_ns));
}

void aos_timer_handler_disable(aos_timer_handler_t *self) {
  aos::TimerHandler *timer_handler =
      reinterpret_cast<aos::TimerHandler *>(ABSL_DIE_IF_NULL(self));
  timer_handler->Disable();
}

void aos_timer_set_name(aos_timer_handler_t *self, const char *name_data,
                        size_t name_size) {
  aos::TimerHandler *timer_handler =
      reinterpret_cast<aos::TimerHandler *>(ABSL_DIE_IF_NULL(self));
  timer_handler->set_name(std::string_view(name_data, name_size));
}

void aos_timer_get_name(aos_timer_handler_t *self, const char **name_data,
                        size_t *name_size) {
  aos::TimerHandler *timer_handler =
      reinterpret_cast<aos::TimerHandler *>(ABSL_DIE_IF_NULL(self));
  const std::string_view name = timer_handler->name();
  *name_data = name.data();
  *name_size = name.size();
}

void aos_exit_handle_destroy(aos_exit_handle_t *self) {
  delete reinterpret_cast<aos::ExitHandle *>(self);
}

void aos_exit_handle_exit(aos_exit_handle_t *self) {
  aos::ExitHandle *exit_handle =
      reinterpret_cast<aos::ExitHandle *>(ABSL_DIE_IF_NULL(self));
  exit_handle->Exit();
}

void aos_exit_handle_exit_with_python_exception(aos_exit_handle_t *self) {
  aos::ExitHandle *exit_handle =
      reinterpret_cast<aos::ExitHandle *>(ABSL_DIE_IF_NULL(self));
  exit_handle->Exit(aos::MakeError(aos::ErrorType::PythonException()));
}

aos_fetcher_t *aos_event_loop_make_fetcher(aos_event_loop_t *self,
                                           const char *channel_name,
                                           const char *channel_type) {
  aos::EventLoop *event_loop =
      reinterpret_cast<aos::EventLoop *>(ABSL_DIE_IF_NULL(self));
  CHECK(event_loop != nullptr);
  const aos::Channel *channel = aos::configuration::GetChannel(
      event_loop->configuration(), channel_name, channel_type,
      event_loop->name(), event_loop->node(), true);
  CHECK(channel != nullptr)
      << ": Can't find channel " << channel_name << " " << channel_type;
  if (!aos::configuration::ChannelIsReadableOnNode(channel,
                                                   event_loop->node())) {
    LOG(FATAL) << ": Channel " << channel_name << " " << channel_type
               << " isn't readable on node " << event_loop->node();
  }
  std::unique_ptr<aos::RawFetcher> fetcher =
      event_loop->MakeRawFetcher(channel);
  return reinterpret_cast<aos_fetcher_t *>(fetcher.release());
}

void aos_event_loop_destroy(aos_event_loop_t *self) {
  delete reinterpret_cast<aos::EventLoop *>(self);
}

aos_sender_t *aos_event_loop_make_sender(aos_event_loop_t *self,
                                         const char *channel_name,
                                         const char *channel_type) {
  aos::EventLoop *event_loop =
      reinterpret_cast<aos::EventLoop *>(ABSL_DIE_IF_NULL(self));
  CHECK(event_loop != nullptr);
  const aos::Channel *channel = aos::configuration::GetChannel(
      event_loop->configuration(), channel_name, channel_type,
      event_loop->name(), event_loop->node(), true);
  CHECK(channel != nullptr)
      << ": Can't find channel " << channel_name << " " << channel_type;
  if (!aos::configuration::ChannelIsSendableOnNode(channel,
                                                   event_loop->node())) {
    LOG(FATAL) << ": Channel " << channel_name << " " << channel_type
               << " isn't sendable on node " << event_loop->node();
  }
  std::unique_ptr<aos::RawSender> sender = event_loop->MakeRawSender(channel);
  return reinterpret_cast<aos_sender_t *>(sender.release());
}

void aos_event_loop_make_watcher(aos_event_loop_t *self,
                                 const char *channel_name,
                                 const char *channel_type,
                                 aos_watcher_callback_t callback,
                                 void *user_data) {
  aos::EventLoop *event_loop =
      reinterpret_cast<aos::EventLoop *>(ABSL_DIE_IF_NULL(self));
  CHECK(event_loop != nullptr);
  const aos::Channel *channel = aos::configuration::GetChannel(
      event_loop->configuration(), channel_name, channel_type,
      event_loop->name(), event_loop->node(), true);
  CHECK(channel != nullptr)
      << ": Can't find channel " << channel_name << " " << channel_type;
  if (!aos::configuration::ChannelIsReadableOnNode(channel,
                                                   event_loop->node())) {
    LOG(FATAL) << ": Channel " << channel_name << " " << channel_type
               << " isn't readable on node " << event_loop->node();
  }
  event_loop->MakeRawWatcher(
      channel,
      [callback, user_data](const aos::Context &context, const void *message) {
        aos_context_t c_context = to_context_t(context);
        callback(&c_context, message, user_data);
      });
}

aos_timer_handler_t *aos_event_loop_add_timer(aos_event_loop_t *self,
                                              aos_timer_callback_t callback,
                                              void *user_data) {
  aos::EventLoop *event_loop =
      reinterpret_cast<aos::EventLoop *>(ABSL_DIE_IF_NULL(self));
  aos::TimerHandler *timer_handler =
      event_loop->AddTimer([callback, user_data]() { callback(user_data); });
  return reinterpret_cast<aos_timer_handler_t *>(timer_handler);
}

int64_t aos_event_loop_monotonic_now(aos_event_loop_t *self) {
  aos::EventLoop *event_loop =
      reinterpret_cast<aos::EventLoop *>(ABSL_DIE_IF_NULL(self));
  return static_cast<int64_t>(
      event_loop->monotonic_now().time_since_epoch().count());
}

void aos_event_loop_on_run(aos_event_loop_t *self,
                           aos_on_run_callback_t callback, void *user_data) {
  aos::EventLoop *event_loop =
      reinterpret_cast<aos::EventLoop *>(ABSL_DIE_IF_NULL(self));
  event_loop->OnRun([callback, user_data]() { callback(user_data); });
}

bool aos_event_loop_is_running(aos_event_loop_t *self) {
  aos::EventLoop *event_loop =
      reinterpret_cast<aos::EventLoop *>(ABSL_DIE_IF_NULL(self));
  return event_loop->is_running();
}

void aos_event_loop_set_runtime_realtime_priority(aos_event_loop_t *self,
                                                  int priority,
                                                  int scheduling_policy,
                                                  int realtime_policy) {
  aos::EventLoop *event_loop =
      reinterpret_cast<aos::EventLoop *>(ABSL_DIE_IF_NULL(self));
  event_loop->SetRuntimeRealtimePriority(
      priority, static_cast<aos::SchedulingPolicy>(scheduling_policy),
      static_cast<aos::RealtimePolicy>(realtime_policy));
}

void aos_event_loop_get_name(aos_event_loop_t *self, const char **name_data,
                             size_t *name_size) {
  aos::EventLoop *event_loop =
      reinterpret_cast<aos::EventLoop *>(ABSL_DIE_IF_NULL(self));
  const std::string_view name = event_loop->name();
  *name_data = name.data();
  *name_size = name.size();
}

aos_error_t *aos_shm_event_loop_run(aos_event_loop_t *self) {
  aos::ShmEventLoop *event_loop = static_cast<aos::ShmEventLoop *>(
      reinterpret_cast<aos::EventLoop *>(ABSL_DIE_IF_NULL(self)));
  return to_error_t(event_loop->Run());
}

aos_exit_handle_t *aos_shm_event_loop_make_exit_handle(aos_event_loop_t *self) {
  aos::ShmEventLoop *event_loop = static_cast<aos::ShmEventLoop *>(
      reinterpret_cast<aos::EventLoop *>(ABSL_DIE_IF_NULL(self)));
  std::unique_ptr<aos::ExitHandle> exit_handle = event_loop->MakeExitHandle();
  return reinterpret_cast<aos_exit_handle_t *>(exit_handle.release());
}

void aos_shm_event_loop_set_name(aos_event_loop_t *self, const char *name_data,
                                 size_t name_size) {
  aos::ShmEventLoop *event_loop = static_cast<aos::ShmEventLoop *>(
      reinterpret_cast<aos::EventLoop *>(ABSL_DIE_IF_NULL(self)));
  event_loop->set_name(std::string_view(name_data, name_size));
}

void aos_init(int *argc, char ***argv) { aos::InitGoogle(argc, argv); }

uint8_t *aos_configuration_read_from_file(const char *file_path) {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(file_path);
  uint8_t *dst = (uint8_t *)malloc(config.span().size() * sizeof(uint8_t));
  CHECK(dst != nullptr);
  memcpy(dst, config.span().data(), config.span().size());
  return dst;
}

void aos_configuration_destroy(uint8_t *self) { free((void *)self); }

aos_event_loop_t *aos_shm_event_loop_create(
    const uint8_t *configuration_buffer) {
  auto event_loop = std::make_unique<aos::ShmEventLoop>(
      flatbuffers::GetRoot<aos::Configuration>(
          ABSL_DIE_IF_NULL(configuration_buffer)));
  return reinterpret_cast<aos_event_loop_t *>(
      static_cast<aos::EventLoop *>(event_loop.release()));
}

void aos_simulated_event_loop_factory_destroy(
    aos_simulated_event_loop_factory_t *self) {
  delete reinterpret_cast<aos::SimulatedEventLoopFactory *>(self);
}

aos_event_loop_t *aos_simulated_event_loop_factory_make_event_loop(
    aos_simulated_event_loop_factory_t *self, const char *name,
    const char *node) {
  aos::SimulatedEventLoopFactory *factory =
      reinterpret_cast<aos::SimulatedEventLoopFactory *>(
          ABSL_DIE_IF_NULL(self));
  std::unique_ptr<aos::EventLoop> event_loop =
      factory->GetNodeEventLoopFactory(node)->MakeEventLoop(name);
  return reinterpret_cast<aos_event_loop_t *>(event_loop.release());
}

void aos_simulated_event_loop_factory_run_for(
    aos_simulated_event_loop_factory_t *self, const int64_t duration_ns) {
  aos::SimulatedEventLoopFactory *factory =
      reinterpret_cast<aos::SimulatedEventLoopFactory *>(
          ABSL_DIE_IF_NULL(self));
  factory->RunFor(std::chrono::nanoseconds(duration_ns));
}

aos_error_t *aos_simulated_event_loop_factory_non_fatal_run_for(
    aos_simulated_event_loop_factory_t *self, const int64_t duration_ns) {
  aos::SimulatedEventLoopFactory *factory =
      reinterpret_cast<aos::SimulatedEventLoopFactory *>(
          ABSL_DIE_IF_NULL(self));
  return to_error_t(
      factory->NonFatalRunFor(std::chrono::nanoseconds(duration_ns)));
}

aos_exit_handle_t *aos_simulated_event_loop_factory_make_exit_handle(
    aos_simulated_event_loop_factory_t *self) {
  aos::SimulatedEventLoopFactory *factory =
      reinterpret_cast<aos::SimulatedEventLoopFactory *>(
          ABSL_DIE_IF_NULL(self));
  std::unique_ptr<aos::ExitHandle> exit_handle = factory->MakeExitHandle();
  return reinterpret_cast<aos_exit_handle_t *>(exit_handle.release());
}

aos_simulated_event_loop_factory_t *aos_simulated_event_loop_factory_create(
    const uint8_t *configuration_buffer) {
  auto factory = std::make_unique<aos::SimulatedEventLoopFactory>(
      flatbuffers::GetRoot<aos::Configuration>(
          ABSL_DIE_IF_NULL(configuration_buffer)));
  return reinterpret_cast<aos_simulated_event_loop_factory_t *>(
      factory.release());
}

void aos_error_destroy(aos_error_t *self) {
  auto error = reinterpret_cast<aos::Status *>(self);
  delete error;
}

int aos_error_code(aos_error_t *self) {
  auto error = reinterpret_cast<aos::Status *>(ABSL_DIE_IF_NULL(self));
  return aos::ResultExitCode(*error);
}
