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

void aos_fetcher_destroy(aos_fetcher_t *self) {
  delete static_cast<aos::RawFetcher *>(ABSL_DIE_IF_NULL(self)->impl);
  free(self);
}

bool aos_fetcher_fetch(aos_fetcher_t *self) {
  aos::RawFetcher *fetcher =
      static_cast<aos::RawFetcher *>(ABSL_DIE_IF_NULL(self)->impl);
  return ABSL_DIE_IF_NULL(fetcher)->Fetch();
}

bool aos_fetcher_fetch_next(aos_fetcher_t *self) {
  aos::RawFetcher *fetcher =
      static_cast<aos::RawFetcher *>(ABSL_DIE_IF_NULL(self)->impl);
  return ABSL_DIE_IF_NULL(fetcher)->FetchNext();
}

aos_context_t aos_fetcher_context(aos_fetcher_t *self) {
  aos::RawFetcher *fetcher =
      static_cast<aos::RawFetcher *>(ABSL_DIE_IF_NULL(self)->impl);
  return to_context_t(ABSL_DIE_IF_NULL(fetcher)->context());
}

void aos_sender_destroy(aos_sender_t *self) {
  delete static_cast<aos::RawSender *>(ABSL_DIE_IF_NULL(self)->impl);
  free(self);
}

size_t aos_sender_size(aos_sender_t *self) {
  aos::RawSender *sender =
      static_cast<aos::RawSender *>(ABSL_DIE_IF_NULL(self)->impl);
  return sender->size();
}

void *aos_sender_data(aos_sender_t *self) {
  aos::RawSender *sender =
      static_cast<aos::RawSender *>(ABSL_DIE_IF_NULL(self)->impl);
  return sender->data();
}

int aos_sender_send_copy(aos_sender_t *self, const void *data, size_t size) {
  aos::RawSender *sender =
      static_cast<aos::RawSender *>(ABSL_DIE_IF_NULL(self)->impl);
  const aos::RawSender::Error status = sender->Send(data, size);
  return static_cast<int>(status);
}

int aos_sender_send(aos_sender_t *self, size_t size) {
  aos::RawSender *sender =
      static_cast<aos::RawSender *>(ABSL_DIE_IF_NULL(self)->impl);
  const aos::RawSender::Error status = sender->Send(size);
  return static_cast<int>(status);
}

void destroy_timer_handler(timer_handler_t *timer_handler) {
  free(ABSL_DIE_IF_NULL(timer_handler));
}

void aos_timer_handler_schedule(aos_timer_handler_t *self, int64_t base_ns,
                                int64_t repeat_offset_ns) {
  aos::TimerHandler *timer_handler =
      static_cast<aos::TimerHandler *>(ABSL_DIE_IF_NULL(self)->impl);
  ABSL_DIE_IF_NULL(timer_handler)
      ->Schedule(
          aos::monotonic_clock::epoch() + std::chrono::nanoseconds(base_ns),
          std::chrono::nanoseconds(repeat_offset_ns));
}

void aos_timer_handler_disable(aos_timer_handler_t *self) {
  aos::TimerHandler *timer_handler =
      static_cast<aos::TimerHandler *>(ABSL_DIE_IF_NULL(self)->impl);
  ABSL_DIE_IF_NULL(timer_handler)->Disable();
}

void aos_exit_handle_destroy(aos_exit_handle_t *self) {
  delete static_cast<aos::ExitHandle *>(ABSL_DIE_IF_NULL(self)->impl);
  free(self);
}

void aos_exit_handle_exit(aos_exit_handle_t *self) {
  aos::ExitHandle *exit_handle =
      static_cast<aos::ExitHandle *>(ABSL_DIE_IF_NULL(self)->impl);
  ABSL_DIE_IF_NULL(exit_handle)->Exit();
}

aos_fetcher_t *aos_event_loop_make_fetcher(aos_event_loop_t *self,
                                           const char *channel_name,
                                           const char *channel_type) {
  aos::EventLoop *event_loop =
      static_cast<aos::EventLoop *>(ABSL_DIE_IF_NULL(self)->impl);
  CHECK(event_loop != nullptr);
  const aos::Channel *channel = aos::configuration::GetChannel(
      event_loop->configuration(), channel_name, channel_type,
      event_loop->name(), event_loop->node(), true);
  CHECK(channel != nullptr)
      << ": Can't find channel " << channel_name << " " << channel_type;
  if (!aos::configuration::ChannelIsReadableOnNode(channel,
                                                   event_loop->node())) {
    LOG(FATAL) << ": Channel " << channel_name << " " << channel_type
               << " isn't sendable on node " << event_loop->node();
  }
  std::unique_ptr<aos::RawFetcher> fetcher =
      event_loop->MakeRawFetcher(channel);
  aos_fetcher_t *c_fetcher = (aos_fetcher_t *)malloc(sizeof(aos_fetcher_t));
  CHECK(c_fetcher != nullptr);
  c_fetcher->impl = fetcher.release();
  c_fetcher->fetch = &aos_fetcher_fetch;
  c_fetcher->fetch_next = &aos_fetcher_fetch_next;
  c_fetcher->context = &aos_fetcher_context;
  return c_fetcher;
}

void aos_event_loop_destroy(aos_event_loop_t *self) {
  delete static_cast<aos::EventLoop *>(ABSL_DIE_IF_NULL(self)->impl);
  free(self);
}

static bool aos_sender_send_copy_bool(aos_sender_t *self, const void *data,
                                      size_t size) {
  return aos_sender_send_copy(self, data, size) ==
         static_cast<int>(aos::RawSender::Error::kOk);
}

aos_sender_t *aos_event_loop_make_sender(aos_event_loop_t *self,
                                         const char *channel_name,
                                         const char *channel_type) {
  aos::EventLoop *event_loop =
      static_cast<aos::EventLoop *>(ABSL_DIE_IF_NULL(self)->impl);
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
  aos_sender_t *c_sender = (aos_sender_t *)malloc(sizeof(aos_sender_t));
  CHECK(c_sender != nullptr);
  c_sender->impl = sender.release();
  c_sender->send = &aos_sender_send_copy_bool;
  return c_sender;
}

void aos_event_loop_make_watcher(aos_event_loop_t *self,
                                 const char *channel_name,
                                 const char *channel_type,
                                 aos_watcher_callback_t callback,
                                 void *user_data) {
  aos::EventLoop *event_loop =
      static_cast<aos::EventLoop *>(ABSL_DIE_IF_NULL(self)->impl);
  CHECK(event_loop != nullptr);
  const aos::Channel *channel = aos::configuration::GetChannel(
      event_loop->configuration(), channel_name, channel_type,
      event_loop->name(), event_loop->node(), true);
  CHECK(channel != nullptr)
      << ": Can't find channel " << channel_name << " " << channel_type;
  if (!aos::configuration::ChannelIsReadableOnNode(channel,
                                                   event_loop->node())) {
    LOG(FATAL) << ": Channel " << channel_name << " " << channel_type
               << " isn't sendable on node " << event_loop->node();
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
      static_cast<aos::EventLoop *>(ABSL_DIE_IF_NULL(self)->impl);
  aos::TimerHandler *timer_handler =
      ABSL_DIE_IF_NULL(event_loop)->AddTimer([callback, user_data]() {
        callback(user_data);
      });
  aos_timer_handler_t *c_timer_handler =
      (aos_timer_handler_t *)malloc(sizeof(aos_timer_handler_t));
  CHECK(c_timer_handler != nullptr);
  c_timer_handler->impl = timer_handler;
  c_timer_handler->schedule = &aos_timer_handler_schedule;
  c_timer_handler->disable = &aos_timer_handler_disable;
  return c_timer_handler;
}

int64_t aos_event_loop_monotonic_now(aos_event_loop_t *self) {
  aos::EventLoop *event_loop =
      static_cast<aos::EventLoop *>(ABSL_DIE_IF_NULL(self)->impl);
  return static_cast<int64_t>(
      ABSL_DIE_IF_NULL(event_loop)->monotonic_now().time_since_epoch().count());
}

void aos_event_loop_on_run(aos_event_loop_t *self,
                           aos_on_run_callback_t callback, void *user_data) {
  aos::EventLoop *event_loop =
      static_cast<aos::EventLoop *>(ABSL_DIE_IF_NULL(self)->impl);
  ABSL_DIE_IF_NULL(event_loop)->OnRun([callback, user_data]() {
    callback(user_data);
  });
}

bool aos_event_loop_is_running(aos_event_loop_t *self) {
  aos::EventLoop *event_loop =
      static_cast<aos::EventLoop *>(ABSL_DIE_IF_NULL(self)->impl);
  return ABSL_DIE_IF_NULL(event_loop)->is_running();
}

aos_error_t *aos_shm_event_loop_run(aos_event_loop_t *self) {
  aos::ShmEventLoop *event_loop =
      static_cast<aos::ShmEventLoop *>(ABSL_DIE_IF_NULL(self)->impl);
  return to_error_t(ABSL_DIE_IF_NULL(event_loop)->Run());
}

static void aos_shm_event_loop_run_void(aos_event_loop_t *self) {
  aos_error_t *error = aos_shm_event_loop_run(self);
  if (error) {
    aos_error_destroy(error);
  }
}

aos_exit_handle_t *aos_shm_event_loop_make_exit_handle(aos_event_loop_t *self) {
  aos::ShmEventLoop *event_loop =
      static_cast<aos::ShmEventLoop *>(ABSL_DIE_IF_NULL(self)->impl);
  std::unique_ptr<aos::ExitHandle> exit_handle =
      ABSL_DIE_IF_NULL(event_loop)->MakeExitHandle();
  aos_exit_handle_t *c_exit_handle =
      (aos_exit_handle_t *)malloc(sizeof(aos_exit_handle_t));
  CHECK(c_exit_handle != nullptr);
  c_exit_handle->impl = exit_handle.release();
  c_exit_handle->exit = &aos_exit_handle_exit;
  return c_exit_handle;
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
  aos_event_loop_t *c_event_loop =
      (aos_event_loop_t *)malloc(sizeof(aos_event_loop_t));
  CHECK(c_event_loop != nullptr);
  c_event_loop->impl = event_loop.release();
  c_event_loop->make_fetcher = &aos_event_loop_make_fetcher;
  c_event_loop->make_sender = &aos_event_loop_make_sender;
  c_event_loop->make_watcher = &aos_event_loop_make_watcher;
  c_event_loop->add_timer = &aos_event_loop_add_timer;
  c_event_loop->monotonic_now = &aos_event_loop_monotonic_now;
  c_event_loop->on_run = &aos_event_loop_on_run;
  c_event_loop->is_running = &aos_event_loop_is_running;
  c_event_loop->run = &aos_shm_event_loop_run_void;
  c_event_loop->make_exit_handle = &aos_shm_event_loop_make_exit_handle;
  return c_event_loop;
}

void aos_simulated_event_loop_factory_destroy(
    aos_simulated_event_loop_factory_t *self) {
  delete static_cast<aos::SimulatedEventLoopFactory *>(
      ABSL_DIE_IF_NULL(self)->impl);
  free(self);
}

aos_event_loop_t *aos_simulated_event_loop_factory_make_event_loop(
    aos_simulated_event_loop_factory_t *self, const char *name,
    const char *node) {
  aos::SimulatedEventLoopFactory *factory =
      static_cast<aos::SimulatedEventLoopFactory *>(
          ABSL_DIE_IF_NULL(self)->impl);
  std::unique_ptr<aos::EventLoop> event_loop =
      ABSL_DIE_IF_NULL(factory)->GetNodeEventLoopFactory(node)->MakeEventLoop(
          name);
  aos_event_loop_t *c_event_loop =
      (aos_event_loop_t *)malloc(sizeof(aos_event_loop_t));
  c_event_loop->impl = event_loop.release();
  c_event_loop->make_fetcher = &aos_event_loop_make_fetcher;
  c_event_loop->make_sender = &aos_event_loop_make_sender;
  c_event_loop->make_watcher = &aos_event_loop_make_watcher;
  c_event_loop->add_timer = &aos_event_loop_add_timer;
  c_event_loop->monotonic_now = &aos_event_loop_monotonic_now;
  c_event_loop->on_run = &aos_event_loop_on_run;
  c_event_loop->is_running = &aos_event_loop_is_running;
  c_event_loop->run = nullptr;
  c_event_loop->make_exit_handle = nullptr;
  return c_event_loop;
}

void aos_simulated_event_loop_factory_run_for(
    aos_simulated_event_loop_factory_t *self, const int64_t duration_ns) {
  aos::SimulatedEventLoopFactory *factory =
      static_cast<aos::SimulatedEventLoopFactory *>(
          ABSL_DIE_IF_NULL(self)->impl);
  ABSL_DIE_IF_NULL(factory)->RunFor(std::chrono::nanoseconds(duration_ns));
}

aos_simulated_event_loop_factory_t *aos_simulated_event_loop_factory_create(
    const uint8_t *configuration_buffer) {
  auto factory = std::make_unique<aos::SimulatedEventLoopFactory>(
      flatbuffers::GetRoot<aos::Configuration>(
          ABSL_DIE_IF_NULL(configuration_buffer)));
  aos_simulated_event_loop_factory_t *c_factory =
      (aos_simulated_event_loop_factory_t *)malloc(
          sizeof(aos_simulated_event_loop_factory_t));
  CHECK(c_factory != nullptr);
  c_factory->impl = factory.release();
  c_factory->make_event_loop =
      &aos_simulated_event_loop_factory_make_event_loop;
  c_factory->run_for = &aos_simulated_event_loop_factory_run_for;
  return c_factory;
}

void aos_error_destroy(aos_error_t *self) {
  auto error = reinterpret_cast<aos::Status *>(self);
  delete error;
}

int aos_error_code(aos_error_t *self) {
  auto error = reinterpret_cast<aos::Status *>(ABSL_DIE_IF_NULL(self));
  return aos::ResultExitCode(*error);
}
