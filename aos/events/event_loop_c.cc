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

static context_t to_context_t(const aos::Context &context) {
  context_t c_context;
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

static bool fetcher_fetch(fetcher_t *self) {
  aos::RawFetcher *fetcher =
      static_cast<aos::RawFetcher *>(ABSL_DIE_IF_NULL(self)->impl);
  return ABSL_DIE_IF_NULL(fetcher)->Fetch();
}

static bool fetcher_fetch_next(fetcher_t *self) {
  aos::RawFetcher *fetcher =
      static_cast<aos::RawFetcher *>(ABSL_DIE_IF_NULL(self)->impl);
  return ABSL_DIE_IF_NULL(fetcher)->FetchNext();
}

static context_t fetcher_context(fetcher_t *self) {
  aos::RawFetcher *fetcher =
      static_cast<aos::RawFetcher *>(ABSL_DIE_IF_NULL(self)->impl);
  return to_context_t(ABSL_DIE_IF_NULL(fetcher)->context());
}

static bool sender_send(sender_t *self, const void *data, size_t size) {
  aos::RawSender *sender =
      static_cast<aos::RawSender *>(ABSL_DIE_IF_NULL(self)->impl);
  const aos::RawSender::Error status =
      ABSL_DIE_IF_NULL(sender)->Send(data, size);
  return status == aos::RawSender::Error::kOk;
}

static void timer_handler_schedule(timer_handler_t *self, int64_t base_ns,
                                   int64_t repeat_offset_ns) {
  aos::TimerHandler *timer_handler =
      static_cast<aos::TimerHandler *>(ABSL_DIE_IF_NULL(self)->impl);
  ABSL_DIE_IF_NULL(timer_handler)
      ->Schedule(
          aos::monotonic_clock::epoch() + std::chrono::nanoseconds(base_ns),
          std::chrono::nanoseconds(repeat_offset_ns));
}

static void timer_handler_disable(timer_handler_t *self) {
  aos::TimerHandler *timer_handler =
      static_cast<aos::TimerHandler *>(ABSL_DIE_IF_NULL(self)->impl);
  ABSL_DIE_IF_NULL(timer_handler)->Disable();
}

static void exit_handle_exit(exit_handle_t *self) {
  aos::ExitHandle *exit_handle =
      static_cast<aos::ExitHandle *>(ABSL_DIE_IF_NULL(self)->impl);
  ABSL_DIE_IF_NULL(exit_handle)->Exit();
}

static fetcher_t *event_loop_make_fetcher(event_loop_t *self,
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
  fetcher_t *c_fetcher = (fetcher_t *)malloc(sizeof(fetcher_t));
  CHECK(c_fetcher != nullptr);
  c_fetcher->impl = fetcher.release();
  c_fetcher->fetch = &fetcher_fetch;
  c_fetcher->fetch_next = &fetcher_fetch_next;
  c_fetcher->context = &fetcher_context;
  return c_fetcher;
}

static sender_t *event_loop_make_sender(event_loop_t *self,
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
  sender_t *c_sender = (sender_t *)malloc(sizeof(sender_t));
  CHECK(c_sender != nullptr);
  c_sender->impl = sender.release();
  c_sender->send = &sender_send;
  return c_sender;
}

static void event_loop_make_watcher(event_loop_t *self,
                                    const char *channel_name,
                                    const char *channel_type,
                                    watcher_callback_t callback,
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
        context_t c_context = to_context_t(context);
        callback(&c_context, message, user_data);
      });
}

static timer_handler_t *event_loop_add_timer(event_loop_t *self,
                                             timer_callback_t callback,
                                             void *user_data) {
  aos::EventLoop *event_loop =
      static_cast<aos::EventLoop *>(ABSL_DIE_IF_NULL(self)->impl);
  aos::TimerHandler *timer_handler =
      ABSL_DIE_IF_NULL(event_loop)->AddTimer([callback, user_data]() {
        callback(user_data);
      });
  timer_handler_t *c_timer_handler =
      (timer_handler_t *)malloc(sizeof(timer_handler_t));
  CHECK(c_timer_handler != nullptr);
  c_timer_handler->impl = timer_handler;
  c_timer_handler->schedule = &timer_handler_schedule;
  c_timer_handler->disable = &timer_handler_disable;
  return c_timer_handler;
}

static int64_t event_loop_monotonic_now(event_loop_t *self) {
  aos::EventLoop *event_loop =
      static_cast<aos::EventLoop *>(ABSL_DIE_IF_NULL(self)->impl);
  return static_cast<int64_t>(
      ABSL_DIE_IF_NULL(event_loop)->monotonic_now().time_since_epoch().count());
}

static void event_loop_on_run(event_loop_t *self, on_run_callback_t callback,
                              void *user_data) {
  aos::EventLoop *event_loop =
      static_cast<aos::EventLoop *>(ABSL_DIE_IF_NULL(self)->impl);
  ABSL_DIE_IF_NULL(event_loop)->OnRun([callback, user_data]() {
    callback(user_data);
  });
}

static bool event_loop_is_running(event_loop_t *self) {
  aos::EventLoop *event_loop =
      static_cast<aos::EventLoop *>(ABSL_DIE_IF_NULL(self)->impl);
  return ABSL_DIE_IF_NULL(event_loop)->is_running();
}

static void shm_event_loop_run(event_loop_t *self) {
  aos::ShmEventLoop *event_loop =
      static_cast<aos::ShmEventLoop *>(ABSL_DIE_IF_NULL(self)->impl);
  ABSL_DIE_IF_NULL(event_loop)->Run();
}

static exit_handle_t *shm_event_loop_make_exit_handle(event_loop_t *self) {
  aos::ShmEventLoop *event_loop =
      static_cast<aos::ShmEventLoop *>(ABSL_DIE_IF_NULL(self)->impl);
  std::unique_ptr<aos::ExitHandle> exit_handle =
      ABSL_DIE_IF_NULL(event_loop)->MakeExitHandle();
  exit_handle_t *c_exit_handle = (exit_handle_t *)malloc(sizeof(exit_handle_t));
  CHECK(c_exit_handle != nullptr);
  c_exit_handle->impl = exit_handle.release();
  c_exit_handle->exit = &exit_handle_exit;
  return c_exit_handle;
}

void init(int *argc, char ***argv) { aos::InitGoogle(argc, argv); }

uint8_t *read_configuration_from_file(const char *file_path) {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(file_path);
  uint8_t *dst = (uint8_t *)malloc(config.span().size() * sizeof(uint8_t));
  CHECK(dst != nullptr);
  memcpy(dst, config.span().data(), config.span().size());
  return dst;
}

void destroy_configuration(uint8_t *configuration_buffer) {
  free((void *)configuration_buffer);
}

event_loop_t *create_shm_event_loop(const uint8_t *configuration_buffer) {
  auto event_loop = std::make_unique<aos::ShmEventLoop>(
      flatbuffers::GetRoot<aos::Configuration>(
          ABSL_DIE_IF_NULL(configuration_buffer)));
  event_loop_t *c_event_loop = (event_loop_t *)malloc(sizeof(event_loop_t));
  CHECK(c_event_loop != nullptr);
  c_event_loop->impl = event_loop.release();
  c_event_loop->make_fetcher = &event_loop_make_fetcher;
  c_event_loop->make_sender = &event_loop_make_sender;
  c_event_loop->make_watcher = &event_loop_make_watcher;
  c_event_loop->add_timer = &event_loop_add_timer;
  c_event_loop->monotonic_now = &event_loop_monotonic_now;
  c_event_loop->on_run = &event_loop_on_run;
  c_event_loop->is_running = &event_loop_is_running;
  c_event_loop->run = &shm_event_loop_run;
  c_event_loop->make_exit_handle = &shm_event_loop_make_exit_handle;
  return c_event_loop;
}

static event_loop_t *simulated_event_loop_factory_make_event_loop(
    simulated_event_loop_factory_t *self, const char *name, const char *node) {
  aos::SimulatedEventLoopFactory *factory =
      static_cast<aos::SimulatedEventLoopFactory *>(
          ABSL_DIE_IF_NULL(self)->impl);
  std::unique_ptr<aos::EventLoop> event_loop =
      ABSL_DIE_IF_NULL(factory)->GetNodeEventLoopFactory(node)->MakeEventLoop(
          name);
  event_loop_t *c_event_loop = (event_loop_t *)malloc(sizeof(event_loop_t));
  c_event_loop->impl = event_loop.release();
  c_event_loop->make_fetcher = &event_loop_make_fetcher;
  c_event_loop->make_sender = &event_loop_make_sender;
  c_event_loop->make_watcher = &event_loop_make_watcher;
  c_event_loop->add_timer = &event_loop_add_timer;
  c_event_loop->monotonic_now = &event_loop_monotonic_now;
  c_event_loop->on_run = &event_loop_on_run;
  c_event_loop->is_running = &event_loop_is_running;
  c_event_loop->run = nullptr;
  c_event_loop->make_exit_handle = nullptr;
  return c_event_loop;
}

static void simulated_event_loop_factory_run_for(
    simulated_event_loop_factory_t *self, const int64_t duration_ns) {
  aos::SimulatedEventLoopFactory *factory =
      static_cast<aos::SimulatedEventLoopFactory *>(
          ABSL_DIE_IF_NULL(self)->impl);
  ABSL_DIE_IF_NULL(factory)->RunFor(std::chrono::nanoseconds(duration_ns));
}

simulated_event_loop_factory_t *create_simulated_event_loop_factory(
    const uint8_t *configuration_buffer) {
  auto factory = std::make_unique<aos::SimulatedEventLoopFactory>(
      flatbuffers::GetRoot<aos::Configuration>(
          ABSL_DIE_IF_NULL(configuration_buffer)));
  simulated_event_loop_factory_t *c_factory =
      (simulated_event_loop_factory_t *)malloc(
          sizeof(simulated_event_loop_factory_t));
  CHECK(c_factory != nullptr);
  c_factory->impl = factory.release();
  c_factory->make_event_loop = &simulated_event_loop_factory_make_event_loop;
  c_factory->run_for = &simulated_event_loop_factory_run_for;
  return c_factory;
}

void destroy_simulated_event_loop_factory(
    simulated_event_loop_factory_t *factory) {
  delete static_cast<aos::SimulatedEventLoopFactory *>(
      ABSL_DIE_IF_NULL(factory)->impl);
  free(factory);
}

void destroy_event_loop(event_loop_t *event_loop) {
  delete static_cast<aos::EventLoop *>(ABSL_DIE_IF_NULL(event_loop)->impl);
  free(event_loop);
}

void destroy_fetcher(fetcher_t *fetcher) {
  delete static_cast<aos::RawFetcher *>(ABSL_DIE_IF_NULL(fetcher)->impl);
  free(fetcher);
}

void destroy_sender(sender_t *sender) {
  delete static_cast<aos::RawSender *>(ABSL_DIE_IF_NULL(sender)->impl);
  free(sender);
}

void destroy_timer_handler(timer_handler_t *timer_handler) {
  free(ABSL_DIE_IF_NULL(timer_handler));
}

void destroy_exit_handle(exit_handle_t *exit_handle) {
  delete static_cast<aos::ExitHandle *>(ABSL_DIE_IF_NULL(exit_handle)->impl);
  free(exit_handle);
}
