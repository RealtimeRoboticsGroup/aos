#ifndef AOS_EVENTS_EVENT_LOOP_C_H_
#define AOS_EVENTS_EVENT_LOOP_C_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

// Forward declarations of C-style structs for corresponding C++ classes.
typedef struct event_loop_t event_loop_t;
typedef struct fetcher_t fetcher_t;
typedef struct sender_t sender_t;
typedef struct timer_handler_t timer_handler_t;
typedef struct exit_handle_t exit_handle_t;
typedef struct context_t context_t;
typedef struct simulated_event_loop_factory_t simulated_event_loop_factory_t;

// Callback types for various EventLoop APIs.
typedef void (*watcher_callback_t)(const context_t *context,
                                   const void *message, void *user_data);
typedef void (*timer_callback_t)(void *user_data);
typedef void (*on_run_callback_t)(void *user_data);

// Wrapper for aos::EventLoop. Member function pointers mimic the aos::EventLoop
// API. Not all members are guaranteed to be set; check for NULL before using
// them. All of these functions can fail and cause your program to crash.
struct event_loop_t {
  void *impl;
  // Creates a fetcher on the specified channel and returns it.
  fetcher_t *(*make_fetcher)(event_loop_t *self, const char *channel_name,
                             const char *channel_type);
  // Creates a sender on the specified channel and returns it.
  sender_t *(*make_sender)(event_loop_t *self, const char *channel_name,
                           const char *channel_type);
  // Creates a watcher on the specified channel with the provided callback.
  void (*make_watcher)(event_loop_t *self, const char *channel_name,
                       const char *channel_type, watcher_callback_t callback,
                       void *user_data);
  // Creates a timer with the provided callback, and returns a timer handler.
  // Use it to schedule the timer.
  timer_handler_t *(*add_timer)(event_loop_t *self, timer_callback_t callback,
                                void *user_data);
  // Returns the current time on the monotonic clock, as nanoseconds since
  // epoch.
  int64_t (*monotonic_now)(event_loop_t *self);
  // Registers the provided callback to be invoked when the event loop is first
  // run.
  void (*on_run)(event_loop_t *self, on_run_callback_t callback,
                 void *user_data);
  // Returns true if the event loop is running.
  bool (*is_running)(event_loop_t *self);
  // Runs the event loop. This blocks until interrupted by a signal or ^C. This
  // is only available on some kinds of event loops.
  void (*run)(event_loop_t *self);
  // Provides a handle that can be used to stop running the event loop.
  exit_handle_t *(*make_exit_handle)(event_loop_t *self);
};

// Wrapper for aos::RawFetcher.
struct fetcher_t {
  void *impl;
  // Fetches the latest message on the channel. Returns true if a new message
  // was fetched.
  bool (*fetch)(fetcher_t *self);
  // Fetches the next message on the channel. Returns true if a new message was
  // fetched.
  bool (*fetch_next)(fetcher_t *self);
  // Returns the context for the current message.
  context_t (*context)(fetcher_t *self);
};

// Wrapper for aos::RawSender.
struct sender_t {
  void *impl;
  // Makes a copy of the provided data and sends it. Returns true on success.
  // TODO(Sanjay): Is bool sufficient?
  // TODO(Sanjay): Replace this with a zero-copy send after we have a
  // flatbuffers builder in Python that can work with a custom allocator.
  bool (*send)(sender_t *self, const void *data, size_t size);
};

// Wrapper for aos::TimerHandler.
struct timer_handler_t {
  void *impl;
  // Schedules the timer to expire at `start_monotonic_ns` and every `period_ns`
  // thereafter. If `period_ns` is 0, the timer only expires once. Every time
  // the timer expires, it invokes the registered callback. `start_monotonic_ns`
  // is nanoseconds since epoch on the monotonic clock, and `period_ns` is
  // nanoseconds.
  //
  // To schedule at the current time, use `monotonic_now` in `event_loop_t`.
  void (*schedule)(timer_handler_t *self, int64_t start_monotonic_ns,
                   int64_t period_ns);
  // Cancels the timer, if scheduled.
  void (*disable)(timer_handler_t *self);
};

// Wrapper for aos::ExitHandle.
struct exit_handle_t {
  void *impl;
  void (*exit)(exit_handle_t *self);
};

// Wrapper for aos::Context. Fields correspond one-to-one with the same names.
// See aos/events/context.h for detailed documentation.
struct context_t {
  int64_t monotonic_event_time;
  int64_t realtime_event_time;
  uint32_t queue_index;
  uint32_t remote_queue_index;
  size_t size;
  const void *data;
};

// TODO(Sanjay): How does this interact with absl-py?
void init(int *argc, char ***argv);
uint8_t *read_configuration_from_file(const char *file_path);
void destroy_configuration(uint8_t *configuration_buffer);

// Factory functions for event loops. Users are responsible for destroying event
// loops, as well any fetchers, senders, and timer handlers they create using
// the event loops. The create* functions can fail and cause your program to
// crash.
event_loop_t *create_shm_event_loop(const uint8_t *configuration_buffer);
void destroy_event_loop(event_loop_t *event_loop);
void destroy_fetcher(fetcher_t *fetcher);
void destroy_sender(sender_t *sender);
void destroy_timer_handler(timer_handler_t *timer_handler);
void destroy_exit_handle(exit_handle_t *exit_handle);

// Wrapper for aos::SimulatedEventLoopFactory.
struct simulated_event_loop_factory_t {
  void *impl;
  event_loop_t *(*make_event_loop)(simulated_event_loop_factory_t *self,
                                   const char *name, const char *node);
  void (*run_for)(simulated_event_loop_factory_t *self, int64_t duration_ns);
};

// Factory functions for the simulated event loop factory. Any event loops
// created usng the factory are the user's responsibility to destroy.
simulated_event_loop_factory_t *create_simulated_event_loop_factory(
    const uint8_t *configuration_buffer);
void destroy_simulated_event_loop_factory(
    simulated_event_loop_factory_t *factory);

#ifdef __cplusplus
}  // extern "C"
#endif  // __cplusplus

#endif  // AOS_EVENTS_EVENT_LOOP_C_H_
