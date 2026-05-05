#ifndef AOS_EVENTS_EVENT_LOOP_C_H_
#define AOS_EVENTS_EVENT_LOOP_C_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

// Forward declarations of C-style names for corresponding C++ classes.
//
// Note that these are all treated as opaque types in the C API.

// Wrapper for aos::EventLoop. See the aos_event_loop_* functions for the member
// functions.
typedef struct aos_event_loop_t aos_event_loop_t;

// Wrapper for aos::RawFetcher. See the aos_fetcher_* functions for the member
// functions.
typedef struct aos_fetcher_t aos_fetcher_t;

// Wrapper for aos::RawSender. See the aos_sender_* functions for the member
// functions.
typedef struct aos_sender_t aos_sender_t;

// Wrapper for aos::TimerHandler. See the aos_timer_handler_* functions for the
// member functions.
typedef struct aos_timer_handler_t aos_timer_handler_t;

// Wrapper for aos::ExitHandle. See the aos_exit_handle_* functions for the
// member functions.
typedef struct aos_exit_handle_t aos_exit_handle_t;
typedef struct aos_context_t aos_context_t;
typedef struct aos_simulated_event_loop_factory_t
    aos_simulated_event_loop_factory_t;
// Wrapper for aos::ErrorType. See aos/util/status.h for detailed documentation.
typedef struct aos_error_t aos_error_t;

// Wrapper for aos::Context. Fields correspond one-to-one with the same names.
// See aos/events/context.h for detailed documentation.
typedef struct aos_context_t {
  int64_t monotonic_event_time;
  int64_t realtime_event_time;
  uint32_t queue_index;
  uint32_t remote_queue_index;
  size_t size;
  const void *data;
} aos_context_t;

// Wrapper for aos::SimulatedEventLoopFactory. See the
// aos_simulated_event_loop_factory_* functions for the member functions.
typedef struct aos_simulated_event_loop_factory_t
    aos_simulated_event_loop_factory_t;

// Wrapper for aos::ErrorType. See aos/util/status.h for detailed documentation.
typedef struct aos_error_t aos_error_t;

// Callback types for various EventLoop APIs.
typedef void (*aos_watcher_callback_t)(const aos_context_t *context,
                                       const void *message, void *user_data);
typedef void (*aos_timer_callback_t)(void *user_data);
typedef void (*aos_on_run_callback_t)(void *user_data);

// These functions will always return the same value each time they are
// executed, although this return value may change in future versions.
// aos_const_raw_sender_error_* are aos::RawSender::Error values.
// aos_const_error_status_code_* are aos::ErrorType::StatusCode vales.
// aos_const_scheduling_policy_* are aos::SchedulingPolicy values.
// aos_const_realtime_policy_* are aos::RealtimePolicy values.
int aos_const_raw_sender_error_ok(void);
int aos_const_raw_sender_error_messages_sent_too_fast(void);
int aos_const_raw_sender_error_invalid_redzone(void);
int aos_const_error_status_code_ok(void);
int aos_const_error_status_code_error(void);
int aos_const_error_status_code_python_exception(void);
int aos_const_scheduling_policy_other(void);
int aos_const_scheduling_policy_fifo(void);
int aos_const_scheduling_policy_rr(void);
int aos_const_realtime_policy_realtime_mode_deny_malloc(void);
int aos_const_realtime_policy_no_mode(void);

void aos_event_loop_destroy(aos_event_loop_t *self);
// Creates a fetcher on the specified channel and returns it.
aos_fetcher_t *aos_event_loop_make_fetcher(aos_event_loop_t *self,
                                           const char *channel_name,
                                           const char *channel_type);
// Creates a sender on the specified channel and returns it.
aos_sender_t *aos_event_loop_make_sender(aos_event_loop_t *self,
                                         const char *channel_name,
                                         const char *channel_type);
// Creates a watcher on the specified channel with the provided callback.
void aos_event_loop_make_watcher(aos_event_loop_t *self,
                                 const char *channel_name,
                                 const char *channel_type,
                                 aos_watcher_callback_t callback,
                                 void *user_data);
// Creates a timer with the provided callback, and returns a timer handler.
// Use it to schedule the timer.
aos_timer_handler_t *aos_event_loop_add_timer(aos_event_loop_t *self,
                                              aos_timer_callback_t callback,
                                              void *user_data);
// Returns the current time on the monotonic clock, as nanoseconds since
// epoch.
int64_t aos_event_loop_monotonic_now(aos_event_loop_t *self);
// Registers the provided callback to be invoked when the event loop is first
// run.
void aos_event_loop_on_run(aos_event_loop_t *self,
                           aos_on_run_callback_t callback, void *user_data);
// Returns true if the event loop is running.
bool aos_event_loop_is_running(aos_event_loop_t *self);
// Sets the realtime priority to run the event loop at.
// scheduling_policy must be one of the aos_const_scheduling_policy_* values.
// realtime_policy must be one of the aos_const_realtime_policy_* values.
void aos_event_loop_set_runtime_realtime_priority(aos_event_loop_t *self,
                                                  int priority,
                                                  int scheduling_policy,
                                                  int realtime_policy);
// "*name_data" is invalidated when the next "aos_event_loop_*" function is
// called on "self".
void aos_event_loop_get_name(aos_event_loop_t *self, const char **name_data,
                             size_t *name_size);

// Runs the event loop. This blocks until interrupted by a signal or ^C. This
// is only available on some kinds of event loops.
// Returns nullptr to indicate Ok().
aos_error_t *aos_shm_event_loop_run(aos_event_loop_t *self);
// Provides a handle that can be used to stop running the event loop.
aos_exit_handle_t *aos_shm_event_loop_make_exit_handle(aos_event_loop_t *self);
// Copies name_size bytes from name_data (no NUL terminator required).
void aos_shm_event_loop_set_name(aos_event_loop_t *self, const char *name_data,
                                 size_t name_size);

void aos_fetcher_destroy(aos_fetcher_t *self);
// Fetches the latest message on the channel. Returns true if a new message
// was fetched.
bool aos_fetcher_fetch(aos_fetcher_t *self);
// Fetches the next message on the channel. Returns true if a new message was
// fetched.
bool aos_fetcher_fetch_next(aos_fetcher_t *self);
// Returns the context for the current message.
aos_context_t aos_fetcher_context(aos_fetcher_t *self);

void aos_sender_destroy(aos_sender_t *self);
size_t aos_sender_size(aos_sender_t *self);
// A buffer of aos_sender_size(self) bytes for the caller to fill in. This value
// and its contents may change after any of the aos_sender_send* functions are
// called.
void *aos_sender_data(aos_sender_t *self);
// Makes a copy of the provided data and sends it. Returns one of the
// aos_const_raw_sender_error_* values.
int aos_sender_send_copy(aos_sender_t *self, const void *data, size_t size);
// Sends the data at the buffer returned by aos_sender_data(self). Returns one
// of the aos_const_raw_sender_error_* values.
int aos_sender_send(aos_sender_t *self, size_t size);

// Schedules the timer to expire at `start_monotonic_ns` and every `period_ns`
// thereafter. If `period_ns` is 0, the timer only expires once. Every time
// the timer expires, it invokes the registered callback. `start_monotonic_ns`
// is nanoseconds since epoch on the monotonic clock, and `period_ns` is
// nanoseconds.
//
// To schedule at the current time, use `monotonic_now` in `aos_event_loop_t`.
void aos_timer_handler_schedule(aos_timer_handler_t *self, int64_t base_ns,
                                int64_t repeat_offset_ns);
// Cancels the timer, if scheduled.
void aos_timer_handler_disable(aos_timer_handler_t *self);
// Copies name_size bytes from name_data (no NUL terminator required).
void aos_timer_set_name(aos_timer_handler_t *self, const char *name_data,
                        size_t name_size);
// "*name_data" is invalidated when the next "aos_timer_*" function is called on
// "self".
void aos_timer_get_name(aos_timer_handler_t *self, const char **name_data,
                        size_t *name_size);

void aos_exit_handle_destroy(aos_exit_handle_t *self);
void aos_exit_handle_exit(aos_exit_handle_t *self);
void aos_exit_handle_exit_with_python_exception(aos_exit_handle_t *self);

// TODO(Sanjay): How does this interact with absl-py?
void aos_init(int *argc, char ***argv);
uint8_t *aos_configuration_read_from_file(const char *file_path);
void aos_configuration_destroy(uint8_t *self);

aos_event_loop_t *aos_shm_event_loop_create(
    const uint8_t *configuration_buffer);

aos_simulated_event_loop_factory_t *aos_simulated_event_loop_factory_create(
    const uint8_t *configuration_buffer);
void aos_simulated_event_loop_factory_destroy(
    aos_simulated_event_loop_factory_t *self);
aos_event_loop_t *aos_simulated_event_loop_factory_make_event_loop(
    aos_simulated_event_loop_factory_t *self, const char *name,
    const char *node);
void aos_simulated_event_loop_factory_run_for(
    aos_simulated_event_loop_factory_t *self, const int64_t duration_ns);
aos_error_t *aos_simulated_event_loop_factory_non_fatal_run_for(
    aos_simulated_event_loop_factory_t *self, const int64_t duration_ns);
aos_exit_handle_t *aos_simulated_event_loop_factory_make_exit_handle(
    aos_simulated_event_loop_factory_t *self);

void aos_error_destroy(aos_error_t *self);
int aos_error_code(aos_error_t *self);

#ifdef __cplusplus
}  // extern "C"
#endif  // __cplusplus

#endif  // AOS_EVENTS_EVENT_LOOP_C_H_
