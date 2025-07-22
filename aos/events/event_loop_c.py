import cffi
import os
from pathlib import Path

from python.runfiles import runfiles

_RUNFILES = runfiles.Create()


def locate(relative_path):
    relative_path = Path(relative_path)
    if relative_path.is_absolute():
        raise ValueError(
            f"Expected relative path that starts with a repository, got absolute path: {relative_path}"
        )
    absolute_path = _RUNFILES.Rlocation(str(relative_path))
    if absolute_path is None or not absolute_path.startswith("/"):
        raise FileNotFoundError(
            f"Failed to locate {relative_path}, got {absolute_path}")
    if not os.path.exists(absolute_path):
        raise FileNotFoundError(
            f"Resolved path to {absolute_path}, but file does not exist")
    return Path(absolute_path)


ffi = cffi.FFI()

ffi.cdef("""typedef struct event_loop_t event_loop_t;
typedef struct fetcher_t fetcher_t;
typedef struct sender_t sender_t;
typedef struct timer_handler_t timer_handler_t;
typedef struct exit_handle_t exit_handle_t;
typedef struct context_t context_t;
typedef struct simulated_event_loop_factory_t simulated_event_loop_factory_t;

typedef void (*watcher_callback_t)(const context_t *context,
                                   const void *message, void *user_data);
typedef void (*timer_callback_t)(void *user_data);
typedef void (*on_run_callback_t)(void *user_data);

struct event_loop_t {
  void *impl;
  fetcher_t *(*make_fetcher)(event_loop_t *self, const char *channel_name,
                             const char *channel_type);
  sender_t *(*make_sender)(event_loop_t *self, const char *channel_name,
                           const char *channel_type);
  void (*make_watcher)(event_loop_t *self, const char *channel_name,
                       const char *channel_type, watcher_callback_t callback,
                       void *user_data);
  timer_handler_t *(*add_timer)(event_loop_t *self, timer_callback_t callback,
                                void *user_data);
  int64_t (*monotonic_now)(event_loop_t *self);
  void (*on_run)(event_loop_t *self, on_run_callback_t callback,
                 void *user_data);
  bool (*is_running)(event_loop_t *self);
  void (*run)(event_loop_t *self);
  exit_handle_t *(*make_exit_handle)(event_loop_t *self);
};

struct fetcher_t {
  void *impl;
  bool (*fetch)(fetcher_t *self);
  bool (*fetch_next)(fetcher_t *self);
  context_t (*context)(fetcher_t *self);
};

struct sender_t {
  void *impl;
  bool (*send)(sender_t *self, const void *data, size_t size);
};

struct timer_handler_t {
  void *impl;
  void (*schedule)(timer_handler_t *self, int64_t start_monotonic_ns,
                   int64_t period_ns);
  void (*disable)(timer_handler_t *self);
};

struct exit_handle_t {
  void *impl;
  void (*exit)(exit_handle_t *self);
};

struct context_t {
  int64_t monotonic_event_time;
  int64_t realtime_event_time;
  uint32_t queue_index;
  uint32_t remote_queue_index;
  size_t size;
  const void *data;
};

void init(int *argc, char ***argv);
uint8_t *read_configuration_from_file(const char *file_path);
void destroy_configuration(uint8_t *configuration_buffer);

event_loop_t *create_shm_event_loop(const uint8_t *configuration_buffer);
void destroy_event_loop(event_loop_t *event_loop);
void destroy_fetcher(fetcher_t *fetcher);
void destroy_sender(sender_t *sender);
void destroy_timer_handler(timer_handler_t *timer_handler);
void destroy_exit_handle(exit_handle_t *exit_handle);

struct simulated_event_loop_factory_t {
  void *impl;
  event_loop_t *(*make_event_loop)(simulated_event_loop_factory_t *self,
                                   const char *name, const char *node);
  void (*run_for)(simulated_event_loop_factory_t *self, int64_t duration_ns);
};

simulated_event_loop_factory_t *create_simulated_event_loop_factory(
    const uint8_t *configuration_buffer);
void destroy_simulated_event_loop_factory(
    simulated_event_loop_factory_t *factory);
""")

lib_path = locate("aos/aos/events/libevent_loop_c.so")
lib = ffi.dlopen(str(lib_path))
