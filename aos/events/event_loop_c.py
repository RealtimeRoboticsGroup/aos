import cffi
import os
from pathlib import Path

from python.runfiles import runfiles

# TODO(Brian): Switch to using out-of-line API mode for speed and robustness.
# That should also allow #including the header file instead of duplicating much of its contents.
# Also switch to extern "Python" instead of ffi.callback: https://cffi.readthedocs.io/en/stable/using.html#extern-python-new-style-callbacks

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

ffi.cdef("""
typedef struct aos_event_loop_t aos_event_loop_t;

typedef struct aos_fetcher_t aos_fetcher_t;

typedef struct aos_sender_t aos_sender_t;

typedef struct aos_timer_handler_t aos_timer_handler_t;

typedef struct aos_exit_handle_t aos_exit_handle_t;

typedef struct aos_context_t {
  int64_t monotonic_event_time;
  int64_t realtime_event_time;
  int64_t monotonic_remote_time;
  int64_t realtime_remote_time;
  int64_t monotonic_remote_transmit_time;
  uint32_t queue_index;
  uint32_t remote_queue_index;
  size_t size;
  const void *data;
  int buffer_index;
  uint8_t source_boot_uuid[16];
} aos_context_t;

typedef struct aos_simulated_event_loop_factory_t
    aos_simulated_event_loop_factory_t;

typedef struct aos_error_t aos_error_t;
typedef struct aos_channel_t aos_channel_t;
typedef struct aos_node_t aos_node_t;
typedef struct aos_configuration_buffer_t aos_configuration_buffer_t;

typedef struct aos_configuration_t aos_configuration_t;

typedef void (*aos_watcher_callback_t)(const aos_context_t *context,
                                       const void *message, void *user_data);
typedef void (*aos_timer_callback_t)(void *user_data);
typedef void (*aos_on_run_callback_t)(void *user_data);
typedef void (*aos_shm_event_loop_fd_callback_t)(void *user_data,
                                                 uint32_t events);

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
aos_fetcher_t *aos_event_loop_make_fetcher(aos_event_loop_t *self,
                                           const char *channel_name,
                                           const char *channel_type);
aos_sender_t *aos_event_loop_make_sender(aos_event_loop_t *self,
                                         const char *channel_name,
                                         const char *channel_type);
void aos_event_loop_make_watcher(aos_event_loop_t *self, const char *channel_name,
                                 const char *channel_type,
                                 aos_watcher_callback_t callback,
                                 void *user_data);
aos_timer_handler_t *aos_event_loop_add_timer(aos_event_loop_t *self,
                                              aos_timer_callback_t callback,
                                              void *user_data);
int64_t aos_event_loop_monotonic_now(aos_event_loop_t *self);
int64_t aos_event_loop_realtime_now(aos_event_loop_t *self);
void aos_event_loop_on_run(aos_event_loop_t *self, aos_on_run_callback_t callback,
                           void *user_data);
bool aos_event_loop_is_running(aos_event_loop_t *self);
void aos_event_loop_set_runtime_realtime_priority(aos_event_loop_t *self,
                                                  int priority,
                                                  int scheduling_policy,
                                                  int realtime_policy);
void aos_event_loop_set_runtime_affinity(aos_event_loop_t *self,
                                         const int32_t *affinity_list,
                                         size_t affinity_list_size);
void aos_event_loop_get_name(aos_event_loop_t *self, const char **name_data,
                             size_t *name_size);
const aos_node_t *aos_event_loop_node(aos_event_loop_t *self);
const aos_configuration_t *aos_event_loop_configuration(aos_event_loop_t *self);

aos_error_t *aos_shm_event_loop_run(aos_event_loop_t *self);
aos_exit_handle_t *aos_shm_event_loop_make_exit_handle(aos_event_loop_t *self);
void aos_shm_event_loop_set_name(aos_event_loop_t *self, const char *name_data,
                                 size_t name_size);
void aos_shm_event_loop_lock_to_thread(aos_event_loop_t *self);
void aos_shm_event_loop_on_fd_events(aos_event_loop_t *self, int fd,
                                     aos_shm_event_loop_fd_callback_t callback,
                                     void *user_data);
void aos_shm_event_loop_delete_fd(aos_event_loop_t *self, int fd);
void aos_shm_event_loop_set_fd_events(aos_event_loop_t *self, int fd,
                                      uint32_t events);
aos_event_loop_t *aos_shm_event_loop_create(
    const aos_configuration_t *configuration);

void aos_fetcher_destroy(aos_fetcher_t *self);
bool aos_fetcher_fetch(aos_fetcher_t *self);
bool aos_fetcher_fetch_next(aos_fetcher_t *self);
aos_context_t aos_fetcher_context(aos_fetcher_t *self);

void aos_sender_destroy(aos_sender_t *self);
size_t aos_sender_size(aos_sender_t *self);
void *aos_sender_data(aos_sender_t *self);
int aos_sender_send_copy(aos_sender_t *self, const void *data, size_t size);
int aos_sender_send(aos_sender_t *self, size_t size);

void aos_timer_handler_schedule(aos_timer_handler_t *self, int64_t base_ns,
                                int64_t repeat_offset_ns);
void aos_timer_handler_disable(aos_timer_handler_t *self);
void aos_timer_set_name(aos_timer_handler_t *self, const char *name_data,
                        size_t name_size);
void aos_timer_get_name(aos_timer_handler_t *self, const char **name_data,
                        size_t *name_size);

void aos_exit_handle_destroy(aos_exit_handle_t *self);
void aos_exit_handle_exit(aos_exit_handle_t *self);
void aos_exit_handle_exit_with_python_exception(aos_exit_handle_t *self);

void aos_init(int *argc, char ***argv);

const void *aos_configuration_buffer_get_data(
    const aos_configuration_buffer_t *self);
size_t aos_configuration_buffer_get_size(
    const aos_configuration_buffer_t *self);
aos_configuration_buffer_t *aos_configuration_buffer_read_from_file(
    const char *file_path);
void aos_configuration_buffer_destroy(aos_configuration_buffer_t *self);

const aos_channel_t *aos_configuration_get_channel(
    const aos_configuration_t *config, const char *name_data, size_t name_size,
    const char *type_data, size_t type_size, const char *application_name_data,
    size_t application_name_size, const aos_node_t *node);
const aos_node_t *aos_configuration_get_node(const aos_configuration_t *config,
                                             const char *name_data,
                                             size_t name_size);
size_t aos_configuration_get_nodes(const aos_configuration_t *config,
                                   const aos_node_t **result,
                                   size_t result_size);
bool aos_configuration_multi_node(const aos_configuration_t *config);
bool aos_configuration_channel_is_sendable_on_node(const aos_channel_t *channel,
                                                   const aos_node_t *node);
bool aos_configuration_channel_is_readable_on_node(const aos_channel_t *channel,
                                                   const aos_node_t *node);

aos_simulated_event_loop_factory_t *aos_simulated_event_loop_factory_create(
    const aos_configuration_t *configuration);
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
""")

import platform
if platform.system() == "Darwin":
    lib_path = locate("aos/aos/events/libevent_loop_c.dylib")
else:
    lib_path = locate("aos/aos/events/libevent_loop_c.so")
lib = ffi.dlopen(str(lib_path))
