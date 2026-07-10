#ifndef AOS_INIT_FOR_RUST_H_
#define AOS_INIT_FOR_RUST_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct aos_flag_info_t {
  char *name;
  char *type;
  char *description;
  char *default_value;
  char *filename;
} aos_flag_info_t;

void aos_init_from_rust();
size_t aos_get_cpp_flags(aos_flag_info_t **flags_out);
void aos_free_cpp_flags(aos_flag_info_t *flags, size_t size);
bool aos_set_command_line_option(const char *name, const char *value);
char *aos_get_command_line_option(const char *name);
void aos_free_command_line_option(char *value);

#ifdef __cplusplus
}
#endif

#endif  // AOS_INIT_FOR_RUST_H_
