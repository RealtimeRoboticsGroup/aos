#define _GNU_SOURCE
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

__attribute__((constructor)) static void init(void) {
  char buf[256];
  ssize_t len = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
  if (len != -1) {
    buf[len] = '\0';
    if (strstr(buf, "python") != NULL) {
      unsetenv("LD_PRELOAD");
    }
  }
}
