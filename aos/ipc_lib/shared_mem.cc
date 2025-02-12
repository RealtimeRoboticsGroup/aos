#include "aos/ipc_lib/shared_mem.h"

#include <fcntl.h>
#include <stdint.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cassert>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ostream>

#include "absl/log/check.h"
#include "absl/log/log.h"

#include "aos/ipc_lib/aos_sync.h"

// the path for the shared memory segment. see shm_open(3) for restrictions
#define AOS_SHM_NAME "/aos_shared_mem"
// Size of the shared mem segment.
// This must fit in the tmpfs for /dev/shm/
#define SIZEOFSHMSEG (4096 * 0x800)

void init_shared_mem_core(aos_shm_core *shm_core) {
  memset(&shm_core->time_offset, 0, sizeof(shm_core->time_offset));
  memset(&shm_core->msg_alloc_lock, 0, sizeof(shm_core->msg_alloc_lock));
  shm_core->queues.pointer = NULL;
  memset(&shm_core->queues.lock, 0, sizeof(shm_core->queues.lock));
  shm_core->queue_types.pointer = NULL;
  memset(&shm_core->queue_types.lock, 0, sizeof(shm_core->queue_types.lock));
}

struct aos_core *global_core = NULL;

void aos_core_use_address_as_shared_mem(void *address, size_t size) {
  global_core->mem_struct = reinterpret_cast<aos_shm_core_t *>(address);
  global_core->size = size;
  global_core->shared_mem =
      (uint8_t *)address + sizeof(*global_core->mem_struct);
  if (global_core->owner) {
    global_core->mem_struct->msg_alloc = (uint8_t *)address + global_core->size;
    init_shared_mem_core(global_core->mem_struct);
    futex_set(&global_core->mem_struct->creation_condition);
  } else {
    if (futex_wait(&global_core->mem_struct->creation_condition) != 0) {
      LOG(FATAL) << "waiting on creation_condition failed";
    }
  }
}
