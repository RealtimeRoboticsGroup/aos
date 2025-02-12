#ifndef _SHARED_MEM_H_
#define _SHARED_MEM_H_

#include <stddef.h>

#include "aos/ipc_lib/shared_mem_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void init_shared_mem_core(aos_shm_core *shm_core);

// Takes the specified memory address and uses it as the shared memory.
// address is the memory address, and size is the size of the memory.
// global_core needs to point to an instance of struct aos_core, and owner
// should be set correctly there.
// The owner should verify that the first sizeof(mutex) of data is set to 0
// before passing the memory to this function.
void aos_core_use_address_as_shared_mem(void *address, size_t size);

#ifdef __cplusplus
}
#endif

#endif
