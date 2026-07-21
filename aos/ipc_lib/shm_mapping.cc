#include "aos/ipc_lib/shm_mapping.h"

#include <atomic>

namespace aos::ipc_lib {

void PageFaultDataWrite(char *data, size_t size, const long page_size) {
  const size_t pages = (size + page_size - 1) / page_size;
  for (size_t i = 0; i < pages; ++i) {
    // We need to ensure there's a writable pagetable entry, but must not modify
    // the data.  Even if you lock the data into memory, some kernels still
    // lazily create the pagetable entries, so we have to perform an actual
    // *write* access to fault in a writable PTE.
    //
    // Getting a real write to *this* page on every architecture is trickier
    // than it looks:
    //   - "compare_exchange(0, 0)" only stores when the byte is already 0.  On
    //     load-linked/store-conditional architectures (e.g. ARM64) the compiler
    //     branches around the store-conditional when the byte is nonzero, so
    //     pages holding existing data get a read-only PTE and a later realtime
    //     write page-faults -- violating this function's guarantee.  (On x86
    //     the CAS is a locked op the MMU treats as a write regardless, which is
    //     why that bug hides there.)
    //   - An identity read-modify-write like fetch_or(0)/fetch_add(0) is worse:
    //     on x86 the compiler folds it to a bare memory barrier (or a dummy
    //     locked op on the stack) that never touches this page at all.
    //
    // So compare-exchange the byte against its own current value.  The store
    // runs (faulting the page writable) when the byte is unchanged; if a
    // concurrent opener/initializer changed it we retry with the new value, so
    // we never clobber their write and every page still ends up write-faulted.
    std::atomic_ref<char> ref(data[i * page_size]);
    char value = ref.load(std::memory_order_relaxed);
    while (!ref.compare_exchange_weak(value, value, std::memory_order_release,
                                      std::memory_order_relaxed)) {
    }
  }
}

void PageFaultDataRead(const char *data, size_t size, const long page_size) {
  const size_t pages = (size + page_size - 1) / page_size;
  for (size_t i = 0; i < pages; ++i) {
    // We just need to ensure there's a readable pagetable entry, so touch one
    // byte per page.
    static_cast<void>(
        std::atomic_ref<char>(const_cast<char &>(data[i * page_size]))
            .load(std::memory_order_relaxed));
  }
}

}  // namespace aos::ipc_lib
