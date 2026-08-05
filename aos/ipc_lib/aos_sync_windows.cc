#if !AOS_SYNC_DEBUG
#undef NDEBUG
#define NDEBUG
#endif

#include "aos/ipc_lib/aos_sync.h"

#ifndef NOMINMAX
#define NOMINMAX
#endif
// clang-format off
// Windows needs to come first.
#include <windows.h>
#include <psapi.h>
// clang-format on

#include <stdio.h>

#include <atomic>
#include <cerrno>
#include <cstdint>

#include "absl/container/flat_hash_map.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/ipc_lib/aos_sync_internal.h"
#include "aos/macros.h"
// Windows is the only platform where aos_sync references //aos:realtime: the
// aos::ScopedNotRealtime use below is Windows-only.  There is no dependency
// cycle here (unlike OSX, where //aos:realtime depends on aos_sync), so we can
// depend on aos/realtime.h directly rather than forward-declaring MarkRealtime.
#include "aos/realtime.h"

// Windows has no cross-process futex: WaitOnAddress and WakeByAddress* are
// scoped strictly to the calling process.  Rather than emulating one badly
// (any userspace scheme layered on counting semaphores can lose or misdeliver
// wakeups -- a waiter for a *later* value change can consume the token meant
// for an earlier waiter, since semaphore tokens are fungible), we split the
// primitives by what they actually need:
//
//  * futex_* and condition_* use WaitOnAddress/WakeByAddress and are
//    PROCESS-LOCAL ONLY on Windows.  Placing one in memory another process
//    could be waiting on (a file-backed mapping) is unsupported and dies with
//    a FATAL in sys_futex_wait/sys_futex_wake.  Nothing in AOS needs a
//    cross-process futex or condition variable on Windows: the lockless
//    queue's data path wakes watchers through the event-loop backend, not
//    futexes.
//
//  * aos_mutex dispatches on where it lives.  In process-private memory it
//    keeps the futex implementation (CAS fast path + WaitOnAddress).  In a
//    file-backed shared mapping it is backed by a named Win32 kernel mutex
//    instead: every acquire goes through WaitForSingleObject and every
//    release through ReleaseMutex.  There is deliberately no userspace fast
//    path for shared mutexes -- if a bare CAS could take the lock without
//    touching the kernel object, a waiter blocked on the kernel mutex would
//    acquire it concurrently and two threads would own the lock.  The only
//    shared aos_mutex in practice is the lockless queue's queue_setup_lock,
//    which is initialization-only, so the syscall per acquire is irrelevant.
//
//    The kernel mutex also buys robustness the futex emulation never had:
//    the kernel tracks which mutexes each thread owns and abandons them on
//    ANY thread termination (clean exit, ExitThread, TerminateThread,
//    TerminateProcess).  The next acquirer gets WAIT_ABANDONED, which we
//    translate into the FUTEX_OWNER_DIED contract.  The shm word remains the
//    authoritative record of ownership (TID | FUTEX_OWNER_DIED): external
//    observers (RobustOwnershipTracker, PrintMutex) decode it, ForceClear
//    stores to it from other processes, and the robust list still marks it on
//    clean thread exit.  The kernel object is only the blocking/abandonment
//    mechanism; after every successful WaitForSingleObject we decode the word
//    to decide whether the previous owner died (nonzero word or
//    FUTEX_OWNER_DIED bit => it never unlocked cleanly => return 1).
//    FUTEX_WAITERS is never set on this path -- the kernel tracks its own
//    waiters -- and the unlock CHECK enforces that: only the portable
//    (process-private) path uses that bit on Windows.
//
//  * death_notification_* stays CAS-only on the word (it never blocks), with
//    RobustOwnershipTracker's thread-start-ticks probe as the liveness
//    backstop.  It gets no kernel object.
//
// The named kernel mutex for a shared aos_mutex is derived from the mapped
// file's path (hashed) plus the offset of the futex within the mapping, so
// every process that maps the same file derives the same name.  Since Win32
// HANDLEs are process-specific, each thread caches the handles it has opened
// in a thread-local map (tl_futex_cache), which also caches the
// private-vs-shared classification of every futex address it has touched.
// Handles are closed via RAII on thread exit.

namespace aos::ipc_lib::sync {
namespace {

// Which kind of memory a futex lives in.  This picks the wait/wake primitive:
// WaitOnAddress is scoped to the calling process, so it only works for memory
// no other process can be waiting on.
enum class FutexMemoryType {
  // Memory private to this process (heap, stack, anonymous mappings).  Uses
  // WaitOnAddress / WakeByAddress{Single,All}.
  kPrivate,
  // Memory backed by a file mapping, which another process may have mapped
  // too.  Only supported for aos_mutex (backed by the named kernel mutex
  // described above); futex_ and condition_ operations FATAL here.
  kShared,
};

// Classifies the memory `addr` lives in, and computes the two values that name
// this futex's kernel mutex if it is shared.
//
// Returns kPrivate for process-private memory; *out_hash and *out_offset are
// left alone in that case.  Returns kShared for memory backed by a file
// mapping, and sets *out_hash to a hash of the mapped file's path and
// *out_offset to the offset of `addr` within that mapping.  Every process
// mapping the same file derives the same pair, and therefore the same
// kernel mutex name, for a given futex.
FutexMemoryType ClassifyFutexAddress(void *addr, uint64_t *out_hash,
                                     uint64_t *out_offset) {
  MEMORY_BASIC_INFORMATION mbi;
  if (VirtualQuery(addr, &mbi, sizeof(mbi)) == 0) {
    ABSL_LOG(FATAL) << "VirtualQuery(" << addr
                    << ") failed: " << GetLastError();
  }

  if (mbi.Type != MEM_MAPPED) {
    return FutexMemoryType::kPrivate;
  }

  char filename[MAX_PATH];
  DWORD length = GetMappedFileNameA(GetCurrentProcess(), mbi.AllocationBase,
                                    filename, sizeof(filename));
  if (length == 0) {
    // Backed by the pagefile/swap rather than a shared file mapping, so no
    // other process can be looking at it.
    return FutexMemoryType::kPrivate;
  }

  // Compute FNV-1a 64-bit hash of the file path.
  uint64_t hash = 14695981039346656037ULL;
  for (DWORD i = 0; i < length; ++i) {
    hash ^= static_cast<uint8_t>(filename[i]);
    hash *= 1099511628211ULL;
  }

  *out_hash = hash;
  *out_offset = reinterpret_cast<uintptr_t>(addr) -
                reinterpret_cast<uintptr_t>(mbi.AllocationBase);

  return FutexMemoryType::kShared;
}

struct FutexCacheEntry {
  // Where this futex lives, as returned by ClassifyFutexAddress.
  FutexMemoryType type;

  // The named kernel mutex backing a shared-memory aos_mutex (NULL for
  // kPrivate).  Only ever waited on by the mutex_ functions; futex_ and
  // condition_ uses of shared memory die before touching it.
  HANDLE hMutex;
};

struct ThreadLocalFutexCache {
  absl::flat_hash_map<void *, FutexCacheEntry> cache;

  ~ThreadLocalFutexCache() {
    for (auto &pair : cache) {
      if (pair.second.hMutex != NULL) {
        // Closing the handle while this thread owns the mutex would not
        // release it; ownership belongs to the thread, and the kernel abandons
        // it at thread exit regardless.
        CloseHandle(pair.second.hMutex);
      }
    }
  }

  // Returns a pointer to the FutexCacheEntry for the given address,
  // creating it if it doesn't exist.
  FutexCacheEntry *GetOrCreate(void *addr) {
    auto it = cache.find(addr);
    if (it != cache.end()) {
      return &it->second;
    }

    uint64_t hash = 0;
    uint64_t offset = 0;
    const FutexMemoryType type = ClassifyFutexAddress(addr, &hash, &offset);

    FutexCacheEntry entry;
    entry.type = type;
    entry.hMutex = NULL;

    if (type == FutexMemoryType::kShared) {
      char name[128];
      snprintf(name, sizeof(name), "Local\\aos_shm_mutex_%llx_%llx", hash,
               offset);
      entry.hMutex = CreateMutexA(NULL, FALSE, name);
      if (entry.hMutex == NULL) {
        ABSL_LOG(FATAL) << "CreateMutexA(" << name
                        << ") failed: " << GetLastError();
      }
    }

    {
      // This is on Windows, and we just don't care...  There is no way this
      // will all be realtime anyways.  Take the easy road here and allocate
      // memory for our mutex.
      aos::ScopedNotRealtime no_rt;
      auto insert_res = cache.emplace(addr, entry);
      return &insert_res.first->second;
    }
  }
};

// Thread-local instance.
thread_local ThreadLocalFutexCache tl_futex_cache;

// Convert a relative timeout timespec into a millisecond count for the Win32
// wait APIs.  Callers reduce any absolute deadline to a relative interval
// before reaching here, so this does no now() subtraction.
DWORD GetTimeoutMs(const struct timespec *timeout) {
  if (timeout == nullptr) {
    return INFINITE;
  }
  int64_t timeout_ms =
      static_cast<int64_t>(timeout->tv_sec) * 1000 +
      (static_cast<int64_t>(timeout->tv_nsec) + 999999) / 1000000;
  if (timeout_ms < 0) {
    timeout_ms = 0;
  } else if (timeout_ms >= static_cast<int64_t>(INFINITE)) {
    timeout_ms = static_cast<int64_t>(INFINITE) - 1;
  }
  return static_cast<DWORD>(timeout_ms);
}

// FATALs if addr classifies as shared memory.  futex_ and condition_
// operations funnel through here: they are built on WaitOnAddress /
// WakeByAddress, which only work within a single process, so putting one in
// memory another process could map is explicitly unsupported on Windows.
// Only aos_mutex supports cross-process use (via a named kernel mutex; see
// the design comment above).
void CheckFutexIsProcessLocal(FutexCacheEntry *entry, void *addr) {
  ABSL_CHECK(entry->type == FutexMemoryType::kPrivate)
      << ": aos futexes and condition variables do not support cross-process "
         "use on Windows, but "
      << addr
      << " is in a file-backed shared memory mapping.  Only aos_mutex works "
         "across processes on Windows; use one of those, or move this "
         "primitive into process-private memory.";
}

// Returns the futex cache entry for m, which says where m lives (and
// therefore which implementation backs it) and holds the kernel mutex handle
// if it is shared.
inline FutexCacheEntry *MutexCacheEntry(aos_mutex *m) {
  return tl_futex_cache.GetOrCreate(&m->futex);
}

// Acquires the kernel mutex backing a shared-memory aos_mutex and stamps the
// value word.  timeout_ms is INFINITE for a blocking lock or 0 for a trylock.
//
// Returns 0 if locked cleanly, 1 if locked but the previous owner never
// unlocked cleanly (it died; possibly already marked by the robust list), or
// 4 for a trylock that lost.  Never returns 2: Windows waits have no EINTR.
inline int shared_mutex_lock(FutexCacheEntry *entry, aos_mutex *m, uint32_t tid,
                             DWORD timeout_ms, my_robust_list::Adder *adder) {
  {
    const uint32_t v =
        std::atomic_ref<uint32_t>(m->futex).load(std::memory_order_acquire);
    // Kernel mutexes are recursive: a second WaitForSingleObject on the
    // owning thread succeeds and bumps a count instead of deadlocking.
    // aos_mutex is not recursive, so catch relocking by hand.  Our TID in the
    // word can only have been stamped by us, so there is no race here.
    if ((v & FUTEX_TID_MASK) == tid) {
      ABSL_LOG(FATAL) << "multiple lock of " << m << " by " << tid;
    }
  }

  const DWORD res = WaitForSingleObject(entry->hMutex, timeout_ms);
  if (res == WAIT_TIMEOUT) {
    return 4;
  }
  ABSL_CHECK(res == WAIT_OBJECT_0 || res == WAIT_ABANDONED)
      << ": WaitForSingleObject(" << entry->hMutex << ") on aos_mutex " << m
      << " failed: " << GetLastError();

  // Mirror the Linux trylock ordering: the mutex is held from here on, so
  // link it into the robust list before the word atomics below, exactly
  // where the futex path's Add() sits relative to mutex_finish_lock().
  // (mutex_do_get passes nullptr and its caller Adds afterwards instead,
  // matching the futex path's asymmetry between get and trylock.)
  if (adder != nullptr) {
    adder->Add();
  }

  // We hold the kernel mutex, so the word is ours: the only other writers are
  // lock/unlock under this same kernel mutex, the robust list walk of the
  // dead prior owner (which ran on the dying thread, before the kernel
  // abandoned the mutex to us), and RobustOwnershipTracker::ForceClear, whose
  // callers serialize against locking with a higher-level lock.
  //
  // Decode the word to learn whether the previous owner unlocked cleanly.  A
  // clean unlock stores 0 before ReleaseMutex, so anything else means the
  // owner died holding it: either a stale TID (the owner was killed without
  // any cleanup running, and kernel abandonment is how we got here) or
  // FUTEX_OWNER_DIED (the robust list walk or
  // mutex_pretend_owner_died_for_testing marked it).  Overwriting the word
  // with our TID consumes the FUTEX_OWNER_DIED bit, exactly like
  // mutex_finish_lock does on the futex path.
  //
  // Note that WAIT_ABANDONED with a zero word is treated as a CLEAN lock: it
  // means the owner died after its store of 0 but before ReleaseMutex, and
  // the unlock had semantically completed.
  // The word only mirrors ownership state which the kernel mutex actually
  // enforces, for the benefit of observers reading shared memory.  We hold
  // the kernel mutex, so nothing may legitimately write the word between
  // this load and the stamp; use a compare-and-swap and LOG(FATAL) on a
  // mismatch rather than retrying, because a mismatch means somebody wrote
  // the mirror outside the locking protocol and fixing it up would hide the
  // corruption.
  const uint32_t v =
      std::atomic_ref<uint32_t>(m->futex).load(std::memory_order_acquire);
  const bool owner_died = v != 0;
  const uint32_t prev = compare_and_swap_val(&m->futex, v, tid);
  ABSL_CHECK_EQ(prev, v) << ": aos_mutex " << m
                         << " changed while its kernel mutex was held";
  if (AOS_UNLIKELY(owner_died)) {
    force_lock_pthread_mutex(m);
    return 1;
  }
  lock_pthread_mutex(m);
  return 0;
}

}  // namespace

pid_t do_get_tid() {
  const DWORD tid = GetCurrentThreadId();
  // Windows Thread IDs are always multiples of 4 (the lowest 2 bits are 0).
  ABSL_CHECK_EQ(tid & 3u, 0u)
      << ": thread id " << tid << " is not a multiple of 4";
  // The futex encoding steals the top 2 bits for FUTEX_WAITERS and
  // FUTEX_OWNER_DIED, and 0 means "unlocked".  Shift the thread ID down by 2
  // bits to fit all 32 bits of thread ID space into the 30-bit FUTEX_TID_MASK
  // without losing any information.
  const DWORD shifted_tid = tid >> 2;
  ABSL_CHECK_NE(shifted_tid, 0u);
  ABSL_CHECK_EQ(shifted_tid & ~static_cast<DWORD>(FUTEX_TID_MASK), 0u)
      << ": shifted thread id " << shifted_tid
      << " does not fit in FUTEX_TID_MASK";
  return static_cast<pid_t>(shifted_tid);
}

// Windows has no fork(2), so there is nothing to hook.
void InstallAtforkHook() {}

int wait_on_address(aos_futex *addr1, int val1,
                    const struct timespec *timeout) {
  CheckFutexIsProcessLocal(tl_futex_cache.GetOrCreate(addr1), addr1);
  uint32_t val = static_cast<uint32_t>(val1);
  // GetTimeoutMs turns a null timeout into INFINITE, which is exactly what
  // an untimed wait wants.
  const BOOL success =
      WaitOnAddress(addr1, &val, sizeof(*addr1), GetTimeoutMs(timeout));
  if (!success) {
    const DWORD error = GetLastError();
    if (error == ERROR_TIMEOUT) {
      return -ETIMEDOUT;
    }
    ABSL_LOG(FATAL) << "WaitOnAddress(" << addr1 << ") failed: " << error;
  }
  return 0;
}

int sys_futex_wake(aos_futex *addr1, int val1) {
  CheckFutexIsProcessLocal(tl_futex_cache.GetOrCreate(addr1), addr1);
  if (val1 == 1) {
    WakeByAddressSingle(addr1);
  } else {
    WakeByAddressAll(addr1);
  }
  return 1;
}

int sys_futex_unlock_pi(aos_futex *addr1) {
  // On Windows the only caller is death_notification_release (mutex_unlock
  // dispatches on its own).  Clear the lock value, which
  // death_notification_release relies on us to do, and wake any same-process
  // waiter directly with WakeByAddress.  A death notification never has
  // blocked waiters -- RobustOwnershipTracker only reads the word and probes
  // thread liveness -- so this wake is defensive only, and deliberately does
  // NOT go through sys_futex_wake: that would FATAL on the shared-memory
  // classification, but a bare word-clear plus process-local wake is correct
  // here regardless of where the word lives.
  std::atomic_ref<uint32_t>(*addr1).store(0, std::memory_order_release);
  WakeByAddressAll(addr1);
  return 0;
}

bool IsValidAddress(void *addr, size_t size) {
  MEMORY_BASIC_INFORMATION mbi;
  if (VirtualQuery(addr, &mbi, sizeof(mbi)) == 0) {
    return false;
  }
  if (mbi.State != MEM_COMMIT) {
    return false;
  }
  DWORD protect = mbi.Protect & ~(PAGE_GUARD | PAGE_NOCACHE);
  if (protect != PAGE_READWRITE && protect != PAGE_WRITECOPY &&
      protect != PAGE_EXECUTE_READWRITE && protect != PAGE_EXECUTE_WRITECOPY) {
    return false;
  }
  uintptr_t limit =
      reinterpret_cast<uintptr_t>(mbi.BaseAddress) + mbi.RegionSize;
  if (reinterpret_cast<uintptr_t>(addr) + size > limit) {
    return false;
  }
  return true;
}

void RobustListCleanerWake(aos_futex *futex) {
  // Wake same-process waiters directly rather than through sys_futex_wake.
  // Two reasons: sys_futex_wake consults the thread-local handle cache, whose
  // destructor may already have run (thread_local destruction order is
  // unspecified and we are inside one now), and it FATALs on shared-memory
  // addresses, which death notifications legitimately are.  WakeByAddress
  // needs neither.  For a shared-memory aos_mutex, waiters are blocked on the
  // kernel mutex, not WaitOnAddress; the kernel abandons it when this thread
  // finishes dying and wakes them with WAIT_ABANDONED, so they need nothing
  // from us.
  WakeByAddressAll(futex);
}

// Shared-memory mutexes are backed by a kernel mutex instead of the futex
// emulation; see the design comment at the top of the file.  These three
// dispatch on where the mutex lives and hand everything process-private to
// the portable futex-word implementations.

int mutex_do_get(aos_mutex *m, bool signals_fail, uint32_t tid) {
  FutexCacheEntry *entry = MutexCacheEntry(m);
  if (entry->type == FutexMemoryType::kShared) {
    // signals_fail is irrelevant here: Windows waits have no EINTR.
    return shared_mutex_lock(entry, m, tid, INFINITE, nullptr);
  }
  return portable_mutex_do_get(m, signals_fail, tid);
}

int mutex_do_trylock(aos_mutex *m, uint32_t tid, my_robust_list::Adder *adder) {
  FutexCacheEntry *entry = MutexCacheEntry(m);
  if (entry->type == FutexMemoryType::kShared) {
    // A zero-timeout wait is the trylock.
    return shared_mutex_lock(entry, m, tid, 0, adder);
  }
  return portable_mutex_do_trylock(m, tid, adder);
}

void mutex_do_unlock(aos_mutex *m, uint32_t tid) {
  FutexCacheEntry *entry = MutexCacheEntry(m);
  if (entry->type == FutexMemoryType::kPrivate) {
    return portable_mutex_do_unlock(m, tid);
  }
  // Clear the word before releasing the kernel mutex: the next acquirer can
  // only run after ReleaseMutex, so it never observes our stale TID and
  // mistakes this unlock for an owner death.  If this thread dies between
  // the store and the release, the kernel abandons the mutex and the next
  // acquirer sees WAIT_ABANDONED with a zero word, which shared_mutex_lock
  // correctly treats as a completed unlock.
  //
  // Compare-and-swap against our own TID rather than blindly storing 0:
  // while we hold the kernel mutex nothing may legitimately change the word,
  // so anything but our TID here means the word was corrupted or a writer
  // broke the locking protocol, and releasing on top of that would destroy
  // the evidence.
  const uint32_t prev = compare_and_swap_val(&m->futex, tid, 0);
  ABSL_CHECK_EQ(prev, tid) << ": aos_mutex " << m
                           << " changed out from under its owner";
  ABSL_CHECK(ReleaseMutex(entry->hMutex) != 0)
      << ": ReleaseMutex on aos_mutex " << m << " failed: " << GetLastError();
}

}  // namespace aos::ipc_lib::sync

// The rest of this is part of the public interface, which lives at the global
// scope.
uint32_t mutex_owner_from_value(uint32_t value) {
  return (value & FUTEX_TID_MASK) << 2;
}
