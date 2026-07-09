#include <mach/mach.h>
#include <mach/thread_policy.h>
#include <malloc/malloc.h>
#include <pthread.h>
#include <stdlib.h>
#include <unistd.h>

#include <cstring>

#include "absl/flags/declare.h"
#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/ipc_lib/aos_sync.h"
#include "aos/realtime.h"
#include "aos/realtime_internal.h"
#include "aos/uuid.h"

ABSL_DECLARE_FLAG(bool, die_on_malloc);
ABSL_DECLARE_FLAG(bool, skip_realtime_scheduler);

namespace logging::internal {

// Implemented in aos/logging/context.cc.
__attribute__((weak)) void ReloadThreadName();

void ReloadThreadName() {}

}  // namespace logging::internal

namespace aos {

namespace {
thread_local int fake_rt_priority;
thread_local int fake_rt_policy;
}  // namespace

// CpuSet methods are in realtime_fake_cpuset.cc (shared with Windows).

namespace {
// thread local variables allocate memory the first time around.  That means, if
// your first call to is_realtime is in a RT section, you can explode.  To fix
// that, use pthread_keys, which don't allocate memory.
static pthread_key_t kIsRealtimeKey;
static pthread_once_t kIsRealtimeKeyOnce = PTHREAD_ONCE_INIT;

void MakeIsRealtimeKey() {
  // Destructor is null because we store a simple value (casted to void*).
  ABSL_PCHECK(pthread_key_create(&kIsRealtimeKey, nullptr) == 0);
}
}  // namespace

bool GetIsRealtime() {
  pthread_once(&kIsRealtimeKeyOnce, MakeIsRealtimeKey);
  return static_cast<bool>(
      reinterpret_cast<uintptr_t>(pthread_getspecific(kIsRealtimeKey)));
}

void SetIsRealtime(bool realtime) {
  pthread_once(&kIsRealtimeKeyOnce, MakeIsRealtimeKey);
  ABSL_PCHECK(pthread_setspecific(kIsRealtimeKey,
                                  reinterpret_cast<void *>(
                                      static_cast<uintptr_t>(realtime))) == 0);
}

// OSX doesn't have the same concepts of RT scheduler as Linux.  We don't really
// care, since the goal here isn't to make a perfect robot, more so to pretend
// to be equivilent to help test.  Track everything explicitly to make it
// easier.
void UnsetCurrentThreadRealtimePriority() {
  MarkRealtime(false);
  fake_rt_priority = 0;
  fake_rt_policy = SCHED_OTHER;
  if (ABSL_VLOG_IS_ON(1)) {
    ABSL_LOG(WARNING) << "No RT scheduler on OSX, ignoring";
  }
}

namespace {
// Same problem, affinity is tracked differently in OSX, fake it here too.
pthread_key_t kCpuSetKey;
pthread_once_t kCpuSetKeyOnce = PTHREAD_ONCE_INIT;

void FreeCpuSet(void *p) { delete static_cast<CpuSet *>(p); }

void CreateCpuSetKey() {
  ABSL_PCHECK(pthread_key_create(&kCpuSetKey, FreeCpuSet) == 0);
}
}  // namespace

void SetCurrentThreadAffinity(const CpuSet &cpuset) {
  pthread_once(&kCpuSetKeyOnce, CreateCpuSetKey);
  CpuSet *current_affinity =
      static_cast<CpuSet *>(pthread_getspecific(kCpuSetKey));
  if (current_affinity == nullptr) {
    current_affinity = new CpuSet();
    ABSL_PCHECK(pthread_setspecific(kCpuSetKey, current_affinity) == 0);
  }
  *current_affinity = cpuset;

  if (cpuset.Empty() || cpuset == DefaultAffinity()) {
    // Clear the affinity policy.
    thread_affinity_policy_data_t policy = {THREAD_AFFINITY_TAG_NULL};
    ABSL_CHECK_EQ(
        thread_policy_set(pthread_mach_thread_np(pthread_self()),
                          THREAD_AFFINITY_POLICY, (thread_policy_t)&policy,
                          THREAD_AFFINITY_POLICY_COUNT),
        KERN_SUCCESS);
  } else {
    integer_t tag = 0;
    // We want to map the cpuset to an affinity tag.  The kernel doesn't give us
    // enough control to do this perfectly, but we can do a decent job by
    // hashing the cpuset.
    for (size_t i = 0; i < CpuSet::kSize; ++i) {
      if (cpuset.IsSet(i)) {
        tag = (tag << 1) ^ i;
      }
    }

    // The affinity policy tells the kernel what to "group".  Group things with
    // the same CpuSet together.
    thread_affinity_policy_data_t policy = {tag};

    ABSL_CHECK_EQ(
        thread_policy_set(pthread_mach_thread_np(pthread_self()),
                          THREAD_AFFINITY_POLICY, (thread_policy_t)&policy,
                          THREAD_AFFINITY_POLICY_COUNT),
        KERN_SUCCESS);
  }
}

void SetCurrentThreadName(const std::string_view name) {
  ABSL_CHECK_LE(name.size(), 16u) << ": thread name '" << name << "' too long";
  ABSL_VLOG(1) << "This thread is changing to '" << name << "'";
  std::string string_name(name);
  pthread_setname_np(string_name.c_str());
  if (&logging::internal::ReloadThreadName != nullptr) {
    logging::internal::ReloadThreadName();
  }
}

CpuSet GetCurrentThreadAffinity() {
  CpuSet result;
  // There is no way to query the affinity back from the kernel, so just return
  // what we were last set to.
  pthread_once(&kCpuSetKeyOnce, CreateCpuSetKey);
  CpuSet *current_affinity =
      static_cast<CpuSet *>(pthread_getspecific(kCpuSetKey));
  if (current_affinity == nullptr) {
    result = DefaultAffinity();
  } else {
    result = *current_affinity;
  }
  return result;
}

void SetCurrentThreadRealtimePriority(int priority, int scheduling_policy,
                                      RealtimePolicy realtime_policy) {
  // Ensure that we won't get expensive reads of /dev/random when the realtime
  // scheduler is running to initialize the pseudo random number generator.
  UUID::Random();

  // Force initialization of the aos_sync primitives on this thread so that we
  // don't take a malloc inside lock/unlock later.
  {
    static aos_mutex kInitMutex = {.next = 0,
                                   .previous = nullptr,
                                   .futex = 0
#ifdef AOS_SANITIZER_thread
                                   ,
                                   .pthread_mutex_init = false
#endif
    };
    (void)mutex_lock(&kInitMutex);
    mutex_unlock(&kInitMutex);
  }

  if (absl::GetFlag(FLAGS_skip_realtime_scheduler)) {
    ABSL_LOG(WARNING)
        << "Ignoring request to switch to the RT scheduler due to "
           "--skip_realtime_scheduler.";
    return;
  }

  // Fake it.
  // TODO(austin): If someone knows a better way to do this, lets do better.
  ABSL_CHECK(scheduling_policy == SCHED_FIFO || scheduling_policy == SCHED_RR)
      << "Specified non-realtime scheduling policy with realtime priority";
  ABSL_CHECK(priority > 0 && priority < 100)
      << "Realtime priority must fall within [1,99]";
  if (ABSL_VLOG_IS_ON(1)) {
    ABSL_LOG(INFO) << "RT priority not implemented on OSX, pretending to be RT";
  }
  fake_rt_priority = priority;
  fake_rt_policy = scheduling_policy;
  if (realtime_policy != RealtimePolicy::NO_MODE) {
    MarkRealtime(true);
  }
}

int GetCurrentThreadRealtimePriority() { return fake_rt_priority; }

int GetCurrentThreadSchedulingPolicy() { return fake_rt_policy; }

// There are multiple ways to hook into malloc, but this appears to be the best.
// We could edit the zone hooks directly to inject ourselves in the middle, do
// this (add a proxy zone), or use a dynamic library to interpose malloc.
// Bazel likes static linking, so let's go this route.  We want to make a zone
// who's entire job is to proxy to the real zone, and enforce malloc checks.

// Basic proxy zone structure
static malloc_zone_t aos_zone;
static malloc_zone_t *system_zone = nullptr;

static size_t AosSize(struct _malloc_zone_t * /*zone*/, const void *ptr) {
  return system_zone->size(system_zone, ptr);
}

static void *AosMalloc(struct _malloc_zone_t * /*zone*/, size_t size) {
  if (GetIsRealtime() && absl::GetFlag(FLAGS_die_on_malloc)) {
    SetIsRealtime(false);
    ABSL_RAW_LOG(FATAL, "Malloced %zu bytes", size);
  }
  return system_zone->malloc(system_zone, size);
}

static void *AosCalloc(struct _malloc_zone_t * /*zone*/, size_t num_items,
                       size_t size) {
  if (GetIsRealtime() && absl::GetFlag(FLAGS_die_on_malloc)) {
    SetIsRealtime(false);
    ABSL_RAW_LOG(FATAL, "Malloced %zu * %zu bytes", num_items, size);
  }
  return system_zone->calloc(system_zone, num_items, size);
}

static void *AosValloc(struct _malloc_zone_t * /*zone*/, size_t size) {
  if (GetIsRealtime() && absl::GetFlag(FLAGS_die_on_malloc)) {
    SetIsRealtime(false);
    ABSL_RAW_LOG(FATAL, "Malloced %zu bytes", size);
  }
  return system_zone->valloc(system_zone, size);
}

static void AosFree(struct _malloc_zone_t * /*zone*/, void *ptr) {
  // This might not be called if system_zone owns ptr.
  if (GetIsRealtime() && absl::GetFlag(FLAGS_die_on_malloc) && ptr != nullptr) {
    SetIsRealtime(false);
    ABSL_RAW_LOG(FATAL, "Deleted %p", ptr);
  }
  system_zone->free(system_zone, ptr);
}

static void *AosRealloc(struct _malloc_zone_t * /*zone*/, void *ptr,
                        size_t size) {
  if (GetIsRealtime() && absl::GetFlag(FLAGS_die_on_malloc)) {
    SetIsRealtime(false);
    ABSL_RAW_LOG(FATAL, "Malloced %p -> %zu bytes", ptr, size);
  }
  return system_zone->realloc(system_zone, ptr, size);
}

static void AosDestroy(struct _malloc_zone_t *zone) {
  system_zone->destroy(zone);
}

static unsigned AosBatchMalloc(struct _malloc_zone_t * /*zone*/, size_t size,
                               void **results, unsigned num_requested) {
  if (GetIsRealtime() && absl::GetFlag(FLAGS_die_on_malloc)) {
    SetIsRealtime(false);
    ABSL_RAW_LOG(FATAL, "Batch Malloced %u * %zu bytes", num_requested, size);
  }
  return system_zone->batch_malloc(system_zone, size, results, num_requested);
}

static void AosBatchFree(struct _malloc_zone_t * /*zone*/, void **to_be_freed,
                         unsigned num_to_be_freed) {
  if (aos::GetIsRealtime() && absl::GetFlag(FLAGS_die_on_malloc) &&
      num_to_be_freed > 0) {
    SetIsRealtime(false);
    ABSL_RAW_LOG(FATAL, "Batch Deleted %u items", num_to_be_freed);
  }
  system_zone->batch_free(system_zone, to_be_freed, num_to_be_freed);
}

static void *AosMemalign(struct _malloc_zone_t * /*zone*/, size_t alignment,
                         size_t size) {
  if (GetIsRealtime() && absl::GetFlag(FLAGS_die_on_malloc)) {
    SetIsRealtime(false);
    ABSL_RAW_LOG(FATAL, "Memaligned %zu bytes", size);
  }
  return system_zone->memalign(system_zone, alignment, size);
}

static void AosFreeDefiniteSize(struct _malloc_zone_t * /*zone*/, void *ptr,
                                size_t size) {
  if (GetIsRealtime() && absl::GetFlag(FLAGS_die_on_malloc) && ptr != nullptr) {
    SetIsRealtime(false);
    ABSL_RAW_LOG(FATAL, "Deleted %p", ptr);
  }
  system_zone->free_definite_size(system_zone, ptr, size);
}

static size_t AosPressureRelief(struct _malloc_zone_t * /*zone*/, size_t goal) {
  return system_zone->pressure_relief(system_zone, goal);
}

static boolean_t AosClaimedAddress(struct _malloc_zone_t * /*zone*/,
                                   void *ptr) {
  return system_zone->claimed_address(system_zone, ptr);
}

static kern_return_t AosEnumerator(task_t task, void *context,
                                   unsigned type_mask,
                                   vm_address_t zone_address,
                                   memory_reader_t reader,
                                   vm_range_recorder_t recorder) {
  return system_zone->introspect->enumerator(task, context, type_mask,
                                             zone_address, reader, recorder);
}

static size_t AosGoodSize(malloc_zone_t * /*zone*/, size_t size) {
  return system_zone->introspect->good_size(system_zone, size);
}

static boolean_t AosCheck(malloc_zone_t * /*zone*/) {
  return system_zone->introspect->check(system_zone);
}

static void AosPrint(malloc_zone_t * /*zone*/, boolean_t verbose) {
  system_zone->introspect->print(system_zone, verbose);
}

static void AosLog(malloc_zone_t * /*zone*/, void *address) {
  system_zone->introspect->log(system_zone, address);
}

static void AosForceLock(malloc_zone_t * /*zone*/) {
  system_zone->introspect->force_lock(system_zone);
}

static void AosForceUnlock(malloc_zone_t * /*zone*/) {
  system_zone->introspect->force_unlock(system_zone);
}

static void AosStatistics(malloc_zone_t * /*zone*/,
                          malloc_statistics_t *stats) {
  system_zone->introspect->statistics(system_zone, stats);
}

static boolean_t AosZoneLocked(malloc_zone_t * /*zone*/) {
  return system_zone->introspect->zone_locked(system_zone);
}

static malloc_introspection_t aos_introspect;  // Zeroed

// Grabbed from jemalloc.
static malloc_zone_t *ZoneDefaultGet(void) {
  malloc_zone_t **zones = NULL;
  unsigned int num_zones = 0;

  /*
   * On OSX 10.12, malloc_default_zone returns a special zone that is not
   * present in the list of registered zones. That zone uses a "lite zone"
   * if one is present (apparently enabled when malloc stack logging is
   * enabled), or the first registered zone otherwise. In practice this
   * means unless malloc stack logging is enabled, the first registered
   * zone is the default.  So get the list of zones to get the first one,
   * instead of relying on malloc_default_zone.
   */
  if (KERN_SUCCESS !=
      malloc_get_all_zones(0, NULL, (vm_address_t **)&zones, &num_zones)) {
    // Reset the value in case the failure happened after it was set.
    num_zones = 0;
  }

  if (num_zones) {
    return zones[0];
  }

  return malloc_default_zone();
}

void InstallHooks() {
  if (ABSL_VLOG_IS_ON(1)) {
    ABSL_RAW_LOG(INFO, "Installing Proxy Malloc Zone");
  }

  // Initialize aos_zone.
  memset(&aos_zone, 0, sizeof(aos_zone));

  aos_zone.version = 8;
  aos_zone.zone_name = "aos_realtime_proxy_zone";

  // Verify the system zone is well formed, and set all the pointers.
  ABSL_CHECK(system_zone->size != nullptr);
  aos_zone.size = AosSize;
  ABSL_CHECK(system_zone->malloc != nullptr);
  aos_zone.malloc = AosMalloc;
  ABSL_CHECK(system_zone->calloc != nullptr);
  aos_zone.calloc = AosCalloc;
  ABSL_CHECK(system_zone->valloc != nullptr);
  aos_zone.valloc = AosValloc;
  ABSL_CHECK(system_zone->free != nullptr);
  aos_zone.free = AosFree;
  ABSL_CHECK(system_zone->realloc != nullptr);
  aos_zone.realloc = AosRealloc;
  ABSL_CHECK(system_zone->destroy != nullptr);
  aos_zone.destroy = AosDestroy;
  ABSL_CHECK(system_zone->batch_malloc != nullptr);
  aos_zone.batch_malloc = AosBatchMalloc;
  ABSL_CHECK(system_zone->batch_free != nullptr);
  aos_zone.batch_free = AosBatchFree;
  ABSL_CHECK(system_zone->memalign != nullptr);
  aos_zone.memalign = AosMemalign;
  ABSL_CHECK(system_zone->free_definite_size != nullptr);
  aos_zone.free_definite_size = AosFreeDefiniteSize;
  ABSL_CHECK(system_zone->pressure_relief != nullptr);
  aos_zone.pressure_relief = AosPressureRelief;
  ABSL_CHECK(system_zone->claimed_address != nullptr);
  aos_zone.claimed_address = AosClaimedAddress;

  // Do the same for introspect.
  aos_zone.introspect = &aos_introspect;
  memset(&aos_introspect, 0, sizeof(aos_introspect));
  ABSL_CHECK(system_zone->introspect->enumerator != nullptr);
  aos_introspect.enumerator = AosEnumerator;
  ABSL_CHECK(system_zone->introspect->good_size != nullptr);
  aos_introspect.good_size = AosGoodSize;
  ABSL_CHECK(system_zone->introspect->check != nullptr);
  aos_introspect.check = AosCheck;
  ABSL_CHECK(system_zone->introspect->print != nullptr);
  aos_introspect.print = AosPrint;
  ABSL_CHECK(system_zone->introspect->log != nullptr);
  aos_introspect.log = AosLog;
  ABSL_CHECK(system_zone->introspect->force_lock != nullptr);
  aos_introspect.force_lock = AosForceLock;
  ABSL_CHECK(system_zone->introspect->force_unlock != nullptr);
  aos_introspect.force_unlock = AosForceUnlock;
  ABSL_CHECK(system_zone->introspect->statistics != nullptr);
  aos_introspect.statistics = AosStatistics;
  ABSL_CHECK(system_zone->introspect->zone_locked != nullptr);
  aos_introspect.zone_locked = AosZoneLocked;
  ABSL_CHECK(system_zone->introspect->enable_discharge_checking == nullptr);
  ABSL_CHECK(system_zone->introspect->disable_discharge_checking == nullptr);
  ABSL_CHECK(system_zone->introspect->discharge == nullptr);

  //  Register our zone.
  malloc_zone_register(&aos_zone);

  size_t loops = 0;
  malloc_zone_t *zone;

  do {
    // Promote to default by making it the "first" zone.
    // We do this by unregistering the system zone and re-registering it.
    // This bumps system zone to the end of the list, leaving aos_zone (added
    // just before) ahead of it.
    malloc_zone_unregister(system_zone);
    malloc_zone_register(system_zone);
    zone = ZoneDefaultGet();

    // Make sure this doesn't spin forever.  At least blow up with a better
    // error.
    ++loops;
    if (loops > 10) {
      ABSL_RAW_LOG(FATAL, "Too many loops");
    }
  } while (zone != &aos_zone);
}

void UninstallHooks() { malloc_zone_unregister(&aos_zone); }

// Forking causes all sorts of problems.  This is pretty rare, so we don't need
// to be all that graceful.  Fix it bye removing the hooks before the fork, and
// re-adding them after the fork, both in the parent and child.
void PrepareFork() {
  if (absl::GetFlag(FLAGS_die_on_malloc)) {
    UninstallHooks();
  }
}

void PostFork() {
  if (absl::GetFlag(FLAGS_die_on_malloc)) {
    InstallHooks();
  }
}

void RegisterMallocHook() {
  if (absl::GetFlag(FLAGS_die_on_malloc)) {
    static bool registered = false;
    if (registered) return;
    registered = true;

    // Capture the current default zone to be used as the actual allocator.
    system_zone = ZoneDefaultGet();
    ABSL_CHECK(system_zone != nullptr) << "Could not get default malloc zone";

    if (!system_zone->zone_name ||
        strcmp(system_zone->zone_name, "DefaultMallocZone") != 0) {
      ABSL_RAW_LOG(FATAL, "Alternative malloc zone registered, %s, aborting",
                   system_zone->zone_name);
    }

    InstallHooks();

    pthread_atfork(PrepareFork, PostFork, PostFork);
  }
}

std::string GetProgramName() { return getprogname(); }

std::string GetThreadName() {
  char thread_name_array[65];
  if (pthread_getname_np(pthread_self(), thread_name_array,
                         sizeof(thread_name_array)) != 0) {
    ABSL_PLOG(FATAL) << "pthread_getname_np failed";
  }
  thread_name_array[sizeof(thread_name_array) - 1] = '\0';
  return std::string(thread_name_array);
}

pid_t GetProcessId() { return getpid(); }
pid_t GetThreadId() {
  uint64_t tid;
  pthread_threadid_np(NULL, &tid);
  return static_cast<pid_t>(tid);
}

}  // namespace aos
