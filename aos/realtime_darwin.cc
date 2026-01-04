#include "aos/realtime.h"

#include <mach/mach.h>
#include <mach/thread_policy.h>
#include <malloc/malloc.h>
#include <pthread.h>
#include <unistd.h>

#include "absl/flags/declare.h"
#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/uuid.h"

ABSL_DECLARE_FLAG(bool, die_on_malloc);
ABSL_DECLARE_FLAG(bool, skip_realtime_scheduler);

namespace logging::internal {

// Implemented in aos/logging/context.cc.
void ReloadThreadName() __attribute__((weak)) {}

}  // namespace logging::internal

#include "aos/realtime_internal.h"

namespace aos {

extern bool has_malloc_hook;
thread_local int fake_rt_priority;
thread_local int fake_rt_policy;

CpuSet::CpuSet() {}

void CpuSet::Set(int cpu) {
  if (cpu >= 0 && cpu < static_cast<int>(set_.size())) {
    set_.set(cpu);
  }
}

void CpuSet::Clear(int cpu) {
  if (cpu >= 0 && cpu < static_cast<int>(set_.size())) {
    set_.reset(cpu);
  }
}

void CpuSet::Clear() { set_.reset(); }

bool CpuSet::IsSet(int cpu) const {
  if (cpu >= 0 && cpu < static_cast<int>(set_.size())) {
    return set_.test(cpu);
  }
  return false;
}

bool CpuSet::Empty() const { return set_.none(); }

bool CpuSet::operator==(const CpuSet &other) const {
  return set_ == other.set_;
}

bool CpuSet::operator!=(const CpuSet &other) const {
  return set_ != other.set_;
}

void UnsetCurrentThreadRealtimePriority() {
  MarkRealtime(false);
  fake_rt_priority = 0;
  fake_rt_policy = SCHED_OTHER;
  if (ABSL_VLOG_IS_ON(1)) {
    ABSL_LOG(WARNING) << "No RT scheduler on OSX, ignoring";
  }
}

namespace {
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
    thread_affinity_policy_data_t policy = {THREAD_AFFINITY_TAG_NULL};
    ABSL_CHECK_EQ(thread_policy_set(pthread_mach_thread_np(pthread_self()),
                                    THREAD_AFFINITY_POLICY,
                                    (thread_policy_t)&policy,
                                    THREAD_AFFINITY_POLICY_COUNT),
                  KERN_SUCCESS);
  } else {
    integer_t tag = 0;
    // We want to map the cpuset to an affinity tag.  The kernel doesn't give us
    // enough control to do this perfectly, but we can do a decent job by hashing
    // the cpuset.
    for (size_t i = 0; i < CpuSet::kSize; ++i) {
      if (cpuset.IsSet(i)) {
        tag = (tag << 1) ^ i;
      }
    }

    thread_affinity_policy_data_t policy = {tag};

    ABSL_CHECK_EQ(thread_policy_set(pthread_mach_thread_np(pthread_self()),
                                    THREAD_AFFINITY_POLICY,
                                    (thread_policy_t)&policy,
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

void SetCurrentThreadRealtimePriority(int priority, int scheduling_policy) {
  // Ensure that we won't get expensive reads of /dev/random when the realtime
  // scheduler is running.
  UUID::Random();

  if (absl::GetFlag(FLAGS_skip_realtime_scheduler)) {
    ABSL_LOG(WARNING)
        << "Ignoring request to switch to the RT scheduler due to "
           "--skip_realtime_scheduler.";
    return;
  }

  ABSL_CHECK(scheduling_policy == SCHED_FIFO || scheduling_policy == SCHED_RR)
      << "Specified non-realtime scheduling policy with realtime priority";
  ABSL_CHECK(priority > 0 && priority < 100)
      << "Realtime priority must fall within [1,99]";
  if (ABSL_VLOG_IS_ON(1)) {
    ABSL_LOG(INFO) << "RT priority not implemented on OSX, pretending to be RT";
  }
  fake_rt_priority = priority;
  fake_rt_policy = scheduling_policy;
  MarkRealtime(true);
}

int GetCurrentThreadRealtimePriority() {
  return fake_rt_priority;
}

int GetCurrentThreadSchedulingPolicy() {
  return fake_rt_policy;
}

void print_zones() {
  vm_address_t *zones = nullptr;
  unsigned int count = 0;
  kern_return_t kr =
      malloc_get_all_zones(mach_task_self(), nullptr, &zones, &count);

  if (kr != KERN_SUCCESS) {
    perror("malloc_get_all_zones failed");
    return;
  }

  ABSL_RAW_LOG(INFO, "Found %u active malloc zones", count);

  for (unsigned int i = 0; i < count; ++i) {
    malloc_zone_t *zone = (malloc_zone_t *)zones[i];
    const char *name = zone->version >= 4 ? zone->zone_name : "unknown";

    ABSL_RAW_LOG(INFO, "Zone [%u]: %s at %p", i, name, zone);
    ABSL_RAW_LOG(INFO, "Zone [%u]: %s->malloc %p", i, name, zone->malloc);
  }
}

// Basic proxy zone structure
static malloc_zone_t aos_zone;
static malloc_zone_t *system_zone = nullptr;

static size_t aos_size(struct _malloc_zone_t *zone, const void *ptr) {
  //ABSL_RAW_LOG(INFO, "aos_size(%p)", ptr);
  if (system_zone && system_zone->size) {
    return system_zone->size(system_zone, ptr);
  }
  return 0;
}

static void *aos_malloc(struct _malloc_zone_t *zone, size_t size) {
  //ABSL_RAW_LOG(INFO, "aos_malloc(%zu)", size);
  if (GetIsRealtime() && absl::GetFlag(FLAGS_die_on_malloc)) {
    SetIsRealtime(false);
    ABSL_RAW_LOG(FATAL, "Malloced %zu bytes", size);
  }
  return system_zone->malloc(system_zone, size);
}

static void *aos_calloc(struct _malloc_zone_t *zone, size_t num_items,
                        size_t size) {
  //ABSL_RAW_LOG(INFO, "aos_calloc(%zu, %zu)", num_items, size);
  if (GetIsRealtime() && absl::GetFlag(FLAGS_die_on_malloc)) {
    SetIsRealtime(false);
    ABSL_RAW_LOG(FATAL, "Malloced %zu * %zu bytes", num_items, size);
  }
  return system_zone->calloc(system_zone, num_items, size);
}

static void *aos_valloc(struct _malloc_zone_t *zone, size_t size) {
  //ABSL_RAW_LOG(INFO, "aos_valloc(%zu)", size);
  if (GetIsRealtime() && absl::GetFlag(FLAGS_die_on_malloc)) {
    SetIsRealtime(false);
    ABSL_RAW_LOG(FATAL, "Malloced %zu bytes", size);
  }
  return system_zone->valloc(system_zone, size);
}

static void aos_free(struct _malloc_zone_t *zone, void *ptr) {
  //ABSL_RAW_LOG(INFO, "aos_free(%p)", ptr);
  // This might not be called if system_zone owns ptr.
  if (GetIsRealtime() && absl::GetFlag(FLAGS_die_on_malloc) &&
      ptr != nullptr) {
    SetIsRealtime(false);
    ABSL_RAW_LOG(FATAL, "Deleted %p", ptr);
  }
  system_zone->free(system_zone, ptr);
}

static void *aos_realloc(struct _malloc_zone_t *zone, void *ptr, size_t size) {
  //ABSL_RAW_LOG(INFO, "aos_realloc(%p, %zu)", ptr, size);
  if (GetIsRealtime() && absl::GetFlag(FLAGS_die_on_malloc)) {
    SetIsRealtime(false);
    ABSL_RAW_LOG(FATAL, "Malloced %p -> %zu bytes", ptr, size);
  }
  return system_zone->realloc(system_zone, ptr, size);
}

static void aos_destroy(struct _malloc_zone_t *zone) {
  //ABSL_RAW_LOG(INFO, "aos_destroy");
  // No-op
}

static unsigned aos_batch_malloc(struct _malloc_zone_t *zone, size_t size,
                                 void **results, unsigned num_requested) {
  //ABSL_RAW_LOG(INFO, "aos_batch_malloc(%zu, %u)", size, num_requested);
  if (GetIsRealtime() && absl::GetFlag(FLAGS_die_on_malloc)) {
    SetIsRealtime(false);
    ABSL_RAW_LOG(FATAL, "Batch Malloced %u * %zu bytes", num_requested, size);
  }
  return system_zone->batch_malloc(system_zone, size, results, num_requested);
}

static void aos_batch_free(struct _malloc_zone_t *zone, void **to_be_freed,
                           unsigned num_to_be_freed) {
  //ABSL_RAW_LOG(INFO, "aos_batch_free(%u)", num_to_be_freed);
  if (aos::GetIsRealtime() && absl::GetFlag(FLAGS_die_on_malloc) &&
      num_to_be_freed > 0) {
    SetIsRealtime(false);
    ABSL_RAW_LOG(FATAL, "Batch Deleted %u items", num_to_be_freed);
  }
  system_zone->batch_free(system_zone, to_be_freed, num_to_be_freed);
}

static void *aos_memalign(struct _malloc_zone_t *zone, size_t alignment,
                          size_t size) {
  //ABSL_RAW_LOG(INFO, "aos_memalign(%zu, %zu)", alignment, size);
  if (GetIsRealtime() && absl::GetFlag(FLAGS_die_on_malloc)) {
    SetIsRealtime(false);
    ABSL_RAW_LOG(FATAL, "Memaligned %zu bytes", size);
  }
  return system_zone->memalign(system_zone, alignment, size);
}

static void aos_free_definite_size(struct _malloc_zone_t *zone, void *ptr,
                                   size_t size) {
  //ABSL_RAW_LOG(INFO, "aos_free_definite_size(%p, %zu)", ptr, size);
  if (GetIsRealtime() && absl::GetFlag(FLAGS_die_on_malloc) &&
      ptr != nullptr) {
    SetIsRealtime(false);
    ABSL_RAW_LOG(FATAL, "Deleted %p", ptr);
  }
  system_zone->free_definite_size(system_zone, ptr, size);
}

static size_t aos_pressure_relief(struct _malloc_zone_t *zone, size_t goal) {
  //ABSL_RAW_LOG(INFO, "aos_pressure_relief");
  return system_zone->pressure_relief(system_zone, goal);
}

static boolean_t aos_claimed_address(struct _malloc_zone_t *zone, void *ptr) {
  // We don't claim anything properly, but we defer.
  // Actually, returning false means we don't own it.
  //ABSL_RAW_LOG(INFO, "aos_claimed_address(%p)", ptr);
  return 0;  // false
}

static kern_return_t aos_enumerator(task_t task, void *context,
                                    unsigned type_mask,
                                    vm_address_t zone_address,
                                    memory_reader_t reader,
                                    vm_range_recorder_t recorder) {
  //ABSL_RAW_LOG(INFO, "aos_enumerator");
  if (system_zone && system_zone->introspect &&
      system_zone->introspect->enumerator) {
    return system_zone->introspect->enumerator(task, context, type_mask,
                                               zone_address, reader, recorder);
  }
  return KERN_SUCCESS;
}

static size_t aos_good_size(malloc_zone_t *zone, size_t size) {
  //ABSL_RAW_LOG(INFO, "aos_good_size(%zu)", size);
  if (system_zone && system_zone->introspect &&
      system_zone->introspect->good_size) {
    return system_zone->introspect->good_size(system_zone, size);
  }
  return size;
}

static boolean_t aos_check(malloc_zone_t *zone) {
  //ABSL_RAW_LOG(INFO, "aos_check");
  if (system_zone && system_zone->introspect &&
      system_zone->introspect->check) {
    return system_zone->introspect->check(system_zone);
  }
  return 1;
}

static void aos_print(malloc_zone_t *zone, boolean_t verbose) {
  //ABSL_RAW_LOG(INFO, "aos_print");
  if (system_zone && system_zone->introspect &&
      system_zone->introspect->print) {
    system_zone->introspect->print(system_zone, verbose);
  }
}

static void aos_log(malloc_zone_t *zone, void *address) {
  //ABSL_RAW_LOG(INFO, "aos_log");
  if (system_zone && system_zone->introspect && system_zone->introspect->log) {
    system_zone->introspect->log(system_zone, address);
  }
}

static void aos_force_lock(malloc_zone_t *zone) {
  //ABSL_RAW_LOG(INFO, "aos_force_lock");
  if (system_zone && system_zone->introspect &&
      system_zone->introspect->force_lock) {
    system_zone->introspect->force_lock(system_zone);
  }
}

static void aos_force_unlock(malloc_zone_t *zone) {
  //ABSL_RAW_LOG(INFO, "aos_force_unlock");
  if (system_zone && system_zone->introspect &&
      system_zone->introspect->force_unlock) {
    system_zone->introspect->force_unlock(system_zone);
  }
}

static void aos_statistics(malloc_zone_t *zone, malloc_statistics_t *stats) {
  //ABSL_RAW_LOG(INFO, "aos_statistics");
  if (system_zone && system_zone->introspect &&
      system_zone->introspect->statistics) {
    system_zone->introspect->statistics(system_zone, stats);
  } else {
    bzero(stats, sizeof(malloc_statistics_t));
  }
}

static boolean_t aos_zone_locked(malloc_zone_t *zone) {
  //ABSL_RAW_LOG(INFO, "aos_zone_locked");
  if (system_zone && system_zone->introspect &&
      system_zone->introspect->zone_locked) {
    return system_zone->introspect->zone_locked(system_zone);
  }
  return 0;
}

static boolean_t aos_enable_discharge_checking(malloc_zone_t *zone) {
  //ABSL_RAW_LOG(INFO, "aos_enable_discharge_checking");
  if (system_zone && system_zone->introspect &&
      system_zone->introspect->enable_discharge_checking) {
    return system_zone->introspect->enable_discharge_checking(system_zone);
  }
  return 0;
}

static void aos_disable_discharge_checking(malloc_zone_t *zone) {
  //ABSL_RAW_LOG(INFO, "aos_disable_discharge_checking");
  if (system_zone && system_zone->introspect &&
      system_zone->introspect->disable_discharge_checking) {
    system_zone->introspect->disable_discharge_checking(system_zone);
  }
}

static void aos_discharge(malloc_zone_t *zone, void *memory) {
  //ABSL_RAW_LOG(INFO, "aos_discharge");
  if (system_zone && system_zone->introspect &&
      system_zone->introspect->discharge) {
    system_zone->introspect->discharge(system_zone, memory);
  }
}

static malloc_introspection_t aos_introspect;  // Zeroed

static malloc_zone_t *zone_default_get(void) {
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
    /*
     * Reset the value in case the failure happened after it was
     * set.
     */
    num_zones = 0;
  }

  if (num_zones) {
    //ABSL_RAW_LOG(INFO, "Got %d zones, returning %p", num_zones, zones[0]);
    return zones[0];
  }

  //ABSL_RAW_LOG(INFO, "Got 0 zones, returning %p", malloc_default_zone());
  return malloc_default_zone();
}

void InstallHooks() {
  if (ABSL_VLOG_IS_ON(1)) {
    ABSL_RAW_LOG(INFO, "Installing Proxy Malloc Zone");
  }
  //ABSL_RAW_LOG(INFO, "Installing Proxy Malloc Zone");
  //print_zones();

  // Initialize aos_zone.
  memset(&aos_zone, 0, sizeof(aos_zone));

  // Version 8 seems standard for modern macOS
  aos_zone.version = 8;
  aos_zone.zone_name = "aos_realtime_proxy_zone";

  aos_zone.size = aos_size;
  aos_zone.malloc = aos_malloc;
  aos_zone.calloc = aos_calloc;
  aos_zone.valloc = aos_valloc;
  aos_zone.free = aos_free;
  aos_zone.realloc = aos_realloc;
  aos_zone.destroy = aos_destroy;
  aos_zone.batch_malloc = aos_batch_malloc;
  aos_zone.batch_free = aos_batch_free;
  aos_zone.memalign = aos_memalign;
  aos_zone.free_definite_size = aos_free_definite_size;
  aos_zone.pressure_relief = aos_pressure_relief;
  aos_zone.claimed_address = aos_claimed_address;

  // We need to provide introspection struct even if empty/minimal to avoid
  // crashes? Copies usually have `introspect` from original. But we are a new
  // zone.
  aos_zone.introspect = &aos_introspect;
  memset(&aos_introspect, 0, sizeof(aos_introspect));
  aos_introspect.enumerator = aos_enumerator;
  aos_introspect.good_size = aos_good_size;
  aos_introspect.check = aos_check;
  aos_introspect.print = aos_print;
  aos_introspect.log = aos_log;
  aos_introspect.force_lock = aos_force_lock;
  aos_introspect.force_unlock = aos_force_unlock;
  aos_introspect.statistics = aos_statistics;
  aos_introspect.zone_locked = aos_zone_locked;
  aos_introspect.enable_discharge_checking = aos_enable_discharge_checking;
  aos_introspect.disable_discharge_checking = aos_disable_discharge_checking;
  aos_introspect.discharge = aos_discharge;

  // Important: To verify `free` works?
  // If we want `free` hooks, we might need `aos_claimed_address` to return
  // true? But then `aos_size` needs to work. If we claim it, we MUST be able to
  // handle it. If we forward `size` to `system_zone`, maybe that works?

  //ABSL_RAW_LOG(INFO, "Before register");
  //print_zones();
  // Register our zone.
  malloc_zone_register(&aos_zone);

  size_t loops = 0;
  malloc_zone_t *zone;
  do {
    //ABSL_RAW_LOG(INFO, "Before unregister system, system %p, default %p",
                 //system_zone, malloc_default_zone());
    //print_zones();

    // Promote to default by making it the "first" zone.
    // We do this by unregistering the system zone and re-registering it.
    // This bumps system zone to the end of the list, leaving aos_zone (added
    // just before) ahead of it.
    malloc_zone_unregister(system_zone);
    //ABSL_RAW_LOG(INFO, "Before register system, system %p, default %p",
                 //system_zone, malloc_default_zone());
    //print_zones();
    malloc_zone_register(system_zone);
    //ABSL_RAW_LOG(INFO, "end, system %p, default %p", system_zone,
                 //malloc_default_zone());
    //print_zones();

    zone = zone_default_get();

    ++loops;
    if (loops > 10) {
      ABSL_RAW_LOG(FATAL, "Too many loops");
    }
  } while (zone != &aos_zone);
}

void UninstallHooks() { malloc_zone_unregister(&aos_zone); }

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

    // Capture the current default zone.
    system_zone = zone_default_get();
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

}  // namespace aos
