#include "aos/ipc_lib/robust_ownership_tracker.h"

namespace aos::ipc_lib {

::std::string RobustOwnershipTracker::DebugString() const {
  ::std::stringstream s;
  const uint32_t futex =
      std::atomic_ref<uint32_t>(const_cast<uint32_t &>(mutex_.futex.value))
          .load(std::memory_order_relaxed);
  s << "{.tid=aos_mutex(" << ::std::hex << futex;

  if (futex != 0) {
    s << ":";
    if (mutex_owner_is_dead_from_value(futex)) {
      s << "FUTEX_OWNER_DIED|";
    }
    s << "tid=" << mutex_owner_from_value(futex);
  }

  s << "),}";
  return s.str();
}

}  // namespace aos::ipc_lib
