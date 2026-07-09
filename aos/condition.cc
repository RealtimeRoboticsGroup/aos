#include "aos/condition.h"

#include <cassert>
#include <cinttypes>
#include <ctime>

#include "absl/log/check.h"
#include "absl/log/log.h"

#include "aos/macros.h"
#include "aos/mutex/mutex.h"
#include "aos/time/time.h"
#include "aos/type_traits/type_traits.h"

namespace aos {

namespace chrono = ::std::chrono;

static_assert(shm_ok<Condition>::value,
              "Condition should work in shared memory");

Condition::Condition(Mutex *m) : impl_(), m_(m) {}

bool Condition::Wait() {
  const int ret = condition_wait(&impl_, &m_->impl_, nullptr);
  assert(AOS_LIKELY(ret == 0 || ret == 1));
  return ret == 1;
}

Condition::WaitResult Condition::WaitTimed(chrono::nanoseconds timeout) {
  struct timespec end_time;
  const bool do_timeout = timeout != chrono::nanoseconds(0);

  if (do_timeout) {
    end_time = aos::time::to_timespec(aos::monotonic_clock::now() + timeout);
  }

  const int ret =
      condition_wait(&impl_, &m_->impl_, do_timeout ? &end_time : nullptr);
  assert(AOS_LIKELY(ret == 0 || ret == 1 || ret == -1));
  switch (ret) {
    case 0:
      return WaitResult::kOk;
    case 1:
      return WaitResult::kOwnerDied;
    default:
      return WaitResult::kTimeout;
  }
}

void Condition::Signal() { condition_signal(&impl_, &m_->impl_); }

void Condition::Broadcast() { condition_broadcast(&impl_, &m_->impl_); }

}  // namespace aos
