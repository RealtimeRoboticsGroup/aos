#include <chrono>
#include <cstdint>
#include <ratio>

#include "aos/time/time.h"
#include "motors/core/kinetis.h"

// The systick interrupt increments this every 1ms.
extern "C" volatile uint32_t systick_millis_count;

namespace aos {

monotonic_clock::time_point monotonic_clock::now() noexcept {
  __disable_irq();
  const uint32_t current_counter = SYST_CVR;
  uint32_t ms_count = systick_millis_count;
  const uint32_t istatus = SCB_ICSR;
  __enable_irq();
  // If the interrupt is pending and the timer has already wrapped from 0 back
  // up to its max, then add another ms.
  if ((istatus & SCB_ICSR_PENDSTSET) && current_counter > 50) {
    ++ms_count;
  }

  // It counts down, but everything we care about counts up.
  const uint32_t counter_up = ((F_CPU / 1000) - 1) - current_counter;

  // "3.2.1.2 System Tick Timer" in the TRM says "The System Tick Timer's clock
  // source is always the core clock, FCLK".
  using systick_duration =
      std::chrono::duration<uint32_t, std::ratio<1, F_CPU>>;

  return time_point(std::chrono::round<std::chrono::nanoseconds>(
      std::chrono::milliseconds(ms_count) + systick_duration(counter_up)));
}

}  // namespace aos
