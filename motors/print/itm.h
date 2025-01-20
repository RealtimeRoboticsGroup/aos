#ifndef MOTORS_PRINT_ITM_
#define MOTORS_PRINT_ITM_

#include "absl/types/span.h"

#include "motors/print/print.h"

namespace frc::motors {

// A printing implementation via the SWO (trace output) pin. This requires an
// attached debugger which is in SWD (Single Wire Debug) mode, has the SWO
// (also known as JTAG_TDO) pin hooked up, and software support.
//
// To decode the output from this, use motors/print/itm_read.py.
// To configure openocd to feed data to that:
//   tpiu config internal /tmp/itm.fifo uart off 120000000 192000
class ItmPrinting final : public PrintingImplementation {
 public:
  ItmPrinting();
  ~ItmPrinting() override = default;

  void Initialize() override {}

  // This goes to stimulus port 0.
  int WriteStdout(absl::Span<const char> buffer) override;
  // This goes to stimulus port 1.
  int WriteDebug(absl::Span<const char> buffer) override;
};

}  // namespace frc::motors

#endif  // MOTORS_PRINT_ITM_
