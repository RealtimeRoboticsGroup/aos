#ifndef MOTORS_PRINT_SEMIHOSTING_H_
#define MOTORS_PRINT_SEMIHOSTING_H_

#include "absl/types/span.h"

#include "motors/print/print.h"

namespace frc::motors {

// A printing implementation which uses the ARM semihosting interface. This
// requries an attached debugger with software support.
//
// You have to do "arm semihosting enable" in openocd to enable this.
// It also seems to be broken with the usb-tiny-h in the openocd version we're
// using, but works fine with the st-link-v2.
// It may also only work if you do this immediately after starting openocd.
//
// Note that this implementation has strange effects on timing even of
// interrupts-disabled code and is in general extremely slow.
class SemihostingPrinting final : public PrintingImplementation {
 public:
  SemihostingPrinting() = default;
  ~SemihostingPrinting() override = default;

  void Initialize() override {}

  int WriteStdout(absl::Span<const char> buffer) override;

  // Could easily implement an optional WriteDebug which goes to a separate
  // file if the name is filled out in the parameters.
};

}  // namespace frc::motors

#endif  // MOTORS_PRINT_SEMIHOSTING_H_
