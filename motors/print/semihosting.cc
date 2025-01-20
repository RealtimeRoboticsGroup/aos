#include "motors/print/semihosting.h"

#include "absl/types/span.h"

#include "motors/core/semihosting.h"

namespace frc::motors {

::std::unique_ptr<PrintingImplementation> CreatePrinting(
    const PrintingParameters & /*parameters*/) {
  return ::std::unique_ptr<PrintingImplementation>(new SemihostingPrinting());
}

extern "C" int _write(const int /*file*/, char *const ptr, const int len) {
  semihosting::Write operation{2 /* stderr */,
                               absl::Span<const char>(ptr, len)};
  return len - operation.Execute();
}

int SemihostingPrinting::WriteStdout(absl::Span<const char> buffer) {
  semihosting::Write operation{2 /* stderr */, buffer};
  return buffer.size() - operation.Execute();
}

}  // namespace frc::motors
