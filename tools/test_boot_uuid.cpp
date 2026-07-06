// Standalone utility to test and verify Windows boot UUID generation stability.
//
// How to run:
//   bazel run //tools:test_boot_uuid
//
// Verification scenarios to test:
//   1. Multi-Process Consistency: Run multiple times and verify the output UUID
//      remains identical.
//   2. NTP Invariance: Run, shift system clock (e.g., forward 5 minutes), run
//      again, and verify the UUID remains identical.
//   3. Reboot Uniqueness: Reboot the machine, run again, and verify the UUID
//      has changed.

#include <iostream>

#include "aos/uuid.h"

int main() {
  aos::UUID boot_uuid = aos::UUID::BootUUID();
  std::cout << "Boot UUID: " << boot_uuid.ToString() << "\n";
  return 0;
}
