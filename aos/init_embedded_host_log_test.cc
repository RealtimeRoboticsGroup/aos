// Tests InitEmbedded() in a process whose host already initialized absl's
// logging, which it can only do once.

#include <cstdlib>

#include "absl/log/absl_check.h"
#include "absl/log/initialize.h"

#include "aos/init.h"

int main() {
  absl::InitializeLog();

  aos::InitOptions options = aos::kEmbeddedInitOptions;
  options.initialize_log = false;
  aos::InitEmbedded(options);
  ABSL_CHECK(aos::IsInitialized());
  return EXIT_SUCCESS;
}
