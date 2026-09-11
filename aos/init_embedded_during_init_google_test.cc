// Tests InitEmbedded() from a library thread while main() is still inside
// InitGoogle().  It has to wait for that initialization rather than run a
// second one.  A thread that gets there before main() starts InitGoogle() is
// the other order, where InitGoogle() dies; see init.h.

#include <atomic>
#include <cstdlib>
#include <thread>

#include "absl/log/absl_check.h"
#include "absl/log/globals.h"

#include "aos/init.h"

int main(int argc, char **argv) {
  ABSL_CHECK(absl::StderrThreshold() != absl::LogSeverityAtLeast::kInfo)
      << ": the threshold this waits for is already set";
  std::atomic<bool> waiting{false};
  std::thread library([&waiting]() {
    waiting = true;
    // InitGoogle() sets this near its start, so once it has, main() is inside
    // InitGoogle() and not yet done.
    while (absl::StderrThreshold() != absl::LogSeverityAtLeast::kInfo) {
    }
    aos::InitEmbedded();
    ABSL_CHECK(aos::IsInitialized());
  });
  // Already watching, so that it is not still starting up when InitGoogle()
  // finishes.
  while (!waiting) {
  }
  aos::InitGoogle(&argc, &argv);
  library.join();
  return EXIT_SUCCESS;
}
