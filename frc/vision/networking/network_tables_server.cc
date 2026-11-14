#include <networktables/NetworkTableInstance.h>

#include <iostream>
#include <thread>

#include "absl/flags/flag.h"

#include "aos/init.h"

ABSL_FLAG(unsigned int, nt_min_log_level, 7,
          "Min log level to use for network tables.");
ABSL_FLAG(unsigned int, nt_max_log_level, UINT_MAX,
          "Max log level to use for network tables.");

namespace frc::vision {
int Main() {
  nt::NetworkTableInstance instance = nt::NetworkTableInstance::GetDefault();
  instance.StartServer();
  instance.AddLogger(absl::GetFlag(FLAGS_nt_min_log_level),
                     absl::GetFlag(FLAGS_nt_max_log_level),
                     [](const nt::Event &event) {
                       const nt::LogMessage &log = *event.GetLogMessage();
                       std::cerr << log.filename << ":" << log.line << "("
                                 << log.level << "): " << log.message << "\n";
                     });

  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return 0;
}

}  // namespace frc::vision

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  return frc::vision::Main();
}
