#include <string>
#include <vector>

#include "absl/log/check.h"

#include "aos/init.h"
#include "aos/util/log_to_mcap_lib.h"

// Converts an AOS log to an MCAP log that can be fed into Foxglove. To try this
// out, run:
//  bazel run //aos/util:log_to_mcap -- --node NODE_NAME /path/to/logfile
//
// Then navigate to http://studio.foxglove.dev (or spin up your own instance
// locally), and use it to open the file (this doesn't upload the file to
// foxglove's servers or anything).
int main(int argc, char *argv[]) {
  aos::InitGoogle(&argc, &argv);
  CHECK_GE(argc, 2) << ": Usage: " << argv[0] << " path/to/log [output.mcap]";

  std::string output_path = "/tmp/log.mcap";
  if (argc >= 3) {
    output_path = std::string(argv[argc - 1]);
    --argc;
  }
  std::vector<std::string> log_paths(argv + 1, argv + argc);
  return aos::util::ConvertLogToMcap(log_paths, output_path);
}
