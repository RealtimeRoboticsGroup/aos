#ifndef AOS_UTIL_LOG_TO_MCAP_LIB_H_
#define AOS_UTIL_LOG_TO_MCAP_LIB_H_

#include <functional>
#include <string>
#include <vector>

#include "aos/configuration_generated.h"
#include "aos/events/logging/log_reader.h"

namespace aos::util {

// Returns a test function that checks if a channel will be dropped from the
// final MCAP. This is called internally by ConvertLogToMcap. It can also be
// useful to determine whether to register send callbacks or not. If the work in
// those callbacks is expensive, then skipping that work can make log conversion
// quite a bit faster.
std::function<bool(const Channel *)> GetChannelShouldBeDroppedTester();

// Converts an AOS log to an MCAP log that can be fed into Foxglove.
//
// This function is intended to be called directly from main(). It's a separate
// file to let AOS users create their own "log_to_mcap" binaries. To try the AOS
// version out, run:
//  bazel run //aos/util:log_to_mcap -- --node NODE_NAME /path/to/logfile
//
// Then navigate to http://app.foxglove.dev (or use the desktop app),
// and use it to open the file (this doesn't upload the file to
// foxglove's servers or anything).
//
// The setup_callback function will be invoked after the underlying `LogReader`
// is instantiated. This allows the user to add their own send callbacks to
// modify the data, for example.
int ConvertLogToMcap(
    const std::vector<std::string> &log_paths, std::string output_path,
    std::function<void(logger::LogReader &)> setup_callback = {});

}  // namespace aos::util

#endif  // AOS_UTIL_LOG_TO_MCAP_LIB_H_
