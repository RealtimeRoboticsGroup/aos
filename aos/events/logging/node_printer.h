#include "absl/flags/flag.h"

#include "aos/aos_cli_utils.h"
#include "aos/events/event_loop.h"
#include "aos/events/simulated_event_loop.h"

ABSL_DECLARE_FLAG(std::string, name);
ABSL_DECLARE_FLAG(std::string, type);
ABSL_DECLARE_FLAG(bool, print);
ABSL_DECLARE_FLAG(int64_t, max_vector_size);
ABSL_DECLARE_FLAG(bool, pretty);
ABSL_DECLARE_FLAG(double, monotonic_start_time);
ABSL_DECLARE_FLAG(double, monotonic_end_time);
ABSL_DECLARE_FLAG(bool, hex);

namespace aos::logging {

// Creates a Printer object based on the command line flags that the user
// specified.
aos::Printer MakePrinter();

// Returns a test function that checks if a channel will be printed to the
// screen. This is called internally by NodePrinter. It can also be useful to
// determine whether a channel will be printed before instantiating a
// NodePrinter.
//
// The argument is the channel definition. The return value is true if the
// channel will be printed. The return value is false if the channel will not be
// printed.
std::function<bool(const aos::Channel *)> GetChannelShouldBePrintedTester();

// This class prints out all data from a node on a boot.
class NodePrinter {
 public:
  NodePrinter(aos::EventLoop *event_loop,
              aos::SimulatedEventLoopFactory *factory, aos::Printer *printer);

  // Tells the printer when the log starts and stops.
  void SetStarted(bool started, aos::monotonic_clock::time_point monotonic_now,
                  aos::realtime_clock::time_point realtime_now);

 private:
  struct MessageInfo {
    std::string node_name;
    std::unique_ptr<aos::RawFetcher> fetcher;
  };

  aos::SimulatedEventLoopFactory *factory_;
  aos::NodeEventLoopFactory *node_factory_;
  aos::EventLoop *event_loop_;

  std::string node_name_;

  bool started_ = false;

  aos::Printer *printer_ = nullptr;
};

}  // namespace aos::logging
